# Cattle Tracker — MQTT Protocol v1

Status: **DRAFT v1.0** — 2026-11-20 (rev. 2: `ts_src=rtc`, failed attempts reset timers). For review by firmware (Yaozong) and backend.
Firmware branch: `p1-protocol`. Topic prefix: `ct/v1/`.

This document is the contract between the collar firmware and any backend
(the production backend, and FleetManager, which is used as a test harness
until the production broker is ready). Both sides implement exactly this.

---

## 1. Ground rules

### 1.1 The device is offline almost all the time

The collar sleeps between reports with the cellular modem **powered off**.
It is reachable only during the ~10–60 s it is connected to publish.
Consequences:

- **Every downlink message MUST be published `retained`, QoS 1.**
  The device uses a clean session; non-retained messages sent while it
  sleeps are lost.
- **Downlink latency is up to one heartbeat interval** (default 10 min in
  development, several hours in production). The backend UI must show
  "sent, waiting for device" vs "applied by device" — see §6.
- **Offline detection is by silence**, not by MQTT session state.
  A device is "offline" when no `telemetry` has arrived for
  `1.5 × heartbeat_min`. Last Will is not used (the device disconnects
  cleanly every cycle, so a Will would never fire on a real failure).

### 1.2 The server cannot trigger a stimulus

There is **no message in this protocol that delivers a shock**.
The server can only set a macro permission (`stim.enabled`, §5.2) and,
later, the fence geometry (§5.4). Whether and when to warn / stimulate is
decided entirely by firmware fence logic, with hard-coded limits the server
cannot change. `stim.enabled = false` always wins.

### 1.3 Conventions

| Item | Convention |
|---|---|
| Encoding | UTF-8 JSON, one object per message, no trailing newline |
| Time | `ts` = Unix seconds, UTC |
| Coordinates | WGS-84 decimal degrees, 6 decimals (~0.1 m) |
| Units | in the field name where not obvious: `_s`, `_min`, `_mv`, `_mg`, `_kmh` |
| Unknown / not measured | field omitted, or `null` where stated |
| Size | uplink ≤ 1 KB per message; downlink limit TBD (see §9) |

---

## 2. Device identity

| | Value | Where it comes from |
|---|---|---|
| `id` | `CT-001`, `CT-002`, … | Written once into the device's NVS over the serial console (factory / first flash). Survives firmware updates. |
| Fallback `id` | `IMEI-<15 digits>` | Used when NVS has no `id` yet. Backend should list such devices as **unregistered**. |
| `imei` | 15 digits | Read from the modem at every boot; reported in `status` (§4.2). |

`id` is the topic segment `{id}` below, and the MQTT Client ID.

---

## 3. Topics

| Topic | Dir | QoS | Retained | Sent when |
|---|---|---|---|---|
| `ct/v1/{id}/status` | up | 1 | **yes** | every connection, first |
| `ct/v1/{id}/telemetry` | up | 1 | no | every report (motion / heartbeat / boot / manual) |
| `ct/v1/{id}/event` | up | 1 | no | reserved — fence events, not used in v1.0 |
| `ct/v1/{id}/config` | down | 1 | **yes** | whenever backend changes settings |
| `ct/v1/{id}/stim` | down | 1 | **yes** | whenever the stimulus permission changes |
| `ct/v1/all/stim` | down | 1 | **yes** | herd-wide stimulus kill switch |
| `ct/v1/{id}/fence` | down | 1 | **yes** | reserved — format TBD (§5.4) |

### 3.1 One connection, in order

```
CONNECT  (clientId = id, clean session, keepalive 60 s)
SUBSCRIBE ct/v1/{id}/config, ct/v1/{id}/stim, ct/v1/all/stim     (QoS 1)
  wait ≤ 3 s for retained messages; validate; apply; save to NVS
PUBLISH  ct/v1/{id}/status      (retained)  — includes the versions now applied
PUBLISH  ct/v1/{id}/telemetry
DISCONNECT
```

`status` is published **after** downlinks are applied, so it is the
acknowledgement of what the device is now running.

---

## 4. Uplink

### 4.1 `telemetry`

```json
{
  "ts": 1763625600, "ts_src": "gnss",
  "boot": 7, "seq": 1234,
  "reason": "heartbeat",
  "gps": { "lat": -1.292066, "lng": 36.821946, "alt": 1661.2,
           "spd_kmh": 0.4, "hdg": 212, "hdop": 0.9, "sats": 14, "ttff_s": 3.2 },
  "bat": { "mv": 3910, "soc": 72 },
  "act": { "event": "still", "act_mg": 0, "still_min": 41 },
  "net": { "rssi": -87 },
  "stim": false
}
```

| Field | Type | Meaning |
|---|---|---|
| `ts` | int | Unix seconds UTC of this report. `0` if no time source yet |
| `ts_src` | string | `gnss` (from RMC date+time this cycle) · `net` (modem network time this cycle) · `rtc` (device clock, set by an earlier `gnss`/`net` sync and kept running through deep sleep) · `none` (never synced since power-on) |
| `boot` | int | Power-on counter (NVS). Increments on every cold boot / reset, not on deep-sleep wake |
| `seq` | int | Message counter within this `boot`, starts at 1. Dedup key = (`id`, `boot`, `seq`) |
| `reason` | string | `motion` · `heartbeat` · `boot` (first report after power-on) · `manual` (IO0 double-click) |
| `gps` | object \| `null` | `null` when no fix was obtained this cycle |
| `gps.lat` / `gps.lng` | number | degrees |
| `gps.alt` | number | metres above MSL (from GGA); omitted if unavailable |
| `gps.spd_kmh` / `gps.hdg` | number | ground speed, course over ground (deg) |
| `gps.hdop` / `gps.sats` | number / int | fix quality — backend should treat `hdop > 5` as unreliable |
| `gps.ttff_s` | number | time from GNSS wake to fix this cycle; omitted for `manual` reports (position taken from the last fix, no wait) |
| `fix_fail` | bool | present and `true` only when a fix was attempted and timed out |
| `bat.mv` / `bat.soc` | int | battery voltage (mV) / estimated state of charge (%) |
| `act.event` | string | motion classification this cycle: `motion` · `still` · `doubt` |
| `act.act_mg` | int | mean frame-to-frame acceleration delta over the 5 s window; `-1` = not sampled |
| `act.still_min` | int | minutes since last `motion` classification |
| `net.rssi` | int | dBm, from `AT+CSQ`; omitted if unknown |
| `stim` | bool | effective stimulus permission on the device (§5.2) |

### 4.2 `status` (retained)

```json
{
  "ts": 1763625600,
  "id": "CT-001", "imei": "865058080796511", "iccid": "8986…",
  "fw": "0.3.0", "hw": "Cattle_T-A",
  "boot": 7, "reset": "deepsleep",
  "applied": { "cfg_ver": 3, "stim_ver": 2, "all_stim_ver": 1, "fence_ver": 0 },
  "cfg": { "heartbeat_min": 10, "motion_min_s": 300, "check_s": 60,
           "fix_timeout_s": 90, "hb_fix_timeout_s": 30, "motion_th_mg": 20 },
  "err": null
}
```

| Field | Meaning |
|---|---|
| `reset` | ESP32 reset reason of this boot: `poweron` · `deepsleep` · `sw` · `panic` · `wdt` · `brownout` · `other` |
| `applied.*_ver` | version of each downlink currently in effect; `0` = never received (using firmware defaults) |
| `cfg` | the **effective** configuration (after defaults and validation) |
| `err` | `null`, or `{ "topic": "config", "ver": 4, "reason": "heartbeat_min must be an integer 5-1440" }` for the last rejected downlink. `topic` is `config` · `stim` · `all/stim`. Cleared when a later message on the same topic is applied |

Because it is retained, `status` always holds the device's latest known
identity, firmware and applied versions — even while it sleeps.

### 4.3 `event` — reserved

Planned types once fence logic exists: `warning`, `stimulus`,
`stim_suppressed`, `escape`, `fence_fault`, `low_battery`.
Format will be defined with the fence design.

---

## 5. Downlink

All downlinks: **retained, QoS 1**, JSON object with an integer `ver`.

Device behaviour, for every downlink topic:

1. `ver` equal to the applied version → ignore (already applied).
2. `ver` different → validate the **whole** message. Any field invalid →
   reject the whole message, keep the previous one, report in `status.err`.
3. Valid → apply from the next cycle, save to NVS, report in `status.applied`.
4. Empty retained payload (topic cleared) → revert to firmware defaults
   (`config`) or `enabled: false` (`stim`).

"Different" rather than "greater", so that a backend database reset
(versions restarting at 1) does not lock devices out.

### 5.1 `config`

```json
{ "ver": 3, "heartbeat_min": 10, "motion_min_s": 300, "check_s": 60,
  "fix_timeout_s": 90, "hb_fix_timeout_s": 30, "motion_th_mg": 20 }
```

| Field | Default | Range | Meaning |
|---|---|---|---|
| `heartbeat_min` | 10 | 5 – 1440 | **Heartbeat interval.** A still animal reports this often (any report restarts the count). The backend marks a collar offline after 1.5 × this with no telemetry |
| `motion_min_s` | 300 | 60 – 3600 | **Report interval while moving.** A moving animal gets one `motion` report this often; after a `motion` report the collar sleeps until the next one is due |
| `check_s` | 60 | 10 – 600 | **Re-check after a small jolt.** Woken by the accelerometer but not really moving (no report): look again after this long. Also the shortest sleep, so keep it ≤ `motion_min_s` (if larger, it wins). Polling build: how often to wake and check |
| `fix_timeout_s` | 90 | 30 – 300 | **GPS wait — moving / boot / manual.** Longest wait for a fix; reports anyway (`fix_fail`) if none |
| `hb_fix_timeout_s` | 30 | 0 – 120 | **GPS wait — heartbeat.** Same for heartbeats. `0` = heartbeat without GNSS |
| `motion_th_mg` | 20 | 5 – 200 | **Moving threshold.** Mean frame-to-frame acceleration delta over the 5 s sample; at or above this = moving. (Waking up needs a 96 mg jolt first — fixed in firmware) |

Sleep in one line: still → until the next heartbeat; moving → one report every `motion_min_s`;
a jolt that isn't real movement → look again after `check_s`. Never longer than the next heartbeat.

Omitted fields take their default. Unknown fields are ignored (forward
compatibility).

Any report resets the heartbeat timer — a `motion` report counts as
"not silent".

A report **attempt that fails** (no coverage, broker unreachable) also resets
the timers. The device does not retry every `check_s` — in a dead zone that
would drain the battery — it tries again at the next heartbeat or motion
report. Backend effect: a collar in a dead zone goes "offline" after
`1.5 × heartbeat_min` and recovers on its own when coverage returns.

### 5.2 `stim` and `all/stim`

```json
{ "ver": 2, "enabled": false }
```

- Effective permission = `ct/v1/{id}/stim.enabled` **AND** `ct/v1/all/stim.enabled`.
- `ct/v1/{id}/stim` never received → `false` (safe default).
- `ct/v1/all/stim` never received → does not block (it is a kill switch; the
  backend should publish `{ "ver": 1, "enabled": true }` once at setup).
- `enabled: true` means the firmware fence logic **may** warn and stimulate.
  It never causes a stimulus by itself.
- v1.0 firmware has no fence logic yet: the value is stored and reported
  (`telemetry.stim`, `status.applied`) but has no effect.

### 5.3 What the backend shows

| Backend state | Condition |
|---|---|
| Applied | `status.applied.<x>_ver` == version last published |
| Pending | published version ≠ `status.applied.<x>_ver`, device not yet reconnected |
| Rejected | `status.err.ver` == version last published |
| Offline | no `telemetry` for `1.5 × heartbeat_min` |

### 5.4 `fence` — reserved

Format depends on how fence decisions are made (GNSS-native geofence vs
firmware point-in-polygon); bench measurements are in progress.
Expected: list of polygons, WGS-84, with a firmware vertex limit.
**Backends should store fences server-side now** but not publish to this
topic until v1.1 defines it.

---

## 6. Firmware defaults and timing (development)

| | Value |
|---|---|
| Heartbeat | 10 min |
| Motion report min interval | 5 min |
| Accelerometer check (polling mode) | 60 s |
| GNSS timeout — motion / heartbeat | 90 s / 30 s |
| Downlink wait after SUBSCRIBE | 3 s |

Production values will be pushed via `config`, no reflash needed.

---

## 7. Connection and security

| | v1.0 (now, test broker) | Production (backend team) |
|---|---|---|
| Transport | TCP 1883 | **TLS 8883** (EG800 `AT+QSSLCFG`, CA cert stored on modem) |
| Credentials | shared test account | **per-collar** username/password (or client cert), stored in device NVS |
| Client ID | `{id}` | `{id}` |
| ACL (recommended) | — | device `{id}` may publish only `ct/v1/{id}/status|telemetry|event`, subscribe only `ct/v1/{id}/config|stim|fence` and `ct/v1/all/stim` |

---

## 8. Versioning

- Breaking changes → new prefix `ct/v2/`. Devices and backend can run both during migration.
- Additive changes (new optional fields) stay in v1; receivers ignore unknown fields.

---

## 9. Open questions (for backend team)

1. **Downlink size limit** — EG800 receive buffer limits the retained message size. Firmware will measure and fix a maximum (expected ≥ 1 KB); fence geometry must fit.
2. **Per-collar credentials format** — username/password or X.509 client certificates? How are they generated and handed to the factory for writing into NVS?
3. **`all/stim` scope** — one herd-wide switch, or per farm / per group (`ct/v1/group/{gid}/stim`)?
4. **Time** — is device `ts` trusted for history, or does the backend also stamp receive time? (Recommend: store both.)
5. **Retention** — how long is telemetry history kept server-side?
