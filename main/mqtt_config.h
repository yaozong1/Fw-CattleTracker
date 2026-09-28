#ifndef MQTT_CONFIG_H
#define MQTT_CONFIG_H

/* ---------------- MQTT 服务器 ----------------
 * 自建 broker（47.89.154.60）。
 * 后端 Web 端走 ws://47.89.154.60:8083/mqtt（WebSocket），
 * 但 EG800 的 AT+QMTOPEN 只能跑裸 TCP 上的原生 MQTT，握不了 WebSocket，
 * 所以这里用同一台机器的 1883 端口，两边数据是通的。
 *
 * 直接写 IP 不写域名：原来的 processengineeringsz.com 已过期
 * （现在解析到万网的 expired.hichina.com 停放页），而且省一次 DNS 解析。
 * ⚠️ 改这个文件时注意别被旧的编辑器缓冲区覆盖回去。 */
#define MQTT_BROKER_HOST    "47.89.154.60"
#define MQTT_BROKER_PORT    1883                 // 1883=明文TCP(模组用这个), 8083=WebSocket(后端用), 8883=TLS

// MQTT客户端认证信息（与后端 MY_PUBLIC_MQTT_USERNAME / PASSWORD 一致）
#define MQTT_CLIENT_ID      "esp32_cattle_tracker_001"      // 客户端ID，确保唯一
#define MQTT_USERNAME       "testuser"             // MQTT用户名
#define MQTT_PASSWORD       "h3341136"             // MQTT密码

/* ---------------- MQTT 主题 ----------------
 * fleet manager 订阅的是 fleet/+/battery，并且**从主题的第二段取设备 ID**
 * （src/services/mqtt-service.ts: topicParts[1]）。
 * 所以主题必须是 fleet/<deviceId>/battery，deviceId 在运行时由模组 IMEI 填入，
 * 不能在这里写死 —— 见 mqtt_module.c 的 mqtt_build_topic()。
 */
#define MQTT_TOPIC_ROOT     "fleet"      // 主题根，需与后端订阅前缀一致
#define MQTT_TOPIC_SUB_BATTERY "battery" // fleet/<id>/battery（后端已订阅）
#define MQTT_TOPIC_SUB_STATUS  "status"  // fleet/<id>/status （后端已订阅，暂未使用）
#define MQTT_TOPIC_SUB_GPS     "gps"     // fleet/<id>/gps    （后端**没有**订阅）

// MQTT QoS等级
// 建议当前阶段使用QoS0以简化发布流程（无需msgId确认，移动网络更稳）
#define MQTT_QOS_LEVEL      0                           // 0=最多一次, 1=至少一次, 2=只有一次

// MQTT保持连接间隔（秒）
#define MQTT_KEEPALIVE      60

// MQTT重连设置
#define MQTT_RECONNECT_TIMEOUT_MS    5000               // 重连超时时间
#define MQTT_MAX_RECONNECT_ATTEMPTS  5                  // 最大重连次数

// 网络APN配置（适用于4G模组）
// 注意：如果APN_NAME为空字符串，将跳过APN设置
// 常见APN: "cmnet", "3gnet", "ctnet", "uninet", "wonet"
#define APN_NAME            ""                          // 网络接入点名称
#define APN_USERNAME        ""                          // APN用户名（通常为空）
#define APN_PASSWORD        ""                          // APN密码（通常为空）

#endif // MQTT_CONFIG_H