#include "ct_config.h"
#include "hardware_config.h"

static const ct_cfg_t s_defaults = {
    .ver              = 0,
    .heartbeat_min    = CT_CFG_HEARTBEAT_MIN,
    .motion_min_s     = CT_CFG_MOTION_MIN_S,
    .check_s          = CT_CFG_CHECK_S,
    .fix_timeout_s    = CT_CFG_FIX_TIMEOUT_S,
    .hb_fix_timeout_s = CT_CFG_HB_FIX_TIMEOUT_S,
    .motion_th_mg     = MOTION_MEAN_DELTA_TH_MG,
};

const ct_cfg_t *ct_cfg(void)
{
    return &s_defaults;
}
