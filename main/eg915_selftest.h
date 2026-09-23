#ifndef EG915_SELFTEST_H
#define EG915_SELFTEST_H

#include <stdbool.h>

// 自测任务（FreeRTOS任务入口），由 app_main 直接 xTaskCreate 调用
// 注意：函数名按要求不用 "eg915_at" 前缀
void Selftest_task(void* pv);

// 单次AT握手（发送"AT"并等待"OK"），返回是否成功
bool eg915_at_handshake_once(unsigned timeout_ms, int retries);

#endif // EG915_SELFTEST_H
