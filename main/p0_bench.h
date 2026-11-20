#ifndef P0_BENCH_H
#define P0_BENCH_H

/* P0 实验固件入口（P0_BENCH=1 时由 app_main 调用，不返回控制权给业务逻辑）。
 * 用法见 p0_bench.c 顶部注释，或串口输入 `p help`。 */
void p0_bench_start(void);

#endif
