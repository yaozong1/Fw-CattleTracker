#ifndef FENCE_BENCH_H
#define FENCE_BENCH_H

/* 电子围栏台架测试控制台。
 * 只在 FENCE_BENCH=1 的构建里编译并调用；生产固件不包含这条路径。
 * 调用后会注册命令并启动 REPL，函数本身立即返回（REPL 跑在自己的任务里）。 */
void fence_bench_start(void);

#endif // FENCE_BENCH_H
