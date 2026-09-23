#ifndef SELF_TEST_H
#define SELF_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

// 运行自检流程；内部循环打印结果，可阻塞当前任务
void self_test_run(void);

#ifdef __cplusplus
}
#endif

#endif // SELF_TEST_H
