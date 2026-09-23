#ifndef RS485_MODULE_H
#define RS485_MODULE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// RS485 封装（基于 UART）
// 默认引脚：TX=IO10 -> 收发器 DI，RX=IO9 <- 收发器 RO
// 默认端口：UART0（与 USB-Serial-JTAG 分离，通常可用）

#ifdef __cplusplus
extern "C" {
#endif

// 初始化 RS485
// baudrate: 波特率，如 9600/115200
// 返回 true 则初始化成功
bool rs485_init(uint32_t baudrate);

// 写入数据
// data: 待发送数据指针
// len:  数据长度
// timeout_ticks: 发送完成等待的 tick 超时（FreeRTOS ticks，注意不是毫秒）
// 返回已写入字节数（>=0），负数表示失败
int rs485_write(const uint8_t* data, size_t len, uint32_t timeout_ticks);

// 读取数据
// buf:  接收缓冲区
// maxlen: 最大读取长度
// timeout_ticks: 读取阻塞等待的 tick 超时（注意不是毫秒）
// 返回实际读取字节数（>=0），负数表示失败
int rs485_read(uint8_t* buf, size_t maxlen, uint32_t timeout_ticks);

// 刷新输入/输出缓冲
void rs485_flush(void);

// 反初始化 RS485
void rs485_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // RS485_MODULE_H