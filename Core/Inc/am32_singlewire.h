#ifndef AM32_SINGLEWIRE_H
#define AM32_SINGLEWIRE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* 当前转接板的DShot1..DShot8分别使用索引0..7。 */
#define AM32_SINGLEWIRE_CHANNEL_COUNT  8U

/** 单线物理层一次收发的结果。 */
typedef enum
{
  AM32_SINGLEWIRE_OK = 0,
  AM32_SINGLEWIRE_BUSY,
  AM32_SINGLEWIRE_INVALID_ARGUMENT,
  AM32_SINGLEWIRE_TIMEOUT,
  AM32_SINGLEWIRE_FRAMING_ERROR
} AM32SingleWireStatus;

/**
 * @brief 初始化AM32单线通信状态和接收EXTI中断。
 *
 * TIM6的时钟、1 MHz计数基准和NVIC由CubeMX生成的MX_TIM6_Init()配置。
 * 本模块只在收发期间动态启动/停止TIM6并修改ARR采样间隔。19200 baud的
 * 一个bit使用52 us，接收起始位由所选GPIO的下降沿EXTI捕获，再在各数据位中心采样。
 * 此函数不会改变DShot引脚的复用模式，只有Transfer开始后才操作选中通道。
 */
void AM32SingleWire_Init(void);

/**
 * @brief 由CubeMX生成的TIM6_DAC_IRQHandler在一次更新事件处理后调用。
 *
 * 函数推进一个19200 baud发送bit或接收采样点；应用层不得直接调用。
 */
void AM32SingleWire_HandleTimerPeriodElapsed(void);

/**
 * @brief 在一根DShot信号线上完成一次半双工“发送后接收”。
 *
 * 发送期间引脚为推挽输出，空闲/停止位为高；最后一个停止位结束后立即切换
 * 为输入上拉并打开下降沿EXTI，因此不会漏掉电调回复的起始位。数据格式固定
 * 为19200 baud、8N1、LSB first。函数等待中断状态机完成，但全局中断保持开启，
 * 所以CAN接收和看门狗等系统功能不会因逐bit忙等而被整体关闭。
 *
 * @param channel_index DShot通道索引，0..7对应DShot1..DShot8。
 * @param tx_data       要连续发送的数据；tx_length大于0时不可为空。
 * @param tx_length     发送字节数，最大32。
 * @param rx_data       接收缓冲区；rx_length大于0时不可为空。
 * @param rx_length     期望接收的准确字节数，最大64；为0表示只发送。
 * @param timeout_ms    包含发送和接收在内的总超时，必须大于0。
 */
AM32SingleWireStatus AM32SingleWire_Transfer(
    uint8_t channel_index,
    const uint8_t* tx_data,
    uint8_t tx_length,
    uint8_t* rx_data,
    uint8_t rx_length,
    uint32_t timeout_ms);

/**
 * @brief 中止当前单线事务并把所选信号线恢复成输入上拉。
 *
 * 查询超时和维护模式异常退出时可安全调用；没有事务时调用也无副作用。
 */
void AM32SingleWire_Abort(void);

/** @brief 返回当前是否仍有发送或接收事务正在进行。 */
bool AM32SingleWire_IsBusy(void);

#ifdef __cplusplus
}
#endif

#endif /* AM32_SINGLEWIRE_H */
