#ifndef __DRONECAN_APP_H__
#define __DRONECAN_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * @brief 初始化 DroneCAN 应用层。
 *
 * CubeMX 只负责生成 CAN 外设的基础初始化代码，例如时钟、GPIO、
 * CAN 位时序和 NVIC 中断。本函数负责更上层的 DroneCAN 启动流程：
 *   1. 初始化 libcanard 协议实例；
 *   2. 配置 DroneCAN 接收用的 bxCAN acceptance filter；
 *   3. 启动 CAN1；
 *   4. 打开 RX FIFO0 接收通知。
 *
 * 当前第一步只把 DroneCAN 基础接收环境搭起来，暂时不启动 DShot 输出，
 * 也暂时不解析 DroneCAN 消息。后续会在接收链路稳定后逐步接上。
 *
 * 本阶段补充：CAN 帧已经会被转交给 libcanard 解析基础 transfer 信息；
 * RawCommand 解码和 DShot 输出仍然没有接入。
 * 本阶段再次补充：RawCommand 已能被接受、解码并暂存供调试观察；
 * DShot 输出仍然没有接入。
 *
 * @retval HAL_OK    DroneCAN 基础层已准备好，可以接收 CAN 中断。
 * @retval HAL_ERROR 启动流程中的某一步失败。
 */
HAL_StatusTypeDef DroneCAN_App_Init(void);

/**
 * @brief 在主循环中执行 DroneCAN 的轻量级周期任务。
 *
 * 当前第一版暂时没有实际任务。后续这个函数会逐步承担：
 *   - 把 CAN RX 队列里的帧交给 libcanard；
 *   - 发送 libcanard 中等待发送的 DroneCAN TX 帧；
 *   - 清理超时的 DroneCAN 传输状态；
 *   - 执行油门超时和 failsafe 检查。
 *
 * 本阶段补充：CAN RX 队列到 libcanard 的接收桥接已经接入；
 * TX、超时清理、油门 failsafe 仍然留到后续步骤。
 */
void DroneCAN_App_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRONECAN_APP_H__ */
