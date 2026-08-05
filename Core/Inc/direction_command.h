#ifndef DIRECTION_COMMAND_H
#define DIRECTION_COMMAND_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "dronecan_dshot.DirectionCommand.h"

/*
 * 允许发送电机换向命令的电脑端 DroneCAN 节点 ID。
 * CAN 硬件过滤器、libcanard 接收判断和本模块的软件校验共用这个定义，
 * 防止以后修改节点 ID 时漏改其中一层。
 */
#define DIRECTION_COMMAND_ALLOWED_SOURCE_NODE_ID  126U

/**
 * @brief 初始化电机换向请求和执行状态。
 *
 * 应在MotorControl_Init()之后、开始接收DirectionCommand之前调用。
 */
void DirectionCommand_Init(void);

/**
 * @brief 校验并暂存一条电机方向设置请求。
 *
 * 本函数只负责“接收入口”这一小步，不会立即发送 DShot 特殊命令。只有满足
 * 以下条件的请求才会被暂存，等待后续状态机处理：
 *   - command 指针有效；
 *   - 来源节点 ID 等于 DIRECTION_COMMAND_ALLOWED_SOURCE_NODE_ID；
 *   - 协议版本、操作类型和确认值与自定义 DSDL 定义一致；
 *   - motor_mask 至少选中一路电机；
 *   - 当前 8 路 DShot 油门命令全部为停止；
 *   - 当前没有其它待处理请求，且 request_id 没有被重复提交。
 *
 * source_node_id 和 timestamp_usec 虽然当前不参与 DShot 波形生成，但后续需要
 * 用于来源审计、请求超时和执行结果记录，因此从一开始就在接口中保留。
 *
 * @param command        DirectionCommand DSDL 解码得到的结构体地址。
 * @param source_node_id 发送该 DroneCAN transfer 的节点 ID。
 * @param timestamp_usec 完整接收该 transfer 的微秒时间戳。
 *
 * @retval true  请求通过校验并已暂存。
 * @retval false 请求无效、重复、电机未停止或已有请求正在等待处理。
 */
bool DirectionCommand_Submit(
    const struct dronecan_dshot_DirectionCommand* command,
    uint8_t source_node_id,
    uint64_t timestamp_usec);

/**
 * @brief 在主循环中推进一次电机换向状态机。
 *
 * 该函数不会阻塞等待，而是根据now_usec判断当前步骤是否已经保持足够时间。
 * 主循环每次调用只推进必要的一步，DShot帧仍由TIM7按固定1.5 ms周期发送。
 *
 * @param now_usec 当前单调递增微秒时间戳。
 */
void DirectionCommand_Poll(uint64_t now_usec);

/**
 * @brief 查询是否有换向请求正在等待或执行。
 *
 * 返回true期间必须忽略普通RawCommand，避免油门命令覆盖DShot特殊命令序列。
 */
bool DirectionCommand_IsBusy(void);

#ifdef __cplusplus
}
#endif

#endif /* DIRECTION_COMMAND_H */
