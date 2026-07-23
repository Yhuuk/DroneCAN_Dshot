#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "dshot.h"

/* DroneCAN uavcan.equipment.esc.RawCommand 中 int14 字段的有效范围。 */
#define MOTOR_CONTROL_RAW_COMMAND_MIN_VALUE  (-8192)
#define MOTOR_CONTROL_RAW_COMMAND_MAX_VALUE  8191

/* 当前硬件提供 DShot1..DShot8，共 8 路电机输出。 */
#define MOTOR_CONTROL_DSHOT_OUTPUT_COUNT      8U

/* 连续 100 ms 没有收到新的有效 RawCommand 时，全部电机命令进入停止状态。 */
#define MOTOR_CONTROL_RAW_COMMAND_TIMEOUT_USEC 100000ULL

/**
 * @brief 初始化电机控制缓存，使8路输出都处于有效的DShot停止状态。
 *
 * DShot停止帧虽然数值是0x0000，但每个逻辑0仍需要对应的PWM高电平时间，
 * 所以不能只依赖RAM上电清零。本函数会同时准备命令、16-bit帧和CCR缓存，
 * 应在开始周期发送DShot之前调用一次。
 */
void MotorControl_Init(void);

/**
 * @brief 立即把8路输出恢复为DShot停止帧，并取消当前命令的新鲜状态。
 *
 * 该接口供启动互锁、映射错误和上层故障保护共同使用。调用后固定周期发送器
 * 仍可继续发送有效的0x0000停止帧，但必须收到新的有效RawCommand后，
 * MotorControl_HasFreshRawCommand()才会重新返回true。
 */
void MotorControl_ForceStop(void);

/**
 * @brief 查询当前是否保存着一条尚未超时的有效RawCommand。
 *
 * @return true表示最近收到的命令有效且未超过100 ms；否则返回false。
 */
bool MotorControl_HasFreshRawCommand(void);

/**
 * @brief 查询当前8路DShot命令是否全部为停止命令0。
 *
 * @return true表示8路全部为0；任一路为普通油门命令时返回false。
 */
bool MotorControl_AreAllDShotCommandsStopped(void);

/**
 * @brief 把一个 DroneCAN RawCommand 值映射成单向 DShot 命令。
 *
 * 当前采用安全的单向电调策略：
 *   - RawCommand -8192..0 映射成 DShot 0，表示停止；
 *   - RawCommand 1..8191 线性映射到 DShot 48..2047；
 *   - 不会由普通油门映射产生 DShot 1..47 的特殊命令。
 *
 * 该函数只进行数值映射，不生成 16-bit DShot 帧，也不操作 TIM、DMA、
 * CCR 或 GPIO。映射成功后，再把 out_dshot_command 交给 DShot_BuildFrame()。
 *
 * @param raw_command       从 RawCommand 解码得到的一个 int14 油门值。
 * @param out_dshot_command 输出映射后的 DShot 命令，范围是 0 或 48..2047。
 *
 * @retval true  映射成功，out_dshot_command 已被写入。
 * @retval false 输入超出 RawCommand 有效范围或输出指针为空，输出保持不变。
 * 
 * 改为了static函数，避免外部调用。外部调用MotorControl_UpdateDShotCommands()即可。
 */
// bool MotorControl_MapRawCommandToDShot(int16_t raw_command,
//                                        uint16_t* out_dshot_command);

/**
 * @brief 把一条 RawCommand 中的多路数据映射并保存为 8 路 DShot 命令。
 *
 * RawCommand 数组与 DShot 输出按索引一一对应：data[0] 对应 DShot1，
 * data[1] 对应 DShot2，以此类推。当前硬件最多使用前 8 个元素：
 *   - 少于 8 路时，缺少的输出保存为 DShot 0（停止）；
 *   - 多于 8 路时，只处理前 8 路，其余元素由本节点忽略；
 *   - 数组为空且长度为 0 时，8 路全部保存为停止；
 *   - 指针无效或任一数值越界时，8 路全部保存为停止并返回 false。
 *
 * 该函数只保存 0 或 48..2047 的 DShot 命令值，尚不生成 16-bit 帧，
 * 也不会操作 TIM、DMA、CCR 或 GPIO。
 * 本阶段补充：8 路命令现在会继续编码并保存为 8 个 16-bit DShot 帧；
 * 仍然不会操作 TIM、DMA、CCR 或 GPIO。
 * 本阶段再次补充：8 个帧现在会继续转换并保存为 8×16 个 CCR 数值；
 * 这些数值只保存在 RAM 中，尚未写入 TIM 的 CCR 寄存器。
 *
 * @param raw_commands      RawCommand 解码后的 cmd.data 数组。
 * @param raw_command_count RawCommand 解码后的 cmd.len。
 * @param timestamp_usec    该条 RawCommand 的接收时间戳，单位为微秒。
 *
 * @retval true  前 8 路范围内的数据全部映射并保存成功。
 * @retval false 输入无效，8 路命令已安全地全部置为停止。
 */
bool MotorControl_UpdateDShotCommands(const int16_t* raw_commands,
                                      uint8_t raw_command_count,
                                      uint64_t timestamp_usec);

/**
 * @brief 检查 RawCommand 是否已经超过 100 ms 没有更新。
 *
 * 该函数应由主循环周期调用。收到过有效 RawCommand 后，如果当前时间与最后一条
 * 有效命令的时间差达到 100 ms，就把 8 路 DShot 命令全部置为 0。超时只处理一次，
 * 后续必须收到新的有效 RawCommand，才会重新开始计时并允许命令更新。
 *
 * @param now_usec 当前单调递增时间戳，单位为微秒。
 */
void MotorControl_Poll(uint64_t now_usec);

/**
 * @brief 读取一路当前保存的 DShot 命令，供后续帧编码步骤使用。
 *
 * @param output_index 输出索引，0..7 分别对应 DShot1..DShot8。
 * @return 对应的 DShot 命令；索引越界时安全返回 0（停止）。
 */
uint16_t MotorControl_GetDShotCommand(uint8_t output_index);

/**
 * @brief 读取一路当前保存的 16-bit DShot 帧。
 *
 * 帧已经包含 11-bit 命令、遥测请求位和 4-bit 校验和。当前统一不请求遥测，
 * 因此编码时传给 DShot_BuildFrame() 的 request_telemetry 固定为 false。
 *
 * @param output_index 输出索引，0..7 分别对应 DShot1..DShot8。
 * @return 对应的完整 16-bit DShot 帧；索引越界时安全返回 0（停止帧）。
 */
uint16_t MotorControl_GetDShotFrame(uint8_t output_index);

/**
 * @brief 读取一路DShot输出中某一个bit对应的CCR值。
 *
 * 每路保存16个CCR值，bit_index=0对应DShot帧的bit15，bit_index=15对应
 * bit0。正常返回值是30（逻辑0）或60（逻辑1）。
 *
 * @param output_index 输出索引，0..7分别对应DShot1..DShot8。
 * @param bit_index    发送顺序索引，0..15分别对应帧的bit15..bit0。
 * @return 对应的CCR值；任一索引越界时返回0。
 */
uint32_t MotorControl_GetDShotCcrValue(uint8_t output_index,
                                       uint8_t bit_index);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H__ */
