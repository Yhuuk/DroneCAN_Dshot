#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "dshot.h"

/* DroneCAN uavcan.equipment.esc.RawCommand 中 int14 字段的有效范围。 */
#define MOTOR_CONTROL_RAW_COMMAND_MIN_VALUE  (-8192)
#define MOTOR_CONTROL_RAW_COMMAND_MAX_VALUE  8191

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
 */
bool MotorControl_MapRawCommandToDShot(int16_t raw_command,
                                       uint16_t* out_dshot_command);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H__ */
