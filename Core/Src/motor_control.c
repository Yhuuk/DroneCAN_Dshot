#include "motor_control.h"

#include <stddef.h>

/*
 * 保存最近一次有效 RawCommand 映射得到的 8 路 DShot 命令。
 * 当前尚未接入帧编码和硬件输出，volatile 用于保证这些调试状态在 RAM 中
 * 保持可观察；后续可以在 Keil debugger 中直接查看每一路的命令值。
 */
static volatile uint16_t g_dshot_commands[MOTOR_CONTROL_DSHOT_OUTPUT_COUNT];

static void MotorControl_StopAllDShotCommands(void)
{
  uint8_t i;

  for (i = 0U; i < MOTOR_CONTROL_DSHOT_OUTPUT_COUNT; i++)
  {
    g_dshot_commands[i] = 0U;
  }
}

bool MotorControl_MapRawCommandToDShot(int16_t raw_command,
                                       uint16_t* out_dshot_command)
{
    //raw_positive_offset是原始命令值减去 RawCommand 正值区间的起点 1，表示当前命令在正值区间中的偏移量。
    //raw_positive_span是 RawCommand 正值区间的宽度，即 8191 - 1 = 8190。
    //dshot_throttle_span是 DShot 普通油门区间的宽度，即 2047 - 48 = 1999。
    //scaled_offset是将 RawCommand 正值区间的偏移量按比例缩放到 DShot 普通油门区间的偏移量，计算公式为 (raw_positive_offset * dshot_throttle_span) / raw_positive_span。
  uint32_t raw_positive_offset;
  uint32_t raw_positive_span;
  uint32_t dshot_throttle_span;
  uint32_t scaled_offset;

  /*
   * RawCommand 在 DSDL 中是 int14，所以只有 -8192..8191 合法。
   * 先完成全部校验再写输出，失败时不会破坏调用者原有的 DShot 命令。
   */
  if ((out_dshot_command == NULL) ||
      (raw_command < MOTOR_CONTROL_RAW_COMMAND_MIN_VALUE) ||
      (raw_command > MOTOR_CONTROL_RAW_COMMAND_MAX_VALUE))
  {
    return false;
  }

  /*
   * 当前按单向电调处理。RawCommand 的负值可用于支持反转的设备，但本项目
   * 暂未启用反转，因此负数和零都安全映射为 DShot 0（停止）。
   */
  if (raw_command <= 0)
  {
    *out_dshot_command = 0U;
    return true;
  }

  /*
   * 把 RawCommand 正值区间 1..8191 线性映射到 DShot 普通油门区间
   * 48..2047。两边都先减去各自起点，再按区间宽度缩放：
   *
   *   raw = 1    -> DShot = 48
   *   raw = 8191 -> DShot = 2047
   *
   * 中间乘法使用 uint32_t，避免 16-bit 乘法溢出。整数除法会舍去小数，
   * 但始终保持单调，并且不会产生 1..47 的 DShot 特殊命令。
   */
  raw_positive_offset = (uint32_t)(raw_command - 1);
  raw_positive_span = (uint32_t)(MOTOR_CONTROL_RAW_COMMAND_MAX_VALUE - 1);
  dshot_throttle_span =
      (uint32_t)(DSHOT_COMMAND_MAX_VALUE - DSHOT_THROTTLE_MIN_VALUE);

  scaled_offset =
      (raw_positive_offset * dshot_throttle_span) / raw_positive_span;

  *out_dshot_command =
      (uint16_t)((uint32_t)DSHOT_THROTTLE_MIN_VALUE + scaled_offset);

  //等价于*out_dshot_command = (uint16_t)(DSHOT_THROTTLE_MIN_VALUE + (raw_positive_offset * dshot_throttle_span) / raw_positive_span);
  // out_dshot_command[0] = (uint16_t)((uint32_t)DSHOT_THROTTLE_MIN_VALUE + scaled_offset);
  return true;
}

bool MotorControl_UpdateDShotCommands(const int16_t* raw_commands,
                                      uint8_t raw_command_count)
{
  uint16_t next_dshot_commands[MOTOR_CONTROL_DSHOT_OUTPUT_COUNT] = {0};
  uint8_t command_count_to_map;
  uint8_t i;

  /*
   * 长度为 0 是合法的“没有电机命令”，此时即使指针为空也可以安全地把
   * 8 路全部置为停止。长度大于 0 时必须提供有效数组地址。
   */
  if ((raw_command_count > 0U) && (raw_commands == NULL))
  {
    MotorControl_StopAllDShotCommands();
    return false;
  }

  /* 本节点只有 8 路输出，因此 RawCommand 超出的元素不参与本节点映射。 */
  command_count_to_map = raw_command_count;
  if (command_count_to_map > MOTOR_CONTROL_DSHOT_OUTPUT_COUNT)
  {
    command_count_to_map = MOTOR_CONTROL_DSHOT_OUTPUT_COUNT;
  }

  /*
   * 先在局部数组中完成全部映射，避免处理到一半时让全局缓存出现部分新值、
   * 部分旧值。局部数组初始为 0，所以 RawCommand 缺少的通道自然保持停止。
   */
  //任一输入越界，都会导致全部输出安全置零。映射失败时不覆盖上一条已经验证有效的命令。
  for (i = 0U; i < command_count_to_map; i++)
  {
    if (!MotorControl_MapRawCommandToDShot(raw_commands[i],
                                           &next_dshot_commands[i]))
    {
      MotorControl_StopAllDShotCommands();
      return false;
    }
  }

  /* 全部映射成功后，再一次性按顺序更新 8 路已保存的目标命令。 */
  for (i = 0U; i < MOTOR_CONTROL_DSHOT_OUTPUT_COUNT; i++)
  {
    g_dshot_commands[i] = next_dshot_commands[i];
  }

  return true;
}

uint16_t MotorControl_GetDShotCommand(uint8_t output_index)
{
  if (output_index >= MOTOR_CONTROL_DSHOT_OUTPUT_COUNT)
  {
    return 0U;
  }

  return g_dshot_commands[output_index];
}
