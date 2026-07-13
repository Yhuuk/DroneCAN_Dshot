#include "motor_control.h"

#include <stddef.h>

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
  return true;
}
