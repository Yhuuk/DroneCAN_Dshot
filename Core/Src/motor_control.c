#include "motor_control.h"

#include <stddef.h>

/*
 * 保存最近一次有效 RawCommand 映射得到的 8 路 DShot 命令。
 * 当前尚未接入帧编码和硬件输出，volatile 用于保证这些调试状态在 RAM 中
 * 保持可观察；后续可以在 Keil debugger 中直接查看每一路的命令值。
 * 本阶段补充：逻辑帧编码已经接入，但仍未接入 TIM、DMA 和 GPIO 硬件输出。
 * 本阶段再次补充：TIM2固定周期发送已经接入；本模块仍只维护目标缓存，
 * 具体DMA启动和发送时序继续由dshot_output与dronecan_app负责。
 */
static volatile uint16_t g_dshot_commands[MOTOR_CONTROL_DSHOT_OUTPUT_COUNT];

/* 保存与 g_dshot_commands 一一对应的 8 个完整 16-bit DShot 帧。 */
static volatile uint16_t g_dshot_frames[MOTOR_CONTROL_DSHOT_OUTPUT_COUNT];

/*
 * 保存8路DShot帧分别转换得到的16个CCR值。第一维是输出通道，第二维是
 * 发送顺序；[output][0]对应帧bit15，[output][15]对应帧bit0。
 */
static uint32_t
    g_dshot_ccr_values[MOTOR_CONTROL_DSHOT_OUTPUT_COUNT][DSHOT_FRAME_BIT_COUNT];

/*
 * 这两个状态只在主循环上下文中访问，所以不需要 volatile。
 * g_has_fresh_raw_command 用来区分“从未收到命令/已经超时”和“正在等待超时”。
 */
static uint64_t g_last_raw_command_timestamp_usec;
static bool g_has_fresh_raw_command;

/* 供 Keil debugger 观察 RawCommand 超时保护实际触发了多少次。 */
static volatile uint32_t g_raw_command_timeout_count;

static void MotorControl_StopAllDShotCommands(void)
{
  uint8_t i;
  uint8_t bit_index;

  for (i = 0U; i < MOTOR_CONTROL_DSHOT_OUTPUT_COUNT; i++)
  {
    g_dshot_commands[i] = 0U;
    g_dshot_frames[i] = 0U;

    /* 停止帧0x0000包含16个逻辑0，所以对应的CCR值都是30，而不是0。 */
    for (bit_index = 0U; bit_index < DSHOT_FRAME_BIT_COUNT; bit_index++)
    {
      g_dshot_ccr_values[i][bit_index] = DSHOT_BIT_0_HIGH_TICKS;
    }
  }
}

void MotorControl_Init(void)
{
  /*
   * 静态RAM上电后虽然默认为0，但CCR=0只会让引脚保持低电平，并不等于
   * 发送一帧有效的DShot 0x0000。本函数显式构建停止帧对应的CCR缓存。
   */
  MotorControl_StopAllDShotCommands();
  g_last_raw_command_timestamp_usec = 0ULL;
  g_has_fresh_raw_command = false;
  g_raw_command_timeout_count = 0U;
}

void MotorControl_ForceStop(void)
{
  MotorControl_StopAllDShotCommands();
  g_has_fresh_raw_command = false;
}

bool MotorControl_HasFreshRawCommand(void)
{
  return g_has_fresh_raw_command;
}

bool MotorControl_AreAllDShotCommandsStopped(void)
{
  uint8_t i;

  for (i = 0U; i < MOTOR_CONTROL_DSHOT_OUTPUT_COUNT; i++)
  {
    if (g_dshot_commands[i] != 0U)
    {
      return false;
    }
  }

  return true;
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
                                      uint8_t raw_command_count,
                                      uint64_t timestamp_usec)
{
  uint16_t next_dshot_commands[MOTOR_CONTROL_DSHOT_OUTPUT_COUNT] = {0};
  uint16_t next_dshot_frames[MOTOR_CONTROL_DSHOT_OUTPUT_COUNT] = {0};
  uint32_t next_dshot_ccr_values[DSHOT_FRAME_BIT_COUNT];
  uint8_t command_count_to_map;
  uint8_t i;
  uint8_t bit_index;

  /*
   * 长度为 0 是合法的“没有电机命令”，此时即使指针为空也可以安全地把
   * 8 路全部置为停止。长度大于 0 时必须提供有效数组地址。
   */
  if ((raw_command_count > 0U) && (raw_commands == NULL))
  {
    MotorControl_StopAllDShotCommands();
    g_has_fresh_raw_command = false;
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
      g_has_fresh_raw_command = false;
      return false;
    }
  }

  /*
   * 把 8 路命令分别编码成完整 DShot 帧。当前阶段不接收电调遥测，所以
   * request_telemetry 固定为 false。缺少的 RawCommand 通道对应命令 0，
   * 编码后得到停止帧 0x0000。
   */
  //这一步是把映射后的 DShot 命令值编码成完整的 16-bit DShot 帧。任一编码失败，都会导致全部输出安全置零。
  //如果收到的比8帧少，next_dshot_commands[i]后面没有的就是0，不会被MotorControl_MapRawCommandToDShot索引到，所以这里也会被编码为0x0000
  for (i = 0U; i < MOTOR_CONTROL_DSHOT_OUTPUT_COUNT; i++)
  {
    if (!DShot_BuildFrame(next_dshot_commands[i],
                          false,
                          &next_dshot_frames[i]))
    {
      MotorControl_StopAllDShotCommands();
      g_has_fresh_raw_command = false;
      return false;
    }
  }

  /*
   * 逐路把完整DShot帧转换成16个CCR值。这里只使用一个16元素局部数组作为
   * 暂存区，避免在函数栈上创建完整的8×16数组。每一路转换成功后再复制到
   * 对应的全局缓存；如果发生错误，全部通道都会恢复为停止状态。
   */
  for (i = 0U; i < MOTOR_CONTROL_DSHOT_OUTPUT_COUNT; i++)
  {
    if (!DShot_BuildCcrValues(next_dshot_frames[i],
                              next_dshot_ccr_values))
    {
      MotorControl_StopAllDShotCommands();
      g_has_fresh_raw_command = false;
      return false;
    }

    for (bit_index = 0U; bit_index < DSHOT_FRAME_BIT_COUNT; bit_index++)
    {
      g_dshot_ccr_values[i][bit_index] =
          next_dshot_ccr_values[bit_index];
    }
  }

  /*
   * 8 路映射和帧编码全部成功后，再一次性更新全局缓存，避免留下部分新数据、
   * 部分旧数据。命令值、完整帧和对应CCR值现在都可以在RAM中读取和检查。
   */
  for (i = 0U; i < MOTOR_CONTROL_DSHOT_OUTPUT_COUNT; i++)
  {
    g_dshot_commands[i] = next_dshot_commands[i];
    g_dshot_frames[i] = next_dshot_frames[i];
  }

  /*
   * 只有全部数据映射成功后才刷新时间。这样非法 RawCommand 不会延长上一条有效
   * 命令的生存时间。长度为 0 的 RawCommand 是合法停止命令，也会重新开始计时。
   */
  g_last_raw_command_timestamp_usec = timestamp_usec;
  g_has_fresh_raw_command = true;

  return true;
}

void MotorControl_Poll(uint64_t now_usec)
{
  //g_has_fresh_raw_command =1表示当前有一条有效的 RawCommand，尚未超时；g_has_fresh_raw_command = 0 表示当前没有有效的 RawCommand，或者已经超时。
  if (!g_has_fresh_raw_command)
  {
    return;
  }

  /*
   * 使用无符号时间差进行比较，不依赖绝对时间值。达到 100 ms（包括恰好等于）
   * 就立即停止全部输出；随后清除 fresh 标志，避免同一次超时被重复计数。
   */
  if ((now_usec - g_last_raw_command_timestamp_usec) >=
      MOTOR_CONTROL_RAW_COMMAND_TIMEOUT_USEC)
  {
    MotorControl_ForceStop();
    g_raw_command_timeout_count++;
  }
}

uint16_t MotorControl_GetDShotCommand(uint8_t output_index)
{
  if (output_index >= MOTOR_CONTROL_DSHOT_OUTPUT_COUNT)
  {
    return 0U;
  }

  return g_dshot_commands[output_index];
}

uint16_t MotorControl_GetDShotFrame(uint8_t output_index)
{
  if (output_index >= MOTOR_CONTROL_DSHOT_OUTPUT_COUNT)
  {
    return 0U;
  }

  return g_dshot_frames[output_index];
}

uint32_t MotorControl_GetDShotCcrValue(uint8_t output_index,
                                       uint8_t bit_index)
{
  if ((output_index >= MOTOR_CONTROL_DSHOT_OUTPUT_COUNT) ||
      (bit_index >= DSHOT_FRAME_BIT_COUNT))
  {
    return 0U;
  }

  return g_dshot_ccr_values[output_index][bit_index];
}
