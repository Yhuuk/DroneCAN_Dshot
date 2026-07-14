#include "dshot.h"

#include <stddef.h>

#define DSHOT_TELEMETRY_BIT_MASK  0x01U
#define DSHOT_CHECKSUM_MASK       0x0FU
#define DSHOT_CHECKSUM_SHIFT      4U
#define DSHOT_FRAME_MSB_MASK      0x8000U

bool DShot_BuildFrame(uint16_t command,
                      bool request_telemetry,
                      uint16_t* out_frame)
{
  uint16_t payload;
  uint16_t checksum;

  /*
   * command 只能占 11 bit。这里对越界值直接报错，不能简单使用位掩码
   * 截断，否则一个错误输入可能被静默转换成另一条合法的 DShot 命令。
   * 先完成全部校验再写 out_frame，失败时调用者原有的输出值不会被破坏。
   */
  if ((command > DSHOT_COMMAND_MAX_VALUE) || (out_frame == NULL))
  {
    return false;
  }

  /*
   * 先组成校验和之前的 12-bit payload：高 11 bit 是命令，最低 1 bit
   * 是遥测请求位。当前阶段通常传入 false，但编码函数保留完整协议能力。
   */
  payload = (uint16_t)(command << 1U);
  if (request_telemetry)
  {
    payload = (uint16_t)(payload | DSHOT_TELEMETRY_BIT_MASK);
  }

  /*
   * DShot 校验和是 12-bit payload 的三个 4-bit 半字节逐位异或，最后只
   * 保留低 4 bit。它不是 CAN CRC，也不是 DroneCAN transfer 的 CRC。
   */
  checksum = (uint16_t)(payload ^
                        (payload >> 4U) ^
                        (payload >> 8U));
  checksum = (uint16_t)(checksum & DSHOT_CHECKSUM_MASK);

  /* 12-bit payload 左移 4 位，最低 4 位放入刚计算出的校验和。 */
  *out_frame = (uint16_t)((payload << DSHOT_CHECKSUM_SHIFT) | checksum);
  return true;
}

bool DShot_BuildCcrValues(uint16_t frame,
                          uint32_t* out_ccr_values)
{
  uint16_t bit_mask;
  uint8_t bit_index;

  if (out_ccr_values == NULL)
  {
    return false;
  }

  /*
   * DShot 规定最高位先发送，所以掩码从 bit15 开始，每处理一位就右移一位。
   * PWM1 模式下，CCR 表示每个 80 计数周期内高电平持续的计数数量。
   */
  bit_mask = DSHOT_FRAME_MSB_MASK;
  for (bit_index = 0U; bit_index < DSHOT_FRAME_BIT_COUNT; bit_index++)
  {
    if ((frame & bit_mask) != 0U)
    {
      out_ccr_values[bit_index] = DSHOT_BIT_1_HIGH_TICKS;
    }
    else
    {
      out_ccr_values[bit_index] = DSHOT_BIT_0_HIGH_TICKS;
    }

    bit_mask = (uint16_t)(bit_mask >> 1U);
  }

  /*
   * 停止帧 0x0000 会得到 16 个 CCR=30 的逻辑 0 脉冲，而不是持续低电平。
   * 后续步骤会在 16 个数据 bit 之后另外追加 CCR=0 的帧间低电平。
   */
  return true;
}
