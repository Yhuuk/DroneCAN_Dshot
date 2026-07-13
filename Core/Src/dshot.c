#include "dshot.h"

#include <stddef.h>

#define DSHOT_TELEMETRY_BIT_MASK  0x01U
#define DSHOT_CHECKSUM_MASK       0x0FU
#define DSHOT_CHECKSUM_SHIFT      4U

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
