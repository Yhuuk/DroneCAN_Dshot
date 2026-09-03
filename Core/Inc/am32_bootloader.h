#ifndef AM32_BOOTLOADER_H
#define AM32_BOOTLOADER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* AM32当前配置区读取48字节；方向字段位于偏移17。 */
#define AM32_BOOTLOADER_EEPROM_READ_LENGTH       48U
#define AM32_BOOTLOADER_DIRECTION_OFFSET         17U

/**
 * @brief AM32内层Bootloader的状态码。
 * AM32BootloaderStatus是一个枚举类型，用于表示AM32内层Bootloader的状态码。它包含以下值：
 * - AM32_BOOTLOADER_OK: 表示操作成功。
 * - AM32_BOOTLOADER_INVALID_ARGUMENT: 表示传入的参数无效。
 * - AM32_BOOTLOADER_TIMEOUT: 表示操作超时。
 * - AM32_BOOTLOADER_TRANSPORT_ERROR: 表示传输错误。
 * - AM32_BOOTLOADER_BAD_IDENTITY: 表示身份验证失败。
 * - AM32_BOOTLOADER_BAD_ACK: 表示接收到的ACK无效。
 * - AM32_BOOTLOADER_BAD_CRC: 表示接收到的CRC无效。
 * - AM32_BOOTLOADER_UNSUPPORTED_PROTOCOL: 表示不支持的协议版本。
 * - AM32_BOOTLOADER_INVALID_DIRECTION: 表示读取的方向数据无效。
 */
typedef enum
{
  AM32_BOOTLOADER_OK = 0,
  AM32_BOOTLOADER_INVALID_ARGUMENT,
  AM32_BOOTLOADER_TIMEOUT,
  AM32_BOOTLOADER_TRANSPORT_ERROR,
  AM32_BOOTLOADER_BAD_IDENTITY,
  AM32_BOOTLOADER_BAD_ACK,
  AM32_BOOTLOADER_BAD_CRC,
  AM32_BOOTLOADER_UNSUPPORTED_PROTOCOL,
  AM32_BOOTLOADER_INVALID_DIRECTION
} AM32BootloaderStatus;

/**
 * 这九个字节分别是：
 *   0-2: ASCII "471"标识AM32/BLHeli Bootloader
 *   3: pin_code
 *   4: flash_size_code
 *   5-6: 保留
 *   7: protocol_version
 *   8: ACK(0x30)
 *  建立连接时AM32返回的9字节设备信息中需要长期保留的字段。 */
typedef struct
{
  uint8_t pin_code;
  uint8_t flash_size_code;
  uint8_t protocol_version;
} AM32BootloaderInfo;

/**
 * @brief 连接一路AM32，读取Flash模拟EEPROM并解析持久化方向。
 *
 * 每次调用都会重新发送BootInit，不依赖上一次会话。协议版本2及以上通过
 * EEPROM魔法地址0x0020定位配置区，再使用READ_FLASH_SIL读取48字节。
 *
 * @param channel_index DShot通道索引0..7。
 * @param out_reversed  成功时写false=Normal、true=Reversed。
 * @param out_info      成功建立连接后写入Bootloader设备信息，可为空。
 * @param out_eeprom    成功读取时复制完整48字节配置，可为空。
 */
AM32BootloaderStatus AM32Bootloader_ReadDirection(
    uint8_t channel_index,
    bool* out_reversed,
    AM32BootloaderInfo* out_info,
    uint8_t* out_eeprom);

/**
 * @brief 在指定信号线上发送四个零字节，请求Bootloader运行电调应用。
 *
 * AM32收到RUN后会立即跳转，通常没有可等待的ACK，所以这里只发送不接收。
 * 即使此前连接失败也可调用，作为统一异常清理的一部分。
 */
AM32BootloaderStatus AM32Bootloader_RunApplication(uint8_t channel_index);

/** @brief 计算AM32内层Bootloader使用的CRC16/ARC。 */
uint16_t AM32Bootloader_Crc16Arc(const uint8_t* data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* AM32_BOOTLOADER_H */
