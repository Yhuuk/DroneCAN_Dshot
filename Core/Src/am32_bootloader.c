#include "am32_bootloader.h"

#include "am32_singlewire.h"

#include <stddef.h>

/* AM32/BLHeli Bootloader内层命令；不是PC端0x2F/0x2E外层4-way封包。 */
#define AM32_BL_CMD_RUN                     0x00U
#define AM32_BL_CMD_READ_FLASH_SIL          0x03U
#define AM32_BL_CMD_SET_ADDRESS             0xFFU
#define AM32_BL_ACK_OK                      0x30U

/* 协议v2起支持0x0020逻辑地址，Bootloader会将其转换成实际配置Flash地址。 */
#define AM32_BL_EEPROM_MAGIC_ADDRESS        0x000020UL
#define AM32_BL_MIN_MAGIC_ADDRESS_PROTOCOL  2U

#define AM32_BL_DEVICE_INFO_LENGTH          9U
#define AM32_BL_READ_RESPONSE_LENGTH        \
    (AM32_BOOTLOADER_EEPROM_READ_LENGTH + 3U)
#define AM32_BL_CONNECT_TIMEOUT_MS           40U
#define AM32_BL_COMMAND_TIMEOUT_MS           25U
#define AM32_BL_READ_TIMEOUT_MS              100U

/*
 * 12个0、0x0D、"BLHeli"、0xF4、0x7D组成兼容ArduPilot/AM32的21字节BootInit。
 * 尚未连接时这条序列不追加普通命令CRC。
 */
static const uint8_t g_am32_boot_init[] =
{
  0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
  0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
  0x0DU, 0x42U, 0x4CU, 0x48U, 0x65U, 0x6CU, 0x69U, 0xF4U, 0x7DU
};

static AM32BootloaderStatus AM32Bootloader_MapTransportStatus(
    AM32SingleWireStatus status);
static AM32BootloaderStatus AM32Bootloader_Connect(
    uint8_t channel_index,
    AM32BootloaderInfo* out_info);
static AM32BootloaderStatus AM32Bootloader_SetEepromAddress(
    uint8_t channel_index);
static AM32BootloaderStatus AM32Bootloader_ReadEeprom(
    uint8_t channel_index,
    uint8_t* out_eeprom);
static void AM32Bootloader_AppendCrc(uint8_t* frame, uint8_t payload_length);

/**
 * 计算CRC16-ARC校验值
 * @param data 输入数据指针
 * @param length 数据长度
 * @return CRC16-ARC校验值
 * 
 * 异或：相同为0，不同为1。当前这个是低位优先的CRC16算法，初始值为0x0000，使用多项式0xA001。
 * 
 *  CRC16-ARC算法的特点：低8位异或，高8位保留作为结果
 * 
 *  初始值：0x0000
    反射多项式：0xA001
    低位优先
 */
uint16_t AM32Bootloader_Crc16Arc(const uint8_t* data, uint16_t length)
{
  uint16_t crc = 0U;
  uint16_t byte_index;
  uint8_t bit_index;

  if ((data == NULL) && (length > 0U))
  {
    return 0U;
  }

  for (byte_index = 0U; byte_index < length; byte_index++)
  {
    /**
     * crc ^= data[byte_index] 将当前字节与CRC进行异或运算，更新CRC的值。
     */
    crc ^= data[byte_index];
    for (bit_index = 0U; bit_index < 8U; bit_index++)
    {
      if ((crc & 1U) != 0U)
      {
        crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
      }
      else
      {
        crc >>= 1U;
      }
    }
  }

  return crc;
}

/**
 * 这是读取单路方向的高级入口。
 * 依次执行:
 *  Connect(channel)
    SetEepromAddress(channel)
    ReadEeprom(channel)
    读取 eeprom[17]

    EEPROM[17] = 0 -> Normal
    EEPROM[17] = 1 -> Reversed
    其他值          -> 数据无效

    成功后返回：
    - reversed
    - 48字节EEPROM副本
    - Bootloader设备信息
 */
AM32BootloaderStatus AM32Bootloader_ReadDirection(
    uint8_t channel_index,
    bool* out_reversed,
    AM32BootloaderInfo* out_info,
    uint8_t* out_eeprom)
{
  AM32BootloaderInfo info = {0};
  uint8_t eeprom[AM32_BOOTLOADER_EEPROM_READ_LENGTH];
  AM32BootloaderStatus status;
  uint8_t i;

  if ((channel_index >= AM32_SINGLEWIRE_CHANNEL_COUNT) ||
      (out_reversed == NULL))
  {
    return AM32_BOOTLOADER_INVALID_ARGUMENT;
  }

  status = AM32Bootloader_Connect(channel_index, &info);
  if (status != AM32_BOOTLOADER_OK)
  {
    return status;
  }

  if (out_info != NULL)
  {
    *out_info = info;
  }

  if (info.protocol_version < AM32_BL_MIN_MAGIC_ADDRESS_PROTOCOL)
  {
    return AM32_BOOTLOADER_UNSUPPORTED_PROTOCOL;
  }

  status = AM32Bootloader_SetEepromAddress(channel_index);
  if (status != AM32_BOOTLOADER_OK)
  {
    return status;
  }

  status = AM32Bootloader_ReadEeprom(channel_index, eeprom);
  if (status != AM32_BOOTLOADER_OK)
  {
    return status;
  }

  if (eeprom[AM32_BOOTLOADER_DIRECTION_OFFSET] > 1U)
  {
    return AM32_BOOTLOADER_INVALID_DIRECTION;
  }

  *out_reversed = eeprom[AM32_BOOTLOADER_DIRECTION_OFFSET] == 1U;
  if (out_eeprom != NULL)
  {
    for (i = 0U; i < AM32_BOOTLOADER_EEPROM_READ_LENGTH; i++)
    {
      out_eeprom[i] = eeprom[i];
    }
  }

  return AM32_BOOTLOADER_OK;
}


/**
 * 发送四个零字节请求，使AM32退出Bootloader并运行正常电调程序，不等待响应
 */
AM32BootloaderStatus AM32Bootloader_RunApplication(uint8_t channel_index)
{
  /* CMD_RUN(0x00)+参数0x00的CRC也为0，因此线上正好是四个连续零字节。 */
  const uint8_t run_frame[4] =
  {
    AM32_BL_CMD_RUN, 0x00U, 0x00U, 0x00U
  };
  AM32SingleWireStatus transport_status;

  if (channel_index >= AM32_SINGLEWIRE_CHANNEL_COUNT)
  {
    return AM32_BOOTLOADER_INVALID_ARGUMENT;
  }

  transport_status = AM32SingleWire_Transfer(
      channel_index,
      run_frame,
      (uint8_t)sizeof(run_frame),
      NULL,
      0U,
      AM32_BL_COMMAND_TIMEOUT_MS);
  return AM32Bootloader_MapTransportStatus(transport_status);
}

/**
 * 发送21字节BootInit：
 * 
 * 接收9字节设备信息:
 *   0-2: ASCII "471"标识AM32/BLHeli Bootloader
    3: pin_code
    4: flash_size_code
    5-6: 保留
    7: protocol_version
    8: ACK(0x30)

    当前要求协议版本至少为2，否则返回 UNSUPPORTED_PROTOCOL。
 */
static AM32BootloaderStatus AM32Bootloader_Connect(
    uint8_t channel_index,
    AM32BootloaderInfo* out_info)
{
  uint8_t response[AM32_BL_DEVICE_INFO_LENGTH];
  AM32SingleWireStatus transport_status;

  transport_status = AM32SingleWire_Transfer(
      channel_index,
      g_am32_boot_init,
      (uint8_t)sizeof(g_am32_boot_init),
      response,
      (uint8_t)sizeof(response),
      AM32_BL_CONNECT_TIMEOUT_MS);
  if (transport_status != AM32_SINGLEWIRE_OK)
  {
    return AM32Bootloader_MapTransportStatus(transport_status);
  }

  /* 当前AM32以ASCII "471"标识这一族Bootloader设备。 */
  if ((response[0] != (uint8_t)'4') ||
      (response[1] != (uint8_t)'7') ||
      (response[2] != (uint8_t)'1'))
  {
    return AM32_BOOTLOADER_BAD_IDENTITY;
  }
  if (response[8] != AM32_BL_ACK_OK)
  {
    return AM32_BOOTLOADER_BAD_ACK;
  }

  out_info->pin_code = response[3];
  out_info->flash_size_code = response[4];
  out_info->protocol_version = response[7];
  return AM32_BOOTLOADER_OK;
}

/**
 * 设置EEPROM地址
 * 发送: FF 00 00 20 CRC_L CRC_H
 * 
 *  0xFF       SET_ADDRESS命令
    00 00 20   EEPROM起始地址0x0020
    CRC_L/H    前4字节CRC16-ARC

    电调应答: 0x30=ACK_OK 或 0x31=ACK_ERROR
 */
static AM32BootloaderStatus AM32Bootloader_SetEepromAddress(
    uint8_t channel_index)
{
  uint8_t frame[6];
  uint8_t ack;
  AM32SingleWireStatus transport_status;

  frame[0] = AM32_BL_CMD_SET_ADDRESS;
  frame[1] = (uint8_t)(AM32_BL_EEPROM_MAGIC_ADDRESS >> 16U);
  frame[2] = (uint8_t)(AM32_BL_EEPROM_MAGIC_ADDRESS >> 8U);
  frame[3] = (uint8_t)AM32_BL_EEPROM_MAGIC_ADDRESS;
  AM32Bootloader_AppendCrc(frame, 4U);

  transport_status = AM32SingleWire_Transfer(
      channel_index,
      frame,
      (uint8_t)sizeof(frame),
      &ack,
      1U,
      AM32_BL_COMMAND_TIMEOUT_MS);
  if (transport_status != AM32_SINGLEWIRE_OK)
  {
    return AM32Bootloader_MapTransportStatus(transport_status);
  }

  return ack == AM32_BL_ACK_OK
             ? AM32_BOOTLOADER_OK
             : AM32_BOOTLOADER_BAD_ACK;
}

/**
 * 读取EEPROM
 * 发送: 03 30 CRC_L CRC_H
 * 
 *  0x03       READ_EEPROM命令， READ_FLASH_SIL
    0x30       读取长度
    CRC_L/H    前4字节CRC16-ARC

    接收：
    48字节 EEPROM数据
    2字节 CRC
    1字节 ACK
    ----------------
    总计51字节

    电调应答: 0x30=ACK_OK 或 0x31=ACK_ERROR

    检查顺序：
    1. 最后一个字节是不是 0x30
    2. 对前48字节计算CRC16-ARC
    3. 与响应第48、49字节比较
    4. 全部正确后复制EEPROM数据
 */
static AM32BootloaderStatus AM32Bootloader_ReadEeprom(
    uint8_t channel_index,
    uint8_t* out_eeprom)
{
  uint8_t frame[4];
  uint8_t response[AM32_BL_READ_RESPONSE_LENGTH];
  uint16_t expected_crc;
  uint16_t received_crc;
  AM32SingleWireStatus transport_status;
  uint8_t i;

  frame[0] = AM32_BL_CMD_READ_FLASH_SIL;
  frame[1] = AM32_BOOTLOADER_EEPROM_READ_LENGTH;
  AM32Bootloader_AppendCrc(frame, 2U);

  transport_status = AM32SingleWire_Transfer(
      channel_index,
      frame,
      (uint8_t)sizeof(frame),
      response,
      (uint8_t)sizeof(response),
      AM32_BL_READ_TIMEOUT_MS);
  if (transport_status != AM32_SINGLEWIRE_OK)
  {
    return AM32Bootloader_MapTransportStatus(transport_status);
  }

  if (response[AM32_BOOTLOADER_EEPROM_READ_LENGTH + 2U] != AM32_BL_ACK_OK)
  {
    return AM32_BOOTLOADER_BAD_ACK;
  }

  expected_crc = AM32Bootloader_Crc16Arc(
      response,
      AM32_BOOTLOADER_EEPROM_READ_LENGTH);
  received_crc =
      (uint16_t)response[AM32_BOOTLOADER_EEPROM_READ_LENGTH] |
      ((uint16_t)response[AM32_BOOTLOADER_EEPROM_READ_LENGTH + 1U] << 8U);
  if (expected_crc != received_crc)
  {
    return AM32_BOOTLOADER_BAD_CRC;
  }

  for (i = 0U; i < AM32_BOOTLOADER_EEPROM_READ_LENGTH; i++)
  {
    out_eeprom[i] = response[i];
  }
  return AM32_BOOTLOADER_OK;
}

/**
 * 对指定帧内容计算CRC，然后追加：frame[length] = CRC低字节，frame[length+1] = CRC高字节
 */
static void AM32Bootloader_AppendCrc(uint8_t* frame, uint8_t payload_length)
{
  uint16_t crc = AM32Bootloader_Crc16Arc(frame, payload_length);

  /* AM32内层协议规定CRC低字节先发送，高字节后发送。 */
  frame[payload_length] = (uint8_t)crc;
  
  //crc >> 8U 取高8位，作为CRC高字节
  frame[payload_length + 1U] = (uint8_t)(crc >> 8U);
}

/**
 * 把单线物理层错误映射为Bootloader层错误：
 *  单线OK       -> Bootloader OK
    单线超时     -> Bootloader TIMEOUT
    参数错误     -> Bootloader INVALID_ARGUMENT
    其他物理错误 -> Bootloader TRANSPORT_ERROR
 */
static AM32BootloaderStatus AM32Bootloader_MapTransportStatus(
    AM32SingleWireStatus status)
{
  if (status == AM32_SINGLEWIRE_OK)
  {
    return AM32_BOOTLOADER_OK;
  }
  if (status == AM32_SINGLEWIRE_TIMEOUT)
  {
    return AM32_BOOTLOADER_TIMEOUT;
  }
  if (status == AM32_SINGLEWIRE_INVALID_ARGUMENT)
  {
    return AM32_BOOTLOADER_INVALID_ARGUMENT;
  }
  return AM32_BOOTLOADER_TRANSPORT_ERROR;
}
