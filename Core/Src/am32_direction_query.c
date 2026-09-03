#include "am32_direction_query.h"

#include "am32_singlewire.h"
#include "dshot_output.h"
#include "motor_control.h"

#include <stddef.h>

/* 查询开始后仍发送零DShot 500 ms，给正在减速的电机留出安全停止时间。 */
#define AM32_QUERY_ZERO_HOLD_USEC          500000ULL

/* 全部信号线拉高3 s，使未armed的AM32应用也能DShot超时复位并留在Bootloader。 */
#define AM32_QUERY_BOOT_ENTRY_HIGH_USEC   3000000ULL
#define AM32_QUERY_MAX_ATTEMPTS                 3U

/* 当前AM32 EEPROM结构中用于诊断的几个稳定前部字段。 */
#define AM32_QUERY_EEPROM_VERSION_OFFSET        1U
#define AM32_QUERY_FW_MAJOR_OFFSET              3U
#define AM32_QUERY_FW_MINOR_OFFSET              4U

typedef enum
{
  AM32_QUERY_STATE_IDLE = 0,
  AM32_QUERY_STATE_HOLD_ZERO,
  AM32_QUERY_STATE_WAIT_BOOTLOADER,
  AM32_QUERY_STATE_READ_CHANNEL,
  AM32_QUERY_STATE_RUN_APPLICATIONS,
  AM32_QUERY_STATE_RESTORE_DSHOT
} AM32DirectionQueryState;

static volatile AM32DirectionQueryState g_query_state;
static AM32DirectionQueryResult g_query_result;
static uint64_t g_query_state_start_usec;
static uint8_t g_query_channel;
static uint8_t g_query_attempt;
static bool g_query_result_available;

/* 这些计数专供Keil/VS Code调试观察，不参与协议判断。 */
static volatile uint32_t g_query_started_count;
static volatile uint32_t g_query_completed_count;
static volatile uint32_t g_query_maintenance_error_count;

static void AM32DirectionQuery_ClearResult(uint8_t motor_mask);
static void AM32DirectionQuery_AdvanceToRequestedChannel(void);
static void AM32DirectionQuery_RecordFinalFailure(
    uint8_t channel,
    AM32BootloaderStatus status);
static void AM32DirectionQuery_Finish(void);

/**
 * 作用：
  - 初始化底层单线模块
  - 状态设为 IDLE
  - 清空上一次结果
  - 清空当前通道、重试次数和调试计数
 */
void AM32DirectionQuery_Init(void)
{
  AM32SingleWire_Init();
  g_query_state = AM32_QUERY_STATE_IDLE;
  g_query_state_start_usec = 0ULL;
  g_query_channel = 0U;
  g_query_attempt = 0U;
  g_query_result_available = false;
  g_query_started_count = 0U;
  g_query_completed_count = 0U;
  g_query_maintenance_error_count = 0U;
  AM32DirectionQuery_ClearResult(0U);
}

/**
 * 开始一次新的8路方向查询任务。
 * 检查条件：
  - motor_mask 不能为0
  - 当前不能已经查询
  - DShot不能已经处于维护模式
  - 电机必须全部停止

  随后：
  1. 按 motor_mask 清空并初始化结果
  2. 当前通道设为0
  3. 重试次数设为0
  4. 保存进入状态的时间
  5. 切换到 HOLD_ZERO
  6. 强制电机停止
 */
bool AM32DirectionQuery_Start(uint8_t motor_mask, uint64_t now_usec)
{
  if ((motor_mask == 0U) ||
      (g_query_state != AM32_QUERY_STATE_IDLE) ||
      DShotOutput_IsMaintenanceMode() ||
      (!MotorControl_AreAllDShotCommandsStopped()))
  {
    return false;
  }

  AM32DirectionQuery_ClearResult(motor_mask);
  g_query_result_available = false;
  g_query_channel = 0U;
  g_query_attempt = 0U;
  g_query_state_start_usec = now_usec;
  g_query_state = AM32_QUERY_STATE_HOLD_ZERO;
  g_query_started_count++;

  /* 清除RawCommand新鲜状态，查询结束后必须重新完成连续零油门互锁。 */
  MotorControl_ForceStop();
  return true;
}

/**
 * 这是整个方向查询的核心状态机。
 * IDLE: 没有查询任务，直接返回
 * 
 * 成功后：
  - 停止DShot输出
  - 把所有DShot引脚变为普通GPIO
  - 全部拉高
  - 状态切到 WAIT_BOOTLOADER

  到时后：
  - 当前通道设为0
  - 当前重试次数设为0
  - 查找第一路被 motor_mask 选中的电调
  - 进入 READ_CHANNEL
 */
void AM32DirectionQuery_Poll(uint64_t now_usec)
{
  switch (g_query_state)
  {
    //空闲状态，就直接返回
    case AM32_QUERY_STATE_IDLE:
      return;

    //强制电机停止，保持零油门500ms
    case AM32_QUERY_STATE_HOLD_ZERO:
      MotorControl_ForceStop();
      if ((now_usec - g_query_state_start_usec) < AM32_QUERY_ZERO_HOLD_USEC)
      {
        return;
      }

      if (DShotOutput_EnterMaintenanceMode() != HAL_OK)
      {
        g_query_result.maintenance_error = true;
        g_query_maintenance_error_count++;
        AM32DirectionQuery_Finish();
        return;
      }

      g_query_state_start_usec = now_usec;
      g_query_state = AM32_QUERY_STATE_WAIT_BOOTLOADER;
      return;

    case AM32_QUERY_STATE_WAIT_BOOTLOADER:
      if ((now_usec - g_query_state_start_usec) <
          AM32_QUERY_BOOT_ENTRY_HIGH_USEC)
      {
        return;
      }

      g_query_channel = 0U;
      g_query_attempt = 0U;
      AM32DirectionQuery_AdvanceToRequestedChannel();
      return;

    case AM32_QUERY_STATE_READ_CHANNEL:
    {
      uint8_t eeprom[AM32_BOOTLOADER_EEPROM_READ_LENGTH];
      AM32BootloaderInfo info = {0};
      AM32BootloaderStatus status;
      bool reversed = false;
      uint8_t channel_bit = (uint8_t)(1U << g_query_channel);

      /*
       * 一个完整尝试包含BootInit、设置EEPROM地址和读取48字节。物理层用中断
       * 逐bit收发，但本函数等待这一小段事务完成；CAN中断在此期间仍可运行。
       */
      status = AM32Bootloader_ReadDirection(
          g_query_channel,
          &reversed,
          &info,
          eeprom);
      g_query_result.status[g_query_channel] = status;
      g_query_result.bootloader_info[g_query_channel] = info;

      if (status == AM32_BOOTLOADER_OK)
      {
        g_query_result.valid_mask |= channel_bit;
        if (reversed)
        {
          g_query_result.reversed_mask |= channel_bit;
        }
        g_query_result.eeprom_version[g_query_channel] =
            eeprom[AM32_QUERY_EEPROM_VERSION_OFFSET];
        g_query_result.firmware_major[g_query_channel] =
            eeprom[AM32_QUERY_FW_MAJOR_OFFSET];
        g_query_result.firmware_minor[g_query_channel] =
            eeprom[AM32_QUERY_FW_MINOR_OFFSET];

        g_query_channel++;
        g_query_attempt = 0U;
        AM32DirectionQuery_AdvanceToRequestedChannel();
        return;
      }

      g_query_attempt++;
      if (g_query_attempt < AM32_QUERY_MAX_ATTEMPTS)
      {
        /* 下一轮主循环重新BootInit，避免复用已经失步的会话或残留字节。 */
        return;
      }

      AM32DirectionQuery_RecordFinalFailure(g_query_channel, status);
      g_query_channel++;
      g_query_attempt = 0U;
      AM32DirectionQuery_AdvanceToRequestedChannel();
      return;
    }

    case AM32_QUERY_STATE_RUN_APPLICATIONS:
      if (g_query_channel < AM32_DIRECTION_QUERY_MOTOR_COUNT)
      {
        /*
         * 进入维护模式时8根线全部保持过高电平，所以即使只请求读取部分通道，
         * 也必须对全部8路发送RUN。失败/未请求通道同样需要退出Bootloader。
         */
        (void)AM32Bootloader_RunApplication(g_query_channel);
        g_query_channel++;
        return;
      }

      g_query_state = AM32_QUERY_STATE_RESTORE_DSHOT;
      return;

    case AM32_QUERY_STATE_RESTORE_DSHOT:
      MotorControl_ForceStop();
      if (DShotOutput_ExitMaintenanceMode() != HAL_OK)
      {
        g_query_result.maintenance_error = true;
        g_query_maintenance_error_count++;
      }
      AM32DirectionQuery_Finish();
      return;

    default:
      AM32SingleWire_Abort();
      g_query_result.maintenance_error = true;
      if (DShotOutput_IsMaintenanceMode())
      {
        (void)DShotOutput_ExitMaintenanceMode();
      }
      AM32DirectionQuery_Finish();
      return;
  }
}

/**
 * 判断当前是否查询中
 */
bool AM32DirectionQuery_IsBusy(void)
{
  return g_query_state != AM32_QUERY_STATE_IDLE;
}

/**
 * 把最近一次查询结果复制给调用者。
 * 只有已经完成查询并且结果可用时才返回成功。
 */
bool AM32DirectionQuery_GetLastResult(AM32DirectionQueryResult* out_result)
{
  if ((!g_query_result_available) || (out_result == NULL))
  {
    return false;
  }

  *out_result = g_query_result;
  return true;
}

/**
 * 内部辅助函数。
 * 清除：
  - valid_mask
  - reversed_mask
  - 各种错误位图
  - 维护模式错误
  - 每一路Bootloader状态
  - Bootloader设备信息
  - EEPROM版本
  - 固件版本
  并记录本次请求的 motor_mask
 */
static void AM32DirectionQuery_ClearResult(uint8_t motor_mask)
{
  uint8_t i;

  g_query_result.requested_mask = motor_mask;
  g_query_result.valid_mask = 0U;
  g_query_result.reversed_mask = 0U;
  g_query_result.timeout_mask = 0U;
  g_query_result.crc_error_mask = 0U;
  g_query_result.unsupported_mask = 0U;
  g_query_result.protocol_error_mask = 0U;
  g_query_result.maintenance_error = false;

  for (i = 0U; i < AM32_DIRECTION_QUERY_MOTOR_COUNT; i++)
  {
    g_query_result.status[i] = AM32_BOOTLOADER_INVALID_ARGUMENT;
    g_query_result.bootloader_info[i].pin_code = 0U;
    g_query_result.bootloader_info[i].flash_size_code = 0U;
    g_query_result.bootloader_info[i].protocol_version = 0U;
    g_query_result.eeprom_version[i] = 0U;
    g_query_result.firmware_major[i] = 0U;
    g_query_result.firmware_minor[i] = 0U;
  }
}

/**
 * 从当前通道开始寻找下一路被 motor_mask 选中的电机。
 * 找不到下一路时，说明选中的电调都处理完了，切换到：RUN_APPLICATIONS
 */
static void AM32DirectionQuery_AdvanceToRequestedChannel(void)
{
  /**
   * g_query_channel 是当前正在处理的电调通道索引，从0到7。g_query_result.requested_mask 是一个8位掩码，表示哪些电调通道被请求查询。
   * 只要有一个通道被请求，就切换状态到 READ_CHANNEL 并返回。如果所有通道都没有被请求，就切换状态到 RUN_APPLICATIONS。
   * 这个while循环从当前通道开始，检查每一位是否在请求掩码中被设置。如果当前通道被请求，就切换状态到 AM32_QUERY_STATE_READ_CHANNEL 并返回。如果当前通道没有被请求，就继续检查下一个通道，直到检查完所有8个通道。
   * 如果所有通道都没有被请求，就说明查询已经完成，切换状态到 AM32_QUERY_STATE_RUN_APPLICATIONS，准备发送 RUN 命令给所有电调。
   */
  while (g_query_channel < AM32_DIRECTION_QUERY_MOTOR_COUNT)
  {
    if ((g_query_result.requested_mask &
         (uint8_t)(1U << g_query_channel)) != 0U)
    {
      g_query_state = AM32_QUERY_STATE_READ_CHANNEL;
      return;
    }
    g_query_channel++;
  }

  /* 所有读取结束后从通道0开始逐路发RUN清理。 */
  g_query_channel = 0U;
  g_query_state = AM32_QUERY_STATE_RUN_APPLICATIONS;
}

static void AM32DirectionQuery_RecordFinalFailure(
    uint8_t channel,
    AM32BootloaderStatus status)
{
  uint8_t channel_bit = (uint8_t)(1U << channel);

  if (status == AM32_BOOTLOADER_TIMEOUT)
  {
    g_query_result.timeout_mask |= channel_bit;
  }
  else if (status == AM32_BOOTLOADER_BAD_CRC)
  {
    g_query_result.crc_error_mask |= channel_bit;
  }
  else if (status == AM32_BOOTLOADER_UNSUPPORTED_PROTOCOL)
  {
    g_query_result.unsupported_mask |= channel_bit;
  }
  else
  {
    g_query_result.protocol_error_mask |= channel_bit;
  }
}

static void AM32DirectionQuery_Finish(void)
{
  MotorControl_ForceStop();
  g_query_state = AM32_QUERY_STATE_IDLE;
  g_query_result_available = true;
  g_query_completed_count++;
}
