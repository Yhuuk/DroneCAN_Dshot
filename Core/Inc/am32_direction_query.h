#ifndef AM32_DIRECTION_QUERY_H
#define AM32_DIRECTION_QUERY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "am32_bootloader.h"

#include <stdbool.h>
#include <stdint.h>

#define AM32_DIRECTION_QUERY_MOTOR_COUNT  8U
#define AM32_DIRECTION_QUERY_ALL_MOTORS   0xFFU

/*
 * AM32维护过程错误位。数值与DirectionQuery DSDL中的MAINTENANCE_ERROR_*
 * 保持一致，因此服务层可以直接把该字段复制到DroneCAN响应中。
 */
#define AM32_DIRECTION_QUERY_MAINTENANCE_ERROR_ENTER_FAILED            0x01U
#define AM32_DIRECTION_QUERY_MAINTENANCE_ERROR_BOOTLOADER_EXIT_FAILED  0x02U
#define AM32_DIRECTION_QUERY_MAINTENANCE_ERROR_DSHOT_RESTORE_FAILED    0x04U

/** 一次查询完成后保留的8路结果；无效通道必须解释为Unknown，不能当作Normal。 */
typedef struct
{
  uint8_t requested_mask;
  uint8_t valid_mask;
  uint8_t reversed_mask;
  uint8_t timeout_mask;
  uint8_t crc_error_mask;
  uint8_t unsupported_mask;
  uint8_t protocol_error_mask;
  /* 上述AM32_DIRECTION_QUERY_MAINTENANCE_ERROR_*按位或后的结果。 */
  uint8_t maintenance_error;

  AM32BootloaderStatus status[AM32_DIRECTION_QUERY_MOTOR_COUNT];
  AM32BootloaderInfo bootloader_info[AM32_DIRECTION_QUERY_MOTOR_COUNT];
  uint8_t eeprom_version[AM32_DIRECTION_QUERY_MOTOR_COUNT];
  uint8_t firmware_major[AM32_DIRECTION_QUERY_MOTOR_COUNT];
  uint8_t firmware_minor[AM32_DIRECTION_QUERY_MOTOR_COUNT];
} AM32DirectionQueryResult;

/** @brief 初始化查询状态和单线物理层。 */
void AM32DirectionQuery_Init(void);

/**
 * @brief 启动一次AM32持久化方向读取。
 *
 * 请求被接受后立即取得8路电机输出控制权：普通油门和DShot方向写入必须由
 * 应用层拒绝。状态机先强制发送500 ms零油门，再暂停DShot、将全部信号线
 * 拉高3 s进入Bootloader，最后依次读取motor_mask选中的电调。
 *
 * @param motor_mask bit0..bit7对应DShot1..DShot8，至少选择一路。
 * @param now_usec   当前单调时间，精度为ms也足够用于长时间安全状态。
 */
bool AM32DirectionQuery_Start(uint8_t motor_mask, uint64_t now_usec);

/** @brief 在主循环中推进查询；进入实际单线事务时单次可能阻塞约100 ms。 */
void AM32DirectionQuery_Poll(uint64_t now_usec);

/** @brief 返回true表示查询已取得输出控制权，调用者必须忽略普通油门。 */
bool AM32DirectionQuery_IsBusy(void);

/**
 * @brief 复制最近一次查询结果。
 * @return false表示从上电后尚未完成过任何一次查询，结果不可用。
 */
bool AM32DirectionQuery_GetLastResult(AM32DirectionQueryResult* out_result);

#ifdef __cplusplus
}
#endif

#endif /* AM32_DIRECTION_QUERY_H */
