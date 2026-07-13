#ifndef __DSHOT_H__
#define __DSHOT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/*
 * DShot 的命令字段宽度是 11 bit，因此合法范围是 0..2047。
 * 其中 0 表示停止，1..47 是协议保留的特殊命令，48..2047 才是普通油门值。
 * 本模块只负责按协议编码，不决定 RawCommand 应该映射成哪个 DShot 命令。
 */
#define DSHOT_COMMAND_MAX_VALUE          2047U
#define DSHOT_SPECIAL_COMMAND_MAX_VALUE  47U
#define DSHOT_THROTTLE_MIN_VALUE         48U

/**
 * @brief 把一个 11-bit DShot 命令编码成完整的 16-bit DShot 帧。
 *
 * DShot 帧从高位到低位依次由以下字段组成：
 *   - 11 bit 命令值；
 *   - 1 bit 遥测请求位；
 *   - 4 bit 校验和。
 *
 * 该函数只生成逻辑上的 16-bit 数据帧，不配置 TIM、DMA 或 GPIO，也不会
 * 产生实际 DShot 波形。后续步骤再把每个 bit 转换成对应的 CCR 占空比。
 *
 * @param command           DShot 命令，合法范围是 0..2047。
 * @param request_telemetry true 表示本帧请求一次遥测，false 表示不请求。
 * @param out_frame         输出完整的 16-bit DShot 帧。
 *
 * @retval true  编码成功，out_frame 已被写入。
 * @retval false command 超出范围或 out_frame 为空，输出值保持不变。
 */
bool DShot_BuildFrame(uint16_t command,
                      bool request_telemetry,
                      uint16_t* out_frame);

#ifdef __cplusplus
}
#endif

#endif /* __DSHOT_H__ */
