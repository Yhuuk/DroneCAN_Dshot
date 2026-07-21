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

/* 一个完整 DShot 帧固定包含 16 bit，并且发送时从最高位开始。 */
#define DSHOT_FRAME_BIT_COUNT             16U

/*
 * 当前 TIM1/TIM2 时钟为 48 MHz，PSC=0、ARR=79，因此一个 DShot600 bit
 * 由 80 个定时器计数组成。逻辑 0 的高电平约占 37.5%，逻辑 1 约占 75%。
 */
#define DSHOT_TIMER_TICKS_PER_BIT         80U
#define DSHOT_BIT_0_HIGH_TICKS            28U
#define DSHOT_BIT_1_HIGH_TICKS            56U

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

/**
 * @brief 把一个 16-bit DShot 帧转换成 16 个定时器 CCR 值。
 *
 * 函数按照 bit15 到 bit0 的顺序处理帧数据：
 *   - 逻辑 1 转换为 CCR=60；
 *   - 逻辑 0 转换为 CCR=30。
 *
 * 输出使用 uint32_t，是为了与当前 CubeMX 中 TIM update DMA 的 word 数据
 * 对齐配置一致。该数组目前只表示一条通道的 16 个数据 bit，尚未添加帧间
 * 低电平，也不能直接作为四通道 TIM DMA burst 缓冲区。
 *
 * @param frame          已经包含命令、遥测位和校验和的完整 DShot 帧。
 * @param out_ccr_values 输出数组地址，调用者必须提供至少 16 个 uint32_t 元素。
 *
 * @retval true  16 个 CCR 值转换完成。
 * @retval false out_ccr_values 为空，没有写入任何数据。
 */
bool DShot_BuildCcrValues(uint16_t frame,
                          uint32_t* out_ccr_values);

#ifdef __cplusplus
}
#endif

#endif /* __DSHOT_H__ */
