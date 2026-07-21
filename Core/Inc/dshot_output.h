#ifndef DSHOT_OUTPUT_H
#define DSHOT_OUTPUT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "dshot.h"
#include "main.h"

#include <stdbool.h>
#include <stdint.h>

/* TIM1和TIM2各使用CH1～CH4驱动4路DShot输出。 */
#define DSHOT_OUTPUT_CHANNELS_PER_TIMER   4U

/*
 * 每次发送包含16个DShot数据周期和2个持续低电平周期。最后2个周期不仅提供
 * 帧间隔，也让CCR预装载和DMA完成处理结束时，4个输出都可靠保持为低电平。
 */
#define DSHOT_OUTPUT_DATA_SLOT_COUNT      DSHOT_FRAME_BIT_COUNT
#define DSHOT_OUTPUT_RESET_SLOT_COUNT     2U
#define DSHOT_OUTPUT_TOTAL_SLOT_COUNT     \
    (DSHOT_OUTPUT_DATA_SLOT_COUNT + DSHOT_OUTPUT_RESET_SLOT_COUNT)

/* 每个slot连续存放CCR1、CCR2、CCR3、CCR4，所以一个TIM需要18×4个word。 */
#define DSHOT_OUTPUT_DMA_BUFFER_LENGTH    \
    (DSHOT_OUTPUT_TOTAL_SLOT_COUNT * DSHOT_OUTPUT_CHANNELS_PER_TIMER)

/**
 * @brief 把连续4路DShot CCR数据排列成一个TIM update DMA burst缓冲区。
 *
 * 缓冲区按slot优先排列：
 *   - [0..3]是bit15对应的CCR1..CCR4；
 *   - [4..7]是bit14对应的CCR1..CCR4；
 *   - 依次排列到bit0；
 *   - 最后2个slot的4个通道全部填0，形成帧间低电平。
 *
 * first_output_index当前使用0或4：0表示DShot1..4，4表示DShot5..8。
 * 参数保留为显式索引，使同一个函数可以同时服务TIM2和TIM1，不需要以后
 * 再增加第二套重复的缓冲区构建函数。
 *
 * 
 * first_output_index是干什么的？它表示当前构建的DMA缓冲区对应的4路DShot输出中的第一路索引。由于一个TIM固定占用连续4路输出，所以first_output_index只能是0或4，分别对应DShot1..4和DShot5..8。
 * @param first_output_index 这一组4路输出中的第一路索引，合法范围是0或4。
 * @param out_dma_buffer     输出DMA缓冲区地址，元素类型必须是uint32_t。
 * @param buffer_capacity   缓冲区容量，至少为DSHOT_OUTPUT_DMA_BUFFER_LENGTH。
 *
 * @retval true  缓冲区构建成功。
 * @retval false 索引、指针或容量无效，没有写入缓冲区。
 */
bool DShotOutput_BuildTimerDmaBuffer(uint8_t first_output_index,
                                     uint32_t* out_dma_buffer,
                                     uint16_t buffer_capacity);

/**
 * @brief 为一个TIM启动update DMA burst，令一次更新事件连续写CCR1～CCR4。
 *
 * HAL会把DMA外设地址设置为TIMx_DMAR，并把TIM的DMA burst基地址设置为CCR1、
 * burst长度设置为4。这样每个TIM update事件会依次把缓冲区中的4个word写到
 * CCR1、CCR2、CCR3、CCR4；72个word正好对应18个PWM周期。
 *
 * 本函数只配置并启动DMA传输，不启动TIM计数器，也不使能4个PWM输出通道。
 * 这些动作留给下一步的高层发送函数统一完成，避免两个TIM的启动流程分散。
 *
 * @param htim              已由CubeMX初始化并连接TIM update DMA的定时器句柄。
 * @param dma_buffer        已按四通道burst格式排列的DMA缓冲区。
 * @param dma_buffer_length DMA数据总长度，当前必须等于72个word。
 *
 * @return HAL_OK表示DMA burst启动成功；其余返回HAL_ERROR或HAL_BUSY。
 */
HAL_StatusTypeDef DShotOutput_StartTimerDma(
    TIM_HandleTypeDef* htim,
    const uint32_t* dma_buffer,
    uint16_t dma_buffer_length);

/**
 * @brief 停止一个TIM的update DMA burst并恢复HAL内部的ready状态。
 *
 * 正常模式DMA完成后，HAL仍要求调用WriteStop关闭TIM update DMA请求，并把
 * DMABurstState从busy恢复为ready，否则下一帧启动时会返回HAL_BUSY。
 * 本函数不会停止PWM通道。固定周期输出启动后，PWM通道会始终保持使能，
 * DMA完成回调只停止计数器和DMA，并以CCR=0主动保持帧间低电平。
 *
 * @param htim 定时器句柄。
 * @return HAL状态。
 */
HAL_StatusTypeDef DShotOutput_StopTimerDma(TIM_HandleTypeDef* htim);

/**
 * @brief 启动由TIM7提供1.5 ms硬件节拍的TIM2周期发送。
 *
 * 调用前必须已经执行MX_TIM2_Init()、MX_TIM7_Init()和MotorControl_Init()。
 * 本函数先把A/B两块DMA缓冲区都构建成完整停止帧，再启动TIM7更新中断；
 * 因此第一次周期中断不依赖主循环准备速度，也不会发送未初始化的数据。
 * TIM2 CH1～CH4也在这里一次性使能，此后帧间通过CCR=0主动输出低电平，
 * 不会在每帧完成后关闭通道并让DShot引脚进入高阻状态。
 *
 * 当前配置要求TIM7输入时钟48 MHz、PSC=47、ARR=1499。周期计算为：
 * 48 MHz / (47 + 1) = 1 MHz，(1499 + 1) / 1 MHz = 1.5 ms。
 *
 * @return HAL_OK表示固定周期已经启动；配置不匹配返回HAL_ERROR；
 *         重复启动返回HAL_BUSY。
 */
HAL_StatusTypeDef DShotOutput_StartTim2Periodic(void);

/**
 * @brief 在主循环中准备TIM2下一周期使用的双缓冲数据。
 *
 * TIM7中断只提出准备请求，本函数在未被DMA使用的另一块缓冲区中构建72个
 * CCR word。全部写完后才发布ready标志，所以中断不会读到半成品。
 * 本函数不等待、不启动DMA；没有准备请求时会立即返回。
 */
void DShotOutput_PollTim2Preparation(void);

/**
 * @brief 处理一个TIM7的1.5 ms更新节拍。
 *
 * 本函数在TIM7中断上下文运行：选择已经准备完成的双缓冲区、提出下一块
 * 缓冲区的准备请求，并启动一次TIM2四通道DShot DMA发送。
 */
void DShotOutput_HandleTim7PeriodElapsed(void);

/**
 * @brief 使用TIM2 CH1～CH4发送一次DShot1～DShot4帧。
 *
 * 本阶段补充：函数现在发送双缓冲中已经准备完成的active缓冲区，不再在
 * 函数内读取motor_control或构建72-word数据。数据准备由主循环中的
 * DShotOutput_PollTim2Preparation()完成，固定调用节拍由TIM7中断提供。
 * TIM2四个PWM通道必须已经由DShotOutput_StartTim2Periodic()一次性使能；
 * 本函数每次只启动DMA和TIM2计数器。DMA传输期间再次调用会返回HAL_BUSY。
 *
 * 本函数只发送一次18-slot波形，不负责固定周期重复发送，也没有在当前阶段
 * 自动接入DroneCAN接收路径。后续TIM1会复用相同的内部四通道启动流程，
 * 不需要修改本函数或现有DMA基础函数的参数。
 * 这个单帧接口继续保留，TIM7固定周期处理函数会调用它；以后TIM1也可以
 * 复用相同的单帧发送结构，不需要修改这里的参数。
 *
 * @return HAL_OK表示本次发送已经启动；其余返回HAL_BUSY或HAL_ERROR。
 */
HAL_StatusTypeDef DShotOutput_SendTim2Once(void);

/**
 * @brief 查询TIM2四通道DShot DMA是否仍在发送。
 * @return true表示缓冲区正被DMA使用；false表示可以启动下一帧。
 */
bool DShotOutput_IsTim2Busy(void);

/**
 * @brief 处理HAL通知的TIM update DMA正常完成事件。
 *
 * 参数形式现在就保留为通用TIM句柄。当前只处理TIM2；后续接入TIM1时只需
 * 在函数体内增加TIM1分支，不需要修改函数声明或HAL回调入口。
 *
 * @param htim 产生DMA完成事件的TIM句柄。
 */
void DShotOutput_HandleTimerPeriodElapsed(TIM_HandleTypeDef* htim);

/**
 * @brief 处理HAL通知的TIM DMA错误事件。
 * @param htim 发生DMA错误的TIM句柄。
 */
void DShotOutput_HandleTimerError(TIM_HandleTypeDef* htim);

#ifdef __cplusplus
}
#endif

#endif /* DSHOT_OUTPUT_H */
