#include "dshot_output.h"

#include "motor_control.h"

#include <stddef.h>

//first_output_index是用来选择是哪个TIM的对应4个Dshot通道的
//DShotOutput_BuildTimerDmaBuffer这个函数就是只能一次使用一组TIM的4个DShot通道，不能同时使用两组TIM的8个DShot通道，所以first_output_index只能是0或4，分别对应DShot1..4和DShot5..8
bool DShotOutput_BuildTimerDmaBuffer(uint8_t first_output_index,
                                     uint32_t* out_dma_buffer,
                                     uint16_t buffer_capacity)
{
  uint32_t dma_index;
  uint8_t slot_index;
  uint8_t channel_index;

  /*
   * 一个TIM固定占用连续4路输出。first_output_index只能是0或4，否则
   * first_output_index + channel_index可能超过DShot8。
   * 所有参数先检查完再写数组，失败时保持调用者原缓冲区不变。
   */
  if ((out_dma_buffer == NULL) ||
      (buffer_capacity < DSHOT_OUTPUT_DMA_BUFFER_LENGTH) ||
      (first_output_index != 0 && first_output_index != 4))
  {
    return false;
  }

  /*
   * 前16个slot按照bit15到bit0排列。每个slot中4个值连续存放，TIM收到
   * 一次update DMA请求后，DMA burst会把它们依次写入CCR1～CCR4。
   */
  for (slot_index = 0U;
       slot_index < DSHOT_OUTPUT_DATA_SLOT_COUNT;
       slot_index++)
  {
    for (channel_index = 0U;
         channel_index < DSHOT_OUTPUT_CHANNELS_PER_TIMER;
         channel_index++)
    {
      dma_index =
          ((uint32_t)slot_index * DSHOT_OUTPUT_CHANNELS_PER_TIMER) +
          channel_index;
      out_dma_buffer[dma_index] =
          MotorControl_GetDShotCcrValue(
              (uint8_t)(first_output_index + channel_index),
              slot_index);
    }
  }

  /*
   * 最后2个slot全部写0。PWM1模式下CCR=0表示整个周期保持低电平，
   * 与停止帧中的逻辑0（CCR=30）含义不同。
   */
  for (slot_index = DSHOT_OUTPUT_DATA_SLOT_COUNT;
       slot_index < DSHOT_OUTPUT_TOTAL_SLOT_COUNT;
       slot_index++)
  {
    for (channel_index = 0U;
         channel_index < DSHOT_OUTPUT_CHANNELS_PER_TIMER;
         channel_index++)
    {
      dma_index =
          ((uint32_t)slot_index * DSHOT_OUTPUT_CHANNELS_PER_TIMER) +
          channel_index;
      out_dma_buffer[dma_index] = 0U;
    }
  }

  return true;
}

HAL_StatusTypeDef DShotOutput_StartTimerDma(
    TIM_HandleTypeDef* htim,
    const uint32_t* dma_buffer,
    uint16_t dma_buffer_length)
{
  /*
   * CubeMX必须已经把hdma[TIM_DMA_ID_UPDATE]连接到对应DMA通道。
   * 长度固定检查为72，避免四通道burst在不完整slot处结束。
   */

   //Instance 是 Register base address
  if ((htim == NULL) ||
      (htim->Instance == NULL) ||
      (htim->hdma[TIM_DMA_ID_UPDATE] == NULL) ||
      (dma_buffer == NULL) ||
      (dma_buffer_length != DSHOT_OUTPUT_DMA_BUFFER_LENGTH))
  {
    return HAL_ERROR;
  }

  /*
   * TIM_DMABASE_CCR1：burst从CCR1开始。
   * TIM_DMA_UPDATE：每个TIM更新事件触发一次burst。这个更新事件是由TIM计数器溢出产生的。也就是产生一位bit的PWM信号时，TIM计数器会溢出一次，从而触发一次DMA burst，把缓冲区中的4个word依次写入CCR1～CCR4。
   * TIM_DMABURSTLENGTH_4TRANSFERS：每次依次更新CCR1、CCR2、CCR3、CCR4。
   * dma_buffer_length：DMA总共搬运72个word，也就是18组四通道CCR数据。
   */
  return HAL_TIM_DMABurst_MultiWriteStart(
      htim,
      TIM_DMABASE_CCR1,
      TIM_DMA_UPDATE,
      dma_buffer,
      TIM_DMABURSTLENGTH_4TRANSFERS,
      dma_buffer_length);
}

HAL_StatusTypeDef DShotOutput_StopTimerDma(TIM_HandleTypeDef* htim)
{
  if ((htim == NULL) ||
      (htim->Instance == NULL) ||
      (htim->hdma[TIM_DMA_ID_UPDATE] == NULL))
  {
    return HAL_ERROR;
  }

  return HAL_TIM_DMABurst_WriteStop(htim, TIM_DMA_UPDATE);
}
