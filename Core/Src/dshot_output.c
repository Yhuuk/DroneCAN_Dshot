#include "dshot_output.h"

#include "motor_control.h"
#include "tim.h"

#include <stddef.h>

/* TIM2固定驱动DShot1～DShot4，所以它使用第一组输出索引0。 */
#define DSHOT_OUTPUT_TIM2_FIRST_OUTPUT_INDEX  0U

/*
 * DMA工作期间缓冲区内容不能被覆盖，因此缓冲区和busy状态都由本模块长期保存。
 * 后续接入TIM1时会增加第二个同尺寸缓冲区，不会改变现有发送接口的参数。
 */
static uint32_t g_tim2_dma_buffer[DSHOT_OUTPUT_DMA_BUFFER_LENGTH];
//表示TIM2 DMA是否正在使用缓冲区，发送期间不能重新修改g_tim2_dma_buffer，否则同一帧可能混入新旧数据。
//true表示TIM2 DMA正在使用缓冲区，false表示TIM2 DMA空闲，可以重新修改g_tim2_dma_buffer。
static volatile bool g_tim2_busy;

/* 供Keil debugger观察TIM2发送完成和错误次数。 */
static volatile uint32_t g_tim2_send_complete_count;
static volatile uint32_t g_tim2_send_error_count;

static HAL_StatusTypeDef DShotOutput_StartFourPwmChannelsLow(
    TIM_HandleTypeDef* htim);
static void DShotOutput_StopTimerGroup(TIM_HandleTypeDef* htim);

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

//启动一个 TIM的Update DMA Burst。
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

//停止Update DMA Burst，并让HAL内部DMA Burst状态恢复为可再次启动。
HAL_StatusTypeDef DShotOutput_StopTimerDma(TIM_HandleTypeDef* htim)
{
  if ((htim == NULL) ||
      (htim->Instance == NULL) ||
      (htim->hdma[TIM_DMA_ID_UPDATE] == NULL))
  {
    return HAL_ERROR;
  }

  /*
     1. 禁用DMA通道
    2. 关闭TIM的Update DMA请求
    3. 把HAL内部DMABurstState恢复成READY
  */
  return HAL_TIM_DMABurst_WriteStop(htim, TIM_DMA_UPDATE);
}

HAL_StatusTypeDef DShotOutput_SendTim2Once(void)
{
  HAL_StatusTypeDef status;

  /*
   * g_tim2_dma_buffer正在被DMA读取时不能重新构建，否则同一帧中可能混入
   * 新旧两组CCR值。busy一直保持到DMA完成回调完成清理。
   */
  if (g_tim2_busy)
  {
    return HAL_BUSY;
  }

  /*
   * 72个word的DMA缓冲区由当前motor_control保存的4路CCR值构建。每路CCR值
   * 对应一个DShot通道的16个bit,后面2个slot全部写0。
   */
  if (!DShotOutput_BuildTimerDmaBuffer(
          DSHOT_OUTPUT_TIM2_FIRST_OUTPUT_INDEX,
          g_tim2_dma_buffer,
          DSHOT_OUTPUT_DMA_BUFFER_LENGTH))
  {
    return HAL_ERROR;
  }

  /*
   * 先以CCR=0启动CH1～CH4，使4个通道都进入HAL的BUSY状态并使能输出。
   * 此时DMA尚未启动，因此即使TIM2短暂计数，引脚也只会保持低电平。
   * helper随后会停止计数并把CNT重新归零，等待统一启动。
   */
  status = DShotOutput_StartFourPwmChannelsLow(&htim2);
  if (status != HAL_OK)
  {
    DShotOutput_StopTimerGroup(&htim2);
    return status;
  }

  /*
   * 4个通道全部准备好之后再武装update DMA。这样启动TIM2计数器时，
   * CH1～CH4从同一个CNT=0起点开始，不会有某一路提前进入数据周期。
   */
  status = DShotOutput_StartTimerDma(&htim2,
                                     g_tim2_dma_buffer,
                                     DSHOT_OUTPUT_DMA_BUFFER_LENGTH);
  if (status != HAL_OK)
  {
    DShotOutput_StopTimerGroup(&htim2);
    return status;
  }

  /* busy必须在使能计数器之前置位，防止极端情况下DMA回调先看到旧状态。 */
  g_tim2_busy = true;

  //__HAL_TIM_SET_COUNTER(&htim2, 0U)是把TIM2的计数器清零，
  //__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE)是清除TIM2的更新事件标志位，
  //__HAL_TIM_ENABLE(&htim2)是使能TIM2计数器开始计数。
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE(&htim2);

  return HAL_OK;
}

bool DShotOutput_IsTim2Busy(void)
{
  return g_tim2_busy;
}

//TIM2 dma正常完成，并计数一次
void DShotOutput_HandleTimerPeriodElapsed(TIM_HandleTypeDef* htim)
{
  if ((htim == NULL) ||
      (htim != &htim2) ||
      (!g_tim2_busy))
  {
    return;
  }

  /*
   * 72个word已经全部传输。当前生效的是低电平slot，因此可以立即停止
   * 计数器、DMA请求和4个PWM通道，并允许下一次发送重新使用缓冲区。
   */
  DShotOutput_StopTimerGroup(htim);
  g_tim2_send_complete_count++;
  g_tim2_busy = false;
}

//TIM2 DMA错误。停止输出，g_tim2_send_error_count统计一次错误
void DShotOutput_HandleTimerError(TIM_HandleTypeDef* htim)
{
  if ((htim == NULL) ||
      (htim != &htim2) ||
      (!g_tim2_busy))
  {
    return;
  }

  /* DMA异常时同样立即停止4路输出，不能让旧CCR继续循环产生波形。 */
  DShotOutput_StopTimerGroup(htim);
  g_tim2_send_error_count++;
  g_tim2_busy = false;
}

//
static HAL_StatusTypeDef DShotOutput_StartFourPwmChannelsLow(
    TIM_HandleTypeDef* htim)
{
  HAL_StatusTypeDef status;
  uint8_t started_channel_count = 0U;

  if ((htim == NULL) || (htim->Instance == NULL))
  {
    return HAL_ERROR;
  }

  /*
   * PWM通道启用了CCR预装载。先写4个CCR=0并产生一次update事件，确保
   * 活动比较值也是0；此时PWM1模式的4个引脚都会保持低电平。
   */
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, 0U);
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, 0U);
  __HAL_TIM_SET_COUNTER(htim, 0U);
  status = HAL_TIM_GenerateEvent(htim, TIM_EVENTSOURCE_UPDATE);
  if (status != HAL_OK)
  {
    return status;
  }
  __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);

  status = HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
  if (status == HAL_OK)
  {
    started_channel_count = 1U;
    status = HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
  }
  if (status == HAL_OK)
  {
    started_channel_count = 2U;
    status = HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
  }
  if (status == HAL_OK)
  {
    started_channel_count = 3U;
    status = HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);
  }
  if (status == HAL_OK)
  {
    started_channel_count = 4U;
  }

  if (status != HAL_OK)
  {
    /* 只回滚已经成功启动的通道，恢复HAL内部的channel ready状态。 */
    if (started_channel_count >= 1U)
    {
      (void)HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
    }
    if (started_channel_count >= 2U)
    {
      (void)HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_2);
    }
    if (started_channel_count >= 3U)
    {
      (void)HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_3);
    }
    return status;
  }

  /*
   * HAL_TIM_PWM_Start()每次都会尝试启动计数器。4个通道此时输出始终为低，
   * 所以全部使能后直接清除CEN并重置CNT，随后由发送函数统一启动。
   * 这里不能使用__HAL_TIM_DISABLE()，因为该宏在通道仍使能时不会清除CEN。
   */
  //CLEAR_BIT(htim->Instance->CR1, TIM_CR1_CEN);是什么？TIM_CR1_CEN是什么标志位
  //CR1: TIM的控制寄存器1，Control Register 1,
  //CEN全称是Counter Enable，即“计数器使能”
  /**
   * 
   * CEN = 1：TIM开始计数，CNT不断增加并产生PWM
   * CEN = 0：TIM停止计数，CNT保持不变
   */
  CLEAR_BIT(htim->Instance->CR1, TIM_CR1_CEN);
  __HAL_TIM_SET_COUNTER(htim, 0U);
  __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);

  return HAL_OK;
}

static void DShotOutput_StopTimerGroup(TIM_HandleTypeDef* htim)
{
  if ((htim == NULL) || (htim->Instance == NULL))
  {
    return;
  }

  /* 先停止计数器，确保清理期间不会再产生新的update事件。 */
  CLEAR_BIT(htim->Instance->CR1, TIM_CR1_CEN);

  if (htim->hdma[TIM_DMA_ID_UPDATE] != NULL)
  {
    //(void) 表示不使用它的返回值，避免编译器警告。DShotOutput_StopTimerDma()会停止DMA并让HAL内部状态恢复为可再次启动。

    (void)DShotOutput_StopTimerDma(htim);
  }

  /*
   * 将4个CCR的预装载值和活动值都恢复为0，再停止PWM通道。这样本帧结束后
   * 引脚保持低电平，下次启动也不会短暂输出上一帧的占空比。
   */
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, 0U);
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, 0U);
  __HAL_TIM_SET_COUNTER(htim, 0U);

  //HAL_TIM_GenerateEvent 是产生Update事件，让CCR =0 真正生效
  (void)HAL_TIM_GenerateEvent(htim, TIM_EVENTSOURCE_UPDATE);

  (void)HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_3);
  (void)HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_4);

  __HAL_TIM_SET_COUNTER(htim, 0U);
  __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
}

/*
 * update DMA正常完成时，HAL内部最终会调用这个官方弱回调。这里只做分发，
 * 以后增加TIM1时继续复用同一个入口。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  DShotOutput_HandleTimerPeriodElapsed(htim);
}

/* TIM update DMA发生错误时，HAL通过这个官方弱回调进入安全清理流程。 */
void HAL_TIM_ErrorCallback(TIM_HandleTypeDef* htim)
{
  DShotOutput_HandleTimerError(htim);
}
