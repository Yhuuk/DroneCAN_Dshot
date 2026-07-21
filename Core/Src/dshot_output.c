#include "dshot_output.h"

#include "motor_control.h"
#include "tim.h"

#include <stddef.h>

/* TIM2固定驱动DShot1～DShot4，所以它使用第一组输出索引0。 */
#define DSHOT_OUTPUT_TIM2_FIRST_OUTPUT_INDEX  0U
#define DSHOT_OUTPUT_TIM2_DMA_BUFFER_COUNT     2U

/*
 * TIM7的输入时钟是48 MHz。PSC=47后计数频率为1 MHz，每计数一次正好是1 us；
 * ARR=1499表示从0计到1499，共1500次，因此更新中断周期严格为1500 us。
 * 这里检查CubeMX生成的配置，避免以后误改TIM7参数后仍以为输出周期是1.5 ms。
 */
#define DSHOT_OUTPUT_TIM7_EXPECTED_PRESCALER   47U
#define DSHOT_OUTPUT_TIM7_EXPECTED_PERIOD      1499U

/*
 * TIM2采用A/B双缓冲，二维数组的第一维就是缓冲区编号：
 *   g_tim2_dma_buffers[0]：A缓冲区，共72个word；
 *   g_tim2_dma_buffers[1]：B缓冲区，共72个word。
 *
 * 双缓冲解决的问题：
 *   1. TIM2 DMA发送时会连续读取当前active缓冲区，这块内存必须保持不变；
 *   2. 主循环可以同时在另一块inactive缓冲区中准备最新4路DShot数据；
 *   3. 主循环只有在72个word全部写完后，才把该缓冲区标记为ready；
 *   4. TIM7中断只切换到已经ready的完整缓冲区，绝不会读取“写了一半”的数据。
 *
 * 例如本周期DMA正在读取A，主循环就在B中构建下一帧。B全部构建完成后发布ready；
 * 下一个1.5 ms的TIM7中断切换到B发送，同时要求主循环重新准备A。之后A/B交替使用。
 * 如果主循环来不及准备下一块，中断会重复发送上一块完整缓冲区，而不是发送损坏帧。
 */
static uint32_t
    g_tim2_dma_buffers[DSHOT_OUTPUT_TIM2_DMA_BUFFER_COUNT]
                      [DSHOT_OUTPUT_DMA_BUFFER_LENGTH];

/* active是下一次DMA读取的缓冲区；prepare是主循环下一次需要写入的缓冲区。 */
static volatile uint8_t g_tim2_active_buffer_index;
static volatile uint8_t g_tim2_prepare_buffer_index;
static volatile uint8_t g_tim2_ready_buffer_index;
static volatile bool g_tim2_prepare_requested;
static volatile bool g_tim2_prepare_in_progress;
static volatile bool g_tim2_ready_buffer_available;
static volatile bool g_tim2_periodic_started;

//表示TIM2 DMA是否正在发送，true期间不能再次启动同一个TIM2 DMA burst。
static volatile bool g_tim2_busy;

/*
 * 供Keil debugger观察固定周期发送和双缓冲是否正常。
 * 正常运行时：
 *   g_tim7_period_count、g_tim2_send_start_count和
 *   g_tim2_send_complete_count应当以基本相同的速度增加；
 *   error、busy_skip和prepare_not_ready通常应保持为0。
 */
static volatile uint32_t g_tim7_period_count;
static volatile uint32_t g_tim2_send_start_count;
static volatile uint32_t g_tim2_send_start_error_count;
static volatile uint32_t g_tim2_send_busy_skip_count;
static volatile uint32_t g_tim2_send_complete_count;
static volatile uint32_t g_tim2_send_error_count;
static volatile uint32_t g_tim2_prepare_count;
static volatile uint32_t g_tim2_prepare_not_ready_count;
static volatile uint32_t g_tim2_prepare_error_count;

static HAL_StatusTypeDef DShotOutput_StartFourPwmChannelsLow(
    TIM_HandleTypeDef* htim);
static void DShotOutput_StopTimerGroup(TIM_HandleTypeDef* htim);
static uint32_t DShotOutput_EnterCritical(void);
static void DShotOutput_ExitCritical(uint32_t previous_primask);

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

HAL_StatusTypeDef DShotOutput_StartTim2Periodic(void)
{
  HAL_StatusTypeDef status;
  uint32_t previous_primask;

  if ((htim7.Instance != TIM7) ||
      (htim7.Init.Prescaler != DSHOT_OUTPUT_TIM7_EXPECTED_PRESCALER) ||
      (htim7.Init.Period != DSHOT_OUTPUT_TIM7_EXPECTED_PERIOD))
  {
    return HAL_ERROR;
  }

  /*
   * 上电时motor_control中是停止命令。先把A、B都完整构建成停止帧，再启动TIM7，
   * 确保第一次中断到来时已经有可直接发送的完整数据，不依赖主循环是否及时运行。
   */
  if ((!DShotOutput_BuildTimerDmaBuffer(
          DSHOT_OUTPUT_TIM2_FIRST_OUTPUT_INDEX,
          g_tim2_dma_buffers[0],
          DSHOT_OUTPUT_DMA_BUFFER_LENGTH)) ||
      (!DShotOutput_BuildTimerDmaBuffer(
          DSHOT_OUTPUT_TIM2_FIRST_OUTPUT_INDEX,
          g_tim2_dma_buffers[1],
          DSHOT_OUTPUT_DMA_BUFFER_LENGTH)))
  {
    return HAL_ERROR;
  }

  previous_primask = DShotOutput_EnterCritical();
  if (g_tim2_periodic_started)
  {
    DShotOutput_ExitCritical(previous_primask);
    return HAL_BUSY;
  }

  g_tim2_active_buffer_index = 0U;
  g_tim2_prepare_buffer_index = 0U;
  g_tim2_ready_buffer_index = 1U;
  g_tim2_prepare_requested = false;
  g_tim2_prepare_in_progress = false;
  g_tim2_ready_buffer_available = true;
  g_tim2_busy = false;

  g_tim7_period_count = 0U;
  g_tim2_send_start_count = 0U;
  g_tim2_send_start_error_count = 0U;
  g_tim2_send_busy_skip_count = 0U;
  g_tim2_send_complete_count = 0U;
  g_tim2_send_error_count = 0U;
  g_tim2_prepare_count = 0U;
  g_tim2_prepare_not_ready_count = 0U;
  g_tim2_prepare_error_count = 0U;

  __HAL_TIM_SET_COUNTER(&htim7, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
  g_tim2_periodic_started = true;
  DShotOutput_ExitCritical(previous_primask);

  status = HAL_TIM_Base_Start_IT(&htim7);
  if (status != HAL_OK)
  {
    previous_primask = DShotOutput_EnterCritical();
    g_tim2_periodic_started = false;
    DShotOutput_ExitCritical(previous_primask);
  }

  return status;
}

void DShotOutput_PollTim2Preparation(void)
{
  uint8_t buffer_index;
  bool build_succeeded;
  uint32_t previous_primask;

  /*
   * TIM7中断只发布一个轻量的prepare_requested请求。主循环在这里接走请求，
   * 避免在中断里遍历并构建72个CCR值，让1.5 ms节拍中断尽量短且稳定。
   */
  previous_primask = DShotOutput_EnterCritical();
  if ((!g_tim2_periodic_started) ||
      (!g_tim2_prepare_requested) ||
      g_tim2_prepare_in_progress)
  {
    DShotOutput_ExitCritical(previous_primask);
    return;
  }

  buffer_index = g_tim2_prepare_buffer_index;
  g_tim2_prepare_requested = false;
  g_tim2_prepare_in_progress = true;
  DShotOutput_ExitCritical(previous_primask);

  /*
   * 构建过程放在临界区之外，CAN和TIM中断仍可正常响应。此时buffer_index必定
   * 不是DMA正在读取的active缓冲区，所以写入不会破坏当前正在发送的DShot帧。
   */
  build_succeeded = DShotOutput_BuildTimerDmaBuffer(
      DSHOT_OUTPUT_TIM2_FIRST_OUTPUT_INDEX,
      g_tim2_dma_buffers[buffer_index],
      DSHOT_OUTPUT_DMA_BUFFER_LENGTH);

  previous_primask = DShotOutput_EnterCritical();
  if (build_succeeded)
  {
    /*
     * __DMB()保证72个word的内存写入先完成，然后才发布ready标志。
     * TIM7中断一旦看到ready=true，就能确定整块缓冲区已经完整可读。
     */
    __DMB();
    g_tim2_ready_buffer_index = buffer_index;
    g_tim2_ready_buffer_available = true;
    g_tim2_prepare_count++;
  }
  else
  {
    /* 构建失败时保留请求，下一轮主循环重试，不发布不完整缓冲区。 */
    g_tim2_prepare_requested = true;
    g_tim2_prepare_error_count++;
  }
  g_tim2_prepare_in_progress = false;
  DShotOutput_ExitCritical(previous_primask);
}

void DShotOutput_HandleTim7PeriodElapsed(void)
{
  HAL_StatusTypeDef status;

  if (!g_tim2_periodic_started)
  {
    return;
  }

  g_tim7_period_count++;

  /*
   * 一帧DShot600约30 us，远小于1.5 ms。若此时仍busy，说明上一次DMA没有
   * 正常结束或调试器长时间暂停，本周期不能覆盖DMA状态，只记录并跳过。
   */
  if (g_tim2_busy)
  {
    g_tim2_send_busy_skip_count++;
    return;
  }

  if (g_tim2_ready_buffer_available)
  {
    /*
     * 主循环通过__DMB()后才置ready。这里先确认写入可见，再把完整缓冲区
     * 切换成active；从此刻到DMA完成，主循环不会再修改这块内存。
     */
    __DMB();
    g_tim2_active_buffer_index = g_tim2_ready_buffer_index;
    g_tim2_ready_buffer_available = false;
  }
  else
  {
    /*
     * 下一块尚未准备完成时继续使用上一块active缓冲区。重复一帧旧命令比
     * 发送一块只写了一半的缓冲区安全，且该异常可由此计数器直接观察。
     */
    g_tim2_prepare_not_ready_count++;
  }

  /*
   * active的另一块就是下一次inactive准备区。只有没有待处理请求且主循环
   * 没在构建时才发布新请求，避免重复覆盖同一个准备任务。
   */
  if ((!g_tim2_prepare_requested) && (!g_tim2_prepare_in_progress))
  {
    g_tim2_prepare_buffer_index =
        (uint8_t)(g_tim2_active_buffer_index ^ 1U);
    g_tim2_prepare_requested = true;
  }

  status = DShotOutput_SendTim2Once();
  if (status == HAL_OK)
  {
    g_tim2_send_start_count++;
  }
  else if (status == HAL_BUSY)
  {
    g_tim2_send_busy_skip_count++;
  }
  else
  {
    g_tim2_send_start_error_count++;
  }
}

HAL_StatusTypeDef DShotOutput_SendTim2Once(void)
{
  HAL_StatusTypeDef status;
  const uint32_t* dma_buffer;

  /*
   * 本函数只发送双缓冲中当前active的完整数据，不再现场读取motor_control。
   * active缓冲区由TIM7中断选择，并且在DMA完成前不会被主循环修改。
   */
  if (g_tim2_busy)
  {
    return HAL_BUSY;
  }

  if (!g_tim2_periodic_started)
  {
    return HAL_ERROR;
  }

  dma_buffer = g_tim2_dma_buffers[g_tim2_active_buffer_index];

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
                                     dma_buffer,
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

static uint32_t DShotOutput_EnterCritical(void)
{
  uint32_t previous_primask = __get_PRIMASK();

  __disable_irq();
  return previous_primask;
}

static void DShotOutput_ExitCritical(uint32_t previous_primask)
{
  /*
   * 如果调用前中断本来就是关闭的，这里不能擅自打开；只有原来允许中断时
   * 才恢复使能。临界区只保护几个双缓冲状态变量，不包含72-word构建过程。
   */
  if (previous_primask == 0U)
  {
    __enable_irq();
  }
}

/*
 * update DMA正常完成时，HAL内部最终会调用这个官方弱回调。这里只做分发，
 * TIM7基本定时中断和TIM2 DMA完成最终都会进入这个同名官方回调，所以必须
 * 先根据句柄区分来源。以后增加TIM1时也继续复用同一个入口。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  if (htim == &htim7)
  {
    DShotOutput_HandleTim7PeriodElapsed();
    return;
  }

  DShotOutput_HandleTimerPeriodElapsed(htim);
}

/* TIM update DMA发生错误时，HAL通过这个官方弱回调进入安全清理流程。 */
void HAL_TIM_ErrorCallback(TIM_HandleTypeDef* htim)
{
  DShotOutput_HandleTimerError(htim);
}
