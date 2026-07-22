#include "dshot_output.h"

#include "motor_control.h"
#include "tim.h"

#include <stddef.h>

/* TIM2驱动DShot1～DShot4，TIM1驱动DShot5～DShot8。 */
#define DSHOT_OUTPUT_TIM2_FIRST_OUTPUT_INDEX   0U
#define DSHOT_OUTPUT_TIM1_FIRST_OUTPUT_INDEX   4U
#define DSHOT_OUTPUT_TIMER_CONTEXT_COUNT       2U
#define DSHOT_OUTPUT_DMA_BUFFER_COUNT          2U

/*
 * TIM7的输入时钟是48 MHz。PSC=47后计数频率为1 MHz，每计数一次正好是1 us；
 * ARR=1499表示从0计到1499，共1500次，因此更新中断周期严格为1500 us。
 * 这里检查CubeMX生成的配置，避免以后误改TIM7参数后仍以为输出周期是1.5 ms。
 */
#define DSHOT_OUTPUT_TIM7_EXPECTED_PRESCALER   47U
#define DSHOT_OUTPUT_TIM7_EXPECTED_PERIOD      1499U

/*
 * 本次整理只改变内部状态的组织方式，不改变外部函数声明和当前TIM2输出行为。
 * 旧变量与新位置的对应关系如下，旧的独立全局标志位已经被结构体成员替换：
 *
 * g_tim2_dma_buffers[]             -> g_tim2_output.dma_buffers[]
 * g_tim2_pwm_channels_started      -> g_tim2_output.pwm_channels_started
 * g_tim2_busy                      -> g_tim2_output.busy
 * g_tim2_send_*_count              -> g_tim2_output.send_*_count
 *
 * g_tim2_active_buffer_index       -> g_dshot_scheduler.active_buffer_index
 * g_tim2_prepare_buffer_index      -> g_dshot_scheduler.prepare_buffer_index
 * g_tim2_ready_buffer_index        -> g_dshot_scheduler.ready_buffer_index
 * g_tim2_prepare_requested         -> g_dshot_scheduler.prepare_requested
 * g_tim2_prepare_in_progress       -> g_dshot_scheduler.prepare_in_progress
 * g_tim2_ready_buffer_available    -> g_dshot_scheduler.ready_buffer_available
 * g_tim2_periodic_started          -> g_dshot_scheduler.periodic_started
 * g_tim7_period_count              -> g_dshot_scheduler.period_count
 * g_tim2_prepare_*_count           -> g_dshot_scheduler.prepare_*_count
 *
 * 第一组状态必须每个TIM各有一份；第二组状态必须由TIM1/TIM2共同使用。
 * 这样既能分别诊断两个DMA，又能保证未来8路输出在同一时刻切换同一批命令。
 */

/*
 * 一个DShotOutputTimerContext只描述“一组TIM四通道输出”的私有状态。
 * TIM1和TIM2各有一个上下文，不能共用busy、DMA缓冲区或完成计数，因为两路
 * DMA可能在不同的时刻完成。后续通用函数只接收上下文，不再复制TIM1/TIM2代码。
 * 
 * 上下文表示:操作某个对象时，需要保存和使用的全部相关信息
 */
typedef struct
{
  /* CubeMX生成的TIM句柄，用来找到寄存器和该TIM自己的update DMA句柄。 */
  TIM_HandleTypeDef* htim;

  /* 0表示本组读取DShot1～4，4表示本组读取DShot5～8。 */
  uint8_t first_output_index;

  /*
   * true表示公共TIM7调度器当前需要管理这组输出。
   * 本步骤先保持TIM2=true、TIM1=false，完成内部重构但不提前改变已验证的硬件输出。
   * 下一步统一TIM1/TIM2位周期后，只需启用TIM1上下文即可接入DShot5～8。
   */
  bool scheduler_enabled;

  /*
   * 每个TIM各自拥有A/B两块72-word缓冲区。DMA读active缓冲区时，主循环只写
   * 另一块prepare缓冲区，防止发送过程中数据被修改。
   */
  uint32_t dma_buffers[DSHOT_OUTPUT_DMA_BUFFER_COUNT]
                      [DSHOT_OUTPUT_DMA_BUFFER_LENGTH];

  /*
   * 4个PWM通道是否已经执行过一次HAL_TIM_PWM_Start()。
   * 这个标志就是原g_tim2_pwm_channels_started的通用版本：只启动一次，
   * 之后帧间保持通道使能并用CCR=0主动输出低电平。
   */
  volatile bool pwm_channels_started;

  /* DMA是否正在读取该上下文的active缓冲区，true时不能再次启动或覆盖它。 */
  volatile bool busy;

  /* 以下计数器只用于Keil调试，分别统计本TIM自己的发送和DMA运行结果。 */
  volatile uint32_t send_start_count;
  volatile uint32_t send_start_error_count;
  volatile uint32_t send_busy_skip_count;
  volatile uint32_t send_complete_count;
  volatile uint32_t send_error_count;
} DShotOutputTimerContext;

/*
 * 公共调度器只保存TIM1/TIM2必须同步共享的状态。
 * 两个定时器使用相同的A/B索引：只有一组命令对应的所有已启用TIM缓冲区都构建
 * 完成，ready才会发布。这样将来启用TIM1后，8路输出不会一半使用新命令、
 * 另一半仍使用旧命令。
 */
typedef struct
{
  /* active是下一次DMA读取的A/B编号，prepare是主循环下一次写入的编号。 */
  volatile uint8_t active_buffer_index;
  volatile uint8_t prepare_buffer_index;
  volatile uint8_t ready_buffer_index;

  /* TIM7是否已经提出准备请求，以及主循环是否正在构建整组缓冲区。 */
  volatile bool prepare_requested;
  volatile bool prepare_in_progress;

  /* true表示ready_buffer_index对应的所有已启用TIM缓冲区都已经完整写完。 */
  volatile bool ready_buffer_available;

  /* TIM7固定1.5 ms调度是否已经启动，防止重复启动和未初始化时发送。 */
  volatile bool periodic_started;

  /* 公共节拍与整组缓冲准备诊断计数，不再属于某一个具体TIM。 */
  volatile uint32_t period_count;
  volatile uint32_t prepare_count;
  volatile uint32_t prepare_not_ready_count;
  volatile uint32_t prepare_error_count;
  volatile uint32_t group_start_error_count;
} DShotOutputScheduler;

/*
 * 当前已验证的TIM2上下文参与调度；TIM1上下文先建立但暂不启用。
 * 这里用指定成员初始化，未列出的缓冲区、状态和计数器都由C语言自动清零。
 */
//创建TIM2的上下文对象
static DShotOutputTimerContext g_tim2_output =
{
  .htim = &htim2,
  .first_output_index = DSHOT_OUTPUT_TIM2_FIRST_OUTPUT_INDEX,
  .scheduler_enabled = true
};

//创建TIM1的上下文对象
static DShotOutputTimerContext g_tim1_output =
{
  .htim = &htim1,
  .first_output_index = DSHOT_OUTPUT_TIM1_FIRST_OUTPUT_INDEX,
  .scheduler_enabled = true
};

/* 公共代码通过这个表遍历两个上下文，顺序按DShot1～4、DShot5～8排列。 */
//这个数组的作用是让公共函数通过循环处理TIM2、TIM1,不必分别复制两套代码了
static DShotOutputTimerContext* const
    g_dshot_timer_contexts[DSHOT_OUTPUT_TIMER_CONTEXT_COUNT] =
{
  &g_tim2_output,
  &g_tim1_output
};

static DShotOutputScheduler g_dshot_scheduler;

static HAL_StatusTypeDef DShotOutput_StartFourPwmChannelsLow(
    TIM_HandleTypeDef* htim);
static DShotOutputTimerContext* DShotOutput_FindTimerContext(
    const TIM_HandleTypeDef* htim);
static bool DShotOutput_BuildEnabledTimerBuffers(uint8_t buffer_index);
static HAL_StatusTypeDef DShotOutput_ArmTimerContext(
    DShotOutputTimerContext* context,
    uint8_t buffer_index);
static HAL_StatusTypeDef DShotOutput_SendEnabledTimerGroupOnce(void);
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
  uint8_t context_index;

  if ((htim7.Instance != TIM7) ||
      (htim7.Init.Prescaler != DSHOT_OUTPUT_TIM7_EXPECTED_PRESCALER) ||
      (htim7.Init.Period != DSHOT_OUTPUT_TIM7_EXPECTED_PERIOD))
  {
    return HAL_ERROR;
  }

  previous_primask = DShotOutput_EnterCritical();
  if (g_dshot_scheduler.periodic_started)
  {
    DShotOutput_ExitCritical(previous_primask);
    return HAL_BUSY;
  }
  DShotOutput_ExitCritical(previous_primask);

  /*
   * 上电时motor_control中是停止命令。公共构建函数会把每个已启用上下文的
   * A、B缓冲区都完整构建好，再启动TIM7。当前只有TIM2参与；以后启用TIM1后，
   * 这里仍会先同时准备好DShot1～DShot8，第一次中断不会发送半组旧数据。
   */
  if ((!DShotOutput_BuildEnabledTimerBuffers(0U)) ||
      (!DShotOutput_BuildEnabledTimerBuffers(1U)))
  {
    return HAL_ERROR;
  }

  /*
   * 共享的A/B索引和准备标志只初始化一次。两个定时器上下文始终使用同一个
   * active_buffer_index，因此未来TIM1与TIM2会发送同一批MotorControl快照。
   */
  previous_primask = DShotOutput_EnterCritical();
  g_dshot_scheduler.active_buffer_index = 0U;
  g_dshot_scheduler.prepare_buffer_index = 0U;
  g_dshot_scheduler.ready_buffer_index = 1U;
  g_dshot_scheduler.prepare_requested = false;
  g_dshot_scheduler.prepare_in_progress = false;
  g_dshot_scheduler.ready_buffer_available = true;

  g_dshot_scheduler.period_count = 0U;
  g_dshot_scheduler.prepare_count = 0U;
  g_dshot_scheduler.prepare_not_ready_count = 0U;
  g_dshot_scheduler.prepare_error_count = 0U;
  g_dshot_scheduler.group_start_error_count = 0U;

  /*
   * busy和发送诊断计数属于具体定时器，必须逐个清零，不能放进公共调度器。
   * pwm_channels_started故意不清零：TIM7启动失败后再次初始化时，已经使能的
   * PWM通道应继续保持CCR=0，不可重复调用HAL_TIM_PWM_Start()。
   */
  for (context_index = 0U;
       context_index < DSHOT_OUTPUT_TIMER_CONTEXT_COUNT;
       context_index++)
  {
    DShotOutputTimerContext* context =
        g_dshot_timer_contexts[context_index];

    context->busy = false;
    context->send_start_count = 0U;
    context->send_start_error_count = 0U;
    context->send_busy_skip_count = 0U;
    context->send_complete_count = 0U;
    context->send_error_count = 0U;
  }

  __HAL_TIM_SET_COUNTER(&htim7, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
  DShotOutput_ExitCritical(previous_primask);

  /*
   * 在TIM7开始产生1.5 ms中断之前，逐个启动已启用上下文的4个PWM通道。
   * 当前循环只会启动TIM2；TIM1上下文存在但scheduler_enabled=false，所以不会
   * 提前改变DShot5～DShot8引脚。以后启用TIM1时不需要复制另一套启动代码。
   *
   * 每个上下文的pwm_channels_started都是“只启动一次”标志。PWM通道保持使能，
   * 帧间通过CCR=0主动输出低电平，不再使用HAL_TIM_PWM_Stop()使引脚悬空。
   */
  for (context_index = 0U;
       context_index < DSHOT_OUTPUT_TIMER_CONTEXT_COUNT;
       context_index++)
  {
    DShotOutputTimerContext* context =
        g_dshot_timer_contexts[context_index];

    if ((!context->scheduler_enabled) || context->pwm_channels_started)
    {
      continue;
    }

    status = DShotOutput_StartFourPwmChannelsLow(context->htim);
    if (status != HAL_OK)
    {
      return status;
    }
    context->pwm_channels_started = true;
  }

  previous_primask = DShotOutput_EnterCritical();
  g_dshot_scheduler.periodic_started = true;
  DShotOutput_ExitCritical(previous_primask);

  //HAL_TIM_Base_Start_IT()函数启动TIM7的基本定时器，并使能TIM7的中断。这样TIM7就会按照预设的时间间隔产生中断，从而触发DShot输出的调度。
  status = HAL_TIM_Base_Start_IT(&htim7);
  if (status != HAL_OK)
  {
    previous_primask = DShotOutput_EnterCritical();
    g_dshot_scheduler.periodic_started = false;
    DShotOutput_ExitCritical(previous_primask);
  }

  return status;
}

/**
 * prepare_requested：有没有CPU构建任务
 * ready_buffer_available：有没有已经构建完成的成品
 * busy：DMA是否正在发送active缓冲区
 * 
 */
void DShotOutput_PollTim2Preparation(void)
{
  uint8_t buffer_index;
  bool build_succeeded;
  uint32_t previous_primask;

  /*
   * TIM7中断只发布一个轻量的prepare_requested请求。主循环在这里接走请求，
   * 避免在中断里构建DMA数据，让1.5 ms节拍中断尽量短且稳定。
   * 一个请求代表“一整组已启用定时器”：当前构建TIM2的一块72-word缓冲；
   * 以后启用TIM1后，同一次请求会同时构建TIM2和TIM1对应编号的两块缓冲。
   */
  previous_primask = DShotOutput_EnterCritical();
  if ((!g_dshot_scheduler.periodic_started) ||
      (!g_dshot_scheduler.prepare_requested) ||
      g_dshot_scheduler.prepare_in_progress)
  {
    DShotOutput_ExitCritical(previous_primask);
    return;
  }

  //buffer_index是当前主循环要构建的缓冲区编号，0或1。g_dshot_scheduler.prepare_buffer_index是TIM7中断设置的下一次准备缓冲区编号，主循环在这里接走请求。
  buffer_index = g_dshot_scheduler.prepare_buffer_index;
  g_dshot_scheduler.prepare_requested = false;
  g_dshot_scheduler.prepare_in_progress = true;
  DShotOutput_ExitCritical(previous_primask);

  /*
   * 构建过程放在临界区之外，CAN和TIM中断仍可正常响应。公共调度器保证这个
   * buffer_index不是DMA正在读取的active编号，所以所有上下文都可以安全写入。
   */
  //DShotOutput_BuildEnabledTimerBuffers函数是把主循环中MotorControl的快照数据转换成TIM的DMA缓冲区格式，写入g_tim2_output.dma_buffers[buffer_index]。
  //如果构建成功，返回true；否则返回false。
  build_succeeded = DShotOutput_BuildEnabledTimerBuffers(buffer_index);

  previous_primask = DShotOutput_EnterCritical();
  if (build_succeeded)
  {
    /*
     * __DMB()保证所有已启用上下文的内存写入先完成，然后才发布ready标志。
     * TIM7中断一旦看到ready=true，就能确定这一组缓冲区已经全部完整可读。
     *
     * __DMB();全称为数据内存保障屏障(Data Memory Barrier)，它是一个内存屏障指令，用于确保在它之前的所有内存写入操作在它之后的内存读写操作之前完成。
     * 也就是说，在执行__DMB()之后，所有对内存的写入操作都已经被提交到内存中，确保数据的一致性和可见性。
     */
    __DMB();
    g_dshot_scheduler.ready_buffer_index = buffer_index;
    g_dshot_scheduler.ready_buffer_available = true;
    g_dshot_scheduler.prepare_count++;
  }
  else
  {
    /* 构建失败时保留请求，下一轮主循环重试，不发布不完整缓冲区。 */
    g_dshot_scheduler.prepare_requested = true;
    g_dshot_scheduler.prepare_error_count++;
  }
  g_dshot_scheduler.prepare_in_progress = false;
  DShotOutput_ExitCritical(previous_primask);
}

void DShotOutput_HandleTim7PeriodElapsed(void)
{
  uint8_t context_index;

  if (!g_dshot_scheduler.periodic_started)
  {
    return;
  }

  g_dshot_scheduler.period_count++;

  /*
   * 一帧DShot600约30 us，远小于1.5 ms。任意一个已启用上下文仍busy，都不能
   * 切换公共active编号，否则可能覆盖尚未完成的那一组DMA缓冲区。
   */
  for (context_index = 0U;
       context_index < DSHOT_OUTPUT_TIMER_CONTEXT_COUNT;
       context_index++)
  {
    DShotOutputTimerContext* context =
        g_dshot_timer_contexts[context_index];

    if (context->scheduler_enabled && context->busy)
    {
      context->send_busy_skip_count++;
      return;
    }
  }

  if (g_dshot_scheduler.ready_buffer_available)
  {
    /*
     * 主循环通过__DMB()后才置ready。这里先确认写入可见，再把完整缓冲区
     * 切换成active；从此刻到DMA完成，主循环不会再修改这块内存。
    */
    __DMB();
    g_dshot_scheduler.active_buffer_index =
        g_dshot_scheduler.ready_buffer_index;
    g_dshot_scheduler.ready_buffer_available = false;
  }
  else
  {
    /*
     * 下一块尚未准备完成时继续使用上一块active缓冲区。重复一帧旧命令比
     * 发送一块只写了一半的缓冲区安全，且该异常可由此计数器直接观察。
     */
    g_dshot_scheduler.prepare_not_ready_count++;
  }

  /*
   * active的另一块就是下一次inactive准备区。只有没有待处理请求且主循环
   * 没在构建时才发布新请求，避免重复覆盖同一个准备任务。
   */
  if ((!g_dshot_scheduler.prepare_requested) &&
      (!g_dshot_scheduler.prepare_in_progress))
  {
    //使用异或运算符^来切换active_buffer_index的值。如果active_buffer_index是0，那么0 ^ 1U的结果是1；如果active_buffer_index是1，那么1 ^ 1U的结果是0。这样就实现了在0和1之间切换，确保下一次准备缓冲区使用的是另一块缓冲区。
    g_dshot_scheduler.prepare_buffer_index =
        (uint8_t)(g_dshot_scheduler.active_buffer_index ^ 1U);
    g_dshot_scheduler.prepare_requested = true;
  }

  /*
   * 公共发送器先武装所有已启用TIM的DMA，全部成功后再集中使能计数器。
   * 当前只发送TIM2；以后打开TIM1上下文时，这一处调用会同时启动两组4通道。
   */
  (void)DShotOutput_SendEnabledTimerGroupOnce();
}


/* TIM1或TIM2的update DMA正常完成后，按句柄找到自己的上下文并完成清理。 */
void DShotOutput_HandleTimerPeriodElapsed(TIM_HandleTypeDef* htim)
{
  DShotOutputTimerContext* context = DShotOutput_FindTimerContext(htim);

  if ((context == NULL) || (!context->busy))
  {
    return;
  }

  /*
   * 72个word已经全部传输。当前生效的是低电平slot，因此可以立即停止
   * 计数器和DMA请求，并允许下一次发送重新使用缓冲区。4个PWM通道继续
   * 保持使能，让CCR=0在两帧之间持续主动输出低电平。
  */
  DShotOutput_StopTimerGroup(htim);
  context->send_complete_count++;
  context->busy = false;
}

/* TIM1或TIM2的update DMA异常后，只停止并记录发生错误的那个定时器。 */
void DShotOutput_HandleTimerError(TIM_HandleTypeDef* htim)
{
  DShotOutputTimerContext* context = DShotOutput_FindTimerContext(htim);

  if ((context == NULL) || (!context->busy))
  {
    return;
  }

  /* DMA异常时立即停止计数和DMA，并用CCR=0让4路引脚继续保持安全低电平。 */
  DShotOutput_StopTimerGroup(htim);
  context->send_error_count++;
  context->busy = false;
}

/*
 * HAL回调只提供TIM句柄，这个函数把句柄重新映射到对应的私有上下文。
 * 因此完成和错误回调不需要分别编写TIM1版、TIM2版，也不会错误修改另一组计数器。
 */
static DShotOutputTimerContext* DShotOutput_FindTimerContext(
    const TIM_HandleTypeDef* htim)
{
  uint8_t context_index;

  if (htim == NULL)
  {
    return NULL;
  }

  for (context_index = 0U;
       context_index < DSHOT_OUTPUT_TIMER_CONTEXT_COUNT;
       context_index++)
  {
    DShotOutputTimerContext* context =
        g_dshot_timer_contexts[context_index];

    if (context->htim == htim)
    {
      return context;
    }
  }

  return NULL;
}

/*
 * 为同一个A/B编号构建所有已启用定时器的DMA缓冲区。
 * 当前只会构建TIM2的DShot1～DShot4；将TIM1的scheduler_enabled改为true后，
 * 同一次调用还会构建TIM1的DShot5～DShot8。只有全部成功才返回true，调用者
 * 才能发布ready标志，所以不会出现前4路已更新、后4路仍是旧命令的情况。
 * 
 * buffer_index 是 0 或 1，分别对应A/B两块缓冲区。当前只有TIM2参与，所以只会构建DShot1～DShot4；以后启用TIM1后，同一次调用还会构建DShot5～DShot8。
 * 这个一次只能构建A或B一块缓冲区，不能同时构建两块缓冲区。因为DMA可能正在读取active缓冲区，主循环只能写另一块prepare缓冲区，防止发送过程中数据被修改。
 */
static bool DShotOutput_BuildEnabledTimerBuffers(uint8_t buffer_index)
{
  uint8_t context_index;
  bool enabled_context_found = false;

  if (buffer_index >= DSHOT_OUTPUT_DMA_BUFFER_COUNT)
  {
    return false;
  }

  for (context_index = 0U;
       context_index < DSHOT_OUTPUT_TIMER_CONTEXT_COUNT;
       context_index++)
  {
    DShotOutputTimerContext* context =
        g_dshot_timer_contexts[context_index];

    if (!context->scheduler_enabled)
    {
      continue;
    }

    enabled_context_found = true;
    if (!DShotOutput_BuildTimerDmaBuffer(
            context->first_output_index,
            context->dma_buffers[buffer_index],
            DSHOT_OUTPUT_DMA_BUFFER_LENGTH))
    {
      return false;
    }
  }

  return enabled_context_found;
}

/*
 * “武装”只完成发送前准备，不立即让定时器开始计数：
 * 1. 让update DMA指向该上下文自己的active缓冲区；
 * 2. 置busy，表示DMA已经占用这块缓冲区；
 * 3. 把CNT和旧update标志恢复到统一起点。
 *
 * 把__HAL_TIM_ENABLE()留给公共组发送器，是为了以后启用TIM1时，先确保两组
 * DMA都准备成功，再依次启动TIM1/TIM2。若第二组准备失败，第一组还没有产生
 * 波形，可以完整回滚，不会只发出前4路。
 */
static HAL_StatusTypeDef DShotOutput_ArmTimerContext(
    DShotOutputTimerContext* context,
    uint8_t buffer_index)
{
  HAL_StatusTypeDef status;

  if ((context == NULL) ||
      (!context->scheduler_enabled) ||
      (!context->pwm_channels_started) ||
      (!g_dshot_scheduler.periodic_started) ||
      (buffer_index >= DSHOT_OUTPUT_DMA_BUFFER_COUNT))
  {
    return HAL_ERROR;
  }

  //busy为true表示DMA正在读取该上下文的active缓冲区，不能再次启动或覆盖它。
  if (context->busy)
  {
    return HAL_BUSY;
  }

  status = DShotOutput_StartTimerDma(
      context->htim,
      context->dma_buffers[buffer_index],
      DSHOT_OUTPUT_DMA_BUFFER_LENGTH);
  if (status != HAL_OK)
  {
    DShotOutput_StopTimerGroup(context->htim);
    return status;
  }

  /* busy必须在使能计数器之前置位，防止DMA回调看到尚未更新的旧状态。 */
  context->busy = true;
  __HAL_TIM_SET_COUNTER(context->htim, 0U);
  __HAL_TIM_CLEAR_FLAG(context->htim, TIM_FLAG_UPDATE);

  return HAL_OK;
}

/*
 * 公共调度器每到一个TIM7节拍只调用这一个发送入口。
 * 第一轮检查所有已启用上下文都处于可发送状态；第二轮逐个武装DMA；全部成功后
 * 第三轮才启动各定时器。当前只有TIM2参与，因此硬件行为与整理前保持一致。
 * 将来启用TIM1后，两个定时器仍各用自己的busy和诊断计数，但共享本次发送时刻。
 */
static HAL_StatusTypeDef DShotOutput_SendEnabledTimerGroupOnce(void)
{
  HAL_StatusTypeDef status;
  uint8_t context_index;
  uint8_t rollback_index;
  bool enabled_context_found = false;

  //看1.5ms定时器是否已经启动，未启动就直接返回错误。
  if (!g_dshot_scheduler.periodic_started)
  {
    return HAL_ERROR;
  }

  /* 先检查整组，避免发现第二个TIM忙时第一个TIM已经被提前武装。 */
  for (context_index = 0U;
       context_index < DSHOT_OUTPUT_TIMER_CONTEXT_COUNT;
       context_index++)
  {
    DShotOutputTimerContext* context =
        g_dshot_timer_contexts[context_index];

    if (!context->scheduler_enabled)
    {
      continue;
    }

    enabled_context_found = true;
    if (context->busy)
    {
      context->send_busy_skip_count++;
      return HAL_BUSY;
    }
    if (!context->pwm_channels_started)
    {
      context->send_start_error_count++;
      g_dshot_scheduler.group_start_error_count++;
      return HAL_ERROR;
    }
  }

  //两组都没有Dshot帧输出
  if (!enabled_context_found)
  {
    g_dshot_scheduler.group_start_error_count++;
    return HAL_ERROR;
  }

  for (context_index = 0U;
       context_index < DSHOT_OUTPUT_TIMER_CONTEXT_COUNT;
       context_index++)
  {
    DShotOutputTimerContext* context =
        g_dshot_timer_contexts[context_index];

    if (!context->scheduler_enabled)
    {
      continue;
    }

    status = DShotOutput_ArmTimerContext(
        context,
        g_dshot_scheduler.active_buffer_index);
    if (status != HAL_OK)
    {
      if (status == HAL_BUSY)
      {
        context->send_busy_skip_count++;
      }
      else
      {
        context->send_start_error_count++;
      }
      g_dshot_scheduler.group_start_error_count++;

      /*
       * 前面的上下文只完成了DMA武装，还没有启动计数器，可以安全撤销。
       * 被撤销的上下文也记录一次start_error，表示本组没有真正发出。
       */
      for (rollback_index = 0U;
           rollback_index < context_index;
           rollback_index++)
      {
        DShotOutputTimerContext* rollback_context =
            g_dshot_timer_contexts[rollback_index];

        if (rollback_context->scheduler_enabled && rollback_context->busy)
        {
          DShotOutput_StopTimerGroup(rollback_context->htim);
          rollback_context->busy = false;
          rollback_context->send_start_error_count++;
        }
      }
      return status;
    }
  }

  /*
   * 所有DMA均已武装，从CNT=0依次使能定时器。软件写寄存器会有极小先后差，
   * 但不会有一组启动失败而另一组已经发送的半组输出。
   */
  for (context_index = 0U;
       context_index < DSHOT_OUTPUT_TIMER_CONTEXT_COUNT;
       context_index++)
  {
    DShotOutputTimerContext* context =
        g_dshot_timer_contexts[context_index];

    if (context->scheduler_enabled)
    {
      __HAL_TIM_ENABLE(context->htim);
      context->send_start_count++;
    }
  }

  return HAL_OK;
}

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
   * 将4个CCR的预装载值和活动值都恢复为0。TIM2计数器虽然停止，但PWM通道
   * 仍保持使能，所以PA0～PA3会由定时器主动输出低电平，而不是变成高阻状态。
   * 这里故意不调用HAL_TIM_PWM_Stop()：该函数会清除CC1E～CC4E，使电调端的
   * 偏置电路有机会把悬空信号线抬高，AM32可能因此误判为反相/双向DShot。
   */
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, 0U);
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, 0U);
  __HAL_TIM_SET_COUNTER(htim, 0U);

  //HAL_TIM_GenerateEvent 是产生Update事件，让CCR =0 真正生效
  (void)HAL_TIM_GenerateEvent(htim, TIM_EVENTSOURCE_UPDATE);

  __HAL_TIM_SET_COUNTER(htim, 0U);
  __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
}

//DShotOutput_EnterCritical()和DShotOutput_ExitCritical()是用来保护共享状态变量的临界区函数。它们通过禁用和恢复中断来确保在访问共享变量时不会被中断打断，从而避免数据竞争和不一致的状态。
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
