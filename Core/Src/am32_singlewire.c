#include "am32_singlewire.h"

#include "main.h"
#include "tim.h"

#include <stddef.h>

/*
 * AM32 Bootloader单线串口固定为19200 8N1。
 * 1 MHz定时器下52个tick对应52 us，实际波特率约19230.8，误差约0.16%。
 */
#define AM32_SW_BIT_TICKS                52U
#define AM32_SW_FIRST_RX_SAMPLE_TICKS    78U
#define AM32_SW_MAX_TX_LENGTH            32U
#define AM32_SW_MAX_RX_LENGTH            64U

typedef enum
{
  AM32_SW_PHASE_IDLE = 0,
  AM32_SW_PHASE_TX,
  AM32_SW_PHASE_RX
} AM32SingleWirePhase;

typedef struct
{
  volatile AM32SingleWirePhase phase;
  volatile AM32SingleWireStatus result;
  volatile bool complete;
  uint8_t channel_index;
  uint16_t pin;

  uint8_t tx_data[AM32_SW_MAX_TX_LENGTH];
  uint8_t tx_length;
  volatile uint8_t tx_byte_index;
  volatile uint8_t tx_bit_index;

  uint8_t* rx_data;
  uint8_t rx_length;
  volatile uint8_t rx_byte_index;
  volatile uint8_t rx_bit_index;
  volatile uint8_t rx_current_byte;
} AM32SingleWireContext;

static AM32SingleWireContext g_am32_sw;

static const uint16_t g_am32_sw_pins[AM32_SINGLEWIRE_CHANNEL_COUNT] =
{
  Dshot1_Pin, Dshot2_Pin, Dshot3_Pin, Dshot4_Pin,
  Dshot5_Pin, Dshot6_Pin, Dshot7_Pin, Dshot8_Pin
};

static void AM32SingleWire_ConfigureOutput(void);
static void AM32SingleWire_ConfigureInput(void);
static void AM32SingleWire_StartTimer(uint16_t ticks);
static void AM32SingleWire_StopTimer(void);
static void AM32SingleWire_EnableSelectedExti(void);
static void AM32SingleWire_DisableSelectedExti(void);
static void AM32SingleWire_Complete(AM32SingleWireStatus result);
static void AM32SingleWire_HandleTimerInterrupt(void);
static void AM32SingleWire_HandleExtiInterrupt(uint16_t pending_pins);


/**
 * 初始化软件状态并打开：
 *- SYSCFG时钟
 *- 对应GPIO的EXTI中断
 *- EXTI0/1/2/3/9_5/15_10 NVIC
 * 
 */
void AM32SingleWire_Init(void)
{
  g_am32_sw.phase = AM32_SW_PHASE_IDLE;
  g_am32_sw.result = AM32_SINGLEWIRE_OK;
  g_am32_sw.complete = true;

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0U, 0U);
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0U, 0U);
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0U, 0U);
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0U, 0U);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0U, 0U);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0U, 0U);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}


/**
 * 这是一次完整单线事务入口: 
 * 
 *  检查通道、长度、缓冲区和超时
    确认当前没有另一个事务
    将发送数据复制到内部缓冲区
    选择DShot通道对应的GPIO
    关闭旧EXTI
    GPIO切为推挽输出
    立即输出起始位低电平
    启动TIM6，每52 μs推进发送状态
    发送结束后切为输入上拉
    用EXTI检测AM32返回数据的起始位
    TIM6在每个数据位中心采样
    等待收满数据或超时

    它从调用者角度是同步函数，但位时序由中断完成，因此全局中断没有被关闭。
 */
AM32SingleWireStatus AM32SingleWire_Transfer(
    uint8_t channel_index,
    const uint8_t* tx_data,
    uint8_t tx_length,
    uint8_t* rx_data,
    uint8_t rx_length,
    uint32_t timeout_ms)
{
  uint8_t i;
  uint32_t start_tick;

  if ((channel_index >= AM32_SINGLEWIRE_CHANNEL_COUNT) ||
      (tx_length == 0U) ||
      (tx_length > AM32_SW_MAX_TX_LENGTH) ||
      (tx_data == NULL) ||
      (rx_length > AM32_SW_MAX_RX_LENGTH) ||
      ((rx_length > 0U) && (rx_data == NULL)) ||
      (timeout_ms == 0U))
  {
    return AM32_SINGLEWIRE_INVALID_ARGUMENT;
  }

  if (g_am32_sw.phase != AM32_SW_PHASE_IDLE)
  {
    return AM32_SINGLEWIRE_BUSY;
  }

  g_am32_sw.channel_index = channel_index;
  g_am32_sw.pin = g_am32_sw_pins[channel_index];
  g_am32_sw.tx_length = tx_length;
  g_am32_sw.tx_byte_index = 0U;
  g_am32_sw.tx_bit_index = 0U;
  g_am32_sw.rx_data = rx_data;
  g_am32_sw.rx_length = rx_length;
  g_am32_sw.rx_byte_index = 0U;
  g_am32_sw.rx_bit_index = 0U;
  g_am32_sw.rx_current_byte = 0U;
  g_am32_sw.result = AM32_SINGLEWIRE_BUSY;
  g_am32_sw.complete = false;

  for (i = 0U; i < tx_length; i++)
  {
    g_am32_sw.tx_data[i] = tx_data[i];
  }

  AM32SingleWire_DisableSelectedExti();
  AM32SingleWire_ConfigureOutput();

  /* 起始位立即拉低；52 us后TIM6中断输出第一个数据位。 */
  GPIOA->BSRR = ((uint32_t)g_am32_sw.pin << 16U);
  g_am32_sw.phase = AM32_SW_PHASE_TX;
  AM32SingleWire_StartTimer(AM32_SW_BIT_TICKS);

  start_tick = HAL_GetTick();
  while (!g_am32_sw.complete)
  {
    /**
     * timeout_ms 是40ms
     */
    if ((uint32_t)(HAL_GetTick() - start_tick) >= timeout_ms)
    {
      AM32SingleWire_Abort();
      return AM32_SINGLEWIRE_TIMEOUT;
    }
  }

  return g_am32_sw.result;
}


/**
 * 中止当前的单线事务
 * 
 *  停止TIM6
    禁用EXTI
    状态回到空闲
    结果设为超时
    GPIO恢复输入上拉

    主要用于事务超时或上层异常恢复。
 */
void AM32SingleWire_Abort(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  AM32SingleWire_StopTimer();
  AM32SingleWire_DisableSelectedExti();
  g_am32_sw.phase = AM32_SW_PHASE_IDLE;
  g_am32_sw.result = AM32_SINGLEWIRE_TIMEOUT;
  g_am32_sw.complete = true;
  AM32SingleWire_ConfigureInput();

  if (primask == 0U)
  {
    __enable_irq();
  }
}

/**
 * 判断底层是否正在收发。
 */
bool AM32SingleWire_IsBusy(void)
{
  return g_am32_sw.phase != AM32_SW_PHASE_IDLE;
}

/**
 * 将当前DShot引脚配置为：推挽输出
 * 
 * 配置前先把输出寄存器设为高，避免模式切换时产生一个错误低脉冲。
*/
static void AM32SingleWire_ConfigureOutput(void)
{
  GPIO_InitTypeDef gpio = {0};

  /* 先把ODR置高，再切换为输出，避免模式切换瞬间产生额外低脉冲。 */
  GPIOA->BSRR = g_am32_sw.pin;
  gpio.Pin = g_am32_sw.pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);
}

/***
 * 将当前引脚配置为 输入上拉
 * 
 * 发送完成后用它接收AM32返回数据。
 */
static void AM32SingleWire_ConfigureInput(void)
{
  GPIO_InitTypeDef gpio = {0};

  if (g_am32_sw.pin == 0U)
  {
    return;
  }

  gpio.Pin = g_am32_sw.pin;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);
}

/**
 * 启动一次 TIM6定时器，ARR设为ticks-1，CNT清零，允许更新中断。
 * 
 * 它根据阶段动态选择：
 *  52 us：正常相邻bit
    78 us：检测到起始位后，采样第一个数据位中心
 */
static void AM32SingleWire_StartTimer(uint16_t ticks)
{
  __HAL_TIM_DISABLE(&htim6);
  __HAL_TIM_SET_AUTORELOAD(&htim6, (uint32_t)ticks - 1U);
  __HAL_TIM_SET_COUNTER(&htim6, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
  __HAL_TIM_ENABLE(&htim6);
}

/** 
 * 停止TIM6定时器
*/
static void AM32SingleWire_StopTimer(void)
{
  __HAL_TIM_DISABLE(&htim6);
  __HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);
  __HAL_TIM_SET_COUNTER(&htim6, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
}

/**
 * 为当前选中的GPIO引脚配置下降沿EXTI。
 * 
 * 下降沿代表AM32开始发送一个字节的起始位：高电平->低电平。EXTI中断服务函数会在下降沿时启动TIM6定时器，定时采样数据位。
 */
static void AM32SingleWire_EnableSelectedExti(void)
{
  uint32_t pin_index = g_am32_sw.channel_index < 4U
                           ? g_am32_sw.channel_index
                           : (uint32_t)g_am32_sw.channel_index + 4U;
  uint32_t exticr_index = pin_index >> 2U;
  uint32_t exticr_shift = (pin_index & 3U) * 4U;

  /* 所有DShot引脚均在GPIOA；EXTICR对应4-bit字段写0即选择Port A。 */
  SYSCFG->EXTICR[exticr_index] &= ~(0xFUL << exticr_shift);
  EXTI->RTSR1 &= ~(uint32_t)g_am32_sw.pin;
  EXTI->FTSR1 |= g_am32_sw.pin;
  EXTI->PR1 = g_am32_sw.pin;
  EXTI->IMR1 |= g_am32_sw.pin;
}

/**
 * 禁止当前通道的EXTI，避免在发送期间或采样期间重复触发。
 * IMR1:Interrupt Mask Register 1,控制 EXTI 0~31 是否允许产生中断 bit = 0:禁止中断，bit = 1:允许中断
 * PR1:Pending Register 1,用于清除 EXTI 0~31 的中断挂起标志
 */
static void AM32SingleWire_DisableSelectedExti(void)
{
  if (g_am32_sw.pin != 0U)
  {
    EXTI->IMR1 &= ~(uint32_t)g_am32_sw.pin;
    EXTI->PR1 = g_am32_sw.pin;
  }
}

/**
 * 完成一次DShot通信
 * 
 * 结束当前物理层事务：
    1. 停止TIM6
    2. 关闭EXTI
    3. 状态回到 IDLE
    4. 保存结果
    5. 最后设置完成标志
 */
static void AM32SingleWire_Complete(AM32SingleWireStatus result)
{
  AM32SingleWire_StopTimer();
  AM32SingleWire_DisableSelectedExti();
  g_am32_sw.phase = AM32_SW_PHASE_IDLE;
  g_am32_sw.result = result;
  g_am32_sw.complete = true;
}

/**
 * 实际逐bit发送和接收的核心。
 * 
 * 接收过程: 
 * EXTI检测到起始位下降沿后，不立即采样，而是等待：1.5 bit = 78us 后采样第一个数据位中心，之后每52 us采样下一个数据位中心。
 * 
 * 采完8位后检查停止位：
  - 高：字节有效
  - 低：帧格式错误
  一个字节完成后：
  - 如果已经收满，结束事务
  - 否则重新打开EXTI，等待下一个字节起始位
 */
static void AM32SingleWire_HandleTimerInterrupt(void)
{
  if (g_am32_sw.phase == AM32_SW_PHASE_TX)
  {
    uint8_t current_byte = g_am32_sw.tx_data[g_am32_sw.tx_byte_index];

    if (g_am32_sw.tx_bit_index < 8U)
    {
       /* 逐bit输出8N1的8个数据位，低位先发。*/
      if ((current_byte & (uint8_t)(1U << g_am32_sw.tx_bit_index)) != 0U)
      {
        GPIOA->BSRR = g_am32_sw.pin;
      }
      else
      {
        GPIOA->BSRR = ((uint32_t)g_am32_sw.pin << 16U);
      }
      g_am32_sw.tx_bit_index++;
      return;
    }

    if (g_am32_sw.tx_bit_index == 8U)
    {
      /* 第9个时隙输出8N1的停止位。 */
      GPIOA->BSRR = g_am32_sw.pin;
      g_am32_sw.tx_bit_index++;
      return;
    }

    g_am32_sw.tx_byte_index++;
    if (g_am32_sw.tx_byte_index < g_am32_sw.tx_length)
    {
      /* 停止位完整保持一个bit后，无额外间隔地开始下一个字节。 */
      g_am32_sw.tx_bit_index = 0U;
      GPIOA->BSRR = ((uint32_t)g_am32_sw.pin << 16U);
      return;
    }

    if (g_am32_sw.rx_length == 0U)
    {
      AM32SingleWire_Complete(AM32_SINGLEWIRE_OK);
      return;
    }

    /* 最后停止位结束：释放推挽输出并立即等待ESC回复的下降沿。 */
    AM32SingleWire_StopTimer();
    AM32SingleWire_ConfigureInput();
    g_am32_sw.phase = AM32_SW_PHASE_RX;
    AM32SingleWire_EnableSelectedExti();
    return;
  }

  if (g_am32_sw.phase == AM32_SW_PHASE_RX)
  {
    if (g_am32_sw.rx_bit_index < 8U)
    {
      /**
       * GPIOA->IDR & g_am32_sw.pin 代表当前DShot引脚的电平状态，如果为高电平，则在接收字节中对应位设为1，否则为0。
       * rx_current_byte |= (1U << rx_bit_index) 将当前采样的bit存入rx_current_byte中，rx_bit_index表示当前采样的是第几个bit。
       */
      if ((GPIOA->IDR & g_am32_sw.pin) != 0U)
      {
        g_am32_sw.rx_current_byte |=
            (uint8_t)(1U << g_am32_sw.rx_bit_index);
      }

      g_am32_sw.rx_bit_index++;
      /* 第一次使用78 us定位bit0中心，后续采样间隔都恢复为52 us。 */
      __HAL_TIM_SET_AUTORELOAD(&htim6, AM32_SW_BIT_TICKS - 1U);
      return;
    }

    /* 停止位中心必须为高，否则说明波特率、极性或线路存在问题。 */
    if ((GPIOA->IDR & g_am32_sw.pin) == 0U)
    {
      AM32SingleWire_Complete(AM32_SINGLEWIRE_FRAMING_ERROR);
      return;
    }

    g_am32_sw.rx_data[g_am32_sw.rx_byte_index] = g_am32_sw.rx_current_byte;
    g_am32_sw.rx_byte_index++;
    if (g_am32_sw.rx_byte_index >= g_am32_sw.rx_length)
    {
      AM32SingleWire_Complete(AM32_SINGLEWIRE_OK);
      return;
    }

    /*
     * 停止位中心到下一字节起始沿约有半个bit（26 us），先停TIM6再重开EXTI，
     * 足够捕获无字节间隔的连续回复。
     */
    AM32SingleWire_StopTimer();
    g_am32_sw.rx_bit_index = 0U;
    g_am32_sw.rx_current_byte = 0U;
    AM32SingleWire_EnableSelectedExti();
  }
}

/**
 * 当前GPIO检测到下降沿时：
    1. 禁止该EXTI，防止重复进入
    2. 清空当前接收字节
    3. 将接收位索引设为0
    4. 启动78 μs定时
    5. 等待到 bit0 中心采样
 */
static void AM32SingleWire_HandleExtiInterrupt(uint16_t pending_pins)
{
  if ((g_am32_sw.phase != AM32_SW_PHASE_RX) ||
      ((pending_pins & g_am32_sw.pin) == 0U))
  {
    return;
  }

  /* 起始位下降沿只负责定位；第一个数据位在1.5 bit后的中心读取。 */
  AM32SingleWire_DisableSelectedExti();
  g_am32_sw.rx_bit_index = 0U;
  g_am32_sw.rx_current_byte = 0U;
  AM32SingleWire_StartTimer(AM32_SW_FIRST_RX_SAMPLE_TICKS);
}

/**
 * 这是TIM6 HAL中断回调到单线模块的公共入口，内部转到 HandleTimerInterrupt()。
 */
void AM32SingleWire_HandleTimerPeriodElapsed(void)
{
  AM32SingleWire_HandleTimerInterrupt();
}

/**
 * EXTI中断函数
 * 
 * 它们负责：
    1. 判断并清除EXTI pending位
    2. 确认是不是当前选中的DShot通道
    3. 调用 HandleExtiInterrupt()
 */

void EXTI0_IRQHandler(void)
{
  EXTI->PR1 = Dshot1_Pin;
  AM32SingleWire_HandleExtiInterrupt(Dshot1_Pin);
}

void EXTI1_IRQHandler(void)
{
  EXTI->PR1 = Dshot2_Pin;
  AM32SingleWire_HandleExtiInterrupt(Dshot2_Pin);
}

void EXTI2_IRQHandler(void)
{
  EXTI->PR1 = Dshot3_Pin;
  AM32SingleWire_HandleExtiInterrupt(Dshot3_Pin);
}

void EXTI3_IRQHandler(void)
{
  EXTI->PR1 = Dshot4_Pin;
  AM32SingleWire_HandleExtiInterrupt(Dshot4_Pin);
}

void EXTI9_5_IRQHandler(void)
{
  uint16_t pending = (uint16_t)(EXTI->PR1 & (Dshot5_Pin | Dshot6_Pin));
  EXTI->PR1 = pending;
  AM32SingleWire_HandleExtiInterrupt(pending);
}

void EXTI15_10_IRQHandler(void)
{
  uint16_t pending =
      (uint16_t)(EXTI->PR1 & (Dshot7_Pin | Dshot8_Pin));
  EXTI->PR1 = pending;
  AM32SingleWire_HandleExtiInterrupt(pending);
}
