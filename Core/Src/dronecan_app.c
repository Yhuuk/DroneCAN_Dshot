#include "dronecan_app.h"

#include "can.h"
#include "canard.h"
#include "dshot_output.h"
#include "motor_control.h"

#include "uavcan.equipment.esc.RawCommand.h"

/*
 * 本模块是 STM32 HAL CAN driver 和 libcanard 之间的边界层。
 *
 * HAL/CubeMX 负责：
 *   - 配置时钟、引脚、CAN bit timing 和 NVIC；
 *   - 提供 hcan1 句柄和 CAN1_RX0_IRQHandler() 中断入口。
 *
 * 本模块负责：
 *   - 配置运行时 CAN filter；
 *   - 启动 CAN 接收；
 *   - 管理 libcanard 实例和回调函数；
 *   - 后续把收到的 CAN frame 转换成 DroneCAN transfer。
 *
 * 本阶段补充：HAL CAN -> libcanard 的接收桥接已经接入；
 * 具体 DroneCAN 数据类型仍然先不接收。
 * 本阶段再次补充：ShouldAcceptTransfer 已允许接收 RawCommand 广播，
 * 但 OnTransferReceived 里的 RawCommand 解码仍未接入。
 * 本阶段再次补充：OnTransferReceived 已能解码并暂存 RawCommand，
 * 当前仍然不会驱动 DShot 输出。
 * 本阶段再次补充：RawCommand 的前 8 路已经会被映射并保存为 DShot 命令，
 * 但仍未生成 16-bit DShot 帧，也没有操作定时器或实际输出引脚。
 * 本阶段再次补充：DShot帧、CCR和TIM2 DMA发送均已接入；TIM7现在每1.5 ms
 * 触发一次TIM2发送，并使用2 s连续零命令启动互锁保护实际油门输出。
 */

 /*
  在你的工程里，最终路径会是：
  CAN 原始帧
  -> HAL_CAN_GetRxMessage()
  -> CanardCANFrame
  -> canardHandleRxFrame()
  -> libcanard 解析成 CanardRxTransfer
  -> DroneCAN_OnTransferReceived()
  -> 你的代码解码 RawCommand
  -> 得到油门值
  -> 转成 DShot 输出
*/

/*
 * 调试阶段临时使用的静态 Node ID。
 *
 * DroneCAN 的节点 ID 范围是 1..127。实物总线上不能和其它节点冲突。
 * 后续如果需要，可以把这里改成参数配置，或者实现 dynamic node allocation。
 */
#define DRONECAN_APP_NODE_ID             42U

/*
 * libcanard 不自己 malloc，而是使用应用层提供的内存池。
 *
 * 这个内存池主要用于：
 *   - RX 多帧传输的重组；
 *   - TX 发送队列缓存。
 *
 * 2048 字节对当前这个小节点来说是一个偏保守的起步值。等真实 DroneCAN
 * 流量跑起来后，可以查看 allocator statistics，再决定是否调大或调小。
 * 
 * 当前 libcanard 默认块大小是 32 字节，所以 2048 字节大约是 64 个内部内存块。
 */
#define DRONECAN_APP_MEM_POOL_SIZE       2048U

/*
 * CAN RX 软件队列长度。
 *
 * HAL 的 RX FIFO 很小，中断里应尽快把硬件 FIFO 读空；但 DroneCAN/libcanard
 * 的协议处理放在主循环里做更稳妥，所以这里加一个小 ring buffer 作为中间层。
 *
 * 这个队列使用“空一格”的环形队列写法，所以实际最多缓存
 * DRONECAN_APP_RX_QUEUE_SIZE - 1 帧。队列满时会丢弃新帧，并增加
 * g_rx_queue_overflow_count，后续调试时可以在 debugger 里观察它。
 */
#define DRONECAN_APP_RX_QUEUE_SIZE       16U

/*
 * 上电、超时或错误后，必须连续收到2 s的零油门RawCommand，才允许非零
 * 命令进入实际输出。等待期间仍固定发送有效的DShot 0x0000停止帧。
 */
#define DRONECAN_APP_DSHOT_ZERO_HOLD_USEC         2000000ULL

//timestamp_usec 是一个 64 位的时间戳，单位是微秒。这个时间戳是 libcanard 用来计算传输超时的。
//DroneCAN_RxQueueItem 是一个结构体，表示从 CAN 接收队列中取出的一帧数据。它包含了 CAN 帧的头信息、数据和时间戳。
typedef struct
{
  CAN_RxHeaderTypeDef header;
  uint8_t data[CANARD_CAN_FRAME_MAX_DATA_LEN];
  uint64_t timestamp_usec;
} DroneCAN_RxQueueItem;

static CanardInstance g_canard;
static uint8_t g_canard_mem_pool[DRONECAN_APP_MEM_POOL_SIZE];

// g_rx_queue 是一个环形队列，用于缓存从 CAN 接收的帧。它的大小是 DRONECAN_APP_RX_QUEUE_SIZE。
static DroneCAN_RxQueueItem g_rx_queue[DRONECAN_APP_RX_QUEUE_SIZE];

//g_rx_queue_head 是环形队列的头索引，表示下一个要写入的位置。g_rx_queue_tail 是环形队列的尾索引，表示下一个要读取的位置。g_rx_queue_overflow_count 是一个计数器，记录在队列满时丢弃的帧数。
static volatile uint8_t g_rx_queue_head = 0U;
static volatile uint8_t g_rx_queue_tail = 0U;
static volatile uint32_t g_rx_queue_overflow_count = 0U;

/*
 * RawCommand 接收调试状态。
 *
 * 当前阶段先把最后一次成功解码的命令保存在 RAM 中，方便通过 debugger
 * 观察 cmd.len 和 cmd.data[]，暂时不把这些油门值交给 DShot。
 * 变量使用 volatile，避免编译器因为应用代码尚未读取它们而省略更新。
 * 解码失败时只增加错误计数，不覆盖上一条已经验证有效的命令。
 */
static volatile struct uavcan_equipment_esc_RawCommand g_last_raw_command;
static volatile uint8_t g_last_raw_command_source_node_id = 0U;
static volatile uint32_t g_raw_command_received_count = 0U;
static volatile uint32_t g_raw_command_decode_error_count = 0U;
static volatile uint32_t g_raw_command_mapping_error_count = 0U;

typedef enum
{
  /* Keil中观察g_dshot_state：0表示仍只允许停止帧，1表示允许实际油门。 */
  DRONECAN_DSHOT_STATE_WAIT_ZERO = 0,
  DRONECAN_DSHOT_STATE_RUNNING
} DroneCAN_DShotState;

/*
 * RawCommand解码和映射成功后只提出一次发送请求，真正启动TIM2 DMA仍放在
 * DroneCAN_App_Poll()主循环上下文中。DMA忙时请求会保留到下一轮，不在
 * libcanard回调中等待，也不会按照主循环的最高速度无条件重复发送。
 * 本阶段补充：一次请求发送机制已替换为TIM7固定1.5 ms调度。RawCommand
 * 回调只更新最新目标值，DShot发送频率不再跟随DroneCAN消息到达间隔变化。
 * 本阶段再次补充：TIM7中断只发送已经准备好的双缓冲区；主循环负责准备
 * 另一块缓冲区，避免在中断里构建72个CCR值。
 */
static volatile DroneCAN_DShotState g_dshot_state =
    DRONECAN_DSHOT_STATE_WAIT_ZERO;
static bool g_dshot_zero_hold_active = false;
static uint64_t g_dshot_zero_hold_start_usec = 0ULL;

/* 供Keil debugger观察启动互锁和超时退回等待状态的执行次数。 */
static volatile uint32_t g_dshot_arm_complete_count = 0U;
static volatile uint32_t g_dshot_zero_hold_reset_count = 0U;
static volatile uint32_t g_dshot_timeout_disarm_count = 0U;

static void DroneCAN_OnTransferReceived(CanardInstance* ins,
                                        CanardRxTransfer* transfer);
static bool DroneCAN_ShouldAcceptTransfer(const CanardInstance* ins,
                                          uint64_t* out_data_type_signature,
                                          uint16_t data_type_id,
                                          CanardTransferType transfer_type,
                                          uint8_t source_node_id);
static HAL_StatusTypeDef DroneCAN_ConfigCanFilter(void);

//DroneCAN_RxQueueNextIndex是要用记录队列的下一个索引。这个函数的作用是计算环形队列的下一个索引，确保索引在队列大小范围内循环。
static uint8_t DroneCAN_RxQueueNextIndex(uint8_t index);

//DroneCAN_RxQueuePushFromIsr是从中断服务例程中调用的函数，用于将接收到的 CAN 帧推入 RX 队列。这个函数会检查队列是否已满，如果满了就丢弃新帧并增加溢出计数，否则将新帧写入队列。
static bool DroneCAN_RxQueuePushFromIsr(const CAN_RxHeaderTypeDef* rx_header,
                                        const uint8_t* rx_data,
                                        uint64_t timestamp_usec);

//DroneCAN_RxQueuePop是从主循环中调用的函数，用于从 RX 队列中弹出一帧 CAN 数据。如果队列为空，返回 false；如果有数据，返回 true 并将数据写入 out_item。
static bool DroneCAN_RxQueuePop(DroneCAN_RxQueueItem* out_item);

//DroneCAN_BuildCanardFrame是将 DroneCAN_RxQueueItem 转换为 libcanard 的 CanardCANFrame 的函数。这个函数会根据 DroneCAN 帧的 ID 和数据长度，填充 CanardCANFrame 的各个字段。
static bool DroneCAN_BuildCanardFrame(const DroneCAN_RxQueueItem* item,
                                      CanardCANFrame* out_frame);

//DroneCAN_GetTimestampUsec是获取当前时间戳的函数，返回值是一个 64 位的微秒时间戳。这个时间戳用于 libcanard 的传输超时计算。                                      
static uint64_t DroneCAN_GetTimestampUsec(void);
static void DroneCAN_ResetDShotZeroHold(void);
static void DroneCAN_EnterDShotWaitZeroState(void);
static void DroneCAN_HandleMappedRawCommand(uint64_t timestamp_usec);
static void DroneCAN_PollDShotState(uint64_t now_usec);
static void DroneCAN_PollDShotSend(uint64_t now_usec);


/**
 * typedef enum
 * {
 *  HAL_OK       = 0x00,
 *   HAL_ERROR    = 0x01,
 *   HAL_BUSY     = 0x02,
 *   HAL_TIMEOUT  = 0x03
 * } HAL_StatusTypeDef;
 * 
 */
HAL_StatusTypeDef DroneCAN_App_Init(void)
{
  /*
   * 在第一次启动DMA之前先构建8路有效停止帧。后面的周期启动函数还会把
   * TIM2的A/B两块DMA缓冲区都准备为停止帧，因此TIM7第一个1.5 ms节拍
   * 到来时就能直接发送完整的0x0000帧。
   */
  MotorControl_Init();
  g_dshot_state = DRONECAN_DSHOT_STATE_WAIT_ZERO;
  DroneCAN_ResetDShotZeroHold();

  /*
   * 先初始化 libcanard。
   *
   * 这里 libcanard 只是在 RAM 里建立协议状态，不会操作 CAN 硬件。
   * 真正启动 CAN 外设是在后面的 HAL_CAN_Start()。
   */
  //现在代码还没有调用 canardHandleRxFrame()，所以这个池子基本只是初始化好了，还没真正忙起来。
  //本阶段补充：canardHandleRxFrame() 已经在 DroneCAN_App_Poll() 中调用；内存池开始用于 libcanard 的 RX 解析状态。
  canardInit(&g_canard,
             g_canard_mem_pool,
             sizeof(g_canard_mem_pool),
             DroneCAN_OnTransferReceived,
             DroneCAN_ShouldAcceptTransfer,
             NULL);

  //g_canard 是 libcanard 的实例，里面有 node_id 字段。这里设置本节点的 Node ID。实例就是一个结构体，g_canard就是这个结构体变量，里面有各种状态和配置参数。node_id 就是 libcanard 里用来标识本节点的 ID。
  //g_canard.node_id = DRONECAN_APP_NODE_ID;这个是什么意思
  canardSetLocalNodeID(&g_canard, DRONECAN_APP_NODE_ID);

  if (DroneCAN_ConfigCanFilter() != HAL_OK)
  {
    return HAL_ERROR;
  }

  /*
   * HAL_CAN_Start() 把 CAN1 从“已初始化”切到“正常工作”状态。
   *
   * HAL_CAN_ActivateNotification() 打开 CAN 外设内部的 RX 中断源。
   * NVIC 这一层的 IRQ 开关已经由 HAL_CAN_MspInit() 打开了。
   */
  //HAL_CAN_Start()是启动CAN外设的函数，返回值是HAL_StatusTypeDef类型，表示函数执行的状态。
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  //HAL_CAN_ActivateNotification()是打开中断 这个函数的第二个参数是一个位掩码，指定要打开哪些 CAN 中断。
  //CAN_IT_RX_FIFO0_MSG_PENDING 是其中一个中断源，表示当 FIFO0 里至少有一帧 CAN 数据时触发。就是有数据就会触发
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /*
   * CAN接收链路准备完成后启动TIM7固定节拍。TIM7_IRQHandler()由CubeMX放在
   * stm32l4xx_it.c中，它调用HAL_TIM_IRQHandler(&htim7)，随后HAL再调用
   * dshot_output.c底部实现的HAL_TIM_PeriodElapsedCallback()。
   */
  if (DShotOutput_StartTim2Periodic() != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

void DroneCAN_App_Poll(void)
{
  DroneCAN_RxQueueItem item;
  uint64_t now_usec;

  /*
   * 当前第一步还没有接入周期任务。
   *
   * 下一步会添加一个小的 CAN RX 队列。到时候这个 Poll 函数会把队列里的
   * CAN frame 取出来，再交给 canardHandleRxFrame() 做 DroneCAN 协议解析。
   *
   * 本阶段补充：RX 队列已经接入。现在 Poll 会从 ring buffer 取出 CAN 帧，
   * 转换成 libcanard 的 CanardCANFrame，然后调用 canardHandleRxFrame()。
   * 目前 DroneCAN_ShouldAcceptTransfer() 仍然返回 false，所以 libcanard 会解析
   * 帧头和 transfer 信息，但还不会真正接收 RawCommand 等具体数据类型。
   * 后续阶段补充：ShouldAcceptTransfer 和 OnTransferReceived 已经实现，现在能够
   * 接收、解码并映射 RawCommand；上面两行保留为早期阶段记录。
   * 本阶段补充：主循环每次处理完 RX 队列后都会检查 RawCommand 的 100 ms
   * 超时保护；超时后 motor_control 会把 8 路 DShot 缓存命令全部置为停止。
   */
  while (DroneCAN_RxQueuePop(&item))
  {
    //CanardCANFrame的字段分别是：id、data、data_len、iface_id、iface_mask、canfd。这个函数会把 DroneCAN_RxQueueItem 里的 header 和 data 转换成 CanardCANFrame 的各个字段。
    //canfd是一个布尔值，表示这个帧是否是 CAN FD 帧。当前这个项目只使用经典 CAN，所以 canfd 设置为 false。
    //iface_mask是一个位掩码，表示这个帧来自哪些 CAN 接口。当前只有一个 CAN 接口（CAN1），所以 iface_mask 设置为 0。
    CanardCANFrame frame;
    int16_t result;

    if (DroneCAN_BuildCanardFrame(&item, &frame))
    {
      //canardHandleRxFrame()是canard库的一个函数，用于处理接收到的 CAN 帧。它会根据帧的 ID 和数据长度，解析出 DroneCAN transfer，并调用应用层的回调函数。
      //result是canardHandleRxFrame()的返回值，表示函数执行的状态。返回值是一个 int16_t 类型的整数，可能是正数、零或负数。正数表示成功处理了多少字节的数据，零表示没有处理任何数据，负数表示发生了错误。
      result = canardHandleRxFrame(&g_canard,
                                   &frame,
                                   item.timestamp_usec);
      /*
       * 当前阶段只把帧送进 libcanard，还不根据返回值做错误统计。
       * 因为 ShouldAcceptTransfer() 暂时返回 false，收到 DroneCAN 帧时
       * 常见返回值会是 CANARD_ERROR_RX_NOT_WANTED 的负值，这是预期现象。
       * 后续阶段补充：ShouldAcceptTransfer() 已经接收 RawCommand；这里仍暂时不做
       * result 错误统计，保留到专门完善 DroneCAN 诊断状态时处理。
       */
      (void)result;
    }
  }

  /*
   * 超时检查放在主循环，不放在 CAN 中断中。即使当前没有收到新 CAN 帧，
   * 只要 DroneCAN_App_Poll() 持续运行，100 ms 保护仍然会按时生效。
   */
  //RX 队列处理完后调用 MotorControl_Poll()，然后比较的是 当前时间 - 新命令时间。只要收到新的有效命令，超时就会重新开始计数
  //这个是放在RX 队列处理完后，可以考虑一下这个处理的顺序问题
  now_usec = DroneCAN_GetTimestampUsec();
  //判断是否超时，若是大于100ms，就把所有的 DShot 命令置为停止。
  MotorControl_Poll(now_usec);

  /*
   * 收到并成功映射RawCommand后，在主循环中使用TIM2发送一次DShot1～DShot4。
   * 一次DMA发送约30 us；如果上一帧仍未完成，就保留pending请求，等完成
   * 回调清除busy后由下一轮Poll重试。多个等待中的RawCommand会自然合并为
   * 一次发送，发送时读取的是motor_control中最新的4路CCR数据。
   * 本阶段补充：现在不再由pending触发发送。下面先更新2 s启动/超时状态机，
   * 再准备TIM7下一个1.5 ms节拍要使用的inactive双缓冲区；实际发送只由
   * TIM7中断触发。因此无命令时也会持续发停止帧，RawCommand到达时间抖动
   * 不会直接变成DShot帧间隔抖动。
   */
  DroneCAN_PollDShotState(now_usec);
  DroneCAN_PollDShotSend(now_usec);
}

// HAL_CAN_RxFifo0MsgPendingCallback()是 HAL CAN driver 的一个回调函数。当 CAN1 的 RX FIFO0 里至少有一帧 CAN 数据时，HAL 会调用这个函数。这个函数的作用是把 FIFO0 里的所有帧都读出来，并暂时丢弃。
//本阶段补充：现在读出来后不再丢弃，而是写入 RX ring buffer，等待主循环交给 libcanard。
//为什么要读空？因为 RX pending 标志只有在实际读出 FIFO 后才会清掉。如果不读，CAN 中断会一直反复进入
//HAL_CAN_RxFifo0MsgPendingCallback()是官方中断回调函数， HAL_CAN_IRQHandler(&hcan1);回调函数的入口
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  //如果不是 CAN1 的中断，就直接返回。因为这个回调函数可能会被多个 CAN 外设调用，但我们只关心 CAN1。
  if (hcan->Instance != CAN1)
  {
    return;
  }

  /*
   * 临时的“只清空 FIFO”接收路径。
   *
   * 当 FIFO0 里至少有一帧 CAN 数据时，HAL 会调用这个 callback。
   * FIFO pending 标志只有在实际读出帧之后才会清掉。所以即使当前还不解析
   * DroneCAN，也必须把 FIFO 读空，否则中断会反复进入。
   *
   * 下一步会把这里的“读出后丢弃”替换成“写入 ring buffer”。随后在
   * DroneCAN_App_Poll() 里取出这些帧，并调用 canardHandleRxFrame()。
   */
  //HAL_CAN_GetRxFifoFillLevel是返回 FIFO0 里有多少帧 CAN 数据的函数。只要 FIFO0 里还有数据，就循环读取。
  //HAL_CAN_GetRxMessage是从 FIFO0 里读出一帧 CAN 数据的函数。它会把帧的头信息放到 rx_header 里，把帧的数据放到 rx_data 里。如果读取失败，就跳出循环。
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
      break;
    }

    //CAN header 和 data分别是什么 目前还没有用到，先显式标记未使用，避免编译器警告。
    //rx_header 和 rx_data 就会被真正转换成 CanardCANFrame
    //本阶段补充：这里已经从“读出后丢弃”改成“写入 RX ring buffer”。
    //中断里只做轻量工作，真正调用 libcanard 的 canardHandleRxFrame() 放在 DroneCAN_App_Poll()。
    (void)DroneCAN_RxQueuePushFromIsr(&rx_header,
                                      rx_data,
                                      DroneCAN_GetTimestampUsec());
  }
}

static uint8_t DroneCAN_RxQueueNextIndex(uint8_t index)
{
  index++;
  if (index >= DRONECAN_APP_RX_QUEUE_SIZE)
  {
    index = 0U;
  }

  return index;
}

static bool DroneCAN_RxQueuePushFromIsr(const CAN_RxHeaderTypeDef* rx_header,
                                        const uint8_t* rx_data,
                                        uint64_t timestamp_usec)
{
  uint8_t next_head;
  DroneCAN_RxQueueItem* item;
  uint8_t i;

  //next_head得到的是g_rx_queue_head的下一个索引
  //这里是空一帧的环形队列写法。g_rx_queue_head指向下一个要写入的位置，g_rx_queue_tail指向下一个要读取的位置。如果 next_head 等于 g_rx_queue_tail，说明队列满了。
  next_head = DroneCAN_RxQueueNextIndex(g_rx_queue_head);
  if (next_head == g_rx_queue_tail)
  {
    /*
     * 队列满时丢弃最新帧。这里不能阻塞等待主循环消费，否则会把 CAN 中断
     * 卡住，反而更容易造成硬件 FIFO 溢出。
     */
    g_rx_queue_overflow_count++;
    return false;
  }

  item = &g_rx_queue[g_rx_queue_head];
  item->header = *rx_header;
  item->timestamp_usec = timestamp_usec;

  for (i = 0U; i < CANARD_CAN_FRAME_MAX_DATA_LEN; i++)
  {
    item->data[i] = rx_data[i];
  }

  /*
   * head 最后更新。这样主循环只有在一帧 header/data/timestamp 都写完之后，
   * 才可能看到这个新队列项。
   */
  g_rx_queue_head = next_head;
  return true;
}

// DroneCAN_RxQueuePop是从主循环中调用的函数，用于从 RX 队列中弹出一帧 CAN 数据。如果队列为空，返回 false；如果有数据，返回 true 并将数据写入 out_item。
static bool DroneCAN_RxQueuePop(DroneCAN_RxQueueItem* out_item)
{
  //g_rx_queue_tail == g_rx_queue_head 表示队列为空。因为 head 指向下一个要写入的位置，tail 指向下一个要读取的位置。如果两者相等，说明没有数据可读。
  if (g_rx_queue_tail == g_rx_queue_head)
  {
    return false;
  }

  //读的话是main loop，写的话是中断。这里不需要临界区保护，因为 head/tail 的更新是原子操作，而且中断里只写 head，主循环只读 tail。
  //out_item是地址，*out_item是解引用，表示把队列里的数据拷贝到 out_item 指向的内存里。g_rx_queue[g_rx_queue_tail] 是当前 tail 指向的队列项。
  *out_item = g_rx_queue[g_rx_queue_tail];
  g_rx_queue_tail = DroneCAN_RxQueueNextIndex(g_rx_queue_tail);
  return true;
}

static bool DroneCAN_BuildCanardFrame(const DroneCAN_RxQueueItem* item,
                                      CanardCANFrame* out_frame)
{
  CanardCANFrame frame = {0};
  uint8_t i;

  /*
   * DroneCAN 使用 29-bit extended data frame。
   * 硬件 filter 当前故意放宽为“全部接收”，所以这里在软件桥接层丢弃
   * standard frame 和 remote frame，避免把非 DroneCAN 帧交给 libcanard。
   */
  //header.IDE是CAN帧的标识符扩展位，表示是否使用扩展ID。CAN_ID_EXT是一个宏(表示扩展ID的值)，表示扩展ID的值。如果不是扩展ID，就返回false，表示不接受这个帧。
  if (item->header.IDE != CAN_ID_EXT)
  {
    return false;
  }

  //header.RTR是CAN帧的远程传输请求位，表示是否是远程帧。CAN_RTR_DATA是一个宏，表示数据帧的值。如果不是数据帧，就返回false，表示不接受这个帧。
  if (item->header.RTR != CAN_RTR_DATA)
  {
    return false;
  }

  //header.DLC是CAN帧的数据长度码，表示数据字段的字节数。CANARD_CAN_FRAME_MAX_DATA_LEN是libcanard定义的最大数据长度。如果DLC大于最大数据长度，就返回false，表示不接受这个帧。
  if (item->header.DLC > CANARD_CAN_FRAME_MAX_DATA_LEN)
  {
    return false;
  }

  //header.ExtId
  //CANARD_CAN_EXT_ID_MASK是libcanard定义的掩码，用于提取扩展ID的有效位。CANARD_CAN_FRAME_EFF是libcanard定义的标志，表示这是一个扩展帧。
  frame.id = (item->header.ExtId & CANARD_CAN_EXT_ID_MASK) | CANARD_CAN_FRAME_EFF;
  //长度是0~8字节，DLC是CAN帧的长度码，表示数据字段的字节数。这里把DLC转换成uint8_t类型，赋值给frame.data_len。
  frame.data_len = (uint8_t)item->header.DLC;
  //frame.iface_id是libcanard定义的CAN接口ID，表示这个帧来自哪个CAN接口。当前只有一个CAN接口（CAN1），所以设置为0。
  frame.iface_id = 0U;

  //这里设置成0和1的区别是什么
  // frame.iface_id = 1U;

  //
  // frame.canfd = false;

  for (i = 0U; i < frame.data_len; i++)
  {
    frame.data[i] = item->data[i];
  }

  *out_frame = frame;
  return true;
}

static uint64_t DroneCAN_GetTimestampUsec(void)
{
  /*
   * libcanard 需要单调递增的微秒时间戳，用于多帧 transfer 超时判断。
   * 当前先用 HAL_GetTick() 的毫秒 tick 转成微秒，精度不高但足够用于
   * 这个阶段的接收桥接验证；后续如果要更精细的超时/统计，可以换成硬件定时器。
   */
  return ((uint64_t)HAL_GetTick()) * 1000ULL;
}

static void DroneCAN_ResetDShotZeroHold(void)
{
  g_dshot_zero_hold_active = false;
  g_dshot_zero_hold_start_usec = 0ULL;
}

static void DroneCAN_EnterDShotWaitZeroState(void)
{
  /*
   * 等待状态始终只允许停止帧进入输出缓存。后续即使没有DroneCAN消息，
   * TIM7的1.5 ms调度器也会继续发送这些有效的0x0000帧。
   */
  MotorControl_ForceStop();
  g_dshot_state = DRONECAN_DSHOT_STATE_WAIT_ZERO;
  DroneCAN_ResetDShotZeroHold();
}

static void DroneCAN_HandleMappedRawCommand(uint64_t timestamp_usec)
{
  if (g_dshot_state != DRONECAN_DSHOT_STATE_WAIT_ZERO)
  {
    return;
  }

  if (!MotorControl_AreAllDShotCommandsStopped())
  {
    /*
     * 启动互锁尚未解除时收到非零油门，必须立即恢复停止缓存并重新等待
     * 一段完整的2 s零命令。RawCommand调试副本仍会保存真实接收值。
     */
    if (g_dshot_zero_hold_active)
    {
      g_dshot_zero_hold_reset_count++;
    }
    MotorControl_ForceStop();
    DroneCAN_ResetDShotZeroHold();
    return;
  }

  /*
   * 第一条有效零命令开始计时。后续零命令会持续刷新motor_control的
   * 100 ms新鲜度，但不改变起点；这样必须连续保持零命令满2 s。
   */
  if (!g_dshot_zero_hold_active)
  {
    g_dshot_zero_hold_active = true;
    g_dshot_zero_hold_start_usec = timestamp_usec;
  }
}

static void DroneCAN_PollDShotState(uint64_t now_usec)
{
  if (g_dshot_state == DRONECAN_DSHOT_STATE_RUNNING)
  {
    /*
     * 运行中超过100 ms没有有效RawCommand时，MotorControl_Poll()已经把
     * 缓存切成停止帧。这里同步退回WAIT_ZERO，恢复输出前必须重新完成
     * 2 s连续零命令过程。
     */
    if (!MotorControl_HasFreshRawCommand())
    {
      DroneCAN_EnterDShotWaitZeroState();
      g_dshot_timeout_disarm_count++;
    }
    return;
  }

  /*
   * 等待期间如果零命令流中断超过100 ms，或缓存不再全零，就取消本轮计时。
   * 这可防止只收到一条零命令、等待2 s后也被误认为完成启动。
   */
  if ((!MotorControl_HasFreshRawCommand()) ||
      (!MotorControl_AreAllDShotCommandsStopped()))
  {
    if (g_dshot_zero_hold_active)
    {
      g_dshot_zero_hold_reset_count++;
    }
    DroneCAN_ResetDShotZeroHold();
    return;
  }

  if (g_dshot_zero_hold_active &&
      ((now_usec - g_dshot_zero_hold_start_usec) >=
       DRONECAN_APP_DSHOT_ZERO_HOLD_USEC))
  {
    g_dshot_state = DRONECAN_DSHOT_STATE_RUNNING;
    DroneCAN_ResetDShotZeroHold();
    g_dshot_arm_complete_count++;
  }
}

static void DroneCAN_PollDShotSend(uint64_t now_usec)
{
  /*
   * 参数保留为当前微秒时间戳，便于以后增加“准备耗时/命令生效延迟”统计，
   * 当前双缓冲准备本身不需要时间值，因此显式标记暂未使用。
   *
   * 这里不再判断1.5 ms是否到达，也不直接启动TIM2。TIM7硬件负责严格节拍，
   * 主循环只响应中断提出的请求，在inactive缓冲区中准备下一帧。
   */
  (void)now_usec;
  DShotOutput_PollTim2Preparation();
}

static HAL_StatusTypeDef DroneCAN_ConfigCanFilter(void)
{
  CAN_FilterTypeDef filter = {0};

  /*
   * 第一阶段调试用 filter：接收所有 CAN frame 到 FIFO0。
   *
   * DroneCAN 使用 29-bit extended CAN ID。后续在接收桥接代码里，我们会丢弃
   * 不是 extended frame 的数据。当前硬件 filter 先放宽，是为了让早期总线
   * 调试更简单；等 RawCommand 接收验证通过后，可以再把 filter 收窄。
   * 
   * CAN_FILTERMODE_IDMASK：ID + mask 模式
   * CAN_FILTERSCALE_32BIT：使用 32 位过滤器格式。DroneCAN 使用 29-bit extended CAN ID，所以用 32-bit scale 合适；16-bit scale 更偏向较短的标准 ID 场景。
   * FilterIdHigh/Low：要匹配的目标 ID，被拆成高 16 位和低 16 位。
   * FilterMaskIdHigh/Low：哪些 ID 位需要比较。当前都是 0，所以不比较任何位，也就是接收全部。
   * FilterFIFOAssignment = CAN_FILTER_FIFO0：通过过滤器的帧放进 RX FIFO0。
   * SlaveStartFilterBank = 14;在单 CAN中这个字段不会生效，真正使用的是 filter.FilterBank = 0;
   */

  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;   //0~27

  return HAL_CAN_ConfigFilter(&hcan1, &filter);
}


//CanardShouldAcceptTransfer这个是libcanard的官方回调函数，DroneCAN_ShouldAcceptTransfer这个是自己命名，但是函数参数格式是官方规定的。
//什么时候调用：libcanard 内部调用。在哪里注册：canardInit()。
/**
 * ins 是 libcanard 的实例指针，但是当前工程只有一个 libcanard 实例（也就是g_canard），所以这个参数暂时没用。
 * out_data_type_signature 是 libcanard 要求应用层提供的类型签名，用于校验多帧 transfer 的 CRC。
 * data_type_id 是 DroneCAN 的数据类型 ID，表示这个 transfer 的类型。判断是不是 uavcan.equipment.esc.RawCommand 就是看这个 ID。
 * transfer_type 是 transfer 的类型，可以是广播、单播等。判断是不是广播消息就看这个参数。
 * source_node_id 是发送这个 transfer 的节点 ID。比如，飞控 Node ID = 10 当前的转换板 Node ID = 42。如果是匿名节点发送的广播消息，这个值就是 0。
 * 现在 source_node_id 没用上，就是不限制广播消息来源
 */
static bool DroneCAN_ShouldAcceptTransfer(const CanardInstance* ins,
                                          uint64_t* out_data_type_signature,
                                          uint16_t data_type_id,
                                          CanardTransferType transfer_type,
                                          uint8_t source_node_id)
{
  (void)ins;
  (void)source_node_id;

  /*
   * libcanard 在收到一个新的 transfer 时，会调用这个回调函数，询问应用层是否要接收这个 transfer。
   *
   * 下一步接好 CAN RX 队列之后，这里会开始接受
   * uavcan.equipment.esc.RawCommand，并返回它的 data type signature。
   *
   * 本阶段补充：CAN RX 队列已经接好，现在只接受广播形式的
   * uavcan.equipment.esc.RawCommand。其他消息和服务仍然返回 false，
   * 避免 libcanard 为当前应用不需要的数据类型分配接收内存。
   */
  //CanardTransferTypeBroadcast是libcanard定义的枚举值，表示这个 transfer 是广播类型。
  if ((transfer_type == CanardTransferTypeBroadcast) &&
      (data_type_id == UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID))
  {
    /*
     * 当回调返回 true 时，libcanard 要求应用同时提供正确的类型签名。
     * libcanard 会使用这个签名校验多帧 transfer 的 CRC；签名不正确时，
     * 即使 CAN 帧已经全部收到，也不能得到一个有效的 RawCommand transfer。
     */
    *out_data_type_signature = UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE;
    return true;
  }

  return false;
}

/*
也是给 libcanard 的回调。只有 ShouldAcceptTransfer() 返回 true，并且 libcanard 已经完整重组出一个 transfer 后，
才会进这里。当前为空；以后会在这里解码 RawCommand，然后转成电调/DShot 输出。
*/


/**
 * 函数名：我们自定义
 * 函数参数格式：libcanard 官方规定
 * 什么时候调用：libcanard 内部调用
 * 在哪里注册：canardInit()
 */

 //CanardOnTransferReception是libcanard的官方回调函数类型，DroneCAN_OnTransferReceived是自己命名，但是函数参数格式是官方规定的。
static void DroneCAN_OnTransferReceived(CanardInstance* ins,
                                        CanardRxTransfer* transfer)
{
  struct uavcan_equipment_esc_RawCommand raw_command = {0};

  (void)ins;

  /*
   * 只有当 DroneCAN_ShouldAcceptTransfer() 返回 true，并且 libcanard 已经把
   * 一个完整 transfer 重组完成后，才会进入这个 callback。
   *
   * 本阶段补充：这里已经接入 RawCommand 解码，但只保存调试数据，
   * 还不会更新定时器、DMA 或任何 DShot 输出。
   * 本阶段再次补充：解码成功后会把前 8 路交给 motor_control 做安全映射，
   * 当前仍然只更新 RAM 中的 DShot 命令缓存。
   */
  if ((transfer->transfer_type != CanardTransferTypeBroadcast) ||
      (transfer->data_type_id != UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID))
  {
    /*
     * ShouldAcceptTransfer 当前只允许 RawCommand 广播。这里再次检查类型，
     * 是为了让以后增加其他 DroneCAN 数据类型时，各类型仍有明确的处理入口。
     */
    return;
  }

  /*
   * 生成的 decode 函数返回 false 表示成功，返回 true 表示 Payload 非法。
   * 它会处理 RawCommand 的 14-bit 有符号数组和多帧 Payload，应用层不需要
   * 自己按照字节或位偏移解析 transfer。
   */
  if (uavcan_equipment_esc_RawCommand_decode(transfer, &raw_command))
  {
    g_raw_command_decode_error_count++;
    return;
  }

  /*
   * cmd.data[0..7] 分别对应 DShot1..DShot8。少于 8 路时，motor_control
   * 会把缺少的输出保存为停止；多于 8 路时，本节点只使用前 8 路。
   * 如果映射失败，motor_control 会把全部输出命令安全置零。
   */
  //raw_command.cmd.data 本身是一个数组，raw_command.cmd.len 是数组的长度。
  //所以传给函数时会自动变成首元素地址
  if (!MotorControl_UpdateDShotCommands(raw_command.cmd.data,
                                         raw_command.cmd.len,
                                         transfer->timestamp_usec))
  {
    //映射失败时只增加错误计数，不覆盖上一条已经验证有效的命令。
    //本阶段补充：为保证安全，motor_control 会把缓存命令全部置零并取消超时计时状态。
    g_raw_command_mapping_error_count++;
    DroneCAN_EnterDShotWaitZeroState();
    return;
  }

  /* 映射和CCR构建全部成功，通知主循环发送最新的DShot1～DShot4。 */
  /*
   * 本阶段补充：发送已经改成TIM7固定1.5 ms，这里不再提出单次pending请求。
   * 回调只更新启动状态：WAIT_ZERO期间非零命令会被强制替换成停止帧；
   * 连续零命令满2 s进入RUNNING后，最新映射值才可由调度器输出。
   */
  DroneCAN_HandleMappedRawCommand(transfer->timestamp_usec);

  /*
   * 只有完整且解码成功的命令才会更新调试状态。received_count 最后增加，
   * 在 debugger 中看到计数变化时，前面的命令内容和来源 Node ID 已经写好。
   */
  g_last_raw_command = raw_command;
  g_last_raw_command_source_node_id = transfer->source_node_id;
  g_raw_command_received_count++;
}
