#include "dronecan_app.h"

#include "can.h"
#include "canard.h"

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

static CanardInstance g_canard;
static uint8_t g_canard_mem_pool[DRONECAN_APP_MEM_POOL_SIZE];

static void DroneCAN_OnTransferReceived(CanardInstance* ins,
                                        CanardRxTransfer* transfer);
static bool DroneCAN_ShouldAcceptTransfer(const CanardInstance* ins,
                                          uint64_t* out_data_type_signature,
                                          uint16_t data_type_id,
                                          CanardTransferType transfer_type,
                                          uint8_t source_node_id);
static HAL_StatusTypeDef DroneCAN_ConfigCanFilter(void);


/**
 * 
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
   * 先初始化 libcanard。
   *
   * 这里 libcanard 只是在 RAM 里建立协议状态，不会操作 CAN 硬件。
   * 真正启动 CAN 外设是在后面的 HAL_CAN_Start()。
   */
  //现在代码还没有调用 canardHandleRxFrame()，所以这个池子基本只是初始化好了，还没真正忙起来。
  canardInit(&g_canard,
             g_canard_mem_pool,
             sizeof(g_canard_mem_pool),
             DroneCAN_OnTransferReceived,
             DroneCAN_ShouldAcceptTransfer,
             NULL);

  //g_canard 是 libcanard 的实例，里面有 node_id 字段。这里设置本节点的 Node ID。
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

  //HAL_CAN_ActivateNotification()是打开中断 这个函数的第二个参数是一个位掩码，指定要打开哪些 CAN 中断。CAN_IT_RX_FIFO0_MSG_PENDING 是其中一个中断源，表示当 FIFO0 里至少有一帧 CAN 数据时触发。
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

void DroneCAN_App_Poll(void)
{
  /*
   * 当前第一步还没有接入周期任务。
   *
   * 下一步会添加一个小的 CAN RX 队列。到时候这个 Poll 函数会把队列里的
   * CAN frame 取出来，再交给 canardHandleRxFrame() 做 DroneCAN 协议解析。
   */
}

// HAL_CAN_RxFifo0MsgPendingCallback()是 HAL CAN driver 的一个回调函数。当 CAN1 的 RX FIFO0 里至少有一帧 CAN 数据时，HAL 会调用这个函数。这个函数的作用是把 FIFO0 里的所有帧都读出来，并暂时丢弃。
//为什么要读空？因为 RX pending 标志只有在实际读出 FIFO 后才会清掉。如果不读，CAN 中断会一直反复进入
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
    (void)rx_header;   //这是 C 语言里常见的“显式标记未使用”的写法，目前就是占位
    (void)rx_data;
  }
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

static bool DroneCAN_ShouldAcceptTransfer(const CanardInstance* ins,
                                          uint64_t* out_data_type_signature,
                                          uint16_t data_type_id,
                                          CanardTransferType transfer_type,
                                          uint8_t source_node_id)
{
  (void)ins;
  (void)out_data_type_signature;
  (void)data_type_id;
  (void)transfer_type;
  (void)source_node_id;

  /*
   * 当前还不接收任何 DroneCAN 数据类型。
   *
   * 下一步接好 CAN RX 队列之后，这里会开始接受
   * uavcan.equipment.esc.RawCommand，并返回它的 data type signature。
   */
  return false;
}

/*
也是给 libcanard 的回调。只有 ShouldAcceptTransfer() 返回 true，并且 libcanard 已经完整重组出一个 transfer 后，
才会进这里。当前为空；以后会在这里解码 RawCommand，然后转成电调/DShot 输出。
*/

static void DroneCAN_OnTransferReceived(CanardInstance* ins,
                                        CanardRxTransfer* transfer)
{
  (void)ins;
  (void)transfer;

  /*
   * 只有当 DroneCAN_ShouldAcceptTransfer() 返回 true，并且 libcanard 已经把
   * 一个完整 transfer 重组完成后，才会进入这个 callback。
   *
   * 当前还没有加入 RawCommand 解码，所以这里先保持为空。
   */
}
