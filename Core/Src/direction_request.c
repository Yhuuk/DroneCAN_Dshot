#include "direction_request.h"

#include "am32_direction_query.h"
#include "direction_command.h"
#include "dronecan_dshot.DirectionQuery.h"

#include <string.h>

/**
 * 定义事务结构
 * 
 */
typedef struct
{
    bool valid;
    uint8_t source_node_id;
    uint16_t request_id;
    uint8_t motor_mask;
  
} DirectionRequestTransaction;

/**
 * 定义模块状态
 */
/* 当前进行中的事务 */
static DirectionRequestTransaction g_active_transaction;

/* 已完成的事务 */
static DirectionRequestTransaction g_completed_transaction;

/*最近一次完成的AM32 查询结果 */
static AM32DirectionQueryResult g_completed_result;

/* 防止没有有效结果时 误读 g_completed_result*/
static bool g_completed_result_valid;

/*
 * 供调试器观察最近一次响应进入libcanard TX队列的结果：
 * 正数是成功入队的CAN帧数，负数是libcanard错误码。
 */
static volatile int16_t g_last_response_enqueue_result;
static volatile uint32_t g_response_enqueue_success_count;
static volatile uint32_t g_response_enqueue_error_count;
static volatile uint32_t g_response_enqueued_frame_count;


/**
 * 内部辅助函数声明
 */
static bool DirectionRequest_TransactionMatches(
    const DirectionRequestTransaction* transaction,
    uint8_t source_node_id,
    uint16_t request_id);

static void DirectionRequest_FillTransactionIdentity(
    struct dronecan_dshot_DirectionQueryResponse* response,
    const DirectionRequestTransaction* transaction);

static void DirectionRequest_FillCompletedResult(
    struct dronecan_dshot_DirectionQueryResponse* response);

static void DirectionRequest_HandleStart(
  const struct dronecan_dshot_DirectionQueryRequest* request,
  uint8_t source_node_id,
  uint64_t timestamp_usec,
  struct dronecan_dshot_DirectionQueryResponse* response);

static void DirectionRequest_HandleGetResult(
  const struct dronecan_dshot_DirectionQueryRequest* request,
  uint8_t source_node_id,
  struct dronecan_dshot_DirectionQueryResponse* response);

static int16_t DirectionRequest_SendResponse(
  CanardInstance* ins,
  CanardRxTransfer* transfer,
  struct dronecan_dshot_DirectionQueryResponse* response);




void DirectionRequest_Init(void)
{
    g_active_transaction = (DirectionRequestTransaction){0};

    g_completed_transaction = (DirectionRequestTransaction){0};
    g_completed_result = (AM32DirectionQueryResult){0};
    g_completed_result_valid = false;
    g_last_response_enqueue_result = 0;
    g_response_enqueue_success_count = 0U;
    g_response_enqueue_error_count = 0U;
    g_response_enqueued_frame_count = 0U;

}

void DirectionRequest_Handle(CanardInstance* ins,
                                CanardRxTransfer* transfer)
{
  /**
   * 该函数进来的时候就已经做过判断了
   */
  // if (transfer->transfer_type != CanardTransferTypeRequest ||
  //     transfer->data_type_id != DRONECAN_DSHOT_DIRECTIONQUERY_ID)
  // {
  //   return;
  // }

  struct dronecan_dshot_DirectionQueryRequest request = {0};
  struct dronecan_dshot_DirectionQueryResponse response = {0};
  int16_t enqueue_result;


  if(ins == NULL || transfer == NULL)
  {
    return;
  }


  /**
   * 解码请求，失败返回1，成功返回0
   */
  if (dronecan_dshot_DirectionQueryRequest_decode(transfer, &request))
  {
    return;
  }

  /**
   * 填充响应协议版本
   */
  response.protocol_version = DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_PROTOCOL_VERSION;

  /**
   * 填充响应请求ID
   */
  response.request_id = request.request_id;

  if(g_active_transaction.valid)
  {

      DirectionRequest_FillTransactionIdentity(&response, &g_active_transaction);
  }
  else if(g_completed_transaction.valid)
  {
      DirectionRequest_FillTransactionIdentity(&response, &g_completed_transaction);
  }
  
  if(request.protocol_version != DRONECAN_DSHOT_DIRECTIONQUERY_REQUEST_PROTOCOL_VERSION)
  {
      response.status = DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_UNSUPPORTED_VERSION;
  }
  else
  {
    switch (request.operation)
    {
      case DRONECAN_DSHOT_DIRECTIONQUERY_REQUEST_OPERATION_START_QUERY:
        DirectionRequest_HandleStart(&request, transfer->source_node_id, transfer->timestamp_usec, &response);

        break;

      case DRONECAN_DSHOT_DIRECTIONQUERY_REQUEST_OPERATION_GET_RESULT:
        DirectionRequest_HandleGetResult(&request, transfer->source_node_id, &response);

        break;

      default:

        response.status = DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_INVALID_REQUEST;
        break;
      
    }
  }

  enqueue_result = DirectionRequest_SendResponse(ins, transfer, &response);
  g_last_response_enqueue_result = enqueue_result;

  if (enqueue_result < 0)
  {
    /* 响应没有成功进入libcanard队列，记录错误供调试器诊断。 */
    g_response_enqueue_error_count++;
  }
  else
  {
    g_response_enqueue_success_count++;
    g_response_enqueued_frame_count += (uint32_t)enqueue_result;
  }
 
}

/**
 * 判断事务是否匹配
 */
static bool DirectionRequest_TransactionMatches(
    const DirectionRequestTransaction* transaction,
    uint8_t source_node_id,
    uint16_t request_id)
{
  return transaction->valid &&
          transaction->source_node_id == source_node_id &&
          transaction->request_id == request_id;

}

/**
 * 填写事务身份字段
 */
static void DirectionRequest_FillTransactionIdentity(
    struct dronecan_dshot_DirectionQueryResponse* response,
    const DirectionRequestTransaction* transaction)
{
    if ((response == NULL) ||
        (transaction == NULL) ||
        (!transaction->valid))
    {
        return;
    }

    response->active_source_node_id =
        transaction->source_node_id;

    response->active_request_id =
        transaction->request_id;

    response->query_motor_mask =
        transaction->motor_mask;
}

/**
 * 填写完成结果
 * 
 */
static void DirectionRequest_FillCompletedResult(
    struct dronecan_dshot_DirectionQueryResponse* response)
{
    response->valid_mask = g_completed_result.valid_mask;

    response->reversed_mask = g_completed_result.reversed_mask;

    response->timeout_mask = g_completed_result.timeout_mask;

    response->crc_error_mask = g_completed_result.crc_error_mask;
    
    response->unsupported_mask = g_completed_result.unsupported_mask;

    response->protocol_error_mask = g_completed_result.protocol_error_mask;

    response->maintenance_error = g_completed_result.maintenance_error;
}

static void DirectionRequest_HandleStart(
  const struct dronecan_dshot_DirectionQueryRequest* request,
  uint8_t source_node_id,
  uint64_t timestamp_usec,
  struct dronecan_dshot_DirectionQueryResponse* response)
{
  if ((request == NULL) || (response == NULL))
  {
    return;
  }

  /*
   * START_QUERY 必须至少选择一路电机，并携带约定的确认值。
   * 这两个条件不满足时不允许改变现有事务或启动 AM32 查询。
   */
  if ((request->motor_mask == 0U) ||
      (request->confirmation !=
       DRONECAN_DSHOT_DIRECTIONQUERY_REQUEST_CONFIRMATION_VALUE))
  {
    response->status =
        DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_INVALID_REQUEST;
    return;
  }

  /*
   * 相同 (source_node_id, request_id) 的活动事务属于重复请求：
   * motor_mask 相同时只报告仍在执行，不能重新启动查询；不同则说明调用者
   * 复用了事务 ID，却改变了事务内容，必须拒绝。
   */
  if (DirectionRequest_TransactionMatches(&g_active_transaction,
                                           source_node_id,
                                           request->request_id))
  {
    DirectionRequest_FillTransactionIdentity(response,
                                               &g_active_transaction);

    response->status =
        (g_active_transaction.motor_mask == request->motor_mask)
            ? DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_IN_PROGRESS
            : DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_INVALID_REQUEST;
    return;
  }

  /*
   * 相同事务如果已经完成，直接返回缓存结果，实现 START_QUERY 的幂等性。
   * 绝不能因为控制板重发请求而再次让电调进入 Bootloader。
   */
  if (DirectionRequest_TransactionMatches(&g_completed_transaction,
                                           source_node_id,
                                           request->request_id))
  {
    DirectionRequest_FillTransactionIdentity(response,
                                               &g_completed_transaction);

    if (g_completed_transaction.motor_mask != request->motor_mask)
    {
      response->status =
          DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_INVALID_REQUEST;
      return;
    }

    if (!g_completed_result_valid)
    {
      response->status =
          DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_INTERNAL_ERROR;
      return;
    }

    DirectionRequest_FillCompletedResult(response);
    response->status =
        (response->maintenance_error == 0U)
            ? DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_COMPLETE
            : DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_INTERNAL_ERROR;
    return;
  }

  /*
   * 存在其它活动事务时，新事务只能得到 BUSY。即使服务层状态与底层状态
   * 暂时不同步，只要 AM32DirectionQuery 仍忙，也不能启动第二次查询。
   */
  if (g_active_transaction.valid || AM32DirectionQuery_IsBusy())
  {
    response->status =
        DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_BUSY;

    if (g_active_transaction.valid)
    {
      DirectionRequest_FillTransactionIdentity(response,
                                                 &g_active_transaction);
    }
    return;
  }

  /*
   * 方向修改状态机和AM32单线查询会共同占用同一组DShot输出资源。
   * DirectionCommand_IsBusy() 同时覆盖“请求已暂存但尚未进入状态机”和
   * “状态机正在执行”两种情况。此时启动查询可能暂停DShot并切换GPIO模式，
   * 从而破坏正在发送的方向特殊命令，因此把新查询判定为当前不安全。
   */
  if (DirectionCommand_IsBusy())
  {
    response->status =
        DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_NOT_SAFE;
    return;
  }


  /*
   * AM32DirectionQuery_Start() 会再次检查 motor_mask、维护模式以及八路电机
   * 是否全部停止。失败表示当前不满足安全启动条件。
   */
  if (!AM32DirectionQuery_Start(request->motor_mask, timestamp_usec))
  {
    response->status =
        DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_NOT_SAFE;
    return;
  }

  /*
   * 只有底层查询确实启动成功后才发布活动事务，避免响应宣称 ACCEPTED，
   * 实际却没有对应的查询任务。
   */
  g_active_transaction.valid = true;
  g_active_transaction.source_node_id = source_node_id;
  g_active_transaction.request_id = request->request_id;
  g_active_transaction.motor_mask = request->motor_mask;

  DirectionRequest_FillTransactionIdentity(response,
                                             &g_active_transaction);
  response->status =
      DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_ACCEPTED;


}

static void DirectionRequest_HandleGetResult(
  const struct dronecan_dshot_DirectionQueryRequest* request,
  uint8_t source_node_id,
  struct dronecan_dshot_DirectionQueryResponse* response)
{
  if ((request == NULL) || (response == NULL))
  {
    return;
  }

  /* 查询仍在运行：只返回事务身份和 IN_PROGRESS，不把未完成结果当成最终结果。 */
  if (DirectionRequest_TransactionMatches(&g_active_transaction,
                                           source_node_id,
                                           request->request_id))
  {
    DirectionRequest_FillTransactionIdentity(response,
                                               &g_active_transaction);
    response->status =
        DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_IN_PROGRESS;
    return;
  }

  /* 找到最近完成的同一事务时，返回已经缓存的最终方向和错误位图。 */
  if (DirectionRequest_TransactionMatches(&g_completed_transaction,
                                           source_node_id,
                                           request->request_id))
  {
    DirectionRequest_FillTransactionIdentity(response,
                                               &g_completed_transaction);

    if (!g_completed_result_valid)
    {
      response->status =
          DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_INTERNAL_ERROR;
      return;
    }

    DirectionRequest_FillCompletedResult(response);
    response->status =
        (response->maintenance_error == 0U)
            ? DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_COMPLETE
            : DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_INTERNAL_ERROR;
    return;
  }

  /* GET_RESULT 的 motor_mask 和 confirmation 按 DSDL 约定忽略。 */
  response->status =
      DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_NOT_FOUND;

}

/**
 * 把响应发送回去
 */
static int16_t DirectionRequest_SendResponse(
  CanardInstance* ins,
  CanardRxTransfer* transfer,
  struct dronecan_dshot_DirectionQueryResponse* response)
{
  uint8_t payload[DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_MAX_SIZE] = {0};
  uint32_t payload_length;

  if ((ins == NULL) || (transfer == NULL) || (response == NULL))
  {
    return -CANARD_ERROR_INVALID_ARGUMENT;
  }

  /* 将响应结构体序列化成 DSDL 定义的 15 字节 Payload。 */
  /**
   * dronecan_dshot_DirectionQueryResponse_encode 返回的是序列化后的字节数
   */
  payload_length =
      dronecan_dshot_DirectionQueryResponse_encode(response, payload);

  if (payload_length > sizeof(payload))
  {
    return -CANARD_ERROR_INTERNAL;
  }

  /*
   * 服务响应必须发回请求节点，并沿用请求的 Transport Transfer-ID 和优先级。
   * 该函数只把响应拆帧后放入 libcanard TX 队列；真正写入 bxCAN 邮箱仍由
   * DroneCAN 发送轮询函数负责。
   * 
   * CanardResponse 表示的是服务响应类型，CanardRequest 表示的是服务请求类型。
   */
  return canardRequestOrRespond(
      ins,
      transfer->source_node_id,
      DRONECAN_DSHOT_DIRECTIONQUERY_SIGNATURE,
      DRONECAN_DSHOT_DIRECTIONQUERY_ID,
      &transfer->transfer_id,
      transfer->priority,
      CanardResponse,
      payload,
      (uint16_t)payload_length);
}

void DirectionRequest_Poll(void)
{
  AM32DirectionQueryResult result;

  /*
   * 没有服务事务正在等待时无需同步结果；底层仍忙时结果尚未最终确定，
   * 此时也必须继续保留g_active_transaction，让GET_RESULT返回IN_PROGRESS。
   */
  if ((!g_active_transaction.valid) || AM32DirectionQuery_IsBusy())
  {
    return;
  }

  /*
   * 运行到这里表示：服务层记录着活动事务，但AM32底层查询已经结束。
   * 先复制事务身份，使后续相同(source_node_id, request_id)的GET_RESULT
   * 可以找到它，也使重复START_QUERY能够直接返回缓存结果而不重新查询。
   */
  g_completed_transaction = g_active_transaction;

  /*
   * 底层正常结束时会发布最近一次查询结果。复制成功后，该结果就和上面
   * 的completed事务绑定，后续响应可安全读取valid/reversed/error等位图。
   */
  if (AM32DirectionQuery_GetLastResult(&result))
  {
    g_completed_result = result;
    g_completed_result_valid = true;
  }
  else
  {
    /*
     * 理论上底层从Busy变为Idle时应当已经有结果。若没有，清除旧结果并
     * 保持valid=false，避免把上一次查询的数据误当成本事务的结果；
     * GET_RESULT随后会返回STATUS_INTERNAL_ERROR。
     */
    memset(&g_completed_result, 0, sizeof(g_completed_result));
    g_completed_result_valid = false;
  }

  /*
   * 当前事务已经转移到completed槽位，清零整个结构体会把valid置为false，
   * 同时清掉旧的来源节点、request_id和motor_mask，防止残留字段被误用。
   * 
   * memset 是将g_active_transaction的成员字段全部置为0
   */
  memset(&g_active_transaction, 0, sizeof(g_active_transaction));
}
