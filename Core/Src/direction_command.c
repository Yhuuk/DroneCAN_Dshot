#include "direction_command.h"

#include "motor_control.h"

/*
 * DirectionCommand 通过校验后保存成一条待处理请求。
 * 后续状态机会在主循环中读取这份数据，并在整个换向过程中持续强制零油门。
 */
typedef struct
{
  struct dronecan_dshot_DirectionCommand command;
  uint8_t source_node_id;
  uint64_t timestamp_usec;
} DirectionCommandPendingRequest;

/*
 * DirectionCommand_Poll()每次只推进一个非阻塞步骤。状态名直接描述当前
 * 正在持续发送的内容，方便在调试器中观察完整执行过程。
 */
typedef enum
{
  DIRECTION_COMMAND_STATE_IDLE = 0,
  DIRECTION_COMMAND_STATE_HOLD_ZERO_BEFORE_COMMAND,
  DIRECTION_COMMAND_STATE_SEND_DIRECTION_COMMAND,
  DIRECTION_COMMAND_STATE_HOLD_ZERO_BETWEEN_COMMANDS,
  DIRECTION_COMMAND_STATE_SEND_SAVE_COMMAND,
  DIRECTION_COMMAND_STATE_HOLD_ZERO_AFTER_COMMAND
} DirectionCommandState;

/*
 * AM32只有在电调已停止并连续识别到相同特殊命令后才执行该命令。
 * 本工程的DShot发送周期是1.5 ms：30 ms大约提供20次发送机会，明显多于
 * AM32要求的6个连续相同帧，并为A/B双缓冲区切换保留了余量。
 */
#define DIRECTION_COMMAND_ZERO_BEFORE_USEC       1000000ULL
#define DIRECTION_COMMAND_SPECIAL_HOLD_USEC        30000ULL
#define DIRECTION_COMMAND_ZERO_BETWEEN_USEC        30000ULL
#define DIRECTION_COMMAND_ZERO_AFTER_USEC         500000ULL

static DirectionCommandPendingRequest g_pending_request;
static bool g_pending_request_available = false;
static volatile DirectionCommandState g_direction_command_state =
    DIRECTION_COMMAND_STATE_IDLE;
static uint64_t g_direction_command_state_started_usec = 0ULL;
static volatile uint16_t g_direction_command_active_dshot_command = 0U;

/*
 * request_id 用于识别电脑端对同一请求的重复发送。
 * ID 数值本身可以为 0，所以额外使用布尔标志区分“尚未接收过请求”。
 */
//弄清 g_has_last_accepted_request_id这个是一个布尔变量，表示是否已经接收过有效的请求。g_last_accepted_request_id是一个16位无符号整数，记录最后一次被接受的请求的request_id。
static bool g_has_last_accepted_request_id = false;
static uint16_t g_last_accepted_request_id = 0U;

/* 供调试器观察通过校验和被拒绝的 DirectionCommand 数量。 */
static volatile uint32_t g_direction_command_accepted_count = 0U;
static volatile uint32_t g_direction_command_rejected_count = 0U;
static volatile uint32_t g_direction_command_started_count = 0U;
static volatile uint32_t g_direction_command_completed_count = 0U;
static volatile uint32_t g_direction_command_execution_error_count = 0U;

static void DirectionCommand_EnterState(DirectionCommandState state,
                                        uint64_t now_usec)
{
  g_direction_command_state = state;
  g_direction_command_state_started_usec = now_usec;
}

static bool DirectionCommand_StateTimeReached(uint64_t now_usec,
                                              uint64_t duration_usec)
{
  return ((now_usec - g_direction_command_state_started_usec) >=
          duration_usec);
}

static void DirectionCommand_Abort(void)
{
  /*
   * 任何内部错误都立即恢复8路停止帧并结束当前请求。已经接受过的request_id
   * 仍然保留，电脑端需要使用新的request_id重新提交，避免旧报文自动重放。
   */
  MotorControl_ForceStop();
  g_direction_command_active_dshot_command = 0U;
  g_pending_request_available = false;
  g_direction_command_state = DIRECTION_COMMAND_STATE_IDLE;
  g_direction_command_execution_error_count++;
}

void DirectionCommand_Init(void)
{
  g_pending_request = (DirectionCommandPendingRequest){0};
  g_pending_request_available = false;
  g_has_last_accepted_request_id = false;
  g_last_accepted_request_id = 0U;
  g_direction_command_state = DIRECTION_COMMAND_STATE_IDLE;
  g_direction_command_state_started_usec = 0ULL;
  g_direction_command_active_dshot_command = 0U;
  g_direction_command_accepted_count = 0U;
  g_direction_command_rejected_count = 0U;
  g_direction_command_started_count = 0U;
  g_direction_command_completed_count = 0U;
  g_direction_command_execution_error_count = 0U;

  MotorControl_ForceStop();
}

bool DirectionCommand_Submit(
    const struct dronecan_dshot_DirectionCommand* command,
    uint8_t source_node_id,
    uint64_t timestamp_usec)
{
  if (command == NULL)
  {
    g_direction_command_rejected_count++;
    return false;
  }

  /*
   * 硬件过滤器和 ShouldAcceptTransfer 已经检查过来源，这里仍再次校验。
   * 这样以后即使调整 CAN 过滤规则，也不会意外放宽换向命令的来源限制。
   */
  if (source_node_id != DIRECTION_COMMAND_ALLOWED_SOURCE_NODE_ID)
  {
    g_direction_command_rejected_count++;
    return false;
  }

  if (command->protocol_version !=
      DRONECAN_DSHOT_DIRECTIONCOMMAND_PROTOCOL_VERSION)
  {
    g_direction_command_rejected_count++;
    return false;
  }

  if ((command->operation !=
       DRONECAN_DSHOT_DIRECTIONCOMMAND_OPERATION_SET_NORMAL) &&
      (command->operation !=
       DRONECAN_DSHOT_DIRECTIONCOMMAND_OPERATION_SET_REVERSED))
  {
    g_direction_command_rejected_count++;
    return false;
  }

  /* motor_mask 为 0 表示没有选中任何输出，这种请求没有实际意义。 */
  if (command->motor_mask == 0U)
  {
    g_direction_command_rejected_count++;
    return false;
  }

  /* 0xA55A 确认值用于降低电脑端误发消息导致电机方向被修改的风险。 */
  if (command->confirmation !=
      DRONECAN_DSHOT_DIRECTIONCOMMAND_CONFIRMATION_VALUE)
  {
    g_direction_command_rejected_count++;
    return false;
  }

  /*
   * 已有请求等待处理时不允许新请求覆盖它。覆盖会造成电脑端以为执行的是
   * 新请求，而状态机实际可能正在处理旧请求，因此这里选择明确拒绝。
   */
  if (g_pending_request_available)
  {
    g_direction_command_rejected_count++;
    return false;
  }

  if (g_has_last_accepted_request_id &&
      (command->request_id == g_last_accepted_request_id))
  {
    g_direction_command_rejected_count++;
    return false;
  }

  /*
   * 换向属于 DShot 特殊命令，绝不能在普通油门输出期间执行。
   * 这里先检查当前软件命令是否全部为 0；后续执行状态机还会再次检查，
   * 并在完整特殊命令序列期间持续保持 8 路零油门。
   */
  if (!MotorControl_AreAllDShotCommandsStopped())
  {
    g_direction_command_rejected_count++;
    return false;
  }

  /*
   * 先写入请求内容，最后再发布 available 标志。后续状态机看到 true 时，
   * command、来源节点和时间戳三部分已经全部准备完成。
   */
  g_pending_request.command = *command;
  g_pending_request.source_node_id = source_node_id;
  g_pending_request.timestamp_usec = timestamp_usec;
  g_last_accepted_request_id = command->request_id;
  g_has_last_accepted_request_id = true;
  g_pending_request_available = true;
  g_direction_command_accepted_count++;

  return true;
}

bool DirectionCommand_IsBusy(void)
{
  /*
   * Submit()先发布pending标志，Poll()稍后才离开IDLE。把两者都纳入判断，
   * 可以从请求刚被接受的那一刻起阻止RawCommand覆盖换向序列。
   */
  return (g_pending_request_available ||
          (g_direction_command_state != DIRECTION_COMMAND_STATE_IDLE));
}

void DirectionCommand_Poll(uint64_t now_usec)
{
  uint16_t direction_dshot_command;

  switch (g_direction_command_state)
  {
    case DIRECTION_COMMAND_STATE_IDLE:
      if (!g_pending_request_available)
      {
        return;
      }

      /*
       * 请求已经在Submit()中通过来源、确认值、操作类型和停止状态校验。
       * 这里正式取得请求执行权，并从1 s连续零帧开始换向流程。
       */
      MotorControl_ForceStop();
      g_direction_command_active_dshot_command = 0U;
      g_direction_command_started_count++;
      DirectionCommand_EnterState(
          DIRECTION_COMMAND_STATE_HOLD_ZERO_BEFORE_COMMAND,
          now_usec);
      return;

    case DIRECTION_COMMAND_STATE_HOLD_ZERO_BEFORE_COMMAND:
      /*
       * 持续维持停止缓存，而不是只在进入状态时写一次。这样即使上层逻辑
       * 被扩展，也不会在等待电调进入安全状态期间意外恢复旧油门。
       */
      MotorControl_ForceStop();

      //这个状态需要维持1秒钟，确保电调已经完全停止。只有在达到这个时间后，才会发送换向命令。
      if (!DirectionCommand_StateTimeReached(
              now_usec,
              DIRECTION_COMMAND_ZERO_BEFORE_USEC))
      {
        return;
      }

      //判断换向指令是设置为正向还是反向，并根据操作类型选择相应的DShot特殊命令编号。
      direction_dshot_command =
          (g_pending_request.command.operation ==
           DRONECAN_DSHOT_DIRECTIONCOMMAND_OPERATION_SET_NORMAL)
              ? DSHOT_COMMAND_SPIN_DIRECTION_NORMAL
              : DSHOT_COMMAND_SPIN_DIRECTION_REVERSED;

      if (!MotorControl_SetDShotSpecialCommand(
              g_pending_request.command.motor_mask,
              direction_dshot_command))
      {
        DirectionCommand_Abort();
        return;
      }

      g_direction_command_active_dshot_command = direction_dshot_command;
      DirectionCommand_EnterState(
          DIRECTION_COMMAND_STATE_SEND_DIRECTION_COMMAND,
          now_usec);
      return;

    case DIRECTION_COMMAND_STATE_SEND_DIRECTION_COMMAND:
      /*
       * 此状态不需要每1.5 ms重新编码。motor_control中的命令缓存保持不变，
       * TIM7调度器会自动重复发送相同方向命令，直到30 ms保持时间结束。
       */
      if (!DirectionCommand_StateTimeReached(
              now_usec,
              DIRECTION_COMMAND_SPECIAL_HOLD_USEC))
      {
        return;
      }

      MotorControl_ForceStop();
      g_direction_command_active_dshot_command = 0U;
      DirectionCommand_EnterState(
          DIRECTION_COMMAND_STATE_HOLD_ZERO_BETWEEN_COMMANDS,
          now_usec);
      return;

    case DIRECTION_COMMAND_STATE_HOLD_ZERO_BETWEEN_COMMANDS:
      MotorControl_ForceStop();
      if (!DirectionCommand_StateTimeReached(
              now_usec,
              DIRECTION_COMMAND_ZERO_BETWEEN_USEC))
      {
        return;
      }

      /*
       * 方向设置只改变AM32的RAM配置；DShot命令12把该配置保存到电调，
       * 使重新上电后仍保持新方向。只对motor_mask选中的电调发送保存命令。
       */
      if (!MotorControl_SetDShotSpecialCommand(
              g_pending_request.command.motor_mask,
              DSHOT_COMMAND_SAVE_SETTINGS))
      {
        DirectionCommand_Abort();
        return;
      }

      g_direction_command_active_dshot_command = DSHOT_COMMAND_SAVE_SETTINGS;
      DirectionCommand_EnterState(
          DIRECTION_COMMAND_STATE_SEND_SAVE_COMMAND,
          now_usec);
      return;

    case DIRECTION_COMMAND_STATE_SEND_SAVE_COMMAND:
      if (!DirectionCommand_StateTimeReached(
              now_usec,
              DIRECTION_COMMAND_SPECIAL_HOLD_USEC))
      {
        return;
      }

      MotorControl_ForceStop();
      g_direction_command_active_dshot_command = 0U;
      DirectionCommand_EnterState(
          DIRECTION_COMMAND_STATE_HOLD_ZERO_AFTER_COMMAND,
          now_usec);
      return;

    case DIRECTION_COMMAND_STATE_HOLD_ZERO_AFTER_COMMAND:
      /* 给电调保存参数和重新回到接收状态留出时间，全程继续发送停止帧。 */
      MotorControl_ForceStop();
      if (!DirectionCommand_StateTimeReached(
              now_usec,
              DIRECTION_COMMAND_ZERO_AFTER_USEC))
      {
        return;
      }

      g_pending_request_available = false;
      g_direction_command_completed_count++;
      DirectionCommand_EnterState(DIRECTION_COMMAND_STATE_IDLE, now_usec);
      return;

    default:
      DirectionCommand_Abort();
      return;
  }
}
