# 说明

## `dronecan_dshot.DirectionQuery_res` 响应字段

1. 
| 数值 | 状态 | 含义 |
|---:|---|---|
| 1 | `STATUS_ACCEPTED` | 新查询已接受并启动 |
| 2 | `STATUS_IN_PROGRESS` | 查询仍在进行 |
| 3 | `STATUS_COMPLETE` | 查询正常结束 |
| 4 | `STATUS_BUSY` | 另一个查询或维护操作正在占用 |
| 5 | `STATUS_NOT_SAFE` | 电机没有安全停止，不能查询 |
| 6 | `STATUS_INVALID_REQUEST` | 请求字段或幂等规则无效 |
| 7 | `STATUS_NOT_FOUND` | 没找到对应request_id |
| 8 | `STATUS_INTERNAL_ERROR` | 维护模式或内部恢复失败 |
| 9 | `STATUS_UNSUPPORTED_VERSION` | DirectionQuery协议版本不支持 |



## `DroneCAN` 的 第`8`个字节 `Transport Tail Byte`
1. DroneCAN（UAVCAN v0）每个物理 CAN 帧的最后一个字节都是 Transport Tail Byt
```text
bit7      bit6      bit5      bit4...bit0
SOT       EOT       Toggle    Transfer-ID
```
- SOT：是否是本次传输的第一帧
- EOT：是否是本次传输的最后一帧
- Toggle：多帧传输时交替变化，用于检测漏帧、重复帧或顺序错误
- Transfer-ID：5 bit，范围 0～31，然后回绕到 0

## `AM32` 的参数
```text
单线OK       -> Bootloader OK
单线超时     -> Bootloader TIMEOUT
参数错误     -> Bootloader INVALID_ARGUMENT
其他物理错误 -> Bootloader TRANSPORT_ERROR
```

## 单线 半双工 串口
```text
波特率：约19200
格式：8N1
空闲状态: 高电平
起始位: 低电平
数据顺序：LSB first
停止位: 高电平
一位时间：52 us
发送和接收：使用同一根信号线，半双工
```

这是我当前的 软件串口 配置
```text
1个TIM6计数 = 1 us
52个计数 ≈ 52 us/bit
实际波特率 = 1 / 52us ≈ 19230.8
```
误差约0.16%，可以满足19200波特率

## 异步请求/查询模式
```text
DroneCAN_Control                         DroneCAN_Dshot
       │                                      │
       │ START_QUERY                          │
       │ mask=0x11, request_id=100            │
       ├─────────────────────────────────────>│
       │                                      │ 验证请求
       │                                      │ 强制停止电机
       │                                      │ 启动AM32查询状态机
       │ STATUS_ACCEPTED                      │
       │<─────────────────────────────────────┤
       │                                      │
       │                                      │ 保持零DShot 500ms
       │                                      │
       │ GET_RESULT, request_id=100           │
       ├─────────────────────────────────────>│
       │ STATUS_IN_PROGRESS                   │
       │<─────────────────────────────────────┤
       │                                      │
       │                                      │ 信号线拉高3s
       │                                      │ 读取DShot1 EEPROM
       │                                      │ 读取DShot5 EEPROM
       │                                      │ 让8路ESC退出Bootloader
       │                                      │ 恢复DShot输出
       │                                      │ 缓存最终结果
       │                                      │
       │ GET_RESULT, request_id=100           │
       ├─────────────────────────────────────>│
       │ STATUS_COMPLETE + masks              │
       │<─────────────────────────────────────┤
```

### 请求Payload：7字节
| 字节偏移 | 长度 | 字段 | 含义 |
|---:|---:|---|---|
| 0 | 1 | `protocol_version` | DirectionQuery应用层协议版本，当前必须为1 |
| 1 | 1 | `operation` | 指定本次请求是启动查询还是读取结果 |
| 2 | 1 | `motor_mask` | 需要查询的电机位掩码 |
| 3～4 | 2 | `request_id` | 控制板分配的应用层事务ID |
| 5～6 | 2 | `confirmation` | START安全确认值，必须为`0x55AA` |

1. `protocol_version` 该值为1，表示版本信息，不是1时，`DroneCAN_Dshot`返回`STATUS_UNSUPPORTED_VERSION`
2. `operation`

| 值 | 常量 | 含义 |
|---:|---|---|
| 1 | `OPERATION_START_QUERY` | 启动一个新的异步AM32方向查询 |
| 2 | `OPERATION_GET_RESULT` | 查询已有事务当前状态或最终结果 |
| 其他 | — | 返回`STATUS_INVALID_REQUEST` |

3. `motor_mask` 这是8路电机掩码，需要查询谁，就把哪一位置1。
- 对于 START_QUERY：不能为0，表示需要读取哪些电调
- 对于 GET_RESULT： 发送方应该填0，DroneCAN_Dshot会忽略它，真正的查询范围来自原始START事务保存的mask。
4. `request_id` 事务完整身份是 `source_node_id, request_id`,就是`OPERATION_START_QUERY`和`OPERATION_GET_RESULT` 都使用相同的 `request_id`
5. `confirmation` 确认值，相当于一个保护，避免误触发。该值必须为`0x55AA`

### 响应Payload：15字节
| 字节偏移 | 长度 | 字段 | 含义 |
|---:|---:|---|---|
| 0 | 1 | `protocol_version` | DroneCAN_Dshot支持的协议版本，当前为1 |
| 1 | 1 | `status` | 本次请求处理结果或查询状态 |
| 2～3 | 2 | `request_id` | 原样返回本次服务请求中的request_id |
| 4 | 1 | `active_source_node_id` | 当前活动事务或最近缓存事务的来源节点ID |
| 5～6 | 2 | `active_request_id` | 当前活动事务或最近缓存事务的request_id |
| 7 | 1 | `query_motor_mask` | 该事务原始START指定的电机mask |
| 8 | 1 | `valid_mask` | 已取得有效方向的电机 |
| 9 | 1 | `reversed_mask` | 有效结果中配置为Reversed的电机 |
| 10 | 1 | `timeout_mask` | 最终查询结果为超时的电机 |
| 11 | 1 | `crc_error_mask` | 最终响应CRC错误的电机 |
| 12 | 1 | `unsupported_mask` | Bootloader协议不支持所需读取操作的电机 |
| 13 | 1 | `protocol_error_mask` | 其他Bootloader、ACK、方向值或帧格式错误 |
| 14 | 1 | `maintenance_error` | 整体维护流程错误位掩码 |

1. `status` 各状态值的含义

| 值 | 状态 | 准确含义 |
|---:|---|---|
| 1 | `STATUS_ACCEPTED` | 新查询已成功启动，但尚未完成 |
| 2 | `STATUS_IN_PROGRESS` | 指定事务存在，并且仍在执行 |
| 3 | `STATUS_COMPLETE` | 查询流程正常结束，但个别电机仍可能查询失败（低概率） |
| 4 | `STATUS_BUSY` | 另一个事务正在占用AM32/DShot维护资源 |
| 5 | `STATUS_NOT_SAFE` | 电机未安全停止、方向修改正忙或其他安全条件不满足 |
| 6 | `STATUS_INVALID_REQUEST` | operation、mask、confirmation或幂等规则不合法 |
| 7 | `STATUS_NOT_FOUND` | 没有找到指定的活动事务或缓存事务 |
| 8 | `STATUS_INTERNAL_ERROR` | 查询遇到内部错误或维护模式错误 |
| 9 | `STATUS_UNSUPPORTED_VERSION` | DroneCAN_Dshot不支持请求中的协议版本 |

2. `maintenance_error` 维护模式错误位掩码

    `maintenance_error`为0表示没有错误

| 位 | 数值 | 含义 |
|---:|---:|---|
| bit0 | `0x01` | 进入维护模式失败 |
| bit1 | `0x02` | 一个或多个ESC退出Bootloader失败 |
| bit2 | `0x04` | DShot GPIO/TIM/DMA恢复失败 |

## 请求响应查询电机状态中调用的函数以及功能说明
1. CAN接收和DroneCAN解析

| 顺序 | 函数 | 作用 |
|---:|---|---|
| 1 | `HAL_CAN_RxFifo0MsgPendingCallback()` | CAN FIFO0接收中断入口，从硬件FIFO读取CAN帧 |
| 2 | `DroneCAN_RxQueuePushFromIsr()` | 在中断中把原始CAN帧放入软件环形队列 |
| 3 | `DroneCAN_App_Poll()` | 主循环入口，处理接收、发送和各状态机 |
| 4 | `DroneCAN_RxQueuePop()` | 从软件队列取出一帧 |
| 5 | `DroneCAN_BuildCanardFrame()` | 将HAL CAN结构转换成`CanardCANFrame` |
| 6 | `canardHandleRxFrame()` | libcanard解析CAN ID、Tail Byte并重组完整Transfer |
| 7 | `DroneCAN_ShouldAcceptTransfer()` | 判断是否接收DirectionQuery并提供DSDL签名 |
| 8 | `DroneCAN_OnTransferReceived()` | 完整请求重组完成后的应用层回调 |
| 9 | `DirectionRequest_Handle()` | 解码7字节请求并根据operation分发 |

2. START_QUERY处理

| 顺序 | 函数 | 作用 |
|---:|---|---|
| 1 | `DirectionRequest_HandleStart()` | 校验mask、confirmation、事务幂等、BUSY和安全状态 |
| 2 | `DirectionRequest_TransactionMatches()` | 比较`source_node_id + request_id`是否属于同一事务 |
| 3 | `DirectionCommand_IsBusy()` | 防止方向修改和AM32查询同时占用DShot资源 |
| 4 | `AM32DirectionQuery_IsBusy()` | 防止同时启动第二个AM32查询 |
| 5 | `AM32DirectionQuery_Start()` | 清空结果、强制停止电机并进入HOLD_ZERO状态 |
| 6 | `DirectionRequest_FillTransactionIdentity()` | 把活动事务ID和motor mask填入响应 |
| 7 | `DirectionRequest_SendResponse()` | 编码并生成`STATUS_ACCEPTED`响应 |
| 8 | `canardRequestOrRespond()` | 将15字节响应加CRC、拆成3帧并加入libcanard TX队列 |
| 9 | `DroneCAN_FlushTXQueue()` | 把libcanard队列中的帧送入CAN硬件邮箱 |
| 10 | `HAL_CAN_AddTxMessage()` | 正式提交给bxCAN硬件发送 |

3. AM32方向查询状态机

| 状态/函数 | 作用 |
|---|---|
| `AM32DirectionQuery_Poll()` | 在主循环中推进整个查询状态机 |
| `HOLD_ZERO` | 强制发送零油门并保持500 ms |
| `DShotOutput_EnterMaintenanceMode()` | 暂停正常DShot，切换信号引脚为维护用途 |
| `WAIT_BOOTLOADER` | 将信号线保持高电平3 s，使AM32进入Bootloader |
| `AM32DirectionQuery_AdvanceToRequestedChannel()` | 根据motor mask跳过未请求通道 |
| `AM32Bootloader_ReadDirection()` | 连接指定ESC、读取EEPROM并解析方向 |
| `AM32DirectionQuery_RecordFinalFailure()` | 3次尝试全部失败后设置对应错误mask |
| `AM32Bootloader_RunApplication()` | 让ESC退出Bootloader并运行正常应用 |
| `DShotOutput_ExitMaintenanceMode()` | 恢复GPIO复用、TIM、DMA和DShot输出 |
| `AM32DirectionQuery_Finish()` | 发布结果、状态回到IDLE |

4. 单路EEPROM读取

`AM32Bootloader_ReadDirection()` 内部主要调用：

| 顺序 | 函数 | 作用 |
|---:|---|---|
| 1 | `AM32Bootloader_Connect()` | 发送BootInit并验证AM32 Bootloader身份和版本 |
| 2 | `AM32Bootloader_SetEepromAddress()` | 设置Flash模拟EEPROM读取地址 |
| 3 | `AM32Bootloader_ReadEeprom()` | 读取48字节AM32配置数据 |
| 4 | `AM32Bootloader_Crc16Arc()` | 生成或检查Bootloader协议CRC |
| 5 | 方向字段解析 | 读取EEPROM方向字段，得到Normal或Reversed |

这些`Bootloader` 命令最终都通过 `AM32SingleWire_Transfer()` 在`Dshot` 信号线上完成半双工单线收发

底层时序由一下函数和中断推进:

| 函数 | 作用 |
|---|---|
| `AM32SingleWire_ConfigureOutput()` | 把当前DShot引脚切换为单线发送输出 |
| `AM32SingleWire_ConfigureInput()` | 发送完成后切换为输入上拉 |
| `AM32SingleWire_StartTimer()` | 启动TIM6产生19200 baud位时序 |
| `AM32SingleWire_HandleTimerPeriodElapsed()` | TIM6中断中发送bit或采样接收bit |
| `AM32SingleWire_HandleExtiInterrupt()` | 通过下降沿检测AM32响应起始位 |
| `AM32SingleWire_Complete()` | 标记本次单线收发成功或失败 |
| `AM32SingleWire_Abort()` | 超时时停止定时器、中断并恢复引脚 |

5. 查询完成与GET_RESULT

| 顺序 | 函数 | 作用 |
|---:|---|---|
| 1 | `AM32DirectionQuery_Finish()` | 底层状态回到IDLE并发布结果 |
| 2 | `DirectionRequest_Poll()` | 把活动DroneCAN事务转存为已完成事务 |
| 3 | `AM32DirectionQuery_GetLastResult()` | 复制最终方向和错误结果 |
| 4 | 控制板发送`GET_RESULT` | 使用原来的request_id查询 |
| 5 | `DirectionRequest_HandleGetResult()` | 判断事务是IN_PROGRESS、COMPLETE还是NOT_FOUND |
| 6 | `DirectionRequest_FillCompletedResult()` | 填入方向和各错误mask |
| 7 | `DirectionRequest_SendResponse()` | 生成最终15字节响应 |
| 8 | `DroneCAN_FlushTXQueue()` | 将多帧响应送到CAN总线 |