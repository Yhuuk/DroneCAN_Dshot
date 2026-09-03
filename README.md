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

2. `maintenance_error` 维护模式错误位掩码

| 位 | 数值 | 含义 |
|---:|---:|---|
| bit0 | `0x01` | 进入维护模式失败 |
| bit1 | `0x02` | 一个或多个ESC退出Bootloader失败 |
| bit2 | `0x04` | DShot GPIO/TIM/DMA恢复失败 |

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
