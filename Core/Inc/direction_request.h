#ifndef _DIRECTION_REQUEST_H
#define _DIRECTION_REQUEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include "canard.h"

/* 当前只允许DroneCAN_Control节点126发起方向查询。 */
#define DIRECTION_REQUEST_ALLOWED_SOURCE_NODE_ID 126U


void DirectionRequest_Init(void);

void DirectionRequest_Handle(CanardInstance* ins,
                                CanardRxTransfer* transfer);

/**
 * @brief 在主循环中把已结束的AM32底层查询同步为已完成的DroneCAN事务。
 *
 * 应在AM32DirectionQuery_Poll()之后周期调用。查询未结束时本函数不改变
 * 活动事务；查询结束后缓存结果，使GET_RESULT能够返回最终状态和位图。
 */
void DirectionRequest_Poll(void);


#ifdef __cplusplus
}
#endif

#endif // _DIRECTION_REQUEST_H
