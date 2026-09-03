
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>




#define DRONECAN_DSHOT_DIRECTIONQUERY_REQUEST_MAX_SIZE 7
#define DRONECAN_DSHOT_DIRECTIONQUERY_REQUEST_SIGNATURE (0x5C392730EE1BD1E4ULL)

#define DRONECAN_DSHOT_DIRECTIONQUERY_REQUEST_ID 200



#define DRONECAN_DSHOT_DIRECTIONQUERY_REQUEST_PROTOCOL_VERSION 1

#define DRONECAN_DSHOT_DIRECTIONQUERY_REQUEST_OPERATION_START_QUERY 1

#define DRONECAN_DSHOT_DIRECTIONQUERY_REQUEST_OPERATION_GET_RESULT 2

#define DRONECAN_DSHOT_DIRECTIONQUERY_REQUEST_CONFIRMATION_VALUE 21930





#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class dronecan_dshot_DirectionQuery_cxx_iface;
#endif


struct dronecan_dshot_DirectionQueryRequest {

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = dronecan_dshot_DirectionQuery_cxx_iface;
#endif




    uint8_t protocol_version;



    uint8_t operation;



    uint8_t motor_mask;



    uint16_t request_id;



    uint16_t confirmation;



};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t _dronecan_dshot_DirectionQueryRequest_encode(struct dronecan_dshot_DirectionQueryRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool _dronecan_dshot_DirectionQueryRequest_decode(const CanardRxTransfer* transfer, struct dronecan_dshot_DirectionQueryRequest* msg);

static inline uint32_t dronecan_dshot_DirectionQueryRequest_encode(struct dronecan_dshot_DirectionQueryRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {

    return _dronecan_dshot_DirectionQueryRequest_encode(msg, buffer
#if CANARD_ENABLE_TAO_OPTION
    , tao
#endif
    );

}

static inline bool dronecan_dshot_DirectionQueryRequest_decode(const CanardRxTransfer* transfer, struct dronecan_dshot_DirectionQueryRequest* msg) {

    return _dronecan_dshot_DirectionQueryRequest_decode(transfer, msg);

}

#if defined(CANARD_DSDLC_INTERNAL)

static inline void __dronecan_dshot_DirectionQueryRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct dronecan_dshot_DirectionQueryRequest* msg, bool tao);
static inline bool __dronecan_dshot_DirectionQueryRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct dronecan_dshot_DirectionQueryRequest* msg, bool tao);
void __dronecan_dshot_DirectionQueryRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct dronecan_dshot_DirectionQueryRequest* msg, bool tao) {

    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;






    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->protocol_version);

    *bit_ofs += 8;






    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->operation);

    *bit_ofs += 8;






    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->motor_mask);

    *bit_ofs += 8;






    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->request_id);

    *bit_ofs += 16;






    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->confirmation);

    *bit_ofs += 16;





}

/*
 decode dronecan_dshot_DirectionQueryRequest, return true on failure, false on success
*/
bool __dronecan_dshot_DirectionQueryRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct dronecan_dshot_DirectionQueryRequest* msg, bool tao) {

    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;





    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->protocol_version);

    *bit_ofs += 8;







    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->operation);

    *bit_ofs += 8;







    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->motor_mask);

    *bit_ofs += 8;







    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->request_id);

    *bit_ofs += 16;







    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->confirmation);

    *bit_ofs += 16;





    return false; /* success */

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct dronecan_dshot_DirectionQueryRequest sample_dronecan_dshot_DirectionQueryRequest_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>



#endif
#endif
