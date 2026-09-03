
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>




#define DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_MAX_SIZE 15
#define DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_SIGNATURE (0x5C392730EE1BD1E4ULL)

#define DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_ID 200



#define DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_PROTOCOL_VERSION 1

#define DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_ACCEPTED 1

#define DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_IN_PROGRESS 2

#define DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_COMPLETE 3

#define DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_BUSY 4

#define DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_NOT_SAFE 5

#define DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_INVALID_REQUEST 6

#define DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_NOT_FOUND 7

#define DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_INTERNAL_ERROR 8

#define DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_UNSUPPORTED_VERSION 9

#define DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_MAINTENANCE_ERROR_ENTER_FAILED 1

#define DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_MAINTENANCE_ERROR_BOOTLOADER_EXIT_FAILED 2

#define DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_MAINTENANCE_ERROR_DSHOT_RESTORE_FAILED 4





#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class dronecan_dshot_DirectionQuery_cxx_iface;
#endif


struct dronecan_dshot_DirectionQueryResponse {

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = dronecan_dshot_DirectionQuery_cxx_iface;
#endif




    uint8_t protocol_version;



    uint8_t status;



    uint16_t request_id;



    uint8_t active_source_node_id;



    uint16_t active_request_id;



    uint8_t query_motor_mask;



    uint8_t valid_mask;



    uint8_t reversed_mask;



    uint8_t timeout_mask;



    uint8_t crc_error_mask;



    uint8_t unsupported_mask;



    uint8_t protocol_error_mask;



    uint8_t maintenance_error;



};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t _dronecan_dshot_DirectionQueryResponse_encode(struct dronecan_dshot_DirectionQueryResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool _dronecan_dshot_DirectionQueryResponse_decode(const CanardRxTransfer* transfer, struct dronecan_dshot_DirectionQueryResponse* msg);

static inline uint32_t dronecan_dshot_DirectionQueryResponse_encode(struct dronecan_dshot_DirectionQueryResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {

    return _dronecan_dshot_DirectionQueryResponse_encode(msg, buffer
#if CANARD_ENABLE_TAO_OPTION
    , tao
#endif
    );

}

static inline bool dronecan_dshot_DirectionQueryResponse_decode(const CanardRxTransfer* transfer, struct dronecan_dshot_DirectionQueryResponse* msg) {

    return _dronecan_dshot_DirectionQueryResponse_decode(transfer, msg);

}

#if defined(CANARD_DSDLC_INTERNAL)

static inline void __dronecan_dshot_DirectionQueryResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct dronecan_dshot_DirectionQueryResponse* msg, bool tao);
static inline bool __dronecan_dshot_DirectionQueryResponse_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct dronecan_dshot_DirectionQueryResponse* msg, bool tao);
void __dronecan_dshot_DirectionQueryResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct dronecan_dshot_DirectionQueryResponse* msg, bool tao) {

    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;






    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->protocol_version);

    *bit_ofs += 8;






    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->status);

    *bit_ofs += 8;






    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->request_id);

    *bit_ofs += 16;






    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->active_source_node_id);

    *bit_ofs += 8;






    canardEncodeScalar(buffer, *bit_ofs, 16, &msg->active_request_id);

    *bit_ofs += 16;






    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->query_motor_mask);

    *bit_ofs += 8;






    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->valid_mask);

    *bit_ofs += 8;






    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->reversed_mask);

    *bit_ofs += 8;






    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->timeout_mask);

    *bit_ofs += 8;






    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->crc_error_mask);

    *bit_ofs += 8;






    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->unsupported_mask);

    *bit_ofs += 8;






    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->protocol_error_mask);

    *bit_ofs += 8;






    canardEncodeScalar(buffer, *bit_ofs, 8, &msg->maintenance_error);

    *bit_ofs += 8;





}

/*
 decode dronecan_dshot_DirectionQueryResponse, return true on failure, false on success
*/
bool __dronecan_dshot_DirectionQueryResponse_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct dronecan_dshot_DirectionQueryResponse* msg, bool tao) {

    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;





    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->protocol_version);

    *bit_ofs += 8;







    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->status);

    *bit_ofs += 8;







    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->request_id);

    *bit_ofs += 16;







    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->active_source_node_id);

    *bit_ofs += 8;







    canardDecodeScalar(transfer, *bit_ofs, 16, false, &msg->active_request_id);

    *bit_ofs += 16;







    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->query_motor_mask);

    *bit_ofs += 8;







    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->valid_mask);

    *bit_ofs += 8;







    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->reversed_mask);

    *bit_ofs += 8;







    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->timeout_mask);

    *bit_ofs += 8;







    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->crc_error_mask);

    *bit_ofs += 8;







    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->unsupported_mask);

    *bit_ofs += 8;







    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->protocol_error_mask);

    *bit_ofs += 8;







    canardDecodeScalar(transfer, *bit_ofs, 8, false, &msg->maintenance_error);

    *bit_ofs += 8;





    return false; /* success */

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct dronecan_dshot_DirectionQueryResponse sample_dronecan_dshot_DirectionQueryResponse_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>



#endif
#endif
