

#define CANARD_DSDLC_INTERNAL
#include <dronecan_dshot.DirectionCommand.h>

#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t _dronecan_dshot_DirectionCommand_encode(struct dronecan_dshot_DirectionCommand* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, DRONECAN_DSHOT_DIRECTIONCOMMAND_MAX_SIZE);
    __dronecan_dshot_DirectionCommand_encode(buffer, &bit_ofs, msg,
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

/*
  return true if the decode is invalid
 */
bool _dronecan_dshot_DirectionCommand_decode(const CanardRxTransfer* transfer, struct dronecan_dshot_DirectionCommand* msg) {
#if CANARD_ENABLE_TAO_OPTION
    if (transfer->tao && (transfer->payload_len > DRONECAN_DSHOT_DIRECTIONCOMMAND_MAX_SIZE)) {
        return true; /* invalid payload length */
    }
#endif
    uint32_t bit_ofs = 0;
    if (__dronecan_dshot_DirectionCommand_decode(transfer, &bit_ofs, msg,
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    )) {
        return true; /* invalid payload */
    }

    const uint32_t byte_len = (bit_ofs+7U)/8U;
#if CANARD_ENABLE_TAO_OPTION
    // if this could be CANFD then the dlc could indicating more bytes than
    // we actually have
    if (!transfer->tao) {
        return byte_len > transfer->payload_len;
    }
#endif
    return byte_len != transfer->payload_len;
}

#ifdef CANARD_DSDLC_TEST_BUILD
struct dronecan_dshot_DirectionCommand sample_dronecan_dshot_DirectionCommand_msg(void) {

    struct dronecan_dshot_DirectionCommand msg;






    msg.protocol_version = (uint8_t)random_bitlen_unsigned_val(8);







    msg.operation = (uint8_t)random_bitlen_unsigned_val(8);







    msg.motor_mask = (uint8_t)random_bitlen_unsigned_val(8);







    msg.request_id = (uint16_t)random_bitlen_unsigned_val(16);







    msg.confirmation = (uint16_t)random_bitlen_unsigned_val(16);





    return msg;

}
#endif
