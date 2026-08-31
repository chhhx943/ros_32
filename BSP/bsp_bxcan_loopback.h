#ifndef __BSP_BXCAN_LOOPBACK_H
#define __BSP_BXCAN_LOOPBACK_H

#include "can.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_BXCAN_LOOPBACK_RESULT_MAGIC 0xBCA11B00UL
#define BSP_BXCAN_LOOPBACK_TEST_STD_ID  0x321U
#define BSP_BXCAN_LOOPBACK_TIMEOUT_MS   100U

typedef enum {
    BSP_BXCAN_LOOPBACK_PASS = 0,
    BSP_BXCAN_LOOPBACK_ERR_NULL_HANDLE = 1,
    BSP_BXCAN_LOOPBACK_ERR_DEINIT = 2,
    BSP_BXCAN_LOOPBACK_ERR_INIT = 3,
    BSP_BXCAN_LOOPBACK_ERR_FILTER = 4,
    BSP_BXCAN_LOOPBACK_ERR_START = 5,
    BSP_BXCAN_LOOPBACK_ERR_TX = 6,
    BSP_BXCAN_LOOPBACK_ERR_TIMEOUT = 7,
    BSP_BXCAN_LOOPBACK_ERR_RX = 8,
    BSP_BXCAN_LOOPBACK_ERR_MISMATCH = 9
} BSP_BXCAN_LoopbackStatus_t;

typedef struct {
    uint32_t magic;
    uint32_t passed;
    uint32_t status;
    uint32_t hal_error;
    uint32_t tx_mailbox;
    uint32_t rx_std_id;
    uint32_t rx_dlc;
    uint32_t rx_ide;
    uint32_t rx_rtr;
    uint32_t ticks_elapsed;
    uint8_t tx_data[8];
    uint8_t rx_data[8];
} BSP_BXCAN_LoopbackResult_t;

extern BSP_BXCAN_LoopbackResult_t g_bxcan_loopback_result;

BSP_BXCAN_LoopbackStatus_t BSP_BXCAN_RunLoopbackSelfTest(CAN_HandleTypeDef *hcan);

#ifdef __cplusplus
}
#endif

#endif
