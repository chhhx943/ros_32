#include "bsp_bxcan_loopback.h"

BSP_BXCAN_LoopbackResult_t g_bxcan_loopback_result;

static const uint8_t g_bxcan_loopback_tx_data[8] = {
    0xA5U, 0x5AU, 0x10U, 0x01U, 0x23U, 0x45U, 0x67U, 0x89U,
};

static void BXCAN_LoopbackResetResult(void)
{
    uint32_t i;

    g_bxcan_loopback_result.magic = BSP_BXCAN_LOOPBACK_RESULT_MAGIC;
    g_bxcan_loopback_result.passed = 0U;
    g_bxcan_loopback_result.status = BSP_BXCAN_LOOPBACK_ERR_TIMEOUT;
    g_bxcan_loopback_result.hal_error = 0U;
    g_bxcan_loopback_result.tx_mailbox = 0xFFFFFFFFU;
    g_bxcan_loopback_result.rx_std_id = 0U;
    g_bxcan_loopback_result.rx_dlc = 0U;
    g_bxcan_loopback_result.rx_ide = 0U;
    g_bxcan_loopback_result.rx_rtr = 0U;
    g_bxcan_loopback_result.ticks_elapsed = 0U;

    for (i = 0U; i < 8U; ++i) {
        g_bxcan_loopback_result.tx_data[i] = g_bxcan_loopback_tx_data[i];
        g_bxcan_loopback_result.rx_data[i] = 0U;
    }
}

static BSP_BXCAN_LoopbackStatus_t BXCAN_LoopbackFinish(CAN_HandleTypeDef *hcan,
                                                       BSP_BXCAN_LoopbackStatus_t status,
                                                       uint32_t start_tick)
{
    g_bxcan_loopback_result.status = (uint32_t)status;
    g_bxcan_loopback_result.passed = (status == BSP_BXCAN_LOOPBACK_PASS) ? 1U : 0U;
    g_bxcan_loopback_result.hal_error = (hcan != 0) ? HAL_CAN_GetError(hcan) : 0U;
    g_bxcan_loopback_result.ticks_elapsed = HAL_GetTick() - start_tick;
    return status;
}

static uint8_t BXCAN_LoopbackRxMatches(const CAN_RxHeaderTypeDef *rx_header, const uint8_t rx_data[8])
{
    uint32_t i;

    if ((rx_header->StdId != BSP_BXCAN_LOOPBACK_TEST_STD_ID) ||
        (rx_header->IDE != CAN_ID_STD) ||
        (rx_header->RTR != CAN_RTR_DATA) ||
        (rx_header->DLC != 8U)) {
        return 0U;
    }

    for (i = 0U; i < 8U; ++i) {
        if (rx_data[i] != g_bxcan_loopback_tx_data[i]) {
            return 0U;
        }
    }

    return 1U;
}

BSP_BXCAN_LoopbackStatus_t BSP_BXCAN_RunLoopbackSelfTest(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef filter = {0};
    CAN_TxHeaderTypeDef tx_header = {0};
    CAN_RxHeaderTypeDef rx_header = {0};
    uint8_t rx_data[8] = {0};
    uint32_t mailbox = 0U;
    uint32_t start_tick;
    uint32_t i;

    BXCAN_LoopbackResetResult();
    start_tick = HAL_GetTick();

    if (hcan == 0) {
        return BXCAN_LoopbackFinish(hcan, BSP_BXCAN_LOOPBACK_ERR_NULL_HANDLE, start_tick);
    }

    if (HAL_CAN_DeInit(hcan) != HAL_OK) {
        return BXCAN_LoopbackFinish(hcan, BSP_BXCAN_LOOPBACK_ERR_DEINIT, start_tick);
    }

    hcan->Init.Mode = CAN_MODE_LOOPBACK;
    if (HAL_CAN_Init(hcan) != HAL_OK) {
        return BXCAN_LoopbackFinish(hcan, BSP_BXCAN_LOOPBACK_ERR_INIT, start_tick);
    }

    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;
    filter.FilterActivation = CAN_FILTER_ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK) {
        return BXCAN_LoopbackFinish(hcan, BSP_BXCAN_LOOPBACK_ERR_FILTER, start_tick);
    }

    if (HAL_CAN_Start(hcan) != HAL_OK) {
        return BXCAN_LoopbackFinish(hcan, BSP_BXCAN_LOOPBACK_ERR_START, start_tick);
    }

    tx_header.StdId = BSP_BXCAN_LOOPBACK_TEST_STD_ID;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8U;
    tx_header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(hcan, &tx_header, (uint8_t *)g_bxcan_loopback_tx_data, &mailbox) != HAL_OK) {
        return BXCAN_LoopbackFinish(hcan, BSP_BXCAN_LOOPBACK_ERR_TX, start_tick);
    }
    g_bxcan_loopback_result.tx_mailbox = mailbox;

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) == 0U) {
        if ((uint32_t)(HAL_GetTick() - start_tick) >= BSP_BXCAN_LOOPBACK_TIMEOUT_MS) {
            return BXCAN_LoopbackFinish(hcan, BSP_BXCAN_LOOPBACK_ERR_TIMEOUT, start_tick);
        }
    }

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
        return BXCAN_LoopbackFinish(hcan, BSP_BXCAN_LOOPBACK_ERR_RX, start_tick);
    }

    g_bxcan_loopback_result.rx_std_id = rx_header.StdId;
    g_bxcan_loopback_result.rx_dlc = rx_header.DLC;
    g_bxcan_loopback_result.rx_ide = rx_header.IDE;
    g_bxcan_loopback_result.rx_rtr = rx_header.RTR;
    for (i = 0U; i < 8U; ++i) {
        g_bxcan_loopback_result.rx_data[i] = rx_data[i];
    }

    if (BXCAN_LoopbackRxMatches(&rx_header, rx_data) == 0U) {
        return BXCAN_LoopbackFinish(hcan, BSP_BXCAN_LOOPBACK_ERR_MISMATCH, start_tick);
    }

    return BXCAN_LoopbackFinish(hcan, BSP_BXCAN_LOOPBACK_PASS, start_tick);
}
