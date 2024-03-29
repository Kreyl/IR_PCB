/*
 * ir.h
 *
 *  Created on: 04.07.2013
 *      Author: kreyl
 */

#pragma once

#include "gd_lib.h"
#include "board.h"

#define IR_TX_ENABLED   TRUE
#define IR_RX_ENABLED   TRUE

// Delays, uS
#define IR_HEADER_uS        2400UL
#define IR_SPACE_uS         600UL
#define IR_ZERO_uS          600UL
#define IR_ONE_uS           1200UL
#define IR_PAUSE_AFTER_uS   2400UL

#if IR_TX_ENABLED // ========================== IR TX ==========================
#define IR_MAX_PWR          255     // Top DAC value

namespace irLed {
    void Init();
    void SetCarrierFreq(uint32_t CarrierFreqHz);
    void TransmitWord(uint16_t wData, int32_t BitCnt, uint8_t Power, ftVoidVoid CallbackI);
    void ResetI();
} // namespace
#endif

#if IR_RX_ENABLED // ========================== IR RX ==========================
#define IR_RX_POLLING_PERIOD_MS     90
#define IR_DEVIATION_uS             150
#define IR_RX_PKT_TIMEOUT_MS        4

namespace irRcvr {
    void Init(ftVoidU32 CallbackI);
    void SetCallback(ftVoidU32 CallbackI);
} // namespace
#endif
