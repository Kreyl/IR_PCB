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
namespace kIr {
inline const uint32_t Header_us = 2400UL;
inline const uint32_t Space_us = 600UL;
inline const uint32_t Zero_us = 600UL;
inline const uint32_t One_us = 1200UL;
inline const uint32_t PauseAfter_us = 2400UL;
} // namespace

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
#define IR_RX_PKT_TIMEOUT_MS        4

namespace irRcvr {
    void Init(ftVoidU32 CallbackI);
    void SetCallback(ftVoidU32 CallbackI);
} // namespace
#endif
