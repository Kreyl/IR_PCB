/*
 * ir.h
 *
 *  Created on: 04.07.2013
 *      Author: kreyl
 */

#pragma once

#include "hal.h"
#include "ch.h"
#include "kl_lib.h"
#include "board.h"
#include "uart.h"
#include "ir_pkt.h"

#define IR_TX_ENABLED   TRUE
#define IR_RX_ENABLED   FALSE

#if IR_TX_ENABLED // ========================== IR TX ==========================
#define IR_CARRIER_HZ       56000UL
#define IR_MAX_PWR          255     // Top DAC value

// Delays, uS
#define IR_HEADER_uS        2400UL
#define IR_SPACE_uS         600UL
#define IR_ZERO_uS          600UL
#define IR_ONE_uS           1200UL
#define IR_PAUSE_AFTER_uS   2400UL

namespace irLed {
    void Init();
    void TransmitWord(uint16_t wData, uint8_t Power, int32_t NRepeat, ftVoidVoid CallbackI);
    void ResetI();
} // namespace
#endif

#if IR_RX_ENABLED // ========================== IR RX ==========================
#define IR_RX_POLLING_PERIOD_MS     90
#define IR_DEVIATION_US             150
#define IR_RX_PKT_TIMEOUT_MS

#define IR_RX_DMA_MODE  STM32_DMA_CR_CHSEL(IR_RX_TIM_DMA_CHNL) | \
                        DMA_PRIORITY_MEDIUM | \
                        STM32_DMA_CR_MSIZE_HWORD | \
                        STM32_DMA_CR_PSIZE_HWORD | \
                        STM32_DMA_CR_MINC |       /* Memory pointer increase */ \
                        STM32_DMA_CR_DIR_P2M |    /* Direction is peripheral to memory */ \
                        STM32_DMA_CR_CIRC         /* Circular buffer enable */

#define IR_RX_BUF_LEN   36

enum IrPktPartType_t {iptHeader, iptZero, iptOne, iptError};

class irReceiver_t {
private:
    Timer_t TmrRx{TMR_IR_RX};
    int32_t SzOld, RIndx;
    uint16_t IRxBuf[IR_RX_BUF_LEN];
    IrPktPartType_t MeasureDuration(uint16_t Duration) {
        if     (IS_LIKE(Duration, IR_HEADER_US, IR_DEVIATION_US)) return iptHeader;
        else if(IS_LIKE(Duration, IR_ZERO_US,   IR_DEVIATION_US)) return iptZero;
        else if(IS_LIKE(Duration, IR_ONE_US,    IR_DEVIATION_US)) return iptOne;
        else return iptError;
    }
    // Parsing
    int IBitCnt = -1; // Header not received
    systime_t IPktStartTime;
    irPkt_t ICurrentPkt;
public:
    void Init();
    irPkt_t LastPkt;
    void ITask();
};

extern irReceiver_t irRx;
#endif
