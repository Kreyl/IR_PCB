/*
 * SnsPins.h
 *
 *  Created on: 17 ���. 2015 �.
 *      Author: Kreyl
 */

/* ================ Documentation =================
 * There are several (may be 1) groups of sensors (say, buttons and USB connection).
 *
 */

#ifndef PIN_SNS_SETTINGS__
#define PIN_SNS_SETTINGS__

#include "SimpleSensors.h"

#ifndef SIMPLESENSORS_ENABLED
#define SIMPLESENSORS_ENABLED   FALSE
#endif

#if SIMPLESENSORS_ENABLED
#define SNS_POLL_PERIOD_MS      72

// Handlers
extern void ProcessInput(PinSnsState_t *PState, uint32_t Len);

const PinSns_t PinSns[] = {
        {INPUT1, pudPullDown, ProcessInput},
        {INPUT2, pudPullDown, ProcessInput},
        {INPUT3, pudPullDown, ProcessInput},
};
#define PIN_SNS_CNT     countof(PinSns)

#endif  // if enabled

#endif // PIN_SNS_SETTINGS__
