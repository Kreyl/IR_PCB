/*
 * app_classes.cpp
 *
 *  Created on: 2 сент. 2024 г.
 *      Author: layst
 */

#include "app_classes.h"

#if 1 // ============================= Pulser pin ==============================
void PulserCallback(void *p) { static_cast<PulserPin*>(p)->IOnTmrDone(); }

void PulserPin::PulseI(uint32_t Dur) {
    itmr.ResetI();
    SetHi();
    if(Dur == 0) SetLo();
    else itmr.SetI(TIME_MS2I(Dur), PulserCallback, (void*)this);
}
void PulserPin::ResetI() {
    itmr.ResetI();
    SetLo();
}
#endif

#if 1 // =========================== PWM input =================================
void PwmInputTimCallback(void *p) {
    static_cast<PwmInputPin*>(p)->TimerCallback();
}
#endif
