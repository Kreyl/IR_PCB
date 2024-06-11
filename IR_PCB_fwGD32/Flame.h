/*
 * Flame.h
 *
 *  Created on: 12 нояб. 2022 г.
 *      Author: layst
 */

#ifndef FLAME_H__
#define FLAME_H__

#include <inttypes.h>
#include "color.h"
#include "Settings.h"
#include "gd_lib.h"
#include "yartos.h"

#define SMOOTH_VAR              450L
#define BRT_MAX                 255L // Do not touch
#define FRAME_PERIOD_ms         7
#define SPARK_DELAY_MIN         18

void OnOffTmrCallback(void *p);

class Flames_t {
private:
    enum PhaseState_t {stIdle, stFadingOut, stFadingIn, stStopping, stStopped} PhaseState;
    int32_t OnOffBrt = 0;
    VirtualTimer_t IOnOffTmr;
    void StartTimerI(uint32_t ms) { IOnOffTmr.SetI(TIME_MS2I(ms), OnOffTmrCallback, nullptr); }
    void StopTimer() { IOnOffTmr.Reset(); }
    FlameSettings_t INewSettings;
    volatile bool INewSettingsAppeared = true;
    ColorHSV_t ClrBattery {0, 0, 0};
    void ApplyNewSettings();
public:
    void Init();
    void FadeIn();
    void FadeOut();
    void Stop();
    void Start();
    void SetNewSettings(FlameSettings_t &ASettings);
    void ShowCharge(ColorHSV_t hsv) { ClrBattery = hsv; }
    void SetBandBrt(uint32_t brts[3]);
    void SetFlameLen(int32_t flen);
    // Inner use
    int32_t flame_len = FLAME_LEN_MAX;
    void IDraw();
    void OnOnOffTmrTickI();
    friend class Spark_t;
};

extern Flames_t Flames;

void PrintSparks();

#endif // FLAME_H__
