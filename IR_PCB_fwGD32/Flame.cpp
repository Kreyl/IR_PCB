/*
 * Flame.cpp
 *
 *  Created on: 12 нояб. 2022 г.
 *      Author: layst
 */

#include "Flame.h"
#include "MsgQ.h"
#include "ws2812bTim.h"
#include <vector>

Flames_t Flames;
extern Neopixels_t NpxLeds;
static FlameSettings_t ISettings;
uint32_t band_brts[3] = {255, 255, 255};

// Set pix pair, mixing it with current color
inline void SetPixRing(int32_t band_indx, int32_t x, Color_t clr) {
//    Printf("%d\r", x);
    if (x >= Flames.flame_len or x < 0) return; // too far
    int32_t indx;
    if  (band_indx == 0) indx = x;
    elif(band_indx == 1) indx = (Flames.flame_len * 2 - 1) - x;
    else indx = (Flames.flame_len * 2) + x;
    // Apply brightness
    clr.SetRGBBrightness(band_brts[band_indx], 255UL);

    NpxLeds.ClrBuf[indx].MixAddingRGBW(clr);

//    NpxLeds.ClrBuf[x].MixAddingRGBW(Clr); // One band
//    NpxLeds.ClrBuf[(FLAME_LEN * 2 - 1) - x] = NpxLeds.ClrBuf[x]; // Second band
//    NpxLeds.ClrBuf[(FLAME_LEN * 2) + x] = NpxLeds.ClrBuf[x]; // Third band
}

#if 1 // ============================ Spark ====================================
void SparkTmrCallback(void *p);

// Proportion for gradients calculation
uint16_t x2Value(uint16_t x, uint16_t fMin, uint16_t fMax) { return Proportion<uint16_t>(0, (Flames.flame_len-1), fMin, fMax, x); }

// Tail Brightness for short tails
#define BRT_TAIL1_1     128
#define BRT_TAIL2_1     85
#define BRT_TAIL2_2     170
#define BRT_TAIL_END    2L

class Spark_t {
private:
    int16_t MoveDelay_ms = 81;
    int16_t Acceleration = 27;
    int32_t x = -1, xMax = 0;
    VirtualTimer_t ITmrMove;
    systime_t SystimeToStart = 540;
    std::vector<Color_t> IBuf;
public:
    int32_t band_indx = 0;
    void Init() { // Recreate IBuf making it's colors black
        IBuf.clear();
        if(ISettings.Sparks.Mode == SPARKS_MODE_RANDOM)
            IBuf.resize(ISettings.Sparks.TailLen + SPARK_CENTER_SZ + ISettings.Sparks.TailLen);
    }

    void Sleep() {
        x = -1; // Sleeping
        SystimeToStart = Sys::GetSysTimeX() + TIME_MS2I(Random::Generate(0, ISettings.Sparks.DelayBeforeRestart));
    }

    void Generate() {
        if(ISettings.Sparks.Mode == SPARKS_MODE_RANDOM) { // Prepare buf which is spark's image
            xMax = (Flames.flame_len - 1) + (int32_t)IBuf.size();
            int32_t N = 0, TailLen = ISettings.Sparks.TailLen, MaxV = ISettings.Sparks.ClrV, BrtRGB, BrtW;
            ColorHSV_t hsv{ISettings.Sparks.GetRandomHue(), 100, (uint8_t)MaxV};
            Color_t rgbw = hsv.ToRGB();
            int32_t MaxW = ISettings.Sparks.GetRandomW();
            rgbw.W = MaxW;
            MaxV = (MaxV * 255L) / 100L; // [0;100] -> [0;255]

            // Head
            for(int32_t i=0; i<TailLen; i++) {
                if(TailLen == 1) {
                    BrtRGB = BRT_TAIL1_1;
                    BrtW = BrtRGB;
                }
                else if(TailLen == 2) {
                    BrtRGB = (i == 0)? BRT_TAIL2_1 : BRT_TAIL2_2;
                    BrtW = BrtRGB;
                }
                else {
                    BrtRGB = Proportion<int32_t>(0, TailLen, BRT_TAIL_END, MaxV, i); // [0;TailLen) -> [1; MaxV)
                    BrtW   = Proportion<int32_t>(0, TailLen, BRT_TAIL_END, MaxW, i); // [0;TailLen) -> [1; MaxW)
                }
                IBuf[N] = rgbw;
                IBuf[N].SetRGBWBrightness(BrtRGB, BrtW, BRT_MAX);
                IBuf[N].ApplyGammaCorrectionRGBW();
                N++;
            }

            // Center
            for(int32_t i=0; i<SPARK_CENTER_SZ; i++) {
                IBuf[N] = rgbw;
                IBuf[N].ApplyGammaCorrectionRGBW();
                N++;
            }

            // Tail
            for(int32_t i=1; i<=TailLen; i++) {
                if(TailLen == 1) {
                    BrtRGB = BRT_TAIL1_1;
                    BrtW = BrtRGB;
                }
                else if(TailLen == 2) {
                    BrtRGB = (i == 1)? BRT_TAIL2_2 : BRT_TAIL2_1;
                    BrtW = BrtRGB;
                }
                else {
                    BrtRGB = Proportion<int32_t>(0, TailLen, MaxV, BRT_TAIL_END, i); // [0;TailLen) -> [MaxV; 1)
                    BrtW   = Proportion<int32_t>(0, TailLen, MaxW, BRT_TAIL_END, i); // [0;TailLen) -> [MaxW; 1)
                }
                IBuf[N] = rgbw;
                IBuf[N].SetRGBWBrightness(BrtRGB, BrtW, BRT_MAX);
                IBuf[N].ApplyGammaCorrectionRGBW();
                N++;
            }
        }
        else { // Gradient
            xMax = Flames.flame_len + ISettings.Sparks.TailLen;
        }
        // === Init coord, speed, acceleration ===
        x = 0;
        MoveDelay_ms = Random::Generate(ISettings.Sparks.StartDelayMin, ISettings.Sparks.StartDelayMax);
        if(MoveDelay_ms < SPARK_DELAY_MIN) MoveDelay_ms = SPARK_DELAY_MIN;
        Acceleration = Random::Generate(ISettings.Sparks.AccMin, ISettings.Sparks.AccMax);
        ITmrMove.Set(TIME_MS2I(MoveDelay_ms), SparkTmrCallback, this);
    }

    void Process() {
        if(x == -1) {
            if(Sys::GetSysTimeX() >= SystimeToStart) Generate();
            else return;
        }
        // Draw it
        if(ISettings.Sparks.Mode == SPARKS_MODE_RANDOM) {
            int32_t Len = IBuf.size();
            int32_t Start = (x < Flames.flame_len)? 0 : x - (Flames.flame_len -1);
            int32_t Stop  = (x < Len)? x : Len - 1;
            int32_t xCurr = (x < Flames.flame_len)? x : (Flames.flame_len - 1);
            for(int32_t i=Start; i<=Stop; i++) SetPixRing(band_indx, xCurr--, IBuf[i]);
        }
        else { // Gradient
            int32_t SparkLen = ISettings.Sparks.TailLen + 1;
            for(int32_t i=0; i<SparkLen; i++) {
                int32_t fx = x - i;

                if(fx < 0) break;
                if(fx >= Flames.flame_len) continue;
                ColorHSV_t hsv{x2Value(fx, ISettings.Sparks.ClrHMin, ISettings.Sparks.ClrHMax), 100, ISettings.Sparks.ClrV};
                Color_t rgbw = hsv.ToRGB();
                rgbw.W = x2Value(fx, ISettings.Sparks.ClrWMin, ISettings.Sparks.ClrWMax);
                SetPixRing(band_indx, fx, rgbw);
            }
        }
    }

    void StopI() {
        ITmrMove.ResetI(); // IsArmed checked inside
    }

    void OnTmrI() {
        x++;
        if(x < xMax) {
            MoveDelay_ms -= Acceleration;
            if(MoveDelay_ms <= SPARK_DELAY_MIN) MoveDelay_ms = SPARK_DELAY_MIN;
            ITmrMove.SetI(TIME_MS2I(MoveDelay_ms), SparkTmrCallback, this);
        }
        else Sleep();
    }

    void Print() {
        Printf("%d %d %d %d; t=%u\r", MoveDelay_ms, Acceleration, x, xMax, SystimeToStart);
    }
};

void SparkTmrCallback(void *p) {
    Sys::LockFromIRQ();
    ((Spark_t*)p)->OnTmrI();
    Sys::UnlockFromIRQ();
}

static std::vector<Spark_t> Sparks;
#endif

// ==== Thread ====
static THD_WORKSPACE(waFlameThread, 512);
__attribute__((noreturn)) static void FlameThread() {
    Flames.IDraw();
}

void Flames_t::Init() {
    Sparks.clear();
    // Create and start thread
    Sys::CreateThd(waFlameThread, sizeof(waFlameThread), NORMALPRIO, FlameThread);
}

__attribute__((noreturn))
void Flames_t::IDraw() {
    while(true) {
        Sys::SleepMilliseconds(FRAME_PERIOD_ms);
        NpxLeds.SetAll((Color_t){0,0,0,0}); // Clear buffer before proceeding

        // Disable everything
        if(MustStop) {
            if(PhaseState == stStopping) {
                NpxLeds.SetCurrentColors();
                Sys::Lock();
                for(Spark_t &Spark : Sparks) Spark.StopI();
                Sys::Unlock();
                PhaseState = stIdle;
            }
            else continue; // Just sleep forever
        }

        // Show charge if needed
        if(ClrBattery.V != 0) {
            for(uint32_t i=0; i<9; i++) NpxLeds.ClrBuf[i] = ClrBattery.ToRGB();
            NpxLeds.SetCurrentColors();
            Sys::SleepMilliseconds(1530);
            for(uint32_t i=0; i<9; i++) NpxLeds.ClrBuf[i] = (Color_t){0,0,0,0};
            ClrBattery.V = 0; // Do not show next time
        }

        if(INewSettingsAppeared) ApplyNewSettings();

        // ==== Draw core ====
        for(uint32_t x=0; x<ISettings.Core.Sz; x++) {
            Color_t Clr;
            Clr.FromHSV(x2Value(x, ISettings.Core.ClrHMin, ISettings.Core.ClrHMax), 100, ISettings.Core.ClrV);
            Clr.W = ISettings.Core.ClrW;
            for(uint32_t i=0; i<NPX_BAND_CNT; i++) SetPixRing(i, x, Clr);
        }
        // ==== Sparks Layer ====
        for(Spark_t &Spark : Sparks) Spark.Process();

        // ==== On-Off Layer ====
        for(Color_t &Clr : NpxLeds.ClrBuf) Clr.SetRGBWBrightness(OnOffBrt, BRT_MAX);

        // ==== Draw it ====
        NpxLeds.SetCurrentColors();
    } // while true
}

#if 1 // ============================ On-Off Layer =============================
void OnOffTmrCallback(void *p) {
    Sys::LockFromIRQ();
    Flames.OnOnOffTmrTickI();
    Sys::UnlockFromIRQ();
}

void Flames_t::FadeIn() {
    PhaseState = stFadingIn;
    Sys::Lock();
    StartTimerI(ClrCalcDelay(OnOffBrt, SMOOTH_VAR));
    Sys::Unlock();
}

void Flames_t::FadeOut() {
    PhaseState = stFadingOut;
    Sys::Lock();
    StartTimerI(ClrCalcDelay(OnOffBrt, SMOOTH_VAR));
    Sys::Unlock();
}

void Flames_t::StopNow() {
    StopTimer();
    PhaseState = stStopping;
    OnOffBrt = 0;
    MustStop = true;
}

void Flames_t::OnOnOffTmrTickI() {
    switch(PhaseState) {
        case stFadingIn:
            if(OnOffBrt == BRT_MAX) PhaseState = stIdle;
            else {
                OnOffBrt++;
                StartTimerI(ClrCalcDelay(OnOffBrt, SMOOTH_VAR));
            }
            break;

        case stFadingOut:
            if(OnOffBrt == 0) {
                PhaseState = stIdle;
                EvtQMain.SendNowOrExitI(EvtMsg_t(EvtId::LedsDone));
            }
            else {
                OnOffBrt--;
                StartTimerI(ClrCalcDelay(OnOffBrt, SMOOTH_VAR));
            }
            break;

        default: break;
    }
}
#endif

void Flames_t::SetNewSettings(FlameSettings_t &ASettings) {
    ASettings.Print();
    Sys::Lock();
    INewSettings = ASettings;
    INewSettingsAppeared = true;
    Sys::Unlock();
}

void Flames_t::ApplyNewSettings() {
    // Stop sparks
    Sys::Lock();
    INewSettingsAppeared = false;
    for(Spark_t &Spark : Sparks) Spark.StopI();
    ISettings = INewSettings;
    Sys::Unlock();
    // Prepare sparks buf
    Sparks.resize(ISettings.Sparks.Cnt * NPX_BAND_CNT);
    // Prepare Sparks
    uint32_t N=0;
    for(int32_t i=0; i<NPX_BAND_CNT; i++) {
        for(int32_t j=0; j<ISettings.Sparks.Cnt; j++) {
            Sparks[N].Init();
            Sparks[N].band_indx = i;
            Sparks[N].Generate();
            N++;
        }
    }
}

void Flames_t::SetBandBrt(uint32_t brts[3]) {
    band_brts[0] = brts[0];
    band_brts[1] = brts[1];
    band_brts[2] = brts[2];
}

void Flames_t::SetFlameLen(int32_t flen) {
    flame_len = flen;
    INewSettingsAppeared = true; // To restart sparks
}

void PrintSparks() {
    for(auto &spark: Sparks) spark.Print();
}
