/*
 * app.cpp
 *
 *  Created on: 14 авг. 2023 г.
 *      Author: layst
 */

#include "app.h"
#include "Settings.h"
#include "kl_lib.h"
#include "shell.h"
#include "ir.h"
#include "ir_pkt.h"
#include "MsgQ.h"
#include "led.h"
#include "beeper.h"
#include "Sequences.h"

int32_t HitCnt, RoundsCnt, MagazinesCnt;
extern Beeper_t Beeper;

#if 1 // ========================== Message queue ==============================
enum class AppEvt : uint16_t {
    Reset,
    StartFire, EndOfIrTx, EndOfDelayBetweenShots, MagazineReloadDone,
    IrRx,
};

union AppMsg_t {
    uint32_t DWord;
    struct {
        uint16_t Data16;
        AppEvt Evt;
    } __attribute__((__packed__));

    AppMsg_t& operator = (const AppMsg_t &Right) {
        DWord = Right.DWord;
        return *this;
    }
    AppMsg_t() : DWord(0) {}
    AppMsg_t(AppEvt AEvt) : Data16(0), Evt(AEvt) {}
    AppMsg_t(AppEvt AEvt, uint16_t Data) : Data16(Data), Evt(AEvt) {}
} __attribute__((__packed__));

static EvtMsgQ_t<AppMsg_t, 9> EvtQ;
#endif

#if 1 // ============================== Controls ===============================
uint32_t In[3];

// ==== Outputs ====
void PulserCallback(virtual_timer_t *vtp, void *p);

class Pulser_t: private PinOutput_t {
private:
    virtual_timer_t ITmr;
public:
    Pulser_t(GPIO_TypeDef *APGPIO, uint16_t APin) : PinOutput_t(APGPIO, APin, omPushPull) {}
    void Init() { PinOutput_t::Init(); }
    void PulseI(uint32_t Dur) {
        chVTResetI(&ITmr);
        SetHi();
        if(Dur == 0) SetLo();
        else chVTSetI(&ITmr, TIME_MS2I(Dur), PulserCallback, (void*)this);
    }
    void ResetI() {
        chVTResetI(&ITmr);
        SetLo();
    }
    void SetHi() { PinOutput_t::SetHi(); }
    void SetLo() { PinOutput_t::SetLo(); }

    // Inner use
    void IOnTmrDone() { SetLo(); }
};

void PulserCallback(virtual_timer_t *vtp, void *p) { ((Pulser_t*)p)->IOnTmrDone(); }

Pulser_t OutPin[OUTPUT_CNT] = { {OUTPUT1}, {OUTPUT2}, {OUTPUT3} };

// ==== Inputs ====
void InputPinIrqHandlerI();
void InputTmrCallback(virtual_timer_t *vtp, void *p) { __NOP(); }

class Input_t : public PinIrq_t {
public:
    virtual_timer_t ITmr;
    Input_t(GPIO_TypeDef *APGpio, uint16_t APinN, PinPullUpDown_t APullUpDown) :
        PinIrq_t(APGpio, APinN, APullUpDown, InputPinIrqHandlerI) {}
    void Init() {
        PinIrq_t::Init(ttRising);
        CleanIrqFlag();
        EnableIrq(IRQ_PRIO_LOW);
    }
    bool CheckIfProcess() {
        if(chVTIsArmedI(&ITmr)) return false; // Not enough time passed
        chVTSetI(&ITmr, TIME_MS2I(INPUT_DEADTIME_ms), InputTmrCallback, nullptr);
        return true;
    }
};

Input_t InputPin[] = { {INPUT1}, {INPUT2}, /*{INPUT3}*/ };

void InputPinIrqHandlerI() {
    uint32_t Flags = EXTI->PR;
    // Pin1
    if(Flags & (1 << InputPin[0].PinN)) {
        if(InputPin[0].CheckIfProcess()) EvtQ.SendNowOrExitI(AppEvt::StartFire);
    }
    // Pin2
    if(Flags & (1 << InputPin[1].PinN)) {
        if(InputPin[0].CheckIfProcess()) EvtQ.SendNowOrExitI(AppEvt::StartFire);
    }
    // Pin3 - unused
//    if(Flags & (1 << InputPin[2].PinN))
}

void SetInputs(uint32_t AIn[3]) {
    if(In[0] == 0 and AIn[0] == 1) EvtQ.SendNowOrExit(AppEvt::StartFire);
    if(In[1] == 0 and AIn[1] == 1) EvtQ.SendNowOrExit(AppEvt::StartFire);
    for(int i=0; i<3; i++) In[i] = AIn[i];
}

bool DoBurstFire() {
    return In[0] == 1 or InputPin[0].IsHi();
}
#endif

#if 1 // =========================== Indication ================================
extern LedSmooth_t SideLEDs[SIDE_LEDS_CNT];
extern LedSmooth_t FrontLEDs[FRONT_LEDS_CNT];

namespace Indication {

enum class IndiState { Idle, Reloading, MagazinesEnded, HitsEnded } State = IndiState::Idle;

void Shot() {
    Beeper.StartOrRestart(bsqShot);
    for(auto& Led : FrontLEDs) Led.StartOrRestart(lsqShot);
    Printf("#Shot; %d/%d left\r\n", RoundsCnt, MagazinesCnt);
}

void RoundsEnded() {
    chSysLock();
    // Beeper
    if(Beeper.GetCurrentSequence() == bsqHit) Beeper.SetNextSequenceI(bsqReloading);
    else Beeper.StartOrRestartI(bsqReloading);
    // Leds
    if(SideLEDs[3].GetCurrentSequence() == lsqHit) SideLEDs[3].SetNextSequenceI(lsqReloading);
    else {
        SideLEDs[0].StopI();
        SideLEDs[1].StopI();
        SideLEDs[2].StopI();
        SideLEDs[3].StartOrRestartI(lsqReloading);
    }
    State = IndiState::Reloading;
    chSysUnlock();
    Printf("#RoundsEnded\r\n");
}

void MagazineReloaded() {
    chSysLock();
    // Beeper
    if(Beeper.GetCurrentSequence() == bsqHit) Beeper.SetNextSequenceI(bsqMagazReloaded);
    else Beeper.StartOrRestartI(bsqMagazReloaded);
    // Leds
    if(SideLEDs[3].GetCurrentSequence() != lsqHit) {
        for(auto& Led : SideLEDs) Led.StopI();
    }
    State = IndiState::Idle;
    chSysUnlock();
    Printf("#MagazineReloaded\r\n");
}

void MagazinesEnded() {
    chSysLock();
    // Beeper
    if(Beeper.GetCurrentSequence() == bsqHit) Beeper.SetNextSequenceI(bsqMagazEnded);
    else Beeper.StartOrRestartI(bsqMagazEnded);
    // Leds
    if(SideLEDs[3].GetCurrentSequence() == lsqHit)
        SideLEDs[3].SetNextSequenceI(lsqMagazinesEnded);
    else {
        SideLEDs[0].StopI();
        SideLEDs[1].StopI();
        SideLEDs[2].StopI();
        SideLEDs[3].StartOrRestartI(lsqMagazinesEnded);
    }
    State = IndiState::MagazinesEnded;
    chSysUnlock();
    Printf("#MagazinesEnded\r\n");
}

void Hit(uint32_t HitFrom) {
    chSysLock();
    OutPin[0].PulseI(Settings.PulseLengthHit_ms);
    Beeper.StartOrRestartI(bsqHit);
    for(auto& Led : SideLEDs) Led.StartOrRestartI(lsqHit);
    switch(State) {
        case IndiState::Idle:
            Beeper.SetNextSequenceI(nullptr);
            SideLEDs[3].SetNextSequenceI(nullptr);
            break;
        case IndiState::HitsEnded:
            Beeper.SetNextSequenceI(bsqHitsEnded);
            SideLEDs[3].SetNextSequenceI(lsqHitsEnded);
            break;
        case IndiState::Reloading:
            Beeper.SetNextSequenceI(bsqReloading);
            SideLEDs[3].SetNextSequenceI(lsqReloading);
            break;
        case IndiState::MagazinesEnded:
            Beeper.SetNextSequenceI(bsqMagazEnded);
            SideLEDs[3].SetNextSequenceI(lsqMagazinesEnded);
            break;
    }
    chSysUnlock();
    Printf("#Hit from %d; %d left\r\n", HitFrom, HitCnt);
}

void HitsEnded() {
    chSysLock();
    // Beeper
    if(Beeper.GetCurrentSequence() == bsqHit) Beeper.SetNextSequenceI(bsqHitsEnded);
    else Beeper.StartOrRestartI(bsqHitsEnded);
    OutPin[1].SetHi();
    for(auto& Led : SideLEDs) Led.StartOrRestartI(lsqHitsEnded);
    chSysUnlock();
    Printf("#Hits Ended\r\n");
}

void Reset(bool quiet) {
    State = IndiState::Idle;
    for(auto& Led : FrontLEDs) Led.Stop();
    for(auto& Led : SideLEDs)  Led.Stop();
    if(!quiet) Printf("#Reset\r\n");
}

} // namespace
#endif

#if 1 // ============================= Hit counter =============================
systime_t PrevHitTime = 0;

void IrRxCallbackI(uint32_t Rcvd) { EvtQ.SendNowOrExitI(AppMsg_t(AppEvt::IrRx, Rcvd)); }

void ProcessRxPkt(IRPkt_t RxPkt) {
//    RxPkt.Print();
    if(RxPkt.FightID != Settings.FightID) return; // Ignore pkt from outside (or crc error)
    if(!RxPkt.IsCrcOk()) return; // Bad pkt
    if(RxPkt.Type == PKT_TYPE_RESET) EvtQ.SendNowOrExit(AppEvt::Reset);
    else if(HitCnt > 0) { // Not reset
        if(RxPkt.GunID == Settings.GunID) return; // Ignore pkt from self
        else if(RxPkt.Type == PKT_TYPE_SHOT) { // Shot incoming
            // Ignore if not enough time passed since last hit
            if(chVTTimeElapsedSinceX(PrevHitTime) < TIME_S2I(Settings.MinDelayBetweenHits)) return;
            // Hit occured, decrement if not infinity
            if(!Settings.HitCnt.IsInfinity()) HitCnt--;
            Indication::Hit(RxPkt.GunID);
            if(HitCnt > 0) PrevHitTime = chVTGetSystemTimeX();
            else Indication::HitsEnded();
        }
    }
}
#endif

#if 1 // =============================== Firing ================================
virtual_timer_t FireTmr;
IRPkt_t PktTx;
systime_t FireStart;

// ==== Delay subsystem ====
void TmrCallback(virtual_timer_t *vtp, void *p) {
    chSysLockFromISR();
    EvtQ.SendNowOrExitI((AppEvt)((uint32_t)p));
    chSysUnlockFromISR();
}

void StartDelay(int32_t ADelay_s, AppEvt AEvt) {
    if(ADelay_s <= 0) EvtQ.SendNowOrExit(AEvt); // Do it immediately
    else chVTSet(&FireTmr, TIME_S2I(ADelay_s), TmrCallback, (void*)((uint32_t)AEvt));
}

void StartDelay_ms(int32_t ADelay_ms, AppEvt AEvt) {
    if(ADelay_ms <= 0) EvtQ.SendNowOrExit(AEvt); // Do it immediately
    else chVTSet(&FireTmr, TIME_MS2I(ADelay_ms), TmrCallback, (void*)((uint32_t)AEvt));
}

void OnIrTxEndI() { EvtQ.SendNowOrExitI(AppEvt::EndOfIrTx); }

bool IsFiring;

void Fire() {
    IsFiring = true;
    if(!Settings.RoundsInMagazine.IsInfinity()) RoundsCnt--;
    // Prepare pkt
    PktTx.Type    = Settings.PktType;
    PktTx.FightID = Settings.FightID;
    PktTx.TeamID  = Settings.TeamID;
    PktTx.GunID   = Settings.GunID;
    PktTx.CalculateCRC();
//    PktTx.Print();
    // Start transmission
    irLed::TransmitWord(PktTx.W16, Settings.TXPwr, OnIrTxEndI);
    FireStart = chVTGetSystemTimeX();
    Indication::Shot();
}
#endif

void Reset(bool quiet) {
    chSysLock();
    irLed::ResetI();
    for(auto& Pin : OutPin) Pin.ResetI();
    for(auto& Pin : InputPin) Pin.CleanIrqFlag();
    IsFiring = false;
    RoundsCnt = Settings.RoundsInMagazine;
    MagazinesCnt = Settings.MagazinesCnt;
    chVTResetI(&FireTmr);
    HitCnt = Settings.HitCnt;
    PrevHitTime = chVTGetSystemTimeX();
    chSysUnlock();
    Indication::Reset(quiet);
}

// ==================================== Thread =================================
static THD_WORKING_AREA(waAppThread, 256);
static void AppThread(void* arg) {
    while(true) {
        AppMsg_t Msg = EvtQ.Fetch(TIME_INFINITE);
        // Will be here when new Evt occur
        if(Msg.Evt == AppEvt::Reset) Reset();
        else if(Msg.Evt == AppEvt::IrRx) ProcessRxPkt(IRPkt_t(Msg.Data16));
        else if(HitCnt > 0) switch(Msg.Evt) { // Do nothing if no hits left
            case AppEvt::StartFire:
                if(!IsFiring and RoundsCnt > 0) Fire();
                break;

            case AppEvt::EndOfIrTx: // Tx of several same pkts just ended
                StartDelay_ms(Settings.ShotsPeriod_ms - TIME_I2MS(chVTTimeElapsedSinceX(FireStart)), AppEvt::EndOfDelayBetweenShots);
                break;

            case AppEvt::EndOfDelayBetweenShots:
                IsFiring = false;
                if(RoundsCnt > 0) { // Fire if needed and there are rounds left
                    if(DoBurstFire()) Fire();
                }
                else { // no more rounds
                    if(MagazinesCnt > 1) { // Reload if possible (more than 0 magazines left)
                        if(!Settings.MagazinesCnt.IsInfinity()) MagazinesCnt--;
                        Indication::RoundsEnded();
                        StartDelay(Settings.MagazineReloadDelay, AppEvt::MagazineReloadDone);
                    }
                    else Indication::MagazinesEnded(); // No magazines left
                }
                break;

            case AppEvt::MagazineReloadDone:
                Indication::MagazineReloaded();
                RoundsCnt = Settings.RoundsInMagazine;
                if(DoBurstFire()) Fire();
                break;

            default: break;
        } // switch Evt
    } // while true
}

void AppInit() {
    EvtQ.Init();
    irLed::Init();
    irRcvr::Init(IrRxCallbackI);
    // Control pins init
    for(auto& Pin : OutPin) Pin.Init();
    for(auto& Pin : InputPin) Pin.Init();
    Reset();

    // Create and start thread
    chThdCreateStatic(waAppThread, sizeof(waAppThread), NORMALPRIO, AppThread, NULL);
}

