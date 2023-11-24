/*
 * app.cpp
 *
 *  Created on: 14.08.2023
 *      Author: layst
 */

#include "app.h"
#include "Settings.h"
#include "gd_lib.h"
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
// ==== Outputs ====
void PulserCallback(void *p);

class Pulser_t: private Pin_t {
private:
    VirtualTimer_t ITmr;
public:
    Pulser_t(GPIO_TypeDef *APGPIO, uint16_t APin) : Pin_t(APGPIO, APin) {}
    void Init() { Pin_t::SetupOut(Gpio::PushPull); }
    void PulseI(uint32_t Dur) {
        ITmr.ResetI();
        SetHi();
        if(Dur == 0) SetLo();
        else ITmr.SetI(TIME_MS2I(Dur), PulserCallback, (void*)this);
    }
    void ResetI() {
        ITmr.ResetI();
        SetLo();
    }
    void SetHi() { Pin_t::SetHi(); }
    void SetLo() { Pin_t::SetLo(); }

    // Inner use
    void IOnTmrDone() { SetLo(); }
};

void PulserCallback(void *p) { ((Pulser_t*)p)->IOnTmrDone(); }

Pulser_t OutPulseOnHit{Output_PulseOnHit};
Pulser_t OutHitsEnded {Output_HitsEnded};

// ==== Inputs ====
void InputPinIrqHandlerI();
void InputTmrCallback(void *p) { __NOP(); }

class Input_t : public PinIrq_t {
public:
    VirtualTimer_t ITmr;
    Input_t(GPIO_TypeDef *APGpio, uint16_t APinN, Gpio::PullUpDown_t APullUpDown) :
        PinIrq_t(APGpio, APinN, APullUpDown, InputPinIrqHandlerI) {}
    void Init() {
        PinIrq_t::Init(ttRising);
        CleanIrqFlag();
        EnableIrq(IRQ_PRIO_LOW);
    }
    bool CheckIfProcess() {
        if(ITmr.IsArmedX()) return false; // Not enough time passed
        else {
            ITmr.SetI(TIME_MS2I(INPUT_DEADTIME_ms), InputTmrCallback, nullptr);
            return true;
        }
    }
};

Input_t InSingleFire{Input_SingleFire};
Input_t InBurstFire {Input_BurstFire};

void InputPinIrqHandlerI() {
    uint32_t Flags = EXTI->PD;
    // InSingleFire
    if(Flags & (1 << InSingleFire.PinN)) {
        if(InSingleFire.CheckIfProcess()) EvtQ.SendNowOrExitI(AppEvt::StartFire);
    }
    // Pin2
    if(Flags & (1 << InBurstFire.PinN)) {
        if(InBurstFire.CheckIfProcess()) EvtQ.SendNowOrExitI(AppEvt::StartFire);
    }
}

// For Debug
static uint32_t IIn[2];
void SetInputs(uint32_t AIn[2]) {
    if(IIn[0] == 0 and AIn[0] == 1) EvtQ.SendNowOrExit(AppEvt::StartFire);
    if(IIn[1] == 0 and AIn[1] == 1) EvtQ.SendNowOrExit(AppEvt::StartFire);
    for(int i=0; i<2; i++) IIn[i] = AIn[i];
}

bool DoBurstFire() { return IIn[0] == 1 or InSingleFire.IsHi(); }
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
    Sys::Lock();
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
    Sys::Unlock();
    Printf("#RoundsEnded\r\n");
}

void MagazineReloaded() {
    Sys::Lock();
    // Beeper
    if(Beeper.GetCurrentSequence() == bsqHit) Beeper.SetNextSequenceI(bsqMagazReloaded);
    else Beeper.StartOrRestartI(bsqMagazReloaded);
    // Leds
    if(SideLEDs[3].GetCurrentSequence() != lsqHit) {
        for(auto& Led : SideLEDs) Led.StopI();
    }
    State = IndiState::Idle;
    Sys::Unlock();
    Printf("#MagazineReloaded\r\n");
}

void MagazinesEnded() {
    Sys::Lock();
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
    Sys::Unlock();
    Printf("#MagazinesEnded\r\n");
}

void Hit(uint32_t HitFrom) {
    Sys::Lock();
    OutPulseOnHit.PulseI(Settings.PulseLenHit_ms);
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
    Sys::Unlock();
    Printf("#Hit from %d; %d left\r\n", HitFrom, HitCnt);
}

void HitsEnded() {
    Sys::Lock();
    // Beeper
    if(Beeper.GetCurrentSequence() == bsqHit) Beeper.SetNextSequenceI(bsqHitsEnded);
    else Beeper.StartOrRestartI(bsqHitsEnded);
    OutHitsEnded.SetHi();
    for(auto& Led : SideLEDs) Led.StartOrRestartI(lsqHitsEnded);
    Sys::Unlock();
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

#if 1 // ========================= Reception processing ========================
systime_t PrevHitTime = 0;

void IrRxCallbackI(uint32_t Rcvd) { EvtQ.SendNowOrExitI(AppMsg_t(AppEvt::IrRx, Rcvd)); }

void ProcessRxPkt(IRPkt_t RxPkt) {
//    RxPkt.Print();
    // Reset
    if(RxPkt.W16 == PKT_RESET) EvtQ.SendNowOrExit(AppEvt::Reset);
    // Shot incoming
    else if(RxPkt.Zero == 0) {
        if(HitCnt <= 0) return; // Nothing to do when no hits left
        if(RxPkt.PlayerID == Settings.PlayerID) return; // Ignore pkt from self
        if(RxPkt.TeamID == Settings.TeamID) return; // Ignore friendly fire
        // Ignore if not enough time passed since last hit
        if(Sys::TimeElapsedSince(PrevHitTime) < TIME_S2I(Settings.MinDelayBetwHits)) return;
        // Hit occured, decrement if not infinity
        if(!Settings.HitCnt.IsInfinity()) HitCnt--;
        Indication::Hit(RxPkt.PlayerID);
        if(HitCnt > 0) PrevHitTime = Sys::GetSysTimeX();
        else Indication::HitsEnded();
    } // if zero
}
#endif

#if 1 // =============================== Firing ================================
VirtualTimer_t FireTmr;
IRPkt_t PktTx;
systime_t FireStart;

// ==== Delay subsystem ====
void TmrCallback(void *p) {
    Sys::LockFromIRQ();
    EvtQ.SendNowOrExitI((AppEvt)((uint32_t)p));
    Sys::UnlockFromIRQ();
}

void StartDelay(int32_t ADelay_s, AppEvt AEvt) {
    if(ADelay_s <= 0) EvtQ.SendNowOrExit(AEvt); // Do it immediately
    else FireTmr.Set(TIME_S2I(ADelay_s), TmrCallback, (void*)((uint32_t)AEvt));
}

void StartDelay_ms(int32_t ADelay_ms, AppEvt AEvt) {
    if(ADelay_ms <= 0) EvtQ.SendNowOrExit(AEvt); // Do it immediately
    else FireTmr.Set(TIME_MS2I(ADelay_ms), TmrCallback, (void*)((uint32_t)AEvt));
}

void OnIrTxEndI() { EvtQ.SendNowOrExitI(AppEvt::EndOfIrTx); }

bool IsFiring;

void Fire() {
    IsFiring = true;
    if(!Settings.RoundsInMagazine.IsInfinity()) RoundsCnt--;
    // Prepare pkt
    if(Settings.PktType == PKT_SHOT) {
        PktTx.W16 = 0;
        PktTx.PlayerID = Settings.PlayerID;
        PktTx.TeamID  = Settings.TeamID;
        PktTx.DamageID = 0; // Which means 1 hit
        irLed::TransmitWord(PktTx.W16, 14, Settings.TXPwr, OnIrTxEndI);
    }
    else {
        PktTx.W16 = (uint16_t)Settings.PktType;
        irLed::TransmitWord(PktTx.W16, 16, Settings.TXPwr, OnIrTxEndI);
    }
//    PktTx.Print();
    FireStart = Sys::GetSysTimeX();
    Indication::Shot();
}
#endif

void Reset(bool quiet) {
    Sys::Lock();
    irLed::ResetI();
    OutHitsEnded.ResetI();
    OutPulseOnHit.ResetI();
    InBurstFire.CleanIrqFlag();
    InSingleFire.CleanIrqFlag();
    IsFiring = false;
    RoundsCnt = Settings.RoundsInMagazine;
    MagazinesCnt = Settings.MagazinesCnt;
    FireTmr.ResetI();
    HitCnt = Settings.HitCnt;
    PrevHitTime = Sys::GetSysTimeX();
    Sys::Unlock();
    Indication::Reset(quiet);
}

// ==================================== Thread =================================
static THD_WORKSPACE(waAppThread, 256);
static void AppThread() {
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
                StartDelay_ms(Settings.ShotsPeriod_ms - TIME_I2MS(Sys::TimeElapsedSince(FireStart)), AppEvt::EndOfDelayBetweenShots);
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
                        StartDelay(Settings.MagazReloadDelay, AppEvt::MagazineReloadDone);
                    }
                    else {
                        MagazinesCnt = 0; // To avoid situation with 1 magaz with 0 rounds
                        Indication::MagazinesEnded(); // No magazines left
                    }
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
    OutHitsEnded.Init();
    OutPulseOnHit.Init();
    InBurstFire.Init();
    InSingleFire.Init();
    Reset();
    // Create and start thread
    Sys::CreateThd(waAppThread, sizeof(waAppThread), NORMALPRIO, AppThread);
}
