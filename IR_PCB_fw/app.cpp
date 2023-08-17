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
#include "Sequences.h"

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

int32_t HitCnt, RoundsCnt, MagazinesCnt;


#if 1 // ============================== Controls ===============================
uint32_t In[3];

void SetInputs(uint32_t AIn[3]) {
    if(In[0] == 0 and AIn[0] == 1) EvtQ.SendNowOrExit(AppEvt::StartFire);
    if(In[1] == 0 and AIn[1] == 1) EvtQ.SendNowOrExit(AppEvt::StartFire);
    for(int i=0; i<3; i++) In[i] = AIn[i];
}

bool DoBurstFire() {
    return In[1] == 1;
}

#endif

#if 1 // =========================== Indication ================================
extern LedSmooth_t SideLEDs[SIDE_LEDS_CNT];
extern LedSmooth_t FrontLEDs[FRONT_LEDS_CNT];

namespace Indication {

enum class IndiState {
    Idle, Reloading, MagazinesEnded, HitsEnded
} State = IndiState::Idle;

void Shot() {
    for(auto& Led : FrontLEDs) Led.StartOrRestart(lsqShot);
    Printf("#Shot; %d/%d left\r\n", RoundsCnt, MagazinesCnt);
}

void RoundsEnded() {
    chSysLock();
    if(SideLEDs[3].GetCurrentSequence() == lsqHit)
        SideLEDs[3].SetNextSequenceI(lsqReloading);
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
    if(SideLEDs[3].GetCurrentSequence() != lsqHit)
        for(auto& Led : SideLEDs) Led.StopI();
    State = IndiState::Idle;
    chSysUnlock();
    Printf("#MagazineReloaded\r\n");
}

void MagazinesEnded() {
    chSysLock();
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
    for(auto& Led : SideLEDs) Led.StartOrRestartI(lsqHit);
    switch(State) {
        case IndiState::Idle:           SideLEDs[3].SetNextSequenceI(nullptr); break;
        case IndiState::HitsEnded:      SideLEDs[3].SetNextSequenceI(lsqHitsEnded); break;
        case IndiState::Reloading:      SideLEDs[3].SetNextSequenceI(lsqReloading); break;
        case IndiState::MagazinesEnded: SideLEDs[3].SetNextSequenceI(lsqMagazinesEnded); break;
    }
    chSysUnlock();
    Printf("#Hit from %d; %d left\r\n", HitFrom, HitCnt);
}

void HitsEnded(uint32_t HitFrom) {
    chSysLock();
    for(auto& Led : SideLEDs) Led.StartOrRestartI(lsqHitsEnded);
    chSysUnlock();
    Printf("#Hit from %d; 0 left\r\n", HitFrom);
}

void Reset() {
    State = IndiState::Idle;
    for(auto& Led : FrontLEDs) Led.Stop();
    for(auto& Led : SideLEDs)  Led.Stop();
    Printf("#Reset\r\n");
}

} // namespace
#endif

#if 1 // ================================= Hull ================================
systime_t PrevHitTime = 0;

void IrRxCallbackI(uint32_t Rcvd) { EvtQ.SendNowOrExitI(AppMsg_t(AppEvt::IrRx, Rcvd)); }

void ProcessRxPkt(IRPkt_t RxPkt) {
//    RxPkt.Print();
    if(RxPkt.FightID != Settings.FightID) return; // Ignore pkt from outside (or crc error)
    if(!RxPkt.IsCrcOk()) return; // Bad pkt
    if(RxPkt.Type == PKT_TYPE_RESET) EvtQ.SendNowOrExitI(AppEvt::Reset);
    else { // Not reset
        if(RxPkt.TeamID == Settings.TeamID) return; // Ignore pkt from our team (or crc error)
        else if(RxPkt.Type == PKT_TYPE_SHOT) { // Shot incoming
            // Ignore if not enough time passed since last hit
            if(chVTTimeElapsedSinceX(PrevHitTime) < TIME_S2I(Settings.MinDelayBetweenHits)) return;
            // Hit occured
            HitCnt--;
            if(HitCnt > 0) {
                PrevHitTime = chVTGetSystemTimeX();
                Indication::Hit(RxPkt.GunID);
            }
            else Indication::HitsEnded(RxPkt.GunID);
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

void Reset() {
    chSysLock();
    irLed::ResetI();
    RoundsCnt = Settings.RoundsInMagazine;
    MagazinesCnt = Settings.MagazinesCnt;
    chVTResetI(&FireTmr);
    HitCnt = Settings.HitCnt;
    PrevHitTime = chVTGetSystemTimeX();
    chSysUnlock();
    Indication::Reset();
}

void OnIrTxEndI() { EvtQ.SendNowOrExitI(AppEvt::EndOfIrTx); }

void Fire() {
    RoundsCnt--;
    // Prepare pkt
    PktTx.Type = PKT_TYPE_SHOT;
    PktTx.FightID = Settings.FightID;
    PktTx.TeamID = Settings.TeamID;
    PktTx.GunID = Settings.GunID;
    PktTx.CalculateCRC();
//    PktTx.Print();
    // Start transmission
    irLed::TransmitWord(PktTx.W16, Settings.TXPwr, OnIrTxEndI);
    FireStart = chVTGetSystemTimeX();
    Indication::Shot();
}
#endif

// ==================================== Thread =================================
static THD_WORKING_AREA(waAppThread, 256);
static void AppThread(void* arg) {
    while(true) {
        AppMsg_t Msg = EvtQ.Fetch(TIME_INFINITE);
        // Will be here when new Evt occur
        if(Msg.Evt == AppEvt::Reset) Reset();
        else if(HitCnt > 0) switch(Msg.Evt) { // Do nothing if no hits left
            case AppEvt::StartFire:
                if(RoundsCnt > 0) Fire();
                break;

            case AppEvt::EndOfIrTx: // Tx of several same pkts just ended
                StartDelay_ms(Settings.ShotsPeriod_ms - TIME_I2MS(chVTTimeElapsedSinceX(FireStart)), AppEvt::EndOfDelayBetweenShots);
                break;

            case AppEvt::EndOfDelayBetweenShots:
                if(RoundsCnt > 0) { // Fire if needed and there are rounds left
                    if(DoBurstFire()) Fire();
                }
                else { // no more rounds
                    if(MagazinesCnt > 1) { // Reload if possible (more than 0 magazines left)
                        MagazinesCnt--;
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

            case AppEvt::IrRx:
                ProcessRxPkt(IRPkt_t(Msg.Data16));
                break;

            default: break;
        } // switch Evt
    } // while true
}

void AppInit() {
    EvtQ.Init();
    irLed::Init();
    irRcvr::Init(IrRxCallbackI);
//    FrontLEDs[0].
    Reset();

    // Control pins init

    // Create and start thread
    chThdCreateStatic(waAppThread, sizeof(waAppThread), NORMALPRIO, AppThread, NULL);
}

