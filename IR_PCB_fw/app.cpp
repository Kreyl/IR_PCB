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

static EvtMsgQ_t<AppMsg_t, 9> EvtQ; // Evt queue

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

namespace Indication {

void ShowRoundsEnd() {
    Printf("#RoundsEnded\r\n");
}

void ShowMagazinesEnded() {
    Printf("#MagazinesEnded\r\n");
}

void ShowMagazineReloaded() {
    Printf("#MagazineReloaded\r\n");
}

void Reset() {
    Printf("#Reset\r\n");
}

} // namespace

#if 1 // ================================= Hull ================================
int32_t HitCnt = 4;

void IrRxCallbackI(uint32_t Rcvd) {
//    PrintfI("Rcvd: 0x%X\r", Rcvd);
    EvtQ.SendNowOrExitI(AppMsg_t(AppEvt::IrRx, Rcvd));

}


#endif

#if 1 // =============================== Firing ================================
virtual_timer_t Tmr;
int32_t RoundsCnt = 9, MagazinesCnt = 4;
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
    else chVTSet(&Tmr, TIME_S2I(ADelay_s), TmrCallback, (void*)((uint32_t)AEvt));
}

void StartDelay_ms(int32_t ADelay_ms, AppEvt AEvt) {
    if(ADelay_ms <= 0) EvtQ.SendNowOrExit(AEvt); // Do it immediately
    else chVTSet(&Tmr, TIME_MS2I(ADelay_ms), TmrCallback, (void*)((uint32_t)AEvt));
}

void Reset() {
    chSysLock();
    irLed::ResetI();
    RoundsCnt = Settings.RoundsInMagazine;
    MagazinesCnt = Settings.MagazinesCnt;
    chVTResetI(&Tmr);
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
    PktTx.PktN++;
    PktTx.CalculateCRC();
    PktTx.Print();
    // Start transmission of several packets
    irLed::TransmitWord(PktTx.W16, Settings.TXPwr, Settings.IrPktsInShot, OnIrTxEndI);
    FireStart = chVTGetSystemTimeX();
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
                    Indication::ShowRoundsEnd();
                    if(MagazinesCnt > 0) { // Reload if possible
                        MagazinesCnt--;
                        StartDelay(Settings.MagazineReloadDelay, AppEvt::MagazineReloadDone);
                    }
                    else Indication::ShowMagazinesEnded(); // No magazines left
                }
                break;

            case AppEvt::MagazineReloadDone:
                Indication::ShowMagazineReloaded();
                RoundsCnt = Settings.RoundsInMagazine;
                if(DoBurstFire()) Fire();
                break;

            case AppEvt::IrRx: {
                IRPkt_t RxPkt(Msg.Data16);
                RxPkt.Print();
            } break;

            default: break;
        } // switch Evt
    } // while true
}

void AppInit() {
    EvtQ.Init();
    irLed::Init();
    irRcvr::Init(IrRxCallbackI);
    Reset();

    // Control pins init

    // Create and start thread
    chThdCreateStatic(waAppThread, sizeof(waAppThread), NORMALPRIO, AppThread, NULL);
}

