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

enum class FirEvt { Reset, StartFire, EndOfIrTx, EndOfFiring, MagazineReloadDone };
EvtMsgQ_t<FirEvt, 9> FirEvtQ; // Evt queue

#if 1 // ============================== Controls ===============================
uint32_t In[3];

void SetInputs(uint32_t AIn[3]) {
    if(In[0] == 0 and AIn[0] == 1) FirEvtQ.SendNowOrExit(FirEvt::StartFire);
    if(In[1] == 0 and AIn[1] == 1) FirEvtQ.SendNowOrExit(FirEvt::StartFire);
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

#endif

#if 1 // =============================== Firing ================================
virtual_timer_t Tmr;
int32_t RoundsCnt = 9, MagazinesCnt = 4;
IRPkt_t PktTx;
systime_t FireStart;

// ==== Delay subsystem ====
void TmrCallback(virtual_timer_t *vtp, void *p) {
    chSysLockFromISR();
    FirEvtQ.SendNowOrExitI((FirEvt)((uint32_t)p));
    chSysUnlockFromISR();
}

void StartDelay(int32_t ADelay_s, FirEvt AEvt) {
    if(ADelay_s <= 0) FirEvtQ.SendNowOrExit(AEvt); // Do it immediately
    else chVTSet(&Tmr, TIME_S2I(ADelay_s), TmrCallback, (void*)((uint32_t)AEvt));
}

void StartDelay_ms(int32_t ADelay_ms, FirEvt AEvt) {
    if(ADelay_ms <= 0) FirEvtQ.SendNowOrExit(AEvt); // Do it immediately
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

void OnIrTxEndI() {
    FirEvtQ.SendNowOrExitI(FirEvt::EndOfIrTx);
}

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

// ==== Thread ====
static THD_WORKING_AREA(waFireThread, 256);
static void FireThread(void* arg) {
    while(true) {
        FirEvt Evt = FirEvtQ.Fetch(TIME_INFINITE);
        // Will be here when new Evt occur
        if(Evt == FirEvt::Reset) Reset();
        else if(HitCnt > 0) switch(Evt) { // Do nothing if no hits left
            case FirEvt::StartFire:
                if(RoundsCnt > 0) Fire();
                break;

            case FirEvt::EndOfIrTx: // Tx of several same pkts just ended
                StartDelay_ms(Settings.ShotsPeriod_ms - TIME_I2MS(chVTTimeElapsedSinceX(FireStart)), FirEvt::EndOfFiring);
                break;

            case FirEvt::EndOfFiring:
                if(RoundsCnt > 0) { // Fire if needed and there are rounds left
                    if(DoBurstFire()) Fire();
                }
                else { // no more rounds
                    Indication::ShowRoundsEnd();
                    if(MagazinesCnt > 0) { // Reload if possible
                        MagazinesCnt--;
                        StartDelay(Settings.MagazineReloadDelay, FirEvt::MagazineReloadDone);
                    }
                    else Indication::ShowMagazinesEnded(); // No magazines left
                }
                break;

            case FirEvt::MagazineReloadDone:
                Indication::ShowMagazineReloaded();
                RoundsCnt = Settings.RoundsInMagazine;
                if(DoBurstFire()) Fire();
                break;

            default: break;
        } // switch Evt
    } // while true
}

void IrRxCallbackI(uint32_t Rcvd) {
    PrintfI("Rcvd: 0x%X\r", Rcvd);
//    IRPkt_t
}

void AppInit() {
    FirEvtQ.Init();
    irLed::Init();
    irRcvr::Init(IrRxCallbackI);
    Reset();

    // Control pins init

    // Create and start thread
    chThdCreateStatic(waFireThread, sizeof(waFireThread), NORMALPRIO, FireThread, NULL);
}
#endif
