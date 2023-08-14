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

#if 1 // ============================== Controls ===============================
namespace Controls {

bool DoBurstFire() {
    return false;
}

}
#endif

namespace Indication {

void ShowRoundsEnd() {
    Printf("#RoundsEnded\r\n");
}

void ShowMagazinesEnded() {
    Printf("#MagazinesEnded\r\n");
}

void Reset() {

}

} // namespace

#if 1 // ================================= Hull ================================
namespace Hull {

int32_t HitCnt = 4;

}
#endif

#if 1 //================================= Gun ==================================
namespace Gun {

void Fire() {

}

void Reset() {
    // Stop TX, disable and reset IRQ
}

}
#endif

#if 1 // =============================== Firing ================================
namespace Firing {

enum class FirEvt { Reset, StartFire, EndOfFiring, MagazineReloadDone };
enum class FirState { Idle, Firing, };


EvtMsgQ_t<FirEvt, 9> EvtQ; // Evt queue
virtual_timer_t Tmr;

FirState State = FirState::Idle;
int32_t RoundsCnt = 9, MagazinesCnt = 4;

void Reset() {
    Gun::Reset();
    State = FirState::Idle;
    RoundsCnt = Settings.RoundsInMagazine;
    MagazinesCnt = Settings.MagazinesCnt;
}

// ==== Delay subsystem ====
void TmrCallback(virtual_timer_t *vtp, void *p) {
    chSysLockFromISR();
    FirEvt Evt = (FirEvt)((uint32_t)p);
    EvtQ.SendNowOrExitI(Evt);
    chSysUnlockFromISR();
}

void StartDelay(uint32_t ADelay_s, FirEvt AEvt) {
    if(ADelay_s == 0) EvtQ.SendNowOrExit(AEvt); // Do it immediately
    else chVTSet(&Tmr, TIME_S2I(ADelay_s), TmrCallback, (void*)((uint32_t)AEvt));
}

void Fire() {
    State = FirState::Firing;
    Gun::Fire();
    RoundsCnt--;
}

// ==== Thread ====
static THD_WORKING_AREA(waFireThread, 256);
static void FireThread(void* arg) {
    while(true) {
        FirEvt Evt = EvtQ.Fetch(TIME_INFINITE);
        // Will be here when new Evt occur
        if(Evt == FirEvt::Reset) Reset();
        else if(Hull::HitCnt == 0) return; // Do nothing if no hits left
        else switch(State) {
            case FirState::Idle:
                switch(Evt) {
                    case FirEvt::StartFire:
                        if(RoundsCnt > 0) Fire();
                        break;
                    case FirEvt::MagazineReloadDone:
                        RoundsCnt = Settings.RoundsInMagazine;
                        if(Controls::DoBurstFire()) Fire();
                        break;
                    default: break; // Ignore other
                } // switch evt
                break;

            case FirState::Firing:
                if(Evt == FirEvt::EndOfFiring) {
                    State = FirState::Idle;
                    if(RoundsCnt > 0) { // Fire if needed and there are rounds left
                        if(Controls::DoBurstFire()) Fire();
                    }
                    else { // no more rounds
                        Indication::ShowRoundsEnd();
                        if(MagazinesCnt > 0) { // Reload if possible
                            MagazinesCnt--;
                            StartDelay(Settings.MagazineReloadDelay, FirEvt::MagazineReloadDone);
                        }
                        else Indication::ShowMagazinesEnded(); // No magazines left
                    }
                }
                break;
        } // switch State
    } // while true
}


void Init() {
    Reset();
    // Create and start thread
    chThdCreateStatic(waFireThread, sizeof(waFireThread), NORMALPRIO, FireThread, NULL);

}


} // Firing
#endif
