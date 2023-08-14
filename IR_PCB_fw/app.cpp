/*
 * app.cpp
 *
 *  Created on: 14 авг. 2023 г.
 *      Author: layst
 */

#include "app.h"
#include "Settings.h"

Firing_h Firing;

#if 1 // ============================== Controls ===============================
namespace Controls {

bool DoBurstFire() {
    return false;
}

}
#endif

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
enum class FirState { Idle, Firing, };

FirState State = FirState::Idle;
int32_t RoundsCnt = 9, MagazinesCnt = 4;

void Reset() {
    Gun::Reset();
    State = FirState::Idle;
    RoundsCnt = Settings.RoundsInMagazine;
    MagazinesCnt = Settings.MagazinesCnt;

}



void StateMachine(FirEvt Evt) {
    if(Evt == FirEvt::Reset) Reset();
    else if(Hull::HitCnt == 0) return; // Do nothing if no hits left

    switch(State) {
        case FirState::Idle:
            if(Evt == FirEvt::StartFire and RoundsCnt > 0) {
                State = FirState::Firing;
                Gun::Fire();
                RoundsCnt--;
            }
            break;

        case FirState::Firing:
            if(Evt == FirEvt::EndOfFiring) {
                if(RoundsCnt == 0) {
                    // Reload if possible
                    if(MagazinesCnt > 0) {
                        MagazinesCnt--;
                        if(Settings.MagazineReloadDelay == 0)
                    }
                }

                // Proceed?
                if(Controls::DoBurstFire() and RoundsCnt > 0) {

                }
            }
            break;
    }
}


} // Firing
#endif
