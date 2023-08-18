/*
 * Settings.cpp
 *
 *  Created on: 20 нояб. 2022 г.
 *      Author: layst
 */

#include "Settings.h"
#include "kl_lib.h"
#include "shell.h"

Settings_t Settings;

void Settings_t::Load() {
    uint32_t *p = (uint32_t*)SETTINGS_STORAGE_ADDR; // Pointer to storage
    uint32_t N = *p++;
    FlameSettings_t *pStp = (FlameSettings_t*)p;
    for(uint32_t i=0; i<N; i++) {
        if(pStp->IsOk()) ISetups.push_back(*pStp);
        else break;
        pStp++;
    }
    Printf("Setups loaded: %u\r", ISetups.size());
    if(ISetups.size() == 0)
        ISetups.resize(1);
    LoadCurrIndx();
}

uint8_t Settings_t::Save() {
    uint8_t Rslt = retvOk;
    uint32_t Addr = SETTINGS_STORAGE_ADDR;
    // Prepare Flash
    chSysLock();
    Flash::LockFlash();
    Flash::UnlockFlash();
    Flash::ClearPendingFlags();
    // Erase flash
    if(Flash::ErasePage(Addr) != retvOk) {
        PrintfI("\rPage Erase fail\r");
        Rslt = retvFail;
        goto End;
    }
    // ==== Write flash ====
    uint32_t N = (sizeof(Settings_t) + 3UL) / 4;
    uint32_t *p = (uint32_t*)this;
    for(uint32_t i=0; i<N; i++) {
        if(Flash::ProgramWord(Addr, *p++) != retvOk) {
            PrintfI("Write Fail\r");
            Rslt = retvFail;
            goto End;
        }
        Addr += sizeof(uint32_t);
    }
    Flash::WaitForLastOperation(TIME_MS2I(450));
    End:
    Flash::LockFlash();
    chSysUnlock();
    return Rslt;
}
