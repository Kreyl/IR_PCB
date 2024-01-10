/*
 * Settings.cpp
 *
 *  Created on: 20 нояб. 2022 г.
 *      Author: layst
 */


#include "Settings.h"
#include "gd_lib.h"
#include "shell.h"

FlameSettings_t& Settings_t::operator[](const uint8_t Indx) {
    if(Indx < ISetups.size()) CurrIndx = Indx;
    else CurrIndx = ISetups.size() - 1;
    SaveCurrIndx();
    return ISetups[CurrIndx];
}

FlameSettings_t& Settings_t::GetNext() {
    if(CurrIndx >= ISetups.size() - 1) CurrIndx = 0;
    else CurrIndx++;
    SaveCurrIndx();
    return ISetups[CurrIndx];
}

FlameSettings_t& Settings_t::GetPrev() {
    if(CurrIndx == 0) CurrIndx = ISetups.size() - 1;
    else CurrIndx--;
    SaveCurrIndx();
    return ISetups[CurrIndx];
}

FlameSettings_t& Settings_t::GetCurrent() {
    return ISetups[CurrIndx];
}

void Settings_t::LoadCurrIndx() {
//    uint8_t SavedValue = RTC->BKP1R;
//    if(SavedValue < ISetups.size()) CurrIndx = SavedValue;
//    else
        CurrIndx = 0;
//    Printf("LoadIndx: %u\r", RTC->BKP1R);
}

void Settings_t::SaveCurrIndx() {
//    RTC->BKP1R = CurrIndx;
}

retv Settings_t::Add(FlameSettings_t &ASetup) {
    if(ISetups.size() >= SETTINGS_MAX_CNT) return retv::Overflow;
    else {
        ISetups.push_back(ASetup);
        return retv::Ok;
    }
}

void Settings_t::Load() {
//    ISetups.clear();
//    uint32_t *p = (uint32_t*)SETTINGS_STORAGE_ADDR; // Pointer to storage
//    uint32_t N = *p++;
//    FlameSettings_t *pStp = (FlameSettings_t*)p;
//    for(uint32_t i=0; i<N; i++) {
//        if(pStp->IsOk()) ISetups.push_back(*pStp);
//        else break;
//        pStp++;
//    }
//    Printf("Setups loaded: %u\r", ISetups.size());
    if(ISetups.size() == 0)
        ISetups.resize(1);
    LoadCurrIndx();
}

retv Settings_t::Save() {
    retv Rslt = retv::Ok;
//    uint32_t Addr = SETTINGS_STORAGE_ADDR;
//    // Prepare Flash
//    chSysLock();
//    Flash::LockFlash();
//    Flash::UnlockFlash();
//    Flash::ClearPendingFlags();
//    // Erase flash
//    if(Flash::ErasePage(Addr) != retvOk) {
//        PrintfI("\rPage Erase fail\r");
//        Rslt = retvFail;
//        goto End;
//    }
//    // ==== Write flash ====
//    // Write setups count
//    if(Flash::ProgramWord(Addr, ISetups.size()) != retvOk) {
//        PrintfI("Write Cnt Fail\r");
//        Rslt = retvFail;
//        goto End;
//    }
//    // Write setups
//    for(FlameSettings_t &fst : ISetups) {
//        uint32_t *p = (uint32_t*)&fst;
//        uint32_t N = (sizeof(FlameSettings_t) + 3) / 4;
//        for(uint32_t i=0; i<N; i++) {
//            Addr += sizeof(uint32_t);
//            if(Flash::ProgramWord(Addr, *p++) != retvOk) {
//                PrintfI("Write Fail\r");
//                Rslt = retvFail;
//                goto End;
//            }
//        }
//    }
//    Flash::WaitForLastOperation(TIME_MS2I(450));
//    End:
//    Flash::LockFlash();
//    chSysUnlock();
    return Rslt;
}
