/*
 * Settings.h
 *
 *  Created on: 20 нояб. 2022 г.
 *      Author: layst
 */


#ifndef SETTINGS_H_
#define SETTINGS_H_

#include <inttypes.h>


class Settings_t {
private:
public:
//    Settings_t() {
//        ISetups.reserve(SETTINGS_MAX_CNT);
//    }
    void Load();
    uint8_t Save();
//    void Clear() {
//        ISetups.clear();
//    }

    // IDs
    uint32_t FightID = 0;
    uint32_t TeamID = 0;

    // Counts
    int32_t HitsCnt = 4;
    int32_t RoundsInMagazine = 9;
    int32_t MagazinesCnt = 4;

    int32_t IrPktsInShot = 1;

    // Delays
    int32_t ShotsPeriod_ms = 99;
    int32_t MagazineReloadDelay = 4;


    // TX power
    int32_t TXPwr = 207;

};

extern Settings_t Settings;

#endif // SETTINGS_H_

