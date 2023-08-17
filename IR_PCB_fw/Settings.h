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
    uint32_t GunID = 1;

    // Counts
    int32_t HitCnt = 4;
    int32_t RoundsInMagazine = 9;
    int32_t MagazinesCnt = 4;

    // Delays
    int32_t ShotsPeriod_ms = 306;
    int32_t MagazineReloadDelay = 4;
    int32_t MinDelayBetweenHits = 0;

    // TX power
    int32_t TXPwr = 207;

};

extern Settings_t Settings;

#endif // SETTINGS_H_

