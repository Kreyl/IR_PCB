/*
 * Settings.h
 *
 *  Created on: 20 нояб. 2022 г.
 *      Author: layst
 */


#ifndef SETTINGS_H_
#define SETTINGS_H_

#include <inttypes.h>

#define FIGHT_ID_MAX        3UL
#define TEAM_ID_MAX         7UL
#define GUN_ID_MAX          64UL
#define HIT_CNT_MAX         254UL
#define HIT_CNT_INF         255UL
#define RND_CNT_MAX         254UL
#define RND_CNT_INF         255UL
#define RND_MGZNS_MAX       254UL
#define RND_MGZNS_INF       255UL
#define SHOTS_PER_MAX       9999UL
#define MGZ_RLD_DLY_MAX     60UL
#define DLY_BTW_HITS_MAX    60UL
#define PULSE_LEN_HIT_MAX   9999UL
#define TX_PWR_MAX          255UL

struct ValueComplex_t {
    uint32_t Value;
    uint32_t Max;
    uint32_t Infinity;
    ValueComplex_t(uint32_t AValue, uint32_t AMax) :
        Value(AValue), Max(AMax), Infinity(0) {}
    ValueComplex_t(uint32_t AValue, uint32_t AMax, uint32_t AInfinity) :
        Value(AValue), Max(AMax), Infinity(AInfinity) {}
};

union Settings_t {
//    void Load();
//    uint8_t Save();

    struct Named_t {
        // IDs
        ValueComplex_t FightID { 0, 3 };
        ValueComplex_t TeamID  { 0, 7 };
        ValueComplex_t GunID   { 1, 63 };

        // Counts
        ValueComplex_t HitCnt           { 4, 254, 255 };
        ValueComplex_t RoundsInMagazine { 9, 254, 255 };
        ValueComplex_t MagazinesCnt     { 4, 254, 255 };

        // Delays
        ValueComplex_t ShotsPeriod_ms { 252, 9999 };
        ValueComplex_t MagazineReloadDelay { 4, 60 };
        ValueComplex_t MinDelayBetweenHits { 0, 60 };
        ValueComplex_t PulseLengthHit_ms   { 450, 9999 };

        // TX power
        ValueComplex_t TXPwr { 207, 255 };
    };

//    ValueComplex_t Arr[sizeof(Named_t) / sizeof(ValueComplex_t)];
};

//extern Settings_t Settings;

#endif // SETTINGS_H_

