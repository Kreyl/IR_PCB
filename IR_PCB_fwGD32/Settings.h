/*
 * Settings.h
 *
 *  Created on: 20.11.2022
 *      Author: layst
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_

#include "types.h"

#define SETTINGS_FILENAME   "Settings.ini"

class Value_t {
public:
    int32_t v;
    const int32_t Default, Min, Max;
    const char* const Section;
    const char* const Name;
    bool IsInfinity() { return v == (Max + 1L); }
    void SetToDefault() { v = Default; }
    bool CheckAndSetIfOk(int32_t AValue) {
        if(AValue < Min or AValue > (Max + 1L)) return false;
        v = AValue;
        return true;
    }
    Value_t(int32_t ADefault, int32_t AMin, int32_t AMax, const char* ASection, const char* AName) :
        v(ADefault), Default(ADefault), Min(AMin), Max(AMax), Section(ASection), Name(AName) {}
    operator uint32_t() const { return v; }
};

class Settings_t {
public:
    void Load();
    retv Save();
    void SetAllToDefault();

#if 0 // ==== DEBUG ====
    // IDs
    Value_t PlayerID { 2, 0, 127, "IDs", "PlayerID" };
    Value_t TeamID   { 1, 0,  3,  "IDs", "TeamID" };
    // Counts
    Value_t HitCnt           { 2, 1, 254, "Counts", "HitCnt" };
    Value_t RoundsInMagazine { 9, 1, 254, "Counts", "RoundsInMagazine" };
    Value_t MagazinesCnt     { 4, 1, 254, "Counts", "MagazinesCnt" };
    // Delays
    Value_t ShotsPeriod_ms   { 1000, 0, 9999, "Delays", "ShotsPeriod_ms" };
    Value_t MagazReloadDelay {   4, 0,   60, "Delays", "MagazReloadDelay" };
    Value_t MinDelayBetwHits {   0, 0,   60, "Delays", "MinDelayBetwHits" };
    Value_t PulseLenHit_ms   { 100, 1, 9999, "Delays", "PulseLenHit_ms" };
    // TX
    Value_t TXPwr   {   200,     1,    255, "IRTX", "TXPwr" };
    Value_t TXFreq  { 56000, 30000,  56000, "IRTX", "TXFreq" };
    Value_t PktType {     0,     0, 0xFFFF, "IRTX", "PktType" };
#else
    // IDs
    Value_t PlayerID { 0, 0, 127, "IDs", "PlayerID" };
    Value_t TeamID   { 0, 0,  3,  "IDs", "TeamID" };
    // Counts
    Value_t HitCnt           { 4, 1, 254, "Counts", "HitCnt" };
    Value_t RoundsInMagazine { 9, 1, 254, "Counts", "RoundsInMagazine" };
    Value_t MagazinesCnt     { 4, 1, 254, "Counts", "MagazinesCnt" };
    // Delays
    Value_t ShotsPeriod_ms   { 252, 0, 9999, "Delays", "ShotsPeriod_ms" };
    Value_t MagazReloadDelay {   4, 0,   60, "Delays", "MagazReloadDelay" };
    Value_t MinDelayBetwHits {   0, 0,   60, "Delays", "MinDelayBetwHits" };
    Value_t PulseLenHit_ms   { 100, 1, 9999, "Delays", "PulseLenHit_ms" };
    // TX
    Value_t TXPwr   {    90,     1,    255, "IRTX", "TXPwr" };
    Value_t TXFreq  { 56000, 30000,  56000, "IRTX", "TXFreq" };
    Value_t PktType {     0,     0, 0xFFFF, "IRTX", "PktType" };
#endif
};

#define SETTINGS_CNT    (sizeof(Settings_t) / sizeof(Value_t))

extern Settings_t Settings;

#endif // SETTINGS_H_
