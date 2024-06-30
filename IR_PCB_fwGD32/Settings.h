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
    const int32_t v_default, v_min, v_max;
    const char* const Section;
    const char* const Name;
    bool IsInfinity() { return v == (v_max + 1L); }
    void SetToDefault() { v = v_default; }
    bool CheckAndSetIfOk(int32_t AValue) {
        if(AValue < v_min or AValue > (v_max + 1L)) return false;
        v = AValue;
        return true;
    }
    Value_t(int32_t ADefault, int32_t AMin, int32_t AMax, const char* ASection, const char* AName) :
        v(ADefault), v_default(ADefault), v_min(AMin), v_max(AMax), Section(ASection), Name(AName) {}
    operator uint32_t() const { return v; }
};

class Settings {
public:
    void Load();
    retv Save();
    void SetAllToDefault();

#if 0 // ==== DEBUG ====
    // IDs
    Value_t player_id { 2, 0, 127, "IDs", "PlayerID" };
    Value_t team_id   { 1, 0,  3,  "IDs", "TeamID" };
    // Counts
    Value_t hit_cnt           { 2, 1, 254, "Counts", "HitCnt" };
    Value_t rounds_in_magaz { 9, 1, 254, "Counts", "RoundsInMagazine" };
    Value_t magazines_cnt     { 4, 1, 254, "Counts", "MagazinesCnt" };
    // Delays
    Value_t shots_period_ms   { 1000, 0, 9999, "Delays", "ShotsPeriod_ms" };
    Value_t magaz_reload_delay {   4, 0,   60, "Delays", "MagazReloadDelay" };
    Value_t min_delay_btw_hits {   0, 0,   60, "Delays", "MinDelayBetwHits" };
    Value_t pulse_len_hit_ms   { 100, 1, 9999, "Delays", "PulseLenHit_ms" };
    // TX
    Value_t ir_tx_pwr   {   200,     1,    255, "IRTX", "TXPwr" };
    Value_t ir_tx_freq  { 56000, 30000,  56000, "IRTX", "TXFreq" };
    Value_t pkt_type {     0,     0, 0xFFFF, "IRTX", "PktType" };
#else
    // IDs
    Value_t player_id { 0, 0, 127, "IDs", "PlayerID" };
    Value_t team_id   { 0, 0,  3,  "IDs", "TeamID" };
    // Counts
    Value_t hit_cnt          { 4, 1, 254, "Counts", "HitCnt" };
    Value_t rounds_in_magaz  { 9, 1, 254, "Counts", "RoundsInMagazine" };
    Value_t magazines_cnt    { 4, 1, 254, "Counts", "MagazinesCnt" };
    // Delays
    Value_t shots_period_ms    { 252, 0, 9999, "Delays", "ShotsPeriod_ms" };
    Value_t magaz_reload_delay {   4, 0,   60, "Delays", "MagazReloadDelay" };
    Value_t min_delay_btw_hits {   0, 0,   60, "Delays", "MinDelayBetwHits" };
    Value_t pulse_len_hit_ms   { 100, 1, 9999, "Delays", "PulseLenHit_ms" };
    // TX
    Value_t ir_tx_pwr   {    90,     1,    255, "IRTX", "TXPwr" };
    Value_t ir_tx_freq  { 56000, 30000,  56000, "IRTX", "TXFreq" };
    Value_t pkt_type    {     0,     0, 0xFFFF, "IRTX", "PktType" };
#endif
};

#define SETTINGS_CNT    (sizeof(Settings) / sizeof(Value_t))

extern Settings settings;

#endif // SETTINGS_H_
