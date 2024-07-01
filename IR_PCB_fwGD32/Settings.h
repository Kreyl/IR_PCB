/*
 * Settings.h
 *
 *  Created on: 20.11.2022
 *      Author: layst
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_

#include "types.h"
#include "ir_pkt.h"

#define SETTINGS_FILENAME   "Settings.ini"

class ValueBase {
public:
    int32_t v;
    const int32_t v_default;
    const char* const section;
    const char* const name;
    operator uint32_t() const { return v; }
    void SetToDefault() { v = v_default; }
    virtual retv CheckAndSetIfOk(int32_t) = 0;
    virtual void Print(Shell*) = 0;
    static const uint32_t kValueNameSz = 16;
    ValueBase(int32_t adefault, const char* asection, const char* aname):
        v(adefault), v_default(adefault), section(asection), name(aname) {}
};

class ValueMinMaxDef : public ValueBase {
public:
    const int32_t v_min, v_max;
    bool IsInfinity() { return v == (v_max + 1L); }
    retv CheckAndSetIfOk(int32_t avalue) {
        if(avalue < v_min or avalue > (v_max + 1L)) return retv::BadValue;
        v = avalue;
        return retv::Ok;
    }
    void Print(Shell *pshell) {
        pshell->Print("%*S = %4u; default = %4u; Min = %u; Max = %4u\r\n",
                kValueNameSz, name, v, v_default, v_min, v_max);
    }
    ValueMinMaxDef(int32_t adefault, int32_t amin, int32_t amax,
            const char* asection, const char* aname) :
                ValueBase(adefault, asection, aname),
                v_min(amin), v_max(amax) {}

};

class ValueDamage : public ValueBase {
public:
    int32_t bits;
    ValueDamage(const char* asection, const char* aname) :
        ValueBase(1, asection, aname), bits(0) {} // 0 means 1 hit
    retv CheckAndSetIfOk(int32_t avalue) {
        StatusOrI32 r = Damage_HitsToId(avalue);
        if(r.Ok()) {
            v = avalue;
            bits = r.v;
            return retv::Ok;
        }
        else return retv::BadValue;
    }
    void Print(Shell *pshell) {
        pshell->Print("%*S = %4u; default = %4u\r\n", kValueNameSz, name, v, v_default);
    }
};

class Settings {
public:
    // IDs
    ValueMinMaxDef player_id { 0, 0, 127, "IDs", "PlayerID" };
    ValueMinMaxDef team_id   { 0, 0,  3,  "IDs", "TeamID" };
    // Counts
    ValueMinMaxDef hit_cnt          { 4, 1, 254, "Counts", "HitCnt" };
    ValueMinMaxDef rounds_in_magaz  { 9, 1, 254, "Counts", "RoundsInMagazine" };
    ValueMinMaxDef magazines_cnt    { 4, 1, 254, "Counts", "MagazinesCnt" };
    // Delays
    ValueMinMaxDef shots_period_ms    { 252, 0, 9999, "Delays", "ShotsPeriod_ms" };
    ValueMinMaxDef magaz_reload_delay {   4, 0,   60, "Delays", "MagazReloadDelay" };
    ValueMinMaxDef min_delay_btw_hits {   0, 0,   60, "Delays", "MinDelayBetwHits" };
    ValueMinMaxDef pulse_len_hit_ms   { 100, 1, 9999, "Delays", "PulseLenHit_ms" };
    // TX
    ValueMinMaxDef ir_tx_pwr   {    90,     1,    255, "IRTX", "TXPwr" };
    ValueMinMaxDef ir_tx_freq  { 56000, 30000,  56000, "IRTX", "TXFreq" };
    ValueMinMaxDef pkt_type    {     0,     0, 0xFFFF, "IRTX", "PktType" };
    ValueDamage    shot_damage { "IRTX", "Damage" };

    // Array of value pointers
    static constexpr uint32_t kValuesCnt = 13;
    ValueBase* const values_arr[kValuesCnt] = {
            &player_id, &team_id,
            &hit_cnt, &rounds_in_magaz, &magazines_cnt,
            &shots_period_ms, &magaz_reload_delay, &min_delay_btw_hits, &pulse_len_hit_ms,
            &ir_tx_pwr, &ir_tx_freq, &pkt_type, &shot_damage,
    };

    void Load();
    retv Save();
    void SetAllToDefault() { for(ValueBase* const pval : values_arr) pval->SetToDefault(); }
};

extern Settings settings;

#endif // SETTINGS_H_
