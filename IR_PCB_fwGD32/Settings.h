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
#include <vector>
#include "app_classes.h"
#include "ff.h"

#define SETTINGS_FILENAME   "Settings.ini"

extern CustomOutPin output_hits_present;

class ValueBase {
protected:
    const char* const section;
    const char* const comment;
    friend class Settings;
public:
    const char* const name;
    int32_t v;
    const int32_t v_default;
    int32_t operator *() const { return v; }
    void SetToDefault() { v = v_default; }
    virtual retv CheckAndSetIfOk(int32_t) = 0;
    virtual void PrintOnGet(Shell*) = 0;
    virtual void PrintOnNew(Shell *pshell) = 0;
    void PrintOnBad(Shell *pshell, int32_t bad_value) { pshell->Print("%S BadValue: %d\r\n", name, bad_value); }
    virtual void Save(FIL* pfile) = 0;
    static const uint32_t kValueNameSz = 16;
    ValueBase(int32_t adefault, const char* asection, const char* aname, const char *acomment):
        section(asection), comment(acomment), name(aname), v(adefault), v_default(adefault) {}
};

// 0 is disabled, 1 is enabled
class ValueEnable : public ValueBase {
public:
    bool IsEnabled() { return v != 0; }
    retv CheckAndSetIfOk(int32_t avalue) {
        if(avalue >= 0) {
            v = avalue == 0? 0 : 1;
            return retv::Ok;
        }
        else return retv::BadValue;
    }
    void PrintOnGet(Shell *pshell) { pshell->Print("%*S = %4d; Def = %d; %S\r\n", kValueNameSz, name, v, v_default, comment); }
    void PrintOnNew(Shell *pshell) { pshell->Print("%S = %d\r\n", name, v); }
    void Save(FIL* pfile) {
        f_printf(pfile, "# %S\r\n", comment);
        f_printf(pfile, "# Default = %D\r\n", v_default);
        f_printf(pfile, "%S = %D\r\n\r\n", name, v);
    }
    ValueEnable(int32_t adefault, const char* asection, const char* aname, const char *acomment) :
                ValueBase(adefault, asection, aname, acomment) {}
};

// PlayerID and TeamID. Default value is generally not used: it is required at power-on only.
class ValueMinMax : public ValueBase {
protected:
    const int32_t v_min, v_max;
public:
    retv CheckAndSetIfOk(int32_t avalue) {
        if(avalue < v_min or avalue > v_max) return retv::BadValue;
        v = avalue;
        return retv::Ok;
    }
    void PrintOnGet(Shell *pshell) {
        pshell->Print("%*S = %4d; Min = %4d Max = %4d; %S\r\n", kValueNameSz, name, v, v_min, v_max, comment);
    }
    void PrintOnNew(Shell *pshell) { pshell->Print("%S = %d\r\n", name, v); }
    void Save(FIL* pfile) {
        f_printf(pfile, "# %S\r\n", comment);
        f_printf(pfile, "# Min = %D, Max = %D\r\n", v_min, v_max);
        f_printf(pfile, "%S = %D\r\n", name, v);
    }
    ValueMinMax(int32_t adefault, int32_t amin, int32_t amax,
            const char* asection, const char* aname, const char *acomment) :
                ValueBase(adefault, asection, aname, acomment),
                v_min(amin), v_max(amax) {}
};

// No infinity here
class ValueMinMaxDef : public ValueBase {
protected:
    const int32_t v_min, v_max;
public:
    retv CheckAndSetIfOk(int32_t avalue) {
        if(avalue < v_min or avalue > v_max) return retv::BadValue;
        v = avalue;
        return retv::Ok;
    }
    void PrintOnGet(Shell *pshell) {
        pshell->Print("%*S = %4d; Min = %4d Max = %4d Def = %3d; %S\r\n",
                kValueNameSz, name, v, v_min, v_max, v_default, comment);
    }
    void PrintOnNew(Shell *pshell) { pshell->Print("%S = %d\r\n", name, v); }
    void Save(FIL* pfile) {
        f_printf(pfile, "# %S\r\n", comment);
        f_printf(pfile, "# Min = %D, Max = %D, Default = %D\r\n", v_min, v_max, v_default);
        f_printf(pfile, "%S = %D\r\n\r\n", name, v);
    }
    ValueMinMaxDef(int32_t adefault, int32_t amin, int32_t amax,
            const char* asection, const char* aname, const char *acomment) :
                ValueBase(adefault, asection, aname, acomment),
                v_min(amin), v_max(amax) {}
};

// Infinity value added
class ValueMinMaxDefInf : public ValueBase {
protected:
    const int32_t v_min, v_max;
public:
    bool IsInfinity() { return v == (v_max + 1L); }
    retv CheckAndSetIfOk(int32_t avalue) {
        if(avalue < v_min or avalue > (v_max + 1L)) return retv::BadValue;
        v = avalue;
        return retv::Ok;
    }
    void PrintOnGet(Shell *pshell) {
        pshell->Print("%*S = %4d; Min = %4d Max = %4d Def = %3d Inf = %3d; %S\r\n",
                kValueNameSz, name, v, v_min, v_max, v_default, v_max+1L, comment);
    }
    void PrintOnNew(Shell *pshell) { pshell->Print("%S = %d\r\n", name, v); }
    void Save(FIL* pfile) {
        f_printf(pfile, "# %S\r\n", comment);
        f_printf(pfile, "# Min = %D, Max = %D, Default = %D, Infinity = %D\r\n", v_min, v_max, v_default, v_max+1L);
        f_printf(pfile, "%S = %D\r\n\r\n", name, v);
    }

    ValueMinMaxDefInf(int32_t adefault, int32_t amin, int32_t amax,
            const char* asection, const char* aname, const char *acomment) :
                ValueBase(adefault, asection, aname, acomment),
                v_min(amin), v_max(amax) {}
};

class ValueGpioMode : public ValueBase {
private:
    CustomOutPin *ppin;
    static constexpr const char* kCommentPre = "Mode of ";
    static constexpr const char* kCommentPost = ": PushPullActiveHi = 0, PushPullActiveLo = 1, OpenDrainActiveHi = 2, OpenDrainActiveLo = 3";
    static constexpr const char* mode_names[4] = {
            "PushPullActiveHi", "PushPullActiveLo",
            "OpenDrainActiveHi", "OpenDrainActiveLo"
    };
public:
    PinMode GetMode() { return static_cast<PinMode>(v); }
    retv CheckAndSetIfOk(int32_t avalue) {
        if(avalue >= 0 and avalue <= 3) {
            v = avalue;
            // Switch pin
            bool is_active = ppin->IsActive();
            ppin->SetMode(GetMode());
            ppin->SetActive(is_active);
            return retv::Ok;
        }
        else return retv::BadValue;
    }
    void PrintOnGet(Shell *pshell) { pshell->Print("%*S = %4d; Def = %d; %S%S%S\r\n",
            kValueNameSz, name, v, v_default, kCommentPre, comment, kCommentPost); }
    void Save(FIL* pfile) {
        f_printf(pfile, "# %S%S%S\r\n", kCommentPre, comment, kCommentPost);
        f_printf(pfile, "# Default = %D\r\n", v_default);
        f_printf(pfile, "%S = %D\r\n\r\n", name, v);
    }
    void PrintOnNew(Shell *pshell) { pshell->Print("%S = %d (%S)\r\n", name, v, mode_names[v]); }
    ValueGpioMode(PinMode adefault, const char* asection, const char* aname, const char *acomment, CustomOutPin *appin) :
                ValueBase(static_cast<int32_t>(adefault), asection, aname, acomment), ppin(appin) {}
};

class ValuePktType : public ValueBase {
public:
    ValuePktType(int32_t adefault, const char* asection, const char* aname) :
        ValueBase(adefault, asection, aname,
                "Supported types: Shot = 0x0000, NewGame = 0x8305, "
                "AddHealth = 0x8000, AddRounds = 0x8100"
              ) {}
    retv CheckAndSetIfOk(int32_t avalue) {
        if(avalue < 0 or avalue > 0xFFFF) return retv::BadValue;
        v = avalue;
        return retv::Ok;
    }
    void PrintOnGet(Shell *pshell) {
        pshell->Print("%*S = 0x%04X; Def = 0x%04X; %S\r\n", kValueNameSz, name, v, v_default, comment);
    }
    void Save(FIL* pfile) {
        f_printf(pfile,
                    "# PktType to transmit:\r\n"
                    "#   * Shot is 0x0000 (PlayerID, TeamID and Damage added automatically)\r\n"
                    "#   * NewGame is 0x8305\r\n"
                    "#   * AddHealth is 0x8000 (number of added health points set in the amount value)\r\n"
                    "#   * AddRounds is 0x8100 (number of added rounds set in the amount value)\r\n");
        f_printf(pfile, "%S = 0x%04X\r\n\r\n", name, v);
    }
    void PrintOnNew(Shell *pshell) { pshell->Print("%S = 0x%04X\r\n", name, v); }
};

class ValueDamage : public ValueBase {
private:
    static constexpr const char* kPossible = "Possible values: 1,2,4,5,7,10,15,17,20,25,30,35,40,50,75,100";
public:
    int32_t bits;
    ValueDamage(const char* asection, const char* aname) :
        ValueBase(1, asection, aname, "damage caused by a single shot"), bits(0) {} // 0 means 1 hit
    retv CheckAndSetIfOk(int32_t avalue) {
        RetvValI32 r = Damage_HitsToId(avalue);
        if(r.IsOk()) {
            v = avalue;
            bits = r.v;
            return retv::Ok;
        }
        else return retv::BadValue;
    }
    void PrintOnGet(Shell *pshell) {
        pshell->Print("%*S = %4u; Def = %4u; %S - %S\r\n", kValueNameSz, name, v, v_default, kPossible, comment);
    }
    void Save(FIL* pfile) {
        f_printf(pfile, "# Hits Damage in 'Shot' pkt. Unusable for other pkt types.\r\n"
                "# %S\r\n", kPossible);
        f_printf(pfile, "%S = %D\r\n\r\n", name, v);
    }
    void PrintOnNew(Shell *pshell) { pshell->Print("%S = %d\r\n", name, v); }
};

class Settings {
private:
    static constexpr const char* kGrpIDs = "IDs";
    static constexpr const char* kGrpCounts = "Counts";
    static constexpr const char* kGrpDelays = "Delays";
    static constexpr const char* kGrpIrRx = "IRRX";
    static constexpr const char* kGrpIrTx = "IRTX";
    static constexpr const char* kGrpGpio = "Gpio";
    static constexpr const char* kGrpBehavior = "Behavior";
    static constexpr const char* kGrpResearch = "Research";
public:
    // IDs
    ValueMinMax player_id { 0, 0, 127, kGrpIDs, "PlayerID", "Player ID, must be unique" };
    ValueMinMax team_id   { 0, 0,  3,  kGrpIDs, "TeamID", "Team ID, must be unique" };
    ValueMinMaxDef super_damage_id { 70, 0, 127, kGrpIDs, "SuperDamageID", "A shot from him completely removes all hits" };
    // Counts
    ValueMinMaxDefInf hit_cnt          { 4, 1, 254, kGrpCounts, "HitCnt", "Number of hits, can be unlimited" };
    ValueMinMaxDefInf rounds_in_magaz  { 9, 1, 254, kGrpCounts, "RoundsInMagazine", "Number of rounds in a single magazine, can be unlimited" };
    ValueMinMaxDefInf magazines_cnt    { 4, 1, 254, kGrpCounts, "MagazinesCnt", "Number of magazines, can be unlimited" };
    // Delays
    ValueMinMaxDef shots_period_ms      { 252, 0, 9999, kGrpDelays, "ShotsPeriod_ms", "Interval between shots in burst fire, ms" };
    ValueMinMaxDef magaz_reload_delay_s {   4, 0,   60, kGrpDelays, "MagazReloadDelay", "Interval between autoreloading of magazines, s" };
    ValueMinMaxDef min_delay_btw_hits_s {   0, 0,   60, kGrpDelays, "MinDelayBetwHits", "Minimum delay between hits loss, s (when 0, it is possible to loose all within a second)" };
    // IR RX
    ValueMinMaxDef ir_rx_deviation { 150, 1, 600, kGrpIrRx, "Deviation", "Deviation of received pulse length, us. Larger is more tolerant" };
    // IR TX
    ValueMinMaxDef ir_tx_pwr  {     90,     1,    255, kGrpIrTx, "TXPwr", "Power of IR output" };
    ValueMinMaxDef ir_tx_freq {  56000, 30000,  56000, kGrpIrTx, "TXFreq", "IR transmission modulation frequency, Hz" };
    ValuePktType   pkt_type   { 0x0000,                kGrpIrTx, "PktType" };
    ValueDamage    tx_damage  {                        kGrpIrTx, "TXDamage" };
    ValueMinMaxDef tx_amount  {      1,     1,    100, kGrpIrTx, "Amount", "Number of things to be added by special packets: AddHealth, AddRounds, etc." };
    // Gpio control
    ValueGpioMode pin_mode_gpio3 { PinMode::PushPullActiveHi, kGrpGpio, "Gpio3Mode", "Gpio3 (hits_present)",  &output_hits_present };
    // Behavior, Modes of operation
    ValueEnable fire_always {0, kGrpBehavior, "FireAlways", "Burst fire always: 1 is enabled, 0 is disabled" };
    // Research
//    ValueEnable print_rx_pkt {1, "Research", "PrintRxPkt", "Print received IR packet when enabled; 1 is enabled, 0 is disabled" };
    ValueEnable transmit_what_rcvd {0, kGrpResearch, "TransmitWhatRcvd", "Transmit last received pkt when firing; 1 is enabled, 0 is disabled" };

    // Array of value pointers
    std::vector<ValueBase*> values_arr = {
            &player_id, &team_id, &super_damage_id,
            &hit_cnt, &rounds_in_magaz, &magazines_cnt,
            &shots_period_ms, &magaz_reload_delay_s, &min_delay_btw_hits_s,
            &ir_rx_deviation,
            &ir_tx_pwr, &ir_tx_freq, &pkt_type, &tx_damage, &tx_amount,
            &pin_mode_gpio3,
            &fire_always,
            /*&print_rx_pkt,*/ &transmit_what_rcvd
    };

    void Load();
    retv Save();
    void SetAllToDefault() { for(ValueBase* const pval : values_arr) pval->SetToDefault(); }
};

extern Settings settings;

#endif // SETTINGS_H_
