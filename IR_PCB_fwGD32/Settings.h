/*
 * Settings.h
 *
 *  Created on: 20 нояб. 2022 г.
 *      Author: layst
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_

#include <inttypes.h>
#include "color.h"
#include <vector>

//#define DEBUG   TRUE

#define SETTINGS_MAX_CNT        27

#define CORE_MAX_SZ             FLAME_LEN_MAX
#define SPARKS_MODE_RANDOM      0
#define SPARKS_MODE_GRADIENT    1
#define SPARK_CENTER_SZ         1
#define SPARK_TAIL_LEN_MAX      27
#define SPARK_LEN_MAX           (SPARK_CENTER_SZ + SPARK_TAIL_LEN_MAX * 2)
#define SPARKS_CNT_MAX          99


struct FlameSettings_t {
    struct {
#if DEBUG
        uint8_t Sz = 0;
#else
        uint8_t Sz = 1;
#endif
        uint16_t ClrHMin = 15, ClrHMax = 15;
        uint8_t ClrV = 28;
        uint8_t ClrW = 0;
        bool IsOk() { return Sz <= CORE_MAX_SZ and ClrHMax <= 360 and ClrHMin <= 360 and ClrV <= 100; }
    } Core;
    struct {
#if DEBUG // DEBUG
        uint8_t Cnt = 1;
        uint8_t TailLen = 0;
        uint16_t ClrHMin = 0, ClrHMax = 120;
        uint8_t ClrV = 100;
        uint8_t ClrWMin = 0, ClrWMax = 0;
        uint16_t DelayBeforeRestart = 540;
        int16_t AccMin = 0, AccMax = 0;
        int16_t StartDelayMin = 200, StartDelayMax = 200;
#else
        uint8_t Cnt = 7;
        uint8_t TailLen = 0;
        uint16_t ClrHMin = 4, ClrHMax = 15;
        uint8_t ClrV = 100;
        uint8_t ClrWMin = 0, ClrWMax = 0;
        uint16_t DelayBeforeRestart = 630;
        int16_t AccMin = 9, AccMax = 27;
        int16_t StartDelayMin = 81, StartDelayMax = 117;
#endif
        uint8_t Mode = 1;
        bool IsOk() { return Cnt <= SPARKS_CNT_MAX and TailLen < SPARK_TAIL_LEN_MAX and ClrHMax <= 360 and ClrHMin <= 360 and ClrV <= 100
                and AccMin < 99 and AccMax < 99; }
        uint16_t GetRandomHue() {
            if(ClrHMin < ClrHMax) return Random::Generate(ClrHMin, ClrHMax);
            else return Random::Generate(ClrHMax, ClrHMin);
        }
        uint8_t GetRandomW() {
            if(ClrWMin < ClrWMax) return Random::Generate(ClrWMin, ClrWMax);
            else return Random::Generate(ClrWMax, ClrWMin);
        }
    } Sparks;

    bool IsOk() { return Core.IsOk() and Sparks.IsOk(); }
    void Print() {
        Printf("Core: Sz=%u ClrHMin=%u ClrHMax=%u ClrV=%u\r"
                "Sparks: Cnt=%u TailLen=%u ClrHMin=%u ClrHMax=%u ClrV=%u "
                "DelayBeforeRestart=%u AccMin=%u AccMax=%u "
                "StartDelayMin=%u StartDelayMax=%u Mode=%u\r",
                Core.Sz, Core.ClrHMin, Core.ClrHMax, Core.ClrV,
                Sparks.Cnt, Sparks.TailLen, Sparks.ClrHMin, Sparks.ClrHMax, Sparks.ClrV,
                Sparks.DelayBeforeRestart, Sparks.AccMin, Sparks.AccMax,
                Sparks.StartDelayMin, Sparks.StartDelayMax, Sparks.Mode);
    }
} __attribute__ ((aligned));


class Settings_t {
private:
    std::vector<FlameSettings_t> ISetups;
    uint8_t CurrIndx = 0;
    void LoadCurrIndx();
    void SaveCurrIndx();
public:
    Settings_t() {
        ISetups.reserve(SETTINGS_MAX_CNT);
    }
    void Load();
    retv Save();
    void Clear() {
        ISetups.clear();
    }
    retv Add(FlameSettings_t &ASetup);
    uint32_t Cnt() {
        return ISetups.size();
    }
    FlameSettings_t& GetNext();
    FlameSettings_t& GetPrev();
    FlameSettings_t& GetCurrent();
    FlameSettings_t& operator[](const uint8_t Indx);
};

#endif // SETTINGS_H_
