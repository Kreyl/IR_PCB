#include "Settings.h"
#include "gd_lib.h"
#include "shell.h"
#include "kl_fs_utils.h"

Settings settings;

void Settings::Load() {
    Value_t *Arr = (Value_t*)this;
    for(uint32_t i=0; i<SETTINGS_CNT; i++) {
        int32_t v;
        if(ini::Read(SETTINGS_FILENAME, Arr[i].Section, Arr[i].Name, &v) != retv::Ok or Arr[i].CheckAndSetIfOk(v) == false) {
            SetAllToDefault();
            Printf("Default settings loaded\r\n");
            return;
        }
//        else Printf("%S %S: 0x%04X\r", Arr[i].Section, Arr[i].Name, v);
    }
    Printf("Settings loaded\r\n");
}

void SaveValueWithMinMax(Value_t &Value) {
    f_printf(&CommonFile, "# Min=%D, Max=%D\r\n", Value.v_min, Value.v_max);
    f_printf(&CommonFile, "%S = %D\r\n", Value.Name, Value.v);
}

void SaveValueWithMinMaxInfDefault(Value_t &Value) {
    f_printf(&CommonFile, "# Min=%D, Max=%D, Infininty=%D, Default=%D\r\n", Value.v_min, Value.v_max, Value.v_max+1, Value.v_default);
    f_printf(&CommonFile, "%S = %D\r\n", Value.Name, Value.v);
}

void SaveValueWithCommentMinMaxInfDefault(Value_t &Value, const char* Comment) {
    f_printf(&CommonFile, "# %S\r\n", Comment);
    f_printf(&CommonFile, "# Min=%D, Max=%D, Infininty=%D, Default=%D\r\n", Value.v_min, Value.v_max, Value.v_max+1, Value.v_default);
    f_printf(&CommonFile, "%S = %D\r\n\r\n", Value.Name, Value.v);
}

void SaveValueWithCommentMinMaxDefault(Value_t &Value, const char* Comment) {
    f_printf(&CommonFile, "# %S\r\n", Comment);
    f_printf(&CommonFile, "# Min=%D, Max=%D, Default=%D\r\n", Value.v_min, Value.v_max, Value.v_default);
    f_printf(&CommonFile, "%S = %D\r\n\r\n", Value.Name, Value.v);
}

void SaveValueWithComment(Value_t &Value, const char* Comment) {
    f_printf(&CommonFile, "# %S\r\n", Comment);
    f_printf(&CommonFile, "%S = %D\r\n\r\n", Value.Name, Value.v);
}


retv Settings::Save() {
    if(TryOpenFileRewrite(SETTINGS_FILENAME, &CommonFile) != retv::Ok) return retv::Fail;
    // Description
    f_printf(&CommonFile, "# IR PCB v3.2 Settings.ini\r\n\r\n");
    // IDs
    f_printf(&CommonFile, "[IDs]\r\n");
    SaveValueWithMinMax(player_id);
    SaveValueWithMinMax(team_id);
    // Counts
    f_printf(&CommonFile, "\r\n[Counts]\r\n");
    SaveValueWithMinMaxInfDefault(hit_cnt);
    SaveValueWithMinMaxInfDefault(rounds_in_magaz);
    SaveValueWithMinMaxInfDefault(magazines_cnt);
    // Delays
    f_printf(&CommonFile, "\r\n[Delays]\r\n");
    SaveValueWithCommentMinMaxInfDefault(shots_period_ms, "Interval between shots in burst fire, ms");
    SaveValueWithCommentMinMaxInfDefault(magaz_reload_delay, "Interval between autoreloading of magazines, s");
    SaveValueWithCommentMinMaxInfDefault(min_delay_btw_hits, "Minimum delay between hits loss, s (when 0, it is possible to loose all within a second)");
    SaveValueWithCommentMinMaxInfDefault(pulse_len_hit_ms, "Duration of side LEDs blink in case of loss a hit");
    // IRTX
    f_printf(&CommonFile, "\r\n[IRTX]\r\n");
    SaveValueWithCommentMinMaxDefault(ir_tx_pwr,  "Power of IR LED impulse");
    SaveValueWithCommentMinMaxDefault(ir_tx_freq, "Carrier frequency of IR LED");
    f_printf(&CommonFile, "# PktType to transmit: SHOT is 0x0000, RESET is 0x8305\r\n");
    f_printf(&CommonFile, "%S = 0x%04X\r\n\r\n", pkt_type.Name, pkt_type.v);
    CloseFile(&CommonFile);
    return retv::Ok;
}

void Settings::SetAllToDefault() {
    Value_t *Arr = (Value_t*)this;
    for(uint32_t i=0; i<SETTINGS_CNT; i++) Arr[i].v = Arr[i].v_default;
}
