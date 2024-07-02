#include "Settings.h"
#include "gd_lib.h"
#include "shell.h"
#include "kl_fs_utils.h"

Settings settings;

void Settings::Load() {
    for(ValueBase* const pval : values_arr) {
        int32_t v;
        if(ini::Read(SETTINGS_FILENAME, pval->section, pval->name, &v) != retv::Ok or
                pval->CheckAndSetIfOk(v) != retv::Ok) {
            SetAllToDefault();
            Printf("Bad %S %S. Default settings loaded\r\n", pval->section, pval->name);
            return;
        }
    }
    Printf("Settings loaded\r\n");
}

void SaveValueWithMinMax(ValueMinMaxDef &value) {
    f_printf(&common_file, "# Min=%D, Max=%D\r\n", value.v_min, value.v_max);
    f_printf(&common_file, "%S = %D\r\n", value.name, value.v);
}

void SaveValueWithMinMaxInfDefault(ValueMinMaxDef &value) {
    f_printf(&common_file, "# Min=%D, Max=%D, Infininty=%D, Default=%D\r\n", value.v_min, value.v_max, value.v_max+1, value.v_default);
    f_printf(&common_file, "%S = %D\r\n", value.name, value.v);
}

void SaveValueWithCommentMinMaxInfDefault(ValueMinMaxDef &value, const char* comment) {
    f_printf(&common_file, "# %S\r\n", comment);
    f_printf(&common_file, "# Min=%D, Max=%D, Infininty=%D, Default=%D\r\n", value.v_min, value.v_max, value.v_max+1, value.v_default);
    f_printf(&common_file, "%S = %D\r\n\r\n", value.name, value.v);
}

void SaveValueWithCommentMinMaxDefault(ValueMinMaxDef &value, const char* comment) {
    f_printf(&common_file, "# %S\r\n", comment);
    f_printf(&common_file, "# Min=%D, Max=%D, Default=%D\r\n", value.v_min, value.v_max, value.v_default);
    f_printf(&common_file, "%S = %D\r\n\r\n", value.name, value.v);
}


retv Settings::Save() {
    if(TryOpenFileRewrite(SETTINGS_FILENAME, &common_file) != retv::Ok) return retv::Fail;
    // Description
    f_printf(&common_file, "# IR PCB v3.2 Settings.ini\r\n\r\n");
    // IDs
    f_printf(&common_file, "[IDs]\r\n");
    SaveValueWithMinMax(player_id);
    SaveValueWithMinMax(team_id);
    // Counts
    f_printf(&common_file, "\r\n[Counts]\r\n");
    SaveValueWithMinMaxInfDefault(hit_cnt);
    SaveValueWithMinMaxInfDefault(rounds_in_magaz);
    SaveValueWithMinMaxInfDefault(magazines_cnt);
    // Delays
    f_printf(&common_file, "\r\n[Delays]\r\n");
    SaveValueWithCommentMinMaxInfDefault(shots_period_ms, "Interval between shots in burst fire, ms");
    SaveValueWithCommentMinMaxInfDefault(magaz_reload_delay, "Interval between autoreloading of magazines, s");
    SaveValueWithCommentMinMaxInfDefault(min_delay_btw_hits, "Minimum delay between hits loss, s (when 0, it is possible to loose all within a second)");
    SaveValueWithCommentMinMaxInfDefault(pulse_len_hit_ms, "Duration of side LEDs blink in case of loss a hit");
    // IRTX
    f_printf(&common_file, "\r\n[IRTX]\r\n");
    SaveValueWithCommentMinMaxDefault(ir_tx_pwr,  "Power of IR LED impulse");
    SaveValueWithCommentMinMaxDefault(ir_tx_freq, "Carrier frequency of IR LED");
    f_printf(&common_file,
            "# PktType to transmit:\r\n"
            "#   SHOT is 0x0000 (PlayerID, TeamID and Damage added automatically"
            "#   RESET is 0x8305\r\n"
            "#   AddHealth is 0x8000 (number of added health points set in the amount value)\r\n"
            "#   AddCartridges is 0x8100 (number of added cartridges set in the amount value)\r\n"
            );
    f_printf(&common_file, "%S = 0x%04X\r\n\r\n", pkt_type.name, pkt_type.v);
    // Special case: damage
    f_printf(&common_file, "# Hits Damage in 'Shot' pkt. Unusable for other pkt types.\r\n"
            "Possible values: 1,2,4,5,7,10,15,17,20,25,30,35,40,50,75,100\r\n");
    f_printf(&common_file, "%S = 0x%04X\r\n\r\n", tx_damage.name, tx_damage.v);
    // Amount
    SaveValueWithCommentMinMaxDefault(tx_amount,
            "For pkt types of 0x80 and 0x81: number of units added");
    CloseFile(&common_file);
    return retv::Ok;
}
