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

void SaveValueWithCommentMinMaxInfDefault(ValueMinMaxDef &value) {
    f_printf(&common_file, "# %S\r\n", value.comment);
    f_printf(&common_file, "# Min=%D, Max=%D, Infininty=%D, Default=%D\r\n", value.v_min, value.v_max, value.v_max+1, value.v_default);
    f_printf(&common_file, "%S = %D\r\n\r\n", value.name, value.v);
}

void SaveValueWithCommentMinMaxDefault(ValueMinMaxDef &value) {
    f_printf(&common_file, "# %S\r\n", value.comment);
    f_printf(&common_file, "# Min=%D, Max=%D, Default=%D\r\n", value.v_min, value.v_max, value.v_default);
    f_printf(&common_file, "%S = %D\r\n\r\n", value.name, value.v);
}

void SaveValueEnable(ValueEnable &value) {
    f_printf(&common_file, "# %S\r\n", value.comment);
    f_printf(&common_file, "# Default=%D\r\n", value.v_default);
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
    SaveValueWithCommentMinMaxDefault(super_damage_id);
    // Counts
    f_printf(&common_file, "\r\n[Counts]\r\n");
    SaveValueWithMinMaxInfDefault(hit_cnt);
    SaveValueWithMinMaxInfDefault(rounds_in_magaz);
    SaveValueWithMinMaxInfDefault(magazines_cnt);
    // Delays
    f_printf(&common_file, "\r\n[Delays]\r\n");
    SaveValueWithCommentMinMaxInfDefault(shots_period_ms);
    SaveValueWithCommentMinMaxInfDefault(magaz_reload_delay_s);
    SaveValueWithCommentMinMaxInfDefault(min_delay_btw_hits_s);
    // IR RX
    f_printf(&common_file, "\r\n[IRRX]\r\n");
    SaveValueWithCommentMinMaxDefault(ir_rx_deviation);
    // IRTX
    f_printf(&common_file, "\r\n[IRTX]\r\n");
    SaveValueWithCommentMinMaxDefault(ir_tx_pwr);
    SaveValueWithCommentMinMaxDefault(ir_tx_freq);
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
            "# Possible values: 1,2,4,5,7,10,15,17,20,25,30,35,40,50,75,100\r\n");
    f_printf(&common_file, "%S = 0x%04X\r\n\r\n", tx_damage.name, tx_damage.v);
    // Amount
    SaveValueWithCommentMinMaxDefault(tx_amount);

    // Gpio
    f_printf(&common_file, "\r\n[Gpio]\r\n");
    f_printf(&common_file, "# %S\r\n", pin_mode_gpio3.comment);
    f_printf(&common_file, "# Default=%D\r\n", pin_mode_gpio3.v_default);
    f_printf(&common_file, "%S = %D\r\n\r\n", pin_mode_gpio3.name, pin_mode_gpio3.v);

    // Research
    f_printf(&common_file, "\r\n[Research]\r\n");
    SaveValueEnable(transmit_what_rcvd);

    CloseFile(&common_file);
    return retv::Ok;
}
