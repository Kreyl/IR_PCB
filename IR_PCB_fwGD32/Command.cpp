/*
 * Command.cpp
 *
 *  Created on: 13/01/2024г.
 *        Author: Kreyl
 */

#include "shell.h"
#include "max98357.h"
#include "ws2812bTim.h"
#include "led.h"

typedef void (*ftVoidPShell)(Shell *pshell);
extern const char* kBuildTime;
__attribute__((unused))
extern Neopixels npx_leds;

extern void Reboot();

class ShellCmd {
public:
    const char *name, *help;
    ftVoidPShell dispatcher;
    ShellCmd(const char* aname, ftVoidPShell adispatcher, const char* ahelp) :
        name(aname), help(ahelp), dispatcher(adispatcher) {}
};

// Dispatchers
static void DoPing(Shell *pshell) { pshell->Ok(); }
static void DoVersion(Shell *pshell) { pshell->Print("%S %S\r", APP_NAME, kBuildTime); }
static void DoReboot(Shell *pshell) { Reboot(); }

static void DoNpxSet(Shell *pshell) {
    Cmd *pcmd = &pshell->cmd;
    uint32_t indx, r, g, b;
    if(pcmd->GetNext(&indx) == retv::Ok) {
        if(pcmd->GetNext(&r) == retv::Ok and pcmd->GetNext(&g) == retv::Ok and pcmd->GetNext(&b) == retv::Ok) {
            npx_leds.clr_buf[indx].R = r;
            npx_leds.clr_buf[indx].G = g;
            npx_leds.clr_buf[indx].B = b;
            npx_leds.SetCurrentColors();
            pshell->Ok();
        }
        else pshell->BadParam();
    }
}

//static void SetLed(Shell *pshell) {
//    Cmd *pcmd = &pshell->cmd;
//    uint32_t v;
//    if(pcmd->GetNext(&v) == retv::Ok) {
//        side_LEDs[0].Set(v);
//        pshell->Ok();
//    }
//    else pshell->BadParam();
//}

//static void Get(Shell *pshell) {
//    pshell->Print("0x%X %u %u\r", TIM0->INTF, TIM0->CH0CV, TIM0->CH1CV);
//}

// Commands
static const ShellCmd cmds[] = {
        {"Ping",    DoPing,    "Just to ask if anyone is there"},
        {"Version", DoVersion, "Show firmware version"},
        {"Reboot",  DoReboot,  "Reboot MCU"},
        // ==== Debug ====
        {"NPXset",  DoNpxSet,  "Set NPX LED of indx to RGB color, ex: 'NPXset 7, 18 255 180'"},
};

// Processing
void OnCmd(Shell *pshell) {
    Cmd *pcmd = &pshell->cmd;
    // Show help if needed
    if(pcmd->NameIs("Help")) {
        pshell->Print("==== Commands available ====\r\n");
        for(auto &scmd : cmds) pshell->Print("%S: %S\r\n", scmd.name, scmd.help);
    }
    else {
        for(auto &scmd : cmds) {
            if(pcmd->NameIs(scmd.name)) {
                scmd.dispatcher(pshell);
                return;
            }
        }
        pshell->CmdUnknown();
    }
}
