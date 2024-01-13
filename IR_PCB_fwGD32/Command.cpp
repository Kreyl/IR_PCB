/*
 * Command.cpp
 *
 *  Created on: 13/01/2024г.
 *        Author: layst
 */

#include "shell.h"
#include "ir.h"
#include "max98357.h"
#include "app.h"
#include "Settings.h"
#include "ws2812bTim.h"

typedef void (*ftVoidPShell)(Shell_t *PShell);
extern const char* FWVersion;
extern Neopixels_t NpxLeds;

extern void Reboot();

class ShellCmd_t {
public:
    const char *Name, *Help;
    ftVoidPShell Dispatcher;
    ShellCmd_t(const char* AName, ftVoidPShell ADispatcher, const char* AHelp) :
        Name(AName), Help(AHelp), Dispatcher(ADispatcher) {}
};

// Dispatchers
static void DoPing(Shell_t *PShell) { PShell->Ok(); }
static void DoVersion(Shell_t *PShell) { PShell->Print("%S %S\r", APP_NAME, FWVersion); }
static void DoReboot(Shell_t *PShell) { Reboot(); }

#if 1 // ====================== Testing related ================================
extern bool IsTesting, Beeped;
extern uint32_t TstIndx;
extern EvtTimer_t TmrTesting;

static const int32_t sinbuf[] = {
0, 1389, 2736, 3999, 5142, 6128, 6928, 7517, 7878, 8000,
7878, 7517, 6928, 6128, 5142, 4000, 2736, 1389, 0, -1389,
-2736, -4000, -5142, -6128, -6928, -7517, -7878, -8000, -7878, -7517,
-6928, -6128, -5142, -3999, -2736, -1389};
#define SIN_SZ    36

void TestIrRxCallbackI(uint32_t Rcvd) { PrintfI("RX: 0x%X\r", Rcvd); }
void I2SDmaDoneCbI() {
    if(IsTesting and TstIndx == 0) {
        Codec::TransmitBuf((void*)sinbuf, SIN_SZ);
    }
}

static void DoTest(Shell_t *PShell) {
    IsTesting = true;
    Beeped = false;
    irRcvr::SetCallback(TestIrRxCallbackI);
    // Gpios
    Gpio::SetupOut(Gpio1, Gpio::PushPull, Gpio::speed2MHz);
    Gpio::SetupOut(Gpio2, Gpio::PushPull, Gpio::speed2MHz);
    Gpio::SetupOut(Gpio3, Gpio::PushPull, Gpio::speed2MHz);
    Gpio::SetupOut(Gpio4, Gpio::PushPull, Gpio::speed2MHz);
    // Audio codec
    if(Codec::SetupSampleRate(48000) == retv::Ok) {
        Codec::I2SDmaDoneCbI = I2SDmaDoneCbI;
        Codec::TransmitBuf((void*)sinbuf, SIN_SZ);
    }
    else Printf("FS setup fail\r");
    TmrTesting.StartOrRestart();
}
#endif

static void GetSta(Shell_t *PShell) { PShell->Print("Hits: %d; Rnds: %d; mgzs: %d\r", HitCnt, RoundsCnt, MagazinesCnt); }
static void Restore(Shell_t *PShell) {
    Reset();
    PShell->Ok();
}

static void GetSettings(Shell_t *PShell) {
    Value_t *Arr = (Value_t*)&Settings;
    for(uint32_t i=0; i<SETTINGS_CNT; i++, Arr++) {
        PShell->Print("%*S = %4u; Min = %u; Max = %4u; default = %4u\r\n",
                16, Arr->Name, Arr->v, Arr->Min, Arr->Max, Arr->Default);
    }
}

static void Set(Shell_t *PShell) {
    Cmd_t *PCmd = &PShell->Cmd;
    const char* Name;
    uint32_t v, N = 0;
    bool Found;
    // Get pairs of values
    while((((Name = PCmd->GetNextString()) != nullptr) and PCmd->GetNext(&v) == retv::Ok)) {
        Found = false;
        Value_t *Arr = (Value_t*)&Settings;
        for(uint32_t i=0; i<SETTINGS_CNT; i++, Arr++) { // Find by name
            if(kl_strcasecmp(Name, Arr->Name) == 0) {
                if(Arr->CheckAndSetIfOk(v)) {
                    PShell->Print("%S = %u\r\n", Name, v);
                    N++;
                    Found = true;
                    break;
                }
                else {
                    PShell->Print("%S BadValue: %u\r\n", Name, v);
                    return;
                }
            } // if
        } // for
        if(!Found) {
            PShell->Print("BadName: %S\r\n", Name);
            break;
        }
    } // while
    PShell->Print("Set %u values\r\n", N);
    Reset();
}

static void SaveSettings(Shell_t *PShell) {
    if(Settings.Save() == retv::Ok) PShell->Print("Saved\r\n");
    else PShell->Print("Saving fail\r\n");
}
static void LoadSettings(Shell_t *PShell) {
    Settings.Load();
    Reset();
}


#if 0 // ==== Debug ====
static void CtrlSet(Shell_t *PShell) {
    uint32_t In[2] = { 0, 0, };
    if(PShell->Cmd.GetParams<uint32_t>(2, &In[0], &In[1]) == retv::Ok) {
        SetInputs(In);
        PShell->Ok();
    }
    else PShell->BadParam();
}

static void IrTx(Shell_t *PShell) {
    Cmd_t *PCmd = &PShell->Cmd;
    uint32_t Word, Pwr, BitCnt;
    if(PCmd->GetNext(&Word) != retv::Ok) { PShell->BadParam(); return; }
    if(PCmd->GetNext(&BitCnt) != retv::Ok) { PShell->BadParam(); return; }
    if(PCmd->GetNext(&Pwr) != retv::Ok) { PShell->BadParam(); return; }
    irLed::TransmitWord(Word, BitCnt, Pwr, nullptr);
    PShell->Ok();
}

static void Npx(Shell_t *PShell) {
    Cmd_t *PCmd = &PShell->Cmd;
    Color_t clr;
    if(PCmd->GetClrRGB(&clr) == retv::Ok) NpxLeds.SetAll(clr);
//        uint32_t i=0;
//        while(PCmd->GetClrRGB(&clr) == retv::Ok) NpxLeds.ClrBuf[i++] = clr;
    NpxLeds.SetCurrentColors();
}
#endif

// Commands
ShellCmd_t Cmds[] = {
        {"Ping",    DoPing,    "Just to ask if anyone is there"},
        {"Version", DoVersion, "Show firmware version"},
        {"Test",    DoTest,    "Start hardware testing"},
        {"Reboot",  DoReboot,  "Reboot MCU"},
        // ==== App ====
        {"GetSta",  GetSta,    "Get current status of device (hit count etc.)"},
        {"Restore", Restore,   "Restore hits, rounds, magazines - everything"},
        {"GetSettings", GetSettings, "Get current values from Settings"},
        {"Set", Set, "Set one or more value of settings, ex: 'Set HitCnt 18, TeamID 7'. This will not save to Flash!"},
        {"SaveSettings", SaveSettings, "Save current settings to Flash"},
        {"LoadSettings", LoadSettings, "Load settings from Flash"},
        // ==== Debug ====
//        {"CtrlSet", CtrlSet,   "Set two control pins to specified value, ex: 'CtrlSet 1, 0'"},
//        {"IrTx",    IrTx,      "'IrTx Word, Pwr, BitCnt': transmit MSB bits by IR LED at specified power. Ex: 'IRTx 0xAA50 90 12"},
//        {"NPX",     Npx,       "Set all NPX LEDs to specified RGB color, ex: 'Npx 18 255 180'"},
};

// Processing
void OnCmd(Shell_t *PShell) {
    Cmd_t *PCmd = &PShell->Cmd;
    // Show help if needed
    if(PCmd->NameIs("Help")) {
        PShell->Print("Commands available:\r\n");
        for(auto &SCmd : Cmds) PShell->Print("%S: %S\r\n", SCmd.Name, SCmd.Help);
    }
    else {
        for(auto &SCmd : Cmds) {
            if(PCmd->NameIs(SCmd.Name)) {
                if(SCmd.Dispatcher == nullptr) PShell->Print("NoDispatsher\r\n");
                else SCmd.Dispatcher(PShell);
                return;
            }
        }
        PShell->CmdUnknown();
    }
}


