#include "gd_lib.h"
#include "gd_uart.h"
#include "yartos.h"
#include "MsgQ.h"
#include "usb_msdcdc.h"
#include "SpiFlash.h"
#include "led.h"
#include "beeper.h"
#include "Sequences.h"
#include "ir.h"
#include "ws2812bTim.h"
#include "max98357.h"
#include "mem_msd_glue.h"
#include "Settings.h"
#include "kl_fs_utils.h"

#include "Flame.h"

#if 1 // ======================== Variables and defines ========================
// Forever
EvtMsgQ_t<EvtMsg_t, MAIN_EVT_Q_LEN> EvtQMain;
static const UartParams_t CmdUartParams(115200, CMD_UART_PARAMS);
CmdUart_t Uart{CmdUartParams};
void OnCmd(Shell_t *PShell);

LedSmooth_t Lumos{LUMOS_PIN};

static const NpxParams NParams{NPX_PARAMS, NPX_DMA, NPX_LED_CNT_MAX, NpxParams::ClrType::RGB};
Neopixels_t NpxLeds{&NParams};
Settings_t Settings;


EvtTimer_t TmrUartCheck(TIME_MS2I(UART_RX_POLL_MS), EvtId::UartCheckTime, EvtTimer_t::Type::Periodic);
EvtTimer_t TmrEverySecond(TIME_MS2I(999), EvtId::EverySecond, EvtTimer_t::Type::Periodic);
#endif

void Reboot() {
    // Use watchdog to reset
    __disable_irq();
    while(true);
}

void TestIrRxCallbackI(uint32_t Rcvd) { PrintfI("RX: 0x%X\r", Rcvd); }


static inline void InitClk() {
    // Initial initialization
    FMC->EnableCashAndPrefetch();
    RCU->EnPwrMgmtUnit();
    FMC->SetLatency(50); // 50 MHz required for NPX LEDs
    // Init Crystal
    retv rXtal = RCU->EnableXTAL();
    if(rXtal == retv::Ok) {
        RCU->SetPllPresel_XTAL();
        RCU->SetPrediv0Sel_XTALorIRC48M(); // XTAL is a source
        // 12MHz div 6 = 2MHz; 2MHz *25 = 50MHz. PLL input freq must be in [1; 25] MHz, 8MHz typical
        RCU->SetPrediv0(6);
        RCU->SetPllSel_Prediv0();
        RCU->SetPllMulti(PllMulti::mul25);
        // Switch clk
        if(RCU->EnablePll() == retv::Ok) {
            RCU->SetAhbPrescaler(AhbPsc::div1);
            RCU->SwitchCkSys2PLL();
        }
    }
    // Setup IRC48M as usb clk and enable CTC trimmer
    if(RCU->EnableIRC48M() == retv::Ok) {
        RCU->SetCK48MSel_CKIRC48M();   // Use IRC48M as clk src instead of PLL
        RCU->EnCTC();              // Enable trimmer
        // Setup trimmer: SOF freq is 1 kHz, rising front, no prescaler, input is UsbSOF
        CTC->Setup(CTC_REFPOL_RISING, CTC_REFSEL_USBSOF, CTC_REFPSC_DIV_1, 1000);
        // If XTAL failed, use IRC as system clock source
        if(rXtal != retv::Ok) {
            RCU->SetPllPresel_CKIRC48M(); // IRC48M is a source
            RCU->SetPrediv0Sel_XTALorIRC48M(); // IRC48M is a source
            // 48MHz div 12 = 4MHz; 4MHz *25 = 100MHz; 100MHz / 2 = 50MHz. PLL input freq must be in [1; 25] MHz, 8MHz typical
            RCU->SetPrediv0(12);
            RCU->SetPllSel_Prediv0();
            RCU->SetPllMulti(PllMulti::mul25);
            // Switch clk
            if(RCU->EnablePll() == retv::Ok) {
                RCU->SetAhbPrescaler(AhbPsc::div2);
                RCU->SwitchCkSys2PLL();
            }
        }
    }
    Clk::UpdateFreqValues();
}

bool Beeped = false;

//FlameSettings_t Setup = {
//    .Core = {
//            .Sz = 4,
//            .ClrHMin = 0, .ClrHMax = 60,
//            .ClrV = 27
//    },
//    .Sparks = {
//            .Cnt = 7,
//            .TailLen = 4,
//            .ClrHMin = 0, .ClrHMax = 60,
//            .ClrV = 100,
//            .DelayBeforeRestart = 630,
//            .AccMin = 9, .AccMax = 27,
//            .StartDelayMin = 99, .StartDelayMax = 153,
//            .Mode = 0
//    }
//};

//21 280 280 18 0
//7 9 280 280 100 0 0 630 9 27 99 153 0

FlameSettings_t Setup = {
    .Core = {
            .Sz = 9,
            .ClrHMin = 290, .ClrHMax = 290,
            .ClrV = 27
    },
    .Sparks = {
            .Cnt = 7,
            .TailLen = 9,
            .ClrHMin = 290, .ClrHMax = 290,
            .ClrV = 100,
            .DelayBeforeRestart = 630,
            .AccMin = 9, .AccMax = 18,
            .StartDelayMin = 99, .StartDelayMax = 153,
            .Mode = 0
    }
};


void main(void) {
//    Watchdog::InitAndStart(999);
    InitClk();
    // ==== Disable JTAG ====
    RCU->EnAFIO();
    AFIO->DisableJtagDP(); // Disable JTAG, leaving SWD and SWO. Otherwise PB3 & PB4 are occupied by JTDO & JTRST

    // ==== UART, RTOS & Event queue ====
    Uart.Init();
    Sys::Init();
    EvtQMain.Init();
    Printf("\r%S %S\r\n", APP_NAME, XSTRINGIFY(BUILD_TIME));
    Clk::PrintFreqs();

    // ==== LEDs ====
    Lumos.Init();
    Lumos.StartOrRestart(lsqFadeIn);
    NpxLeds.Init();
    NpxLeds.SetAll(clBlack);
    NpxLeds.SetCurrentColors();
    Flames.SetNewSettings(Settings.GetCurrent());
    Flames.SetNewSettings(Setup);
    Flames.Init();
    Flames.FadeIn();

//    TmrEverySecond.StartOrRestart();

    // ==== USB ====
//    UsbMsdCdc.Init();
//    UsbMsdCdc.Connect();

//    Gpio::SetupOut(GPIOB, 3, Gpio::PushPull);
//    Gpio::SetHi(GPIOB, 3);

    // ==== Main evt cycle ====
    TmrUartCheck.StartOrRestart();
    while(true) {
        EvtMsg_t Msg = EvtQMain.Fetch(TIME_INFINITE);
        switch(Msg.ID) {
            case EvtId::UartCheckTime:
                Watchdog::Reload();
                while(Uart.TryParseRxBuff() == retv::Ok) OnCmd((Shell_t*)&Uart);

                break;

            case EvtId::UsbCdcDataRcvd:
                while(UsbMsdCdc.TryParseRxBuff() == retv::Ok) OnCmd((Shell_t*)&UsbMsdCdc);
                break;

            case EvtId::EverySecond: {
            } break;

            case EvtId::UsbReady:
                Printf("Usb ready\r");
                CTC->Enable(); // Start autotrimming of IRC48M
                break;

            default: break;
        } // switch
    }
}

//template <typename T>
//static inline T Proportion(T MinX, T MaxX, T MinY, T MaxY, T x) {
//    return (((x - MaxX) * (MaxY - MinY)) / (MaxX - MinX)) + MaxY;
//}

static const uint32_t gamma_table[12] = {
    2,5,9,14,20,28,37,47,58,71,85,100,
};

retv GetSetup(Cmd_t *PCmd, FlameSettings_t &FSett) {
    if(     PCmd->GetNextU8(&FSett.Core.Sz) == retv::Ok and
            PCmd->GetNextU16(&FSett.Core.ClrHMin) == retv::Ok and
            PCmd->GetNextU16(&FSett.Core.ClrHMax) == retv::Ok and
            PCmd->GetNextU8(&FSett.Core.ClrV) == retv::Ok and
            PCmd->GetNextU8(&FSett.Sparks.Cnt) == retv::Ok and
            PCmd->GetNextU8(&FSett.Sparks.TailLen) == retv::Ok and
            PCmd->GetNextU16(&FSett.Sparks.ClrHMin) == retv::Ok and
            PCmd->GetNextU16(&FSett.Sparks.ClrHMax) == retv::Ok and
            PCmd->GetNextU8(&FSett.Sparks.ClrV) == retv::Ok and
            PCmd->GetNextU16(&FSett.Sparks.DelayBeforeRestart) == retv::Ok and
            PCmd->GetNextI16(&FSett.Sparks.AccMin) == retv::Ok and
            PCmd->GetNextI16(&FSett.Sparks.AccMax) == retv::Ok and
            PCmd->GetNextI16(&FSett.Sparks.StartDelayMin) == retv::Ok and
            PCmd->GetNextI16(&FSett.Sparks.StartDelayMax) == retv::Ok and
            PCmd->GetNextU8(&FSett.Sparks.Mode) == retv::Ok
        ) return retv::Ok;
    else return retv::Fail;
}

void OnCmd(Shell_t *PShell) {
    Cmd_t *PCmd = &PShell->Cmd;
    // Handle command
    if(PCmd->NameIs("Ping")) PShell->Ok();
    else if(PCmd->NameIs("Version")) PShell->Print("%S %S\r", APP_NAME, XSTRINGIFY(BUILD_TIME));
    else if(PCmd->NameIs("Reboot")) Reboot();

    else if(PCmd->NameIs("Start")) { Flames.Start(); PShell->Ok(); }
    else if(PCmd->NameIs("Stop"))  { Flames.Stop(); PShell->Ok(); }


    else if(PCmd->NameIs("Npx")) {
        Color_t clr;
        if(PCmd->GetClrRGB(&clr) == retv::Ok) NpxLeds.SetAll(clr);
        else PShell->BadParam();
        uint32_t i=0;
        while(PCmd->GetClrRGB(&clr) == retv::Ok) NpxLeds.ClrBuf[i++] = clr;
        NpxLeds.SetCurrentColors();
    }

    else if(PCmd->NameIs("SetFlameLen")) {
        uint32_t flen;
        if(PCmd->GetNext(&flen) == retv::Ok and flen <= FLAME_LEN_MAX) {
            Flames.SetFlameLen(flen);
            PShell->Ok();
        }
        else PShell->BadParam();
    }

    else if(PCmd->NameIs("Grad")) {
        Color_t clr;
        if(PCmd->GetClrRGB(&clr) == retv::Ok) {
            ColorHSV_t hsv;
            hsv.FromRGB(clr);
            for(int32_t i=0; i<Flames.flame_len; i++) {
                hsv.V = gamma_table[Flames.flame_len-i-1];
                NpxLeds.ClrBuf[i] = hsv.ToRGB();
                Printf("%u\r", hsv.V);
            }
            NpxLeds.SetCurrentColors();
        }
        else PShell->BadParam();
    }

    else if(PCmd->NameIs("Setup")) {
        FlameSettings_t FSett;
        if(GetSetup(PCmd, FSett) == retv::Ok) {
            Flames.SetNewSettings(FSett);
            PShell->Ok();
        }
        else PShell->BadParam();
    }

    else if(PCmd->NameIs("BandBrt")) {
        uint32_t brts[3];
        if(PCmd->GetArray(brts, 3) == retv::Ok) {
            Flames.SetBandBrt(brts);
            PShell->Ok();
        }
        else PShell->BadParam();
    }

    else PShell->CmdUnknown();
}
