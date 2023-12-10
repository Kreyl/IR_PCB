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
#include "app.h"

#if 1 // ======================== Variables and defines ========================
// Forever
EvtMsgQ_t<EvtMsg_t, MAIN_EVT_Q_LEN> EvtQMain;
static const UartParams_t CmdUartParams(115200, CMD_UART_PARAMS);
CmdUart_t Uart{CmdUartParams};
void OnCmd(Shell_t *PShell);

LedSmooth_t Lumos{LUMOS_PIN};
LedSmooth_t SideLEDs[SIDE_LEDS_CNT] = { {LED_PWM1}, {LED_PWM2}, {LED_PWM3}, {LED_PWM4} };
LedSmooth_t FrontLEDs[FRONT_LEDS_CNT] = { {LED_FRONT1}, {LED_FRONT2} };

static const NpxParams NParams{NPX_PARAMS, NPX_DMA, 17, NpxParams::ClrType::RGB};
Neopixels_t NpxLeds{&NParams};

Beeper_t Beeper {BEEPER_PIN};

SpiFlash_t SpiFlash(SPI0);
FATFS FlashFS;

EvtTimer_t TmrUartCheck(TIME_MS2I(UART_RX_POLL_MS), EvtId::UartCheckTime, EvtTimer_t::Type::Periodic);
EvtTimer_t TmrTesting(TIME_MS2I(540), EvtId::TestingTime, EvtTimer_t::Type::Periodic);

// Testing variables
static const int32_t sinbuf[] = {
0, 1389, 2736, 3999, 5142, 6128, 6928, 7517, 7878, 8000,
7878, 7517, 6928, 6128, 5142, 4000, 2736, 1389, 0, -1389,
-2736, -4000, -5142, -6128, -6928, -7517, -7878, -8000, -7878, -7517,
-6928, -6128, -5142, -3999, -2736, -1389};
#define SIN_SZ    36

#define TESTING_NPX_BRT     72
bool IsTesting = false;
uint32_t TstIndx = 0;
#endif

void Reboot() {
    // Use watchdog to reset
    __disable_irq();
    while(true);
}

void TestIrRxCallbackI(uint32_t Rcvd) { PrintfI("RX: 0x%X\r", Rcvd); }

void I2SDmaDoneCbI() {
    if(IsTesting and TstIndx == 0) {
        Codec::TransmitBuf((void*)sinbuf, SIN_SZ);
    }
}

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

void main(void) {
    Watchdog::InitAndStart(999);
    InitClk();
    // ==== Disable JTAG ====
    RCU->EnAFIO();
    AFIO->DisableJtagDP(); // Disable JTAG, leaving SWD. Otherwise PB3 & PB4 are occupied by JTDO & JTRST

    // ==== UART, RTOS & Event queue ====
    Uart.Init();
    Sys::Init();
    EvtQMain.Init();
    Printf("\r%S %S\r\n", APP_NAME, XSTRINGIFY(BUILD_TIME));
    Clk::PrintFreqs();

    // ==== LEDs ====
    Lumos.Init();
    Lumos.StartOrRestart(lsqFadeIn);
    for(auto &Led : SideLEDs) Led.Init();
    for(auto &Led : FrontLEDs) Led.Init();
    NpxLeds.Init();
    NpxLeds.SetAll(clBlack);
    NpxLeds.SetCurrentColors();

    // ==== Audio ====
    Codec::Init();
    Beeper.Init();

    // ==== Spi Flash, MsdGlue, filesystem ====
    AFIO->RemapSPI0_PB345();
    SpiFlash.Init();
    SpiFlash.Reset();
    SpiFlash_t::MemParams_t mp = SpiFlash.GetParams();
    MsdMem::BlockCnt = mp.SectorCnt;
    MsdMem::BlockSz = mp.SectorSz;
    Printf("Flash: %u sectors of %u bytes\r", mp.SectorCnt, mp.SectorSz);
    if(mp.SectorCnt == 0 or mp.SectorSz == 0) Reboot();
    // Init filesystem
    if(f_mount(&FlashFS, "", 0) != FR_OK) Printf("FS error\r\n");

    // ==== USB ====
    UsbMsdCdc.Init();
    UsbMsdCdc.Connect();

    // ==== IR ====
    irLed::Init();
    irRcvr::Init(IrRxCallbackI);

    // ==== App ====
    Settings.Load();
    AppInit();

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

            case EvtId::TestingTime:
                if(IsTesting) {
                    // Npx
                    switch(TstIndx) {
                        case 0: NpxLeds.SetAll({TESTING_NPX_BRT, 0, 0}); break;
                        case 1: NpxLeds.SetAll({0, TESTING_NPX_BRT, 0}); break;
                        case 2: NpxLeds.SetAll({0, 0, TESTING_NPX_BRT}); break;
                        case 3: NpxLeds.SetAll({TESTING_NPX_BRT, TESTING_NPX_BRT, TESTING_NPX_BRT}); break;
                        default: NpxLeds.SetAll(clCyan); break;
                    }
                    NpxLeds.SetCurrentColors();
                    // Send IR pkt
                    irLed::TransmitWord(0xCA11, 16, 180, nullptr);
                    // Side Leds
                    SideLEDs[TstIndx].StartOrRestart(lsqFadeInOut);
                    // Front LEDs
                    FrontLEDs[TstIndx & 0x01].StartOrRestart(lsqFadeInOut);
                    // Gpios
                    Gpio::Set(Gpio1, TstIndx == 0);
                    Gpio::Set(Gpio2, TstIndx == 1);
                    Gpio::Set(Gpio3, TstIndx == 2);
                    Gpio::Set(Gpio4, TstIndx == 3);
                    // Buzzer
                    if(TstIndx == 0 and !Beeped) {
                        Beeper.StartOrRestart(bsqBeepBeep);
                        Beeped = true;
                    }
                    // Increment TstIndx
                    if(TstIndx < 3) TstIndx++;
                    else TstIndx = 0;
                }
                break;

            case EvtId::UsbReady:
                Printf("Usb ready\r");
                CTC->Enable(); // Start autotrimming of IRC48M
                break;

            default: break;
        } // switch
    }
}

void OnCmd(Shell_t *PShell) {
    Cmd_t *PCmd = &PShell->Cmd;
    // Handle command
    if(PCmd->NameIs("Ping")) PShell->Ok();
    else if(PCmd->NameIs("Version")) PShell->Print("%S %S\r", APP_NAME, XSTRINGIFY(BUILD_TIME));

    else if(PCmd->NameIs("Print")) {
        uint32_t N;
        if(PCmd->GetNext(&N) == retv::Ok) {
            uint32_t i = 1;
            uint32_t N10 = N / 10;
            while(N10--) {
                PShell->Print("%08u\r\n", i++);
                N -= 10;
            }
            i = 1;
            while(N--) PShell->Print("%u", i++);
//            PShell->PrintEOL();
        }
        else PShell->BadParam();
    }

    else if(PCmd->NameIs("Test")) {
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

    else if(PCmd->NameIs("Reboot")) Reboot();

    // ==== App ====
    else if(PCmd->NameIs("GetSta")) {
        PShell->Print("Hits: %d; Rnds: %d; mgzs: %d\r", HitCnt, RoundsCnt, MagazinesCnt);
    }
    else if(PCmd->NameIs("Restore")) {
        Reset();
        PShell->Ok();
    }

    else if(PCmd->NameIs("GetSettings")) {
        Value_t *Arr = (Value_t*)&Settings;
        for(uint32_t i=0; i<SETTINGS_CNT; i++, Arr++) {
            PShell->Print("%*S = %4u; Min = %u; Max = %4u; default = %4u\r\n",
                    16, Arr->Name, Arr->v, Arr->Min, Arr->Max, Arr->Default);
        }
    }

    else if(PCmd->NameIs("Set")) {
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

    else if(PCmd->NameIs("SaveSettings")) {
        if(Settings.Save() == retv::Ok) PShell->Print("Saved\r\n");
        else PShell->Print("Saving fail\r\n");
    }

    else if(PCmd->NameIs("LoadSettings")) {
        Settings.Load();
        Reset();
    }

    // ==== Debug ====
    else if(PCmd->NameIs("CtrlSet")) {
        uint32_t In[2] = { 0, 0, };
        for(int i=0; i<2; i++) {
            if(PCmd->GetNext(&In[i]) != retv::Ok) break;
        }
        SetInputs(In);
        PShell->Ok();
    }

    else if(PCmd->NameIs("irtx")) {
        uint32_t Word, Pwr, BitCnt;
        if(PCmd->GetNext(&Word) != retv::Ok) { PShell->BadParam(); return; }
        if(PCmd->GetNext(&BitCnt) != retv::Ok) { PShell->BadParam(); return; }
        if(PCmd->GetNext(&Pwr) != retv::Ok) { PShell->BadParam(); return; }
        irLed::TransmitWord(Word, BitCnt, Pwr, nullptr);
        PShell->Ok();
    }

//    else if(PCmd->NameIs("IRTX")) {
//        uint16_t w, Pwr;
//        if(PCmd->GetParams<uint16_t>(2, &w, &Pwr) == retv::Ok) {
//            irLed::TransmitWord(w, 16, Pwr, nullptr);
//            PShell->Ok();
//        }
//        else PShell->BadParam();
//    }

    else if(PCmd->NameIs("Npx")) {
        Color_t clr;
        if(PCmd->GetClrRGB(&clr) == retv::Ok) NpxLeds.SetAll(clr);
//        uint32_t i=0;
//        while(PCmd->GetClrRGB(&clr) == retv::Ok) NpxLeds.ClrBuf[i++] = clr;
        NpxLeds.SetCurrentColors();
    }

    else PShell->CmdUnknown();
}
