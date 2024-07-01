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
#include "ir_pkt.h"

#if 1 // ======================== Variables and defines ========================
// Forever
const char* FWVersion = XSTRINGIFY(BUILD_TIME);
EvtMsgQ<EvtMsg_t, MAIN_EVT_Q_LEN> EvtQMain;
static const UartParams_t CmdUartParams(115200, CMD_UART_PARAMS);
CmdUart_t Uart{CmdUartParams};
extern void OnCmd(Shell *PShell); // See Command.cpp

LedBlinker sys_LED{LUMOS_PIN};
LedSmooth side_LEDs[SIDE_LEDS_CNT] = { {LED_PWM1}, {LED_PWM2}, {LED_PWM3}, {LED_PWM4} };
LedSmooth front_LEDs[FRONT_LEDS_CNT] = { {LED_FRONT1}, {LED_FRONT2} };

static const NpxParams nparams{NPX_PARAMS, NPX_DMA, 17, NpxParams::ClrType::RGB};
Neopixels_t NpxLeds{&nparams};

Beeper beeper {BEEPER_PIN};

SpiFlash_t SpiFlash(SPI0);
FATFS FlashFS;

EvtTimer tmr_uart_check(TIME_MS2I(UART_RX_POLL_MS), EvtId::UartCheckTime, EvtTimer::Type::Periodic);
EvtTimer tmr_testing(TIME_MS2I(540), EvtId::TestingTime, EvtTimer::Type::Periodic);

// Testing variables
#define TESTING_NPX_BRT     72
bool is_testing = false;
uint32_t tst_indx = 0;
bool beeped = false;
#endif

// Use watchdog to reset
void Reboot() {
    __disable_irq();
    while(true);
}

static inline void InitClk() {
    // Initial initialization
    FMC->EnableCashAndPrefetch();
    RCU->EnPwrMgmtUnit();
    FMC->SetLatency(50); // 50 MHz required for NPX LEDs
    // Init Crystal
    retv state_xtal = RCU->EnableXTAL();
    if(state_xtal == retv::Ok) {
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
        if(state_xtal != retv::Ok) {
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

void main(void) {
//    Watchdog::InitAndStart(999);
    InitClk();
    // ==== Disable JTAG ====
    RCU->EnAFIO();
    AFIO->DisableJtagDP(); // Disable JTAG, leaving SWD. Otherwise PB3 & PB4 are occupied by JTDO & JTRST

    // ==== UART, RTOS & Event queue ====
    Uart.Init();
    Sys::Init();
    EvtQMain.Init();
    Printf("\r%S %S\r\n", APP_NAME, FWVersion);
    Clk::PrintFreqs();

    // ==== LEDs ====
    sys_LED.Init();
    sys_LED.StartOrRestart(lbsqStart);
    for(auto &led : side_LEDs) led.Init();
    for(auto &led : front_LEDs) led.Init();
    NpxLeds.Init();
    NpxLeds.SetAll(clBlack);
    NpxLeds.SetCurrentColors();

    // ==== Audio ====
    Codec::Init();
    beeper.Init();

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

    // ==== IR ==== TX LED SetFreq is called within App Reset, which is called within AppInit
    irLed::Init();
    irRcvr::Init(IrRxCallbackI);

    // ==== App ====
    settings.Load();
    Printf("Pkt type: 0x%04X\r", settings.pkt_type.v);
    AppInit();

    // ==== Main evt cycle ====
    tmr_uart_check.StartOrRestart();
    while(true) {
        EvtMsg_t msg = EvtQMain.Fetch(TIME_INFINITE);
        switch(msg.ID) {
            case EvtId::UartCheckTime:
                Watchdog::Reload();
                while(Uart.TryParseRxBuff() == retv::Ok) OnCmd((Shell*)&Uart);
                break;

            case EvtId::UsbCdcDataRcvd:
                while(UsbMsdCdc.TryParseRxBuff() == retv::Ok) OnCmd((Shell*)&UsbMsdCdc);
                break;

            case EvtId::TestingTime:
                if(is_testing) {
                    // Npx
                    switch(tst_indx) {
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
                    side_LEDs[tst_indx].StartOrRestart(lsqFadeInOut);
                    // Front LEDs
                    front_LEDs[tst_indx & 0x01].StartOrRestart(lsqFadeInOut);
                    // Gpios
                    Gpio::Set(Gpio1, tst_indx == 0);
                    Gpio::Set(Gpio2, tst_indx == 1);
                    Gpio::Set(Gpio3, tst_indx == 2);
                    Gpio::Set(Gpio4, tst_indx == 3);
                    // Buzzer
                    if(tst_indx == 0 and !beeped) {
                        beeper.StartOrRestart(bsqBeepBeep);
                        beeped = true;
                    }
                    // Increment TstIndx
                    if(tst_indx < 3) tst_indx++;
                    else tst_indx = 0;
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
