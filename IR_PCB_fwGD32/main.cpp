#include "gd_lib.h"
#include "gd_uart.h"
#include "yartos.h"
#include "MsgQ.h"
#include "usb_msdcdc.h"
#include "SpiFlash.h"
#include "led.h"
#include "beeper.h"
#include "Sequences.h"
#include "ws2812bTim.h"
#include "max98357.h"
#include "mem_msd_glue.h"
#include "kl_fs_utils.h"
#include "ir_pkt.h"

#pragma region // ======================== Variables and defines ========================
// Forever
extern const char *kBuildTime, *kBuildCfgName;
EvtMsgQ<EvtMsg, MAIN_EVT_Q_LEN> evt_q_main;
static const UartParams cmd_uart_params(115200, CMD_UART_PARAMS);
CmdUart dbg_uart{cmd_uart_params};
extern void OnCmd(Shell *pshell); // See Command.cpp

LedBlinker sys_LED{LUMOS_PIN};
LedSmooth side_LEDs[SIDE_LEDS_CNT] = { {LED_PWM1}, {LED_PWM2}, {LED_PWM3}, {LED_PWM4} };
LedSmooth front_LEDs[FRONT_LEDS_CNT] = { {LED_FRONT1}, {LED_FRONT2} };

static const int32_t kLedCnt = 36L;
static const NpxParams nparams{NPX_PARAMS, NPX_DMA, kLedCnt, NpxParams::ClrType::RGB};
Neopixels npx_leds{&nparams};

Beeper beeper {BEEPER_PIN};

SpiFlash spi_flash(SPI0);
FATFS flash_fs;

EvtTimer tmr_uart_check(TIME_MS2I(UART_RX_POLL_MS), EvtId::UartCheckTime, EvtTimer::Type::Periodic);
TimHw tmr_enc{TIM2};
#pragma endregion

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

const int32_t kTicsPerLed = 3L;
const int32_t kTopVal = kLedCnt * kTicsPerLed;

enum class LedsMode { 
    OneColor,
    GradientZeroTop,
    GradientBand
} mode = LedsMode::GradientZeroTop;


const Color_t kOneColor = Color_t(0, 7, 0);
const int32_t kGradStartH = 240, kGradEndH = 0;

void ProcessEncValue(int32_t value) {
    value = kTopVal - value;
    value /= kTicsPerLed;
    // Protect us
    if(value < 0) value = 0;
    if(value >= kLedCnt) value = kLedCnt-1;
    Printf("%d\r", value);

    switch(mode) {
        case LedsMode::OneColor:
            for(int32_t i = 0; i < kLedCnt; i++) {
                npx_leds.clr_buf[i] = i < value ? kOneColor : clBlack; 
            }
            break;

        case LedsMode::GradientZeroTop:
            for(int32_t i = 0; i < kLedCnt; i++) {
                if(i < value) {
                    int32_t H = (i * (kGradEndH - kGradStartH)) / kLedCnt + kGradStartH;
                    if(kGradStartH > kGradEndH) H = 360 - H;
                    npx_leds.clr_buf[i].FromHSV(H, 100, 100);
                }
                else npx_leds.clr_buf[i] = clBlack; 
            }
            break;

        case LedsMode::GradientBand: {

            }
            break;

        default:
            break;
    } // switch
    // Update LEDs 
    npx_leds.SetCurrentColors();
}


void main(void) {
    Watchdog::InitAndStart(999);
    InitClk();
    // ==== Disable JTAG ====
    RCU->EnAFIO();
    AFIO->DisableJtagDP(); // Disable JTAG, leaving SWD. Otherwise PB3 & PB4 are occupied by JTDO & JTRST
  
    // ==== UART, RTOS & Event queue ====
    dbg_uart.Init();
    Sys::Init();
    evt_q_main.Init();
    Printf("\r%S %S\r\n", APP_NAME, kBuildTime);
    Clk::PrintFreqs();

    // ==== LEDs ====
    sys_LED.Init();
    sys_LED.StartOrRestart(lbsqStart);
    for(auto &led : side_LEDs) led.Init();
    for(auto &led : front_LEDs) led.Init();
    npx_leds.Init();
    npx_leds.SetAll(clBlack);
    npx_leds.SetCurrentColors();

    // ==== Audio ====
    Codec::Init();
    beeper.Init();

    // ==== Spi Flash, MsdGlue, filesystem ====
    AFIO->RemapSPI0_PB345();
    spi_flash.Init();
    spi_flash.Reset();
    SpiFlash::MemParams mp = spi_flash.GetParams();
    MsdMem::block_cnt = mp.sector_cnt;
    MsdMem::block_sz = mp.sector_sz;
    Printf("Flash: %u sectors of %u bytes\r", mp.sector_cnt, mp.sector_sz);
    if(mp.sector_cnt == 0 or mp.sector_sz == 0) Reboot();
    // Init filesystem
    if(f_mount(&flash_fs, "", 0) != FR_OK) Printf("FS error\r\n");

    // ==== USB ====
    usb_msd_cdc.Init();
    usb_msd_cdc.Connect();

    // ==== Encoder ====
    Gpio::SetupInput(PA6, Gpio::PullDown);
    Gpio::SetupInput(PA7, Gpio::PullDown);
    tmr_enc.Init();
    // tmr_enc.SetTopValue(0xFFFF);
    tmr_enc.SetTopValue(kTopVal);
    tmr_enc.SetFilterBits(0b1111);
    tmr_enc.SetQEncoderMode0();
    tmr_enc.SetCounter(0);
    tmr_enc.Enable();
    uint32_t enc_old_value = 0, enc_new_value = 0;

    // ==== Main evt cycle ====
    tmr_uart_check.StartOrRestart();
    while(true) {
        EvtMsg msg = evt_q_main.Fetch(TIME_INFINITE);
        switch(msg.id) {
            case EvtId::UartCheckTime:
                Watchdog::Reload();
                while(dbg_uart.TryParseRxBuff() == retv::Ok) OnCmd((Shell*)&dbg_uart);
                // Check encoder
                enc_new_value = tmr_enc.GetCounter();
                if(enc_new_value != enc_old_value) {
                    enc_old_value = enc_new_value;
                    ProcessEncValue(enc_old_value);
                    // Printf("Enc: %u %u\r", enc_old_value, tmr_enc.GetDir());
                }
                break;

            case EvtId::UsbCdcDataRcvd:
                while(usb_msd_cdc.TryParseRxBuff() == retv::Ok) OnCmd((Shell*)&usb_msd_cdc);
                break;

            case EvtId::UsbReady:
                Printf("Usb ready\r");
                CTC->Enable(); // Start autotrimming of IRC48M
                break;

            default: break;
        } // switch
    }
}
