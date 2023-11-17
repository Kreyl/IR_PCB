#include "gd_lib.h"
#include "gd_uart.h"
#include "yartos.h"
#include "MsgQ.h"
#include "usb_msdcdc.h"
#include "usb.h"
#include "SpiFlash.h"
#include "led.h"
#include "mem_msd_glue.h"
#include "kl_fs_utils.h"

/* Here is bootloader's logic.
 * If GPIO4 is Low, run USB MSD until reboot.
 * Else if there is firmware file on ext flash:
 *     1) write it to app flash
 *     2) delete the file
 *     3) jump to app
 * else (there is no firmware file):
 *     1) if app memory is not empty - jump to app
 *     2) else (app mem is empty) run USB MSD until reboot.
 */

#if 1 // ======================== Variables and defines ========================
// Forever
EvtMsgQ_t<EvtMsg_t, MAIN_EVT_Q_LEN> EvtQMain;
static const UartParams_t CmdUartParams(115200, CMD_UART_PARAMS);
CmdUart_t Uart{CmdUartParams};
void OnCmd(Shell_t *PShell);

LedSmooth_t Lumos{LUMOS_PIN};
SpiFlash_t SpiFlash(SPI0);
FATFS FlashFS;
FIL sFile;

uint32_t Buf[(FLASH_PAGE_SZ / sizeof(uint32_t))];

const LedSmoothChunk_t lsqWriting[] = {
        {Chunk::Setup, 0, 255}, {Chunk::Wait, 54},
        {Chunk::Setup, 0, 0},   {Chunk::Wait, 54},
        {Chunk::Repeat, 2},
        {Chunk::Wait, 450},
        {Chunk::Goto, 0}
};

const LedSmoothChunk_t lsqError[] = {
        {Chunk::Setup, 0, 255}, {Chunk::Wait, 99},
        {Chunk::Setup, 0, 0},   {Chunk::Wait, 99},
        {Chunk::Goto, 0}
};

EvtTimer_t TmrUartCheck(TIME_MS2I(UART_RX_POLL_MS), EvtId::UartCheckTime, EvtTimer_t::Type::Periodic);
#endif

void RunUSB();

void JumpToApp() {
    Watchdog::Reload();
    Sys::Lock();
    __disable_irq();
    void (*app)(void);
    uint32_t *p = (uint32_t*)APP_START_ADDR; // get a pointer to app
    SCB->VTOR = APP_START_ADDR; // offset the vector table
    __set_MSP(*p++);            // set main stack pointer to app Reset_Handler
    app = (void (*)(void))(*p);
    app();
    while(true); // Will not be here
}

static inline bool AppIsEmpty() { return *(uint32_t*)APP_START_ADDR == 0xFFFFFFFF; }

void CheckAppAndJumpIfNotEmpty(const char *Reason) {
    Printf("%S\r\n", Reason);
    Watchdog::Reload();
    Sys::Sleep(7);
    if(AppIsEmpty()) {
        Printf("App area is empty\r\n");
        RunUSB();
    }
    else JumpToApp();
}

void PrintErrorAndReboot(const char *Reason) {
    Lumos.StartOrRestart(lsqError);
    Printf("%S\r", Reason);
    Sys::Sleep(7);
    while(true); // Will reboot due to watchdog
}

static inline void InitClk() {
    FMC->EnableCashAndPrefetch();
    RCU->EnPwrMgmtUnit();
    FMC->SetLatency(48);
    if(RCU->EnableIRC48M() == retv::Ok) {
        RCU->SetCK48MSel_CKIRC48M();   // Use IRC48M as clk src instead of PLL
        RCU->EnCTC();              // Enable trimmer
        // Setup trimmer: SOF freq is 1 kHz, rising front, no prescaler, input is UsbSOF
        CTC->Setup(CTC_REFPOL_RISING, CTC_REFSEL_USBSOF, CTC_REFPSC_DIV_1, 1000);
        // Setup system clock
        RCU->SetPllPresel_CKIRC48M(); // IRC48M is a source
        RCU->SetPrediv0Sel_XTALorIRC48M(); // IRC48M is a source
        // 48MHz div 6 = 8MHz; 8MHz *6 = 48MHz. PLL input freq must be in [1; 25] MHz, 8MHz typical
        RCU->SetPrediv0(6);
        RCU->SetPllSel_Prediv0();
        RCU->SetPllMulti(PllMulti::mul06);
        // Switch clk
        if(RCU->EnablePll() == retv::Ok) {
            RCU->SetAhbPrescaler(AhbPsc::div1);
            RCU->SwitchCkSys2PLL();
        }
    }
    Clk::UpdateFreqValues();
}

void main() {
    Watchdog::InitAndStart(1800);
    InitClk();
    // Init peripheral
    RCU->EnAFIO();
    AFIO->DisableJtagDP(); // Disable JTAG, leaving SWD. Otherwise PB3 & PB4 are occupied by JTDO & JTRST
    Uart.Init();

    // RTOS & Event queue
    Sys::Init();
    EvtQMain.Init();
    Printf("\r%S %S\r\n", APP_NAME, XSTRINGIFY(BUILD_TIME));
    Clk::PrintFreqs();

    Lumos.Init();

    // Spi Flash and Msd glue
    AFIO->RemapSPI0_PB345();
    SpiFlash.Init();
    SpiFlash.Reset();
    SpiFlash_t::MemParams_t mp = SpiFlash.GetParams();
    MsdMem::BlockCnt = mp.SectorCnt;
    MsdMem::BlockSz = mp.SectorSz;
    Printf("Flash: %u sectors of %u bytes\r", mp.SectorCnt, mp.SectorSz);

    // Check if GPIO4 id Low
    Gpio::SetupInput(Gpio4, Gpio::PullUp);
    Sys::SleepMilliseconds(7);
    if(Gpio::IsLo(Gpio4)) RunUSB();

    // Gpio4 is hi, check if there is file to load
    // Init filesystem
    if(f_mount(&FlashFS, "", 0) != FR_OK) CheckAppAndJumpIfNotEmpty("FS error");
    // Try find file
    if(f_findfirst(&Dir, &FileInfo, "", FILENAME_PATTERN) != FR_OK)  // Failure, maybe not formatted
        CheckAppAndJumpIfNotEmpty("File search fail");
    // Is there fw file?
    if(FileInfo.fname[0] == 0) // Not found
        CheckAppAndJumpIfNotEmpty("Not found");
    Printf("Found: %S\r", FileInfo.fname);
    if(TryOpenFileRead(FileInfo.fname, &CommonFile) != retv::Ok)
        CheckAppAndJumpIfNotEmpty("File Open Error");
    // File found
    uint32_t TotalLen = f_size(&CommonFile);
    if(TotalLen < 128UL) CheckAppAndJumpIfNotEmpty("Empty File");
    else if(TotalLen > MAX_APP_SZ) CheckAppAndJumpIfNotEmpty("Too Large File");
    // Len is ok, write file

#if 1 // ======= Reading and flashing =======
    Lumos.StartOrRestart(lsqWriting);
    Flash::UnlockFlash();
    // Read file block by block
    uint32_t BytesToWrite, CurrentAddr = APP_START_ADDR;
    while(TotalLen != 0) {
        Watchdog::Reload();
        BytesToWrite = MIN_(TotalLen, FLASH_PAGE_SZ); // How many bytes to read?
        // Read block
        if(f_read(&CommonFile, Buf, BytesToWrite, &BytesToWrite) != FR_OK) {
            Flash::LockFlash();
            PrintErrorAndReboot("Read error");
        }
        // Write page
        if(Flash::ProgramBuf(Buf, BytesToWrite, CurrentAddr) != retv::Ok) {
            Flash::LockFlash();
            PrintErrorAndReboot("Write error");
        }
        // Writing succeded
        Printf(".");
        CurrentAddr += BytesToWrite;
        TotalLen -= BytesToWrite;
    } // while
#endif
    // Writing done
    f_close(&CommonFile);
    // Remove firmware file
    f_unlink(FileInfo.fname);
    Flash::LockFlash();
    CheckAppAndJumpIfNotEmpty("\r\nWriting done");
}

void RunUSB() {
    Watchdog::Reload();
    UsbMsdCdc.Init();
    UsbMsdCdc.Connect();
    TmrUartCheck.StartOrRestart();
    while(true) {
        EvtMsg_t Msg = EvtQMain.Fetch(TIME_INFINITE);
        switch(Msg.ID) {
            case EvtId::UartCheckTime:
                Watchdog::Reload();
                Gpio::Toggle(PA10);
                while(Uart.TryParseRxBuff() == retv::Ok) OnCmd((Shell_t*)&Uart);
                break;
            case EvtId::UsbCdcDataRcvd:
                while(UsbMsdCdc.TryParseRxBuff() == retv::Ok) OnCmd((Shell_t*)&UsbMsdCdc);
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
    if(PCmd->NameIs("Ping")) PShell->Ok();
    else if(PCmd->NameIs("Version")) PShell->Print("%S %S\r", APP_NAME, XSTRINGIFY(BUILD_TIME));

    else PShell->CmdUnknown();
}
