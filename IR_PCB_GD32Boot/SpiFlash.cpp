/*
 * SpiFlash.cpp
 *
 *  Created on: 26 окт. 2023 г.
 *      Author: laurelindo
 */

#include "SpiFlash.h"
#include "yartos.h"
#include "shell.h"

// DMA
#define DMA_RX_MODE DMA_PRIO_HIGH | DMA_MEMSZ_8_BIT | DMA_PERSZ_8_BIT | \
    DMA_MEM_INC | DMA_DIR_PER2MEM | DMA_TCIE

// Mode for reading, i.e. no irq, no mem inc (as it is required to send zeroes only)
#define DMA_TX_MODE_NOIRQ_NOINC DMA_PRIO_HIGH | DMA_MEMSZ_8_BIT | DMA_PERSZ_8_BIT | DMA_DIR_MEM2PER
// Mode for writing
#define DMA_TX_MODE_IRQ_INC DMA_PRIO_HIGH | DMA_MEMSZ_8_BIT | DMA_PERSZ_8_BIT | \
    DMA_MEM_INC | DMA_DIR_MEM2PER | DMA_TCIE

void SpiFlashDmaCb(void *p, uint32_t W32) {
    Sys::LockFromIRQ();
    Sys::WakeI(&((SpiFlash_t*)p)->PThd, retv::Ok);
    Sys::UnlockFromIRQ();
}

SpiFlash_t::SpiFlash_t(SPI_TypeDef *pspi) : spi(pspi),
        DmaTx(SPIFLASH_DMA_TX, SpiFlashDmaCb, this),
        DmaRx(SPIFLASH_DMA_RX, SpiFlashDmaCb, this)
        {}

void SpiFlash_t::Init() {
    // Init GPIO
    Nss.SetupOut(Gpio::PushPull, Gpio::speed50MHz);
    Nss.SetHi();
    Gpio::SetupAlterFunc(FLASH_SCK,  Gpio::PushPull, Gpio::speed50MHz);
    Gpio::SetupAlterFunc(FLASH_MISO, Gpio::PushPull, Gpio::speed50MHz);
    Gpio::SetupAlterFunc(FLASH_MOSI, Gpio::PushPull, Gpio::speed50MHz);
    Gpio::SetupAlterFunc(FLASH_IO2,  Gpio::PushPull, Gpio::speed50MHz);
    Gpio::SetupAlterFunc(FLASH_IO3,  Gpio::PushPull, Gpio::speed50MHz);
    // ==== SPI ====    MSB first, master, ClkLowIdle, FirstEdge, BitNum = 8
    spi.Setup(BitOrder::MSB, Spi_t::cpol::IdleLow, Spi_t::cpha::FirstEdge, SPIFLASH_CLK_FREQ_Hz);
//    spi.Setup(BitOrder::MSB, Spi_t::cpol::IdleLow, Spi_t::cpha::FirstEdge, 1000000);
    spi.Enable();
    // ==== DMA ====
    DmaRx.Init();
    DmaRx.SetPeriphAddr(&spi.PSpi->DATA);
    DmaRx.SetMode(DMA_RX_MODE);
    DmaTx.Init();
    DmaTx.SetPeriphAddr(&spi.PSpi->DATA);
}

SpiFlash_t::MemParams_t SpiFlash_t::GetParams() {
    MemParams_t r;
    // Read JEDEC ID
    uint32_t JID;
    Nss.SetLo();
    spi.WriteRead(0x9F); // Cmd code Read JEDEC ID
    JID = spi.WriteRead(0x00); // MfgId
    JID = (JID << 8) | spi.WriteRead(0x00); // MemType
    JID = (JID << 8) | spi.WriteRead(0x00); // Capacity
    Nss.SetHi();
//    Printf("JID: 0x%X\r", JID);
    switch(JID) {
        case 0xEF4016: // W25Q32
            r.SectorCnt = 1024UL;
            r.SectorSz = 4096UL;
            break;
        case 0xEF4017: // W25Q64
            r.SectorCnt = 2048UL;
            r.SectorSz = 4096UL;
            break;
        default: Printf("Unknown Flash JID: 0x%X\r", JID); break;
    } // switch
    return r;
}

#if 1 // ======================= Read / Write / Erase ==========================
retv SpiFlash_t::Read(uint32_t Addr, uint8_t *PBuf, uint32_t ALen) {
    Nss.SetLo();
    WriteCmdAndAddr(0x0B, Addr); // Cmd FastRead
    spi.WriteRead(0x00); // 8 dummy clocks
    // ==== Read Data ====
    spi.ClearRxBuf();
    spi.Disable();
    spi.SetRxOnly();   // Will not set if enabled
    spi.EnRxDma();
    DmaRx.SetMemoryAddr(PBuf);
    DmaRx.SetTransferDataCnt(ALen);
    // Start
    Sys::Lock();
    PThd = Sys::GetSelfThd();
    DmaRx.Enable();
    spi.Enable();
    retv r = Sys::SleepS(TIME_MS2I(270)); // Wait IRQ
    // Will be here after timeout or IRQ
    DmaRx.Disable();
    Sys::Unlock();
    Nss.SetHi();
    spi.Disable();
    spi.SetFullDuplex();   // Remove read-only mode
    spi.DisRxDma();
    spi.Enable();
    spi.PSpi->WaitForTBEHiAndTransLo(); // }
    spi.ClearRxBuf();                   // } Clear bufs and flags
    return r;
}

retv SpiFlash_t::ReadQ(uint32_t Addr, uint8_t *PBuf, uint32_t ALen) {
    Nss.SetLo();
    spi.WriteRead(0xEB); // Write cmd in single mode
    spi.ClearRxBuf();
    spi.EnQuadWrite();
    // Send Addr and 2 dummy clocks
    spi.WriteRead(0xFF & (Addr >> 16));
    spi.WriteRead(0xFF & (Addr >> 8));
    spi.WriteRead(0xFF & (Addr >> 0));
    spi.WriteRead(0xF0); // 2 dummy clocks, must be Fx (see memory datasheet)
    // Switch to read mode
    spi.EnQuadRead();
    // Send 4 more clocks
    spi.Write(0x00);
    spi.Write(0x00);
    // ==== Read Data ====
    DmaRx.SetMemoryAddr(PBuf);
    DmaRx.SetTransferDataCnt(ALen);
    uint8_t Dummy = 0;
    DmaTx.SetMemoryAddr(&Dummy);
    DmaTx.SetTransferDataCnt(ALen);
    DmaTx.SetMode(DMA_TX_MODE_NOIRQ_NOINC);
    spi.PSpi->WaitForTBEHiAndTransLo();
    spi.ClearRxBuf();
    spi.EnRxTxDma();
    // Start
    Sys::Lock();
    PThd = Sys::GetSelfThd();
    DmaRx.Enable();
    DmaTx.Enable();
    retv r = Sys::SleepS(TIME_MS2I(270)); // Wait IRQ
    // Will be here after timeout or IRQ
    DmaRx.Disable();
    DmaTx.Disable();
    Sys::Unlock();
    Nss.SetHi();
    spi.DisRxTxDma();
    spi.PSpi->WaitForTBEHiAndTransLo(); // }
    spi.ClearRxBuf();                   // } Clear bufs and flags
    spi.DisQuad();
    return r;
}


retv SpiFlash_t::WritePage(uint32_t Addr, uint8_t *PBuf, uint32_t ALen) {
    WriteEnable();
    Nss.SetLo();
    WriteCmdAndAddr(0x02, Addr);
    // Write data
    for(uint32_t i=0; i < ALen; i++) spi.WriteRead(*PBuf++);
    Nss.SetHi();
    return BusyWait(); // Wait completion
}

retv SpiFlash_t::WritePageQ(uint32_t Addr, uint8_t *PBuf, uint32_t ALen) {
    WriteEnable();
    Nss.SetLo();
    spi.WriteRead(0x32); // Write cmd and addr in single mode
    // Send Addr and 2 dummy clocks
    spi.WriteRead(0xFF & (Addr >> 16));
    spi.WriteRead(0xFF & (Addr >> 8));
    spi.WriteRead(0xFF & (Addr >> 0));
    // Write data
    DmaTx.SetMemoryAddr(PBuf);
    DmaTx.SetTransferDataCnt(ALen);
    DmaTx.SetMode(DMA_TX_MODE_IRQ_INC);
    spi.EnQuadWrite();
    spi.EnTxDma();
    Sys::Lock();
    PThd = Sys::GetSelfThd();
    DmaTx.Enable();
    retv r = Sys::SleepS(TIME_MS2I(270)); // Wait IRQ
    // Will be here after timeout or IRQ
    DmaTx.Disable();
    Sys::Unlock();
    spi.PSpi->WaitForTBEHiAndTransLo(); // }
    spi.ClearRxBuf();                   // } Clear bufs and flags
    Nss.SetHi();
    spi.DisTxDma();
    spi.DisQuad();
    if(r == retv::Ok) return BusyWait(); // Wait completion
    else return r;
}

retv SpiFlash_t::EraseSector4k(uint32_t Addr) {
    WriteEnable();
    Nss.SetLo();
    WriteCmdAndAddr(0x20, Addr);
    Nss.SetHi();
    return BusyWait(); // Wait completion
}

retv SpiFlash_t::EraseBlock32k(uint32_t Addr) {
    WriteEnable();
    Nss.SetLo();
    WriteCmdAndAddr(0x52, Addr);
    Nss.SetHi();
    return BusyWait(); // Wait completion
}

retv SpiFlash_t::EraseBlock64k(uint32_t Addr) {
    WriteEnable();
    Nss.SetLo();
    WriteCmdAndAddr(0xD8, Addr);
    Nss.SetHi();
    return BusyWait(); // Wait completion
}
#endif

// ========================= Control Instructions ==============================
uint8_t SpiFlash_t::ReleasePowerDown() {
    Nss.SetLo();
    spi.WriteRead(0xAB); // Cmd code
    spi.WriteRead(0x00); // }
    spi.WriteRead(0x00); // }
    spi.WriteRead(0x00); // } Three dummy bytes
    uint8_t r = spi.WriteRead(0x00);
    Nss.SetHi();
    return r;
}


/* UNUSED, left for dbg purposes
SpiFlash_t::MfrDevId_t SpiFlash_t::ReadMfrDevId() {
    MfrDevId_t r;
    Nss.SetLo();
    spi.WriteRead(0x90);
    spi.WriteRead(0x00);
    spi.WriteRead(0x00);
    spi.WriteRead(0x00);
    r.Mfr = spi.WriteRead(0x00);
    r.DevID = spi.WriteRead(0x00);
    Nss.SetHi();
    return r;
}

SpiFlash_t::MfrDevId_t SpiFlash_t::ReadMfrDevIdQ() {
    MfrDevId_t r;
    Nss.SetLo();
    spi.WriteRead(0x94); // Write cmd in single mode
    spi.EnQuadWrite();
    // Send 8 clocks
    spi.Write(0x00);
    spi.Write(0x00);
    spi.Write(0x00);
    spi.Write(0x00);
    // Switch to read mode
    spi.EnQuadRead();
    // Send 4 more clocks
    spi.Write(0x00);
    spi.Write(0x00);
    spi.PSpi->WaitForTBEHiAndTransLo();
    spi.ClearRxBuf();
    // Read payload
    r.Mfr = spi.WriteRead(0x00);
    r.DevID = spi.WriteRead(0x00);
    Nss.SetHi();
    spi.DisQuad();
    return r;
}
*/

#if 1 // ========================== Service ====================================
void SpiFlash_t::Reset() {
    // Software Reset sequence: Enable Reset (66h) & Reset (99h).
    Sys::Lock();
    Nss.SetLo();
    spi.WriteRead(0x66); // Enable Reset
    Nss.SetHi();
    __NOP(); __NOP(); __NOP(); __NOP();
    Nss.SetLo();
    spi.WriteRead(0x99); // Do Reset
    Nss.SetHi();
    Sys::Unlock();
    Sys::SleepMilliseconds(1); // device will take approximately 30μS (tRST) to reset
}

void SpiFlash_t::WriteCmdAndAddr(uint8_t Cmd, uint32_t Addr) {
    spi.WriteRead(Cmd);
    spi.WriteRead(0xFF & (Addr >> 16));
    spi.WriteRead(0xFF & (Addr >> 8));
    spi.WriteRead(0xFF & (Addr >> 0));
}

// Write cmd and then read next byte
uint8_t SpiFlash_t::WriteCmdRead1Byte(uint8_t Cmd) {
    Nss.SetLo();
    spi.WriteRead(Cmd);
    uint8_t r = spi.WriteRead(0x00);
    Nss.SetHi();
    return r;
}

void SpiFlash_t::WriteEnable() {
    Nss.SetLo();
    spi.WriteRead(0x06);
    Nss.SetHi();
}

retv SpiFlash_t::BusyWait() {
    Sys::SleepMilliseconds(1);
    systime_t Start = Sys::GetSysTimeX();
    retv r = retv::Timeout;
    Nss.SetLo();
    spi.WriteRead(0x05); // Read StatusReg1
    while(Sys::TimeElapsedSince(Start) < TIME_MS2I(SPIFLASH_TIMEOUT_ms)) {
        Sys::SleepMilliseconds(1);
        uint8_t b = spi.WriteRead(0);
//        Printf("%X\r", r);
        if((b & 0x01) == 0) { // BUSY bit == 0
            r = retv::Ok;
            break;
        }
    }
    Nss.SetHi();
    return r;
}
#endif
