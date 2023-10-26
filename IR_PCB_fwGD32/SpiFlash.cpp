/*
 * SpiFlash.cpp
 *
 *  Created on: 26 окт. 2023 г.
 *      Author: laurelindo
 */

#include "SpiFlash.h"
#include "yartos.h"

// DMA
#define MEM_RX_DMA_MODE STM32_DMA_CR_CHSEL(0) |   /* dummy */ \
                        DMA_PRIORITY_HIGH | \
                        STM32_DMA_CR_MSIZE_BYTE | \
                        STM32_DMA_CR_PSIZE_BYTE | \
                        STM32_DMA_CR_MINC |       /* Memory pointer increase */ \
                        STM32_DMA_CR_DIR_P2M |    /* Direction is peripheral to memory */ \
                        STM32_DMA_CR_TCIE         /* Enable Transmission Complete IRQ */

void SpiFlash_t::Init(
        GPIO_TypeDef *AGpioNss, uint32_t APinNss,
        GPIO_TypeDef *AGpioSck, uint32_t APinSck,
        GPIO_TypeDef *AGpioMiso, uint32_t APinMiso,
        GPIO_TypeDef *AGpioMosi, uint32_t APinMosi,
        GPIO_TypeDef *AGpioIO2, uint32_t APinIO2,
        GPIO_TypeDef *AGpioIO3, uint32_t APinIO3) {
    // Init GPIO
    Nss.PGpio = AGpioNss;
    Nss.PinN = APinNss;
    Nss.SetupOut(Gpio::PushPull, Gpio::speed50MHz);
    Nss.SetHi();
    Gpio::SetupAlterFunc(AGpioSck,  APinSck,  Gpio::PushPull, Gpio::speed50MHz);
    Gpio::SetupAlterFunc(AGpioMiso, APinMiso, Gpio::PushPull, Gpio::speed50MHz);
    Gpio::SetupAlterFunc(AGpioMosi, APinMosi, Gpio::PushPull, Gpio::speed50MHz);
    Gpio::SetupAlterFunc(AGpioIO2,  APinIO2,  Gpio::PushPull, Gpio::speed50MHz);
    Gpio::SetupAlterFunc(AGpioIO2,  APinIO3,  Gpio::PushPull, Gpio::speed50MHz);

    // ==== SPI ====    MSB first, master, ClkLowIdle, FirstEdge, BitNum = 8
//    spi.Setup(BitOrder::MSB, Spi_t::cpol::IdleLow, Spi_t::cpha::FirstEdge, SPIFLASH_CLK_FREQ_Hz);
    spi.Setup(BitOrder::MSB, Spi_t::cpol::IdleLow, Spi_t::cpha::FirstEdge, 1000000);
    spi.Enable();
    // DMA

    //
}

// ========================= Read / Write / Erase ==============================
retv SpiFlash_t::Read(uint32_t Addr, uint8_t *PBuf, uint32_t ALen) {
    Nss.SetLo();
    WriteCmdAndAddr(0x0B, Addr); // Cmd FastRead
    spi.WriteRead(0x00); // 8 dummy clocks
    // ==== Read Data ====
    while(ALen--) {
        *PBuf++ = spi.WriteRead(0);
    }


//    spi.Disable();
//    spi.SetRxOnly();   // Will not set if enabled
//    spi.EnableRxDma();
//    dmaStreamSetMemory0(SPI1_DMA_RX, PBuf);
//    dmaStreamSetTransactionSize(SPI1_DMA_RX, ALen);
//    dmaStreamSetMode   (SPI1_DMA_RX, MEM_RX_DMA_MODE);
    // Start
//    dmaStreamEnable    (SPI1_DMA_RX);
//    spi.Enable();
//    chThdSuspendS(&trp);    // Wait IRQ
//    dmaStreamDisable(SPI1_DMA_RX);
    Nss.SetHi();
//    ISpi.Disable();
//    ISpi.SetFullDuplex();   // Remove read-only mode
//    ISpi.DisableRxDma();
//    ISpi.Enable();
//    ISpi.ClearRxBuf();
//    chSysUnlock();
    return retv::Ok;
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
    // XXX
    WriteEnable();
    Nss.SetLo();
    WriteCmdAndAddr(0x02, Addr);
    // Write data
    for(uint32_t i=0; i < ALen; i++) spi.WriteRead(*PBuf++);
    Nss.SetHi();
    return BusyWait(); // Wait completion
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
    spi.PSpi->ClearRxBuf();
    // Read payload
    r.Mfr = spi.WriteRead(0x00);
    r.DevID = spi.WriteRead(0x00);
    Nss.SetHi();
    spi.DisQuad();
    return r;
}

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
    systime_t Start = Sys::GetSysTime();
    retv r = retv::Timeout;
    Nss.SetLo();
    spi.WriteRead(0x05); // Read StatusReg1
    while(Sys::TimeElapsedSince(Start) < TIME_MS2I(270)) {
        Sys::SleepMilliseconds(1);
        uint8_t b = spi.WriteRead(0);
//        Printf(">%X\r", r);
        if((b & 0x01) == 0) { // BUSY bit == 0
            r = retv::Ok;
            break;
        }
    }
    Nss.SetHi();
    return r;
}
#endif

void SpiFlash_t::WriteStatusReg1(uint8_t b) {

}
