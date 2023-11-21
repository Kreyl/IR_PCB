/*
 * SpiFlash.h
 *
 *  Created on: 26 окт. 2023 г.
 *      Author: laurelindo
 */

#ifndef LIB_SPIFLASH_H_
#define LIB_SPIFLASH_H_

#include "gd32e11x_kl.h"
#include "gd_lib.h"

/* ReadData allows 50Mhz only, so it is not implemented. Read method uses
 * Fast Read instead. For voltages below 3v0, set 104MHz. */
//#define SPIFLASH_CLK_FREQ_Hz    133000000UL // For voltages 3v0...3v6
//#define SPIFLASH_CLK_FREQ_Hz    1000000UL // Do it unhurriedly
#define SPIFLASH_CLK_FREQ_Hz    104000000UL // For voltages 2v7...3v0
#define SPIFLASH_PAGE_SZ        256UL    // Defined in datasheet
#define SPIFLASH_BLOCK32_SZ     32768UL  // 128 pages
#define SPIFLASH_BLOCK64_SZ     65536UL  // 256 pages

#define SPIFLASH_TIMEOUT_ms     999UL

class SpiFlash_t {
private:
    Spi_t spi;
    Pin_t Nss{FLASH_NSS};
    DMA_t DmaTx, DmaRx;
    Thread_t* PThd = nullptr;
    uint8_t WriteCmdRead1Byte(uint8_t Cmd);
    void WriteCmdAndAddr(uint8_t Cmd, uint32_t Addr);
    void WriteEnable();
    retv BusyWait();
    friend void SpiFlashDmaCb(void *p, uint32_t W32);
public:
    SpiFlash_t(SPI_TypeDef *pspi);
    void Init();

    struct MemParams_t {
        uint32_t SectorCnt = 0, SectorSz = 0;
    };

    MemParams_t GetParams();

    // ==== Instructions ====
    // Read / Write / Erase
    retv Read(uint32_t Addr, uint8_t *PBuf, uint32_t ALen);
    retv ReadQ(uint32_t Addr, uint8_t *PBuf, uint32_t ALen);

    retv WritePage(uint32_t Addr, uint8_t *PBuf, uint32_t ALen);
    retv WritePageQ(uint32_t Addr, uint8_t *PBuf, uint32_t ALen);

    retv EraseSector4k(uint32_t Addr);
    retv EraseBlock32k(uint32_t Addr);
    retv EraseBlock64k(uint32_t Addr);
    retv EraseAndWriteSector4k(uint32_t Addr, uint8_t *PBuf);

    // Control
    void Reset();
    uint8_t ReleasePowerDown(); // Returns ID[7:0]

    // Status Regs
    uint8_t ReadStatusReg1() { return WriteCmdRead1Byte(0x05); }
    uint8_t ReadStatusReg2() { return WriteCmdRead1Byte(0x35); }
    uint8_t ReadStatusReg3() { return WriteCmdRead1Byte(0x15); }
//    void WriteStatusReg1(uint8_t b);
//    void WriteStatusReg2(uint8_t b);
//    void WriteStatusReg3(uint8_t b);

    void PowerDown();
};

#endif /* LIB_SPIFLASH_H_ */
