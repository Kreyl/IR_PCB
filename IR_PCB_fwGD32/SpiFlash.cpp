/*
 * SpiFlash.cpp
 *
 *  Created on: 26 окт. 2023 г.
 *      Author: laurelindo
 */

#include "SpiFlash.h"
#include "yartos.h"

// DMA
#define DMA_RX_MODE DMA_PRIO_HIGH | DMA_MEMSZ_8_BIT | DMA_PERSZ_8_BIT | \
    DMA_MEM_INC | DMA_DIR_PER2MEM | DMA_TCIE

// Mode for reading, i.e. no irq, no mem inc (as it is required to send zeroes only)
#define DMA_TX_MODE_NOIRQ_NOINC DMA_PRIO_HIGH | DMA_MEMSZ_8_BIT | DMA_PERSZ_8_BIT | DMA_DIR_MEM2PER
// Mode for writing
#define DMA_TX_MODE_IRQ_INC DMA_PRIO_HIGH | DMA_MEMSZ_8_BIT | DMA_PERSZ_8_BIT | \
    DMA_MEM_INC | DMA_DIR_MEM2PER | DMA_TCIE

void SpiFlashDmaCb(void *p, uint32_t W32) {
    Sys::IrqPrologue();
    Sys::LockFromIRQ();
    Sys::WakeI(&((SpiFlash_t*)p)->PThd, retv::Ok);
    Sys::UnlockFromIRQ();
    Sys::IrqEpilogue();
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
//    spi.Setup(BitOrder::MSB, Spi_t::cpol::IdleLow, Spi_t::cpha::FirstEdge, SPIFLASH_CLK_FREQ_Hz);
    spi.Setup(BitOrder::MSB, Spi_t::cpol::IdleLow, Spi_t::cpha::FirstEdge, 1000000);
    spi.Enable();
    // ==== DMA ====
    DmaRx.Init();
    DmaRx.SetPeriphAddr(&spi.PSpi->DATA);
    DmaRx.SetMode(DMA_RX_MODE);
    DmaTx.Init();
    DmaTx.SetPeriphAddr(&spi.PSpi->DATA);
}

// ========================= Read / Write / Erase ==============================
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
    spi.ClearRxBuf();
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
    systime_t Start = Sys::GetSysTimeX();
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


// ========================================= Definition ============================================

#define EXT_FLASH_XX25_TEST_SEGMENT_AMOUNT          4
#define EXT_FLASH_XX25_TEST_PAGE_AMOUNT             8
#define EXT_FLASH_XX25_TEST_MAGIC_NUMBER            42

/// Таймаут стирания минимального сегмента для семейства микросхем N25xx, мс
#define FLASH_N25_SEGMENT_ERASE_TIMEOUT             800
/// Таймаут записи страницы для семейства микросхем N25xx, мс
#define FLASH_N25_PAGE_PROGRAM_TIMEOUT              5

/// Таймаут стирания минимального сегмента для семейства микросхем M25xx, мс
#define FLASH_M25_SEGMENT_ERASE_TIMEOUT             600
/// Таймаут записи страницы для семейства микросхем M25xx, мс
#define FLASH_M25_PAGE_PROGRAM_TIMEOUT              1

/// Таймаут стирания минимального сегмента для семейства микросхем MX25xx, мс
#define FLASH_MX25_SEGMENT_ERASE_TIMEOUT            120
/// Таймаут записи страницы для семейства микросхем MX25xx, мс
#define FLASH_MX25_PAGE_PROGRAM_TIMEOUT             2

/// Таймаут стирания минимального сегмента для семейства микросхем WQ25xx, мс
#define FLASH_WQ25_SEGMENT_ERASE_TIMEOUT            420
/// Таймаут записи страницы для семейства микросхем WQ25xx, мс
#define FLASH_WQ25_PAGE_PROGRAM_TIMEOUT             4

/// Размер сектора, байт
#define EXT_FLASH_XX45_SECTOR_SIZE                  65536
/// Размер подсектора (доступен не во всех микросхемах), байт
#define FLASH_XX25_SUBSECTOR_SIZE                   4096

/**
 * @defgroup FLASH_XX25_CMD
 * Команды, для работы с микросхемой FLASH
 * @{
 */
#define FLASH_XX25_CMD__READ_ID                     0x9F
#define FLASH_XX25_CMD__READ_DATA_BYTES             0x03
#define FLASH_XX25_CMD__READ_STATUS_REGISTER        0x05
#define FLASH_XX25_CMD__PAGE_PROGRAM                0x02
#define FLASH_XX25_CMD__WRITE_ENABLE                0x06
#define FLASH_XX25_CMD__WRITE_DISABLE               0x04
#define FLASH_XX25_CMD__SECTOR_ERASE                0xD8
#define FLASH_XX25_CMD__SUBSECTOR_ERASE             0x20
/** @} */

/// Полный объем Flash памяти M25P16 в Кб
#define FLASH_M25P16_TOTAL_SIZE                     2048

/// Полный объем Flash памяти N25Q128 в Кб
#define FLASH_N25Q128_TOTAL_SIZE                    16384

/// Полный объем Flash памяти N25Q032 в Кб
#define FLASH_N25Q032_TOTAL_SIZE                    4096

/// Полный объем Flash памяти MX25L128 в Кб
#define FLASH_MX25L128_TOTAL_SIZE                   16384

/// Полный объем Flash памяти WQ25F32 в Кб
#define FLASH_WQ25F32_TOTAL_SIZE                    4096

#define EXT_FLASH_XX25_STATUS_WRITE_IN_PROG_MSK     0x01

#pragma pack(push, 1)
typedef union
{
  struct
  {
    uint8_t write_in_progress   : 1;
    uint8_t write_enable_latch  : 1;
    uint8_t block_protect       : 3;
    uint8_t reserved            : 2;
    uint8_t write_protect       : 1;
  } m25;

  struct
  {
    uint8_t write_in_progress   : 1;
    uint8_t write_enable_latch  : 1;
    uint8_t block_protect       : 3;
    /// 1 - bottom
    /// 0 - top
    uint8_t top_bottom          : 1;
    uint8_t reserved            : 1;
    uint8_t write_protect       : 1;
  } n25;

  struct
  {
    uint8_t write_in_progress   : 1;
    uint8_t write_enable_latch  : 1;
    uint8_t block_protect       : 4;
    uint8_t quad_en             : 1;
    uint8_t write_protect       : 1;
  } mx25;

  struct
  {
    uint8_t write_in_progress   : 1;
    uint8_t write_enable_latch  : 1;
    uint8_t block_protect       : 3;
    uint8_t top_bottom_protect  : 1;
    uint8_t sector_protect      : 1;
    uint8_t status_reg_protect  : 1;
  } wq25;

  uint8_t msk;
} ext_flash_xx25_status_t;
#pragma pack(pop)

// ========================================= Declaration ===========================================

uint32_t ext_flash_xx25_get_type(void);

// ======================================== Implementation =========================================
/*
error_t ext_flash_xx25_autodefine(void)
{
  error_t res;

  ext_flash_xx25.inited = 0;
  ext_flash_xx25.type = EXT_FLASH_XX25_TYPE__UNKNOWN;
  ext_flash_xx25.size_in_kb = 0;
  ext_flash_xx25.erasable_segment_size = 0;
  ext_flash_xx25.erase_segment_timeout = 0;

  // первый вызов - холостой
  ext_flash_xx25.type = (ext_flash_xx25_type_t)ext_flash_xx25_get_type();
  ext_flash_xx25.type = (ext_flash_xx25_type_t)ext_flash_xx25_get_type();

  switch (ext_flash_xx25.type)
  {
    case EXT_FLASH_XX25_TYPE__M25P16:
      ext_flash_xx25.size_in_kb = FLASH_M25P16_TOTAL_SIZE;
      ext_flash_xx25.erase_subsector_enable = false;
      ext_flash_xx25.erasable_segment_size = EXT_FLASH_XX45_SECTOR_SIZE;
      ext_flash_xx25.erase_segment_timeout = FLASH_M25_SEGMENT_ERASE_TIMEOUT;
      ext_flash_xx25.program_page_timeout = FLASH_M25_PAGE_PROGRAM_TIMEOUT;
      res = ERR_OK;
    break;

    case EXT_FLASH_XX25_TYPE__N25Q128:
      ext_flash_xx25.size_in_kb = FLASH_N25Q128_TOTAL_SIZE;
      ext_flash_xx25.erase_subsector_enable = true;
      ext_flash_xx25.erasable_segment_size = FLASH_XX25_SUBSECTOR_SIZE;
      ext_flash_xx25.erase_segment_timeout = FLASH_N25_SEGMENT_ERASE_TIMEOUT;
      ext_flash_xx25.program_page_timeout = FLASH_N25_PAGE_PROGRAM_TIMEOUT;
      res = ERR_OK;
    break;

    case EXT_FLASH_XX25_TYPE__N25Q032:
      ext_flash_xx25.size_in_kb = FLASH_N25Q032_TOTAL_SIZE;
      ext_flash_xx25.erase_subsector_enable = true;
      ext_flash_xx25.erasable_segment_size = FLASH_XX25_SUBSECTOR_SIZE;
      ext_flash_xx25.erase_segment_timeout = FLASH_N25_SEGMENT_ERASE_TIMEOUT;
      ext_flash_xx25.program_page_timeout = FLASH_N25_PAGE_PROGRAM_TIMEOUT;
      res = ERR_OK;
    break;

    case EXT_FLASH_XX25_TYPE__MX25L128:
      ext_flash_xx25.size_in_kb = FLASH_MX25L128_TOTAL_SIZE;
      ext_flash_xx25.erase_subsector_enable = true;
      ext_flash_xx25.erasable_segment_size = FLASH_XX25_SUBSECTOR_SIZE;
      ext_flash_xx25.erase_segment_timeout = FLASH_MX25_SEGMENT_ERASE_TIMEOUT;
      ext_flash_xx25.program_page_timeout = FLASH_MX25_PAGE_PROGRAM_TIMEOUT;
      res = ERR_OK;
    break;

    case EXT_FLASH_XX25_TYPE__W25Q32F:
      ext_flash_xx25.size_in_kb = FLASH_WQ25F32_TOTAL_SIZE;
      ext_flash_xx25.erase_subsector_enable = true;
      ext_flash_xx25.erasable_segment_size = FLASH_XX25_SUBSECTOR_SIZE;
      ext_flash_xx25.erase_segment_timeout = FLASH_WQ25_SEGMENT_ERASE_TIMEOUT;
      ext_flash_xx25.program_page_timeout = FLASH_WQ25_PAGE_PROGRAM_TIMEOUT;
      res = ERR_OK;
    break;

    default:
      res = ERR_RESULT;
    break;
  }

  if (res == ERR_OK)
  {
    ext_flash_xx25.inited = true;
  }

  return res;
}
*/
