/*
 * mem_msd_glue.cpp
 *
 *  Created on: 30 ���. 2016 �.
 *      Author: Kreyl
 */

#include "mem_msd_glue.h"
#include "shell.h"
#include "SpiFlash.h"

//#include "diskio.h"

extern SpiFlash_t SpiFlash;

namespace MsdMem {

uint32_t BlockCnt, BlockSz;

retv Read(uint32_t BlockAddress, uint8_t *Ptr, uint32_t BlocksCnt) {
//    Printf("R %u %u\r", BlockAddress, BlocksCnt);
    return SpiFlash.ReadQ(BlockAddress * BlockSz, Ptr, BlocksCnt * BlockSz);

//    Mem.Read(BlockAddress * MSD_BLOCK_SZ, Ptr, BlocksCnt * MSD_BLOCK_SZ);
//    if(disk_read(0, Ptr, BlockAddress, BlocksCnt) == RES_OK) return retvOk;
//    else return retvFail;
//    memcpy(Ptr, &buf[BlockAddress*MsdBlockSz], BlocksCnt*MsdBlockSz);
//    return retv::Ok;
}

retv Write(uint32_t BlockAddress, uint8_t *Ptr, uint32_t BlocksCnt) {
//    Printf("W %u %u\r", BlockAddress, BlocksCnt);
    uint32_t PageCnt = (BlocksCnt * BlockSz) / SPIFLASH_PAGE_SZ;
    // Erase sectors
    uint32_t Addr = BlockAddress * BlockSz;
    while(BlocksCnt) {
        if(SpiFlash.EraseSector4k(Addr) != retv::Ok) return retv::Fail;
        Addr += BlockSz;
        BlocksCnt--;
    }
    // Write data page by page
    Addr = BlockAddress * BlockSz;
    while(PageCnt) {
//        if(SpiFlash.WritePage(Addr, Ptr, SPIFLASH_PAGE_SZ) != retv::Ok) {
        if(SpiFlash.WritePageQ(Addr, Ptr, SPIFLASH_PAGE_SZ) != retv::Ok) return retv::Fail;
        Addr += SPIFLASH_PAGE_SZ;
        Ptr += SPIFLASH_PAGE_SZ;
        PageCnt--;
    }
    return retv::Ok;
//    return SpiFlash.wr
//    memcpy(&buf[BlockAddress*MsdBlockSz], Ptr, BlocksCnt*MsdBlockSz);
//    if(disk_write(0, Ptr, BlockAddress, BlocksCnt) == RES_OK) return retvOk;
//    else return retvFail;
//    while(BlocksCnt != 0) {
        // Calculate Mem Sector addr
//        uint32_t SectorStartAddr = BlockAddress * MEM_SECTOR_SZ;
        // Write renewed sector
//        if(Mem.EraseAndWriteSector4k(SectorStartAddr, Ptr) != OK) return FAILURE;
        // Process variables
//        BlockAddress += MSD_BLOCK_SZ;
//        BlocksCnt--;
//    }
//    return retv::Ok;
}

} // namespace
