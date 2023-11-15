/*
 * mem_msd_glue.cpp
 *
 *  Created on: 30 ���. 2016 �.
 *      Author: Kreyl
 */

#include "mem_msd_glue.h"
#include "shell.h"
//#include "diskio.h"

uint32_t MsdBlockCnt = 32UL, MsdBlockSz = 512UL;
static uint8_t buf[32 * 512];

retv MSDRead(uint32_t BlockAddress, uint8_t *Ptr, uint32_t BlocksCnt) {
    Printf("R Addr: %u; Cnt: %u\r", BlockAddress, BlocksCnt);
//    Mem.Read(BlockAddress * MSD_BLOCK_SZ, Ptr, BlocksCnt * MSD_BLOCK_SZ);
//    if(disk_read(0, Ptr, BlockAddress, BlocksCnt) == RES_OK) return retvOk;
//    else return retvFail;
    memcpy(Ptr, &buf[BlockAddress*MsdBlockSz], BlocksCnt*MsdBlockSz);
    return retv::Ok;
}

retv MSDWrite(uint32_t BlockAddress, uint8_t *Ptr, uint32_t BlocksCnt) {
    Printf("WRT Addr: %u; Cnt: %u\r", BlockAddress, BlocksCnt);
    memcpy(&buf[BlockAddress*MsdBlockSz], Ptr, BlocksCnt*MsdBlockSz);
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
    return retv::Ok;
}

//MemParams_t MSDGetMemParams() {
//    MemParams_t r;
//    r.Rslt = retv::Ok;
//    r->BlockSz = 512;
//    r->BlockCnt = 32768;
//    return r;
//}
