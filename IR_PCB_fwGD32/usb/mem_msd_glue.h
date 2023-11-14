/*
 * mem_msd_glue.h
 *
 *  Created on: 30 ���. 2016 �.
 *      Author: Kreyl
 */

#ifndef MEM_MSD_GLUE_H__
#define MEM_MSD_GLUE_H__

#include "types.h"

//#define MSD_BLOCK_CNT   SDCD1.capacity
//#define MSD_BLOCK_SZ    512

extern uint32_t MsdBlockCnt, MsdBlockSz;

retv MSDRead(uint32_t BlockAddress, uint8_t *Ptr, uint32_t BlocksCnt);
retv MSDWrite(uint32_t BlockAddress, uint8_t *Ptr, uint32_t BlocksCnt);

#endif // MEM_MSD_GLUE_H__
