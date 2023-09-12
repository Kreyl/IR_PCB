/*
 * ir_pkt.h
 *
 *  Created on: 13 авг. 2023 г.
 *      Author: layst
 */

#ifndef IR_PKT_H_
#define IR_PKT_H_

#include <inttypes.h>
#include "shell.h"

#define PKT_RESET   0x8305
#define PKT_SHOT    0x0000

static const uint32_t DamageTable[16] = {
        1,2,4,5,7,10,15,17,20,25,30,35,40,50,75,100
};

union IRPkt_t {
    uint16_t W16;
    struct { // LSB to MSB
        uint32_t _reserved: 2; // Not used in 14-bit pkt
        uint32_t DamageID: 4;
        uint32_t TeamID: 2;
        uint32_t PlayerID: 7;
        uint32_t Zero: 1;
    };
    IRPkt_t() : W16(0) {}
    IRPkt_t(uint16_t AW16) : W16(AW16) {}


    void PrintI() { PrintfI("Pkt: Word %04X; PlayerID %u; TeamID %u; Damage %u\r",
            W16, PlayerID, TeamID, DamageTable[DamageID]); }
    void Print() {
        chSysLock();
        PrintI();
        chSysUnlock();
    }

    IRPkt_t& operator =(const IRPkt_t &Right) { W16 = Right.W16; return *this; }
};

#endif /* IR_PKT_H_ */
