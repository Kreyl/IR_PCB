/*
 * ir_pkt.h
 *
 *  Created on: 13 авг. 2023 г.
 *      Author: layst
 */

#ifndef IR_PKT_H_
#define IR_PKT_H_

#include <inttypes.h>

class IRPkt_t {
private:

public:
    // Body
    union {
        uint32_t DW32; // For mem alignment
        uint16_t W16;
        struct {
            uint32_t Type: 3;
            uint32_t FightID: 2;
            uint32_t TeamID: 3;
            uint32_t PktN: 5;
            uint32_t crc: 3;
        };
    }; // union

};



#endif /* IR_PKT_H_ */
