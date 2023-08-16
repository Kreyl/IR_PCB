/*
 * app.h
 *
 *  Created on: 14 авг. 2023 г.
 *      Author: layst
 */

#ifndef APP_H_
#define APP_H_

#include <inttypes.h>
#include "MsgQ.h"

extern int32_t RoundsCnt, MagazinesCnt;

void AppInit();
void SetInputs(uint32_t AIn[3]);
void Reset();

#endif /* APP_H_ */
