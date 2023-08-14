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


namespace Firing {

void Init();

}


namespace CtrlPins {

void Init();

void SetInputs(uint32_t AIn[3]);


}

#endif /* APP_H_ */
