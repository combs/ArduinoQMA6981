/*
 * QMA6981.h
 *
 *  Created on: Jul 9, 2019
 *      Author: adam
 */

#ifndef QMA6981_H_
#define QMA6981_H_

#include "Arduino.h"


typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} accel_t;

bool QMA6981_setup(void);
void QMA6981_poll(accel_t* currentAccel);
bool QMA6981_enable_interrupts(uint8_t interrupt_number, bool step_unsimilar, bool step_quit, bool step, bool d_tap, bool s_tap, bool orient, bool fob, bool fwm, bool ffull, bool data, bool low, bool high_en_z, bool high_en_y, bool high_en_x);


#endif /* QMA6981_H_ */
