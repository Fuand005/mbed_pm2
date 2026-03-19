#ifndef TEST_DC_MOTOR_H_
#define TEST_DC_MOTOR_H_

#include "mbed.h"

void dc_motor_init(int loops_per_second);
void dc_motor_task(DigitalOut& led);
void dc_motor_reset(DigitalOut& led);
void dc_motor_print();

#endif /* TEST_DC_MOTOR_H_ */
