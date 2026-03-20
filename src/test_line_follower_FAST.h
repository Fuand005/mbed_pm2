#ifndef TEST_LINE_FOLLOWER_FAST_H_
#define TEST_LINE_FOLLOWER_FAST_H_

#include "mbed.h"

void line_follower_fast_init(int loops_per_second);
void line_follower_fast_task(DigitalOut& led);
void line_follower_fast_reset(DigitalOut& led);
void line_follower_fast_print();

#endif /* TEST_LINE_FOLLOWER_FAST_H_ */
