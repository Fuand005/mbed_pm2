#include "mbed.h"
#include "PESBoardPinMap.h"
#include "DebounceIn.h"

// === SELECT ACTIVE TEST (edit test_config.h) ===

#include "test_config.h"

#if defined(TEST_SERVO)
    #include "test_servo.h"
    #define TEST_INIT(lps)    servo_init(lps)
    #define TEST_TASK(led)    servo_task(led)
    #define TEST_RESET(led)   servo_reset(led)
    #define TEST_PRINT()      servo_print()

#elif defined(TEST_COLOR_SENSOR)
    #include "test_color_sensor.h"
    #define TEST_INIT(lps)    color_sensor_init(lps)
    #define TEST_TASK(led)    color_sensor_task(led)
    #define TEST_RESET(led)   color_sensor_reset(led)
    #define TEST_PRINT()      color_sensor_print()

#elif defined(TEST_COLOR_SENSOR_CALIB)
    #include "test_color_sensor_calib.h"
    #define TEST_INIT(lps)    color_sensor_calib_init(lps)
    #define TEST_TASK(led)    color_sensor_calib_task(led)
    #define TEST_RESET(led)   color_sensor_calib_reset(led)
    #define TEST_PRINT()      color_sensor_calib_print()

#elif defined(TEST_LINE_FOLLOWER)
    #include "test_line_follower.h"
    #define TEST_INIT(lps)  line_follower_init(lps)
    #define TEST_TASK(led)  line_follower_task(led)
    #define TEST_RESET(led) line_follower_reset(led)
    #define TEST_PRINT()    line_follower_print()

#elif defined(TEST_LINE_FOLLOWER_SLOW)
    #include "test_line_follower_SLOW.h"
    #define TEST_INIT(lps)  line_follower_slow_init(lps)
    #define TEST_TASK(led)  line_follower_slow_task(led)
    #define TEST_RESET(led) line_follower_slow_reset(led)
    #define TEST_PRINT()    line_follower_slow_print()

#elif defined(TEST_LINE_FOLLOWER_FAST)
    #include "test_line_follower_FAST.h"
    #define TEST_INIT(lps)  line_follower_fast_init(lps)
    #define TEST_TASK(led)  line_follower_fast_task(led)
    #define TEST_RESET(led) line_follower_fast_reset(led)
    #define TEST_PRINT()    line_follower_fast_print()

#elif defined(TEST_DC_MOTOR)
    #include "test_dc_motor.h"
    #define TEST_INIT(lps)    dc_motor_init(lps)
    #define TEST_TASK(led)    dc_motor_task(led)
    #define TEST_RESET(led)   dc_motor_reset(led)
    #define TEST_PRINT()      dc_motor_print()


#else
    #error "No test selected. Open test_config.h and uncomment one #define."
#endif
// ================================================

bool do_execute_main_task = false;
bool do_reset_all_once    = false;

DebounceIn user_button(BUTTON1);
void toggle_do_execute_main_fcn();

int main()
{
    user_button.fall(&toggle_do_execute_main_fcn);

    const int main_task_period_ms = 20;
    Timer main_task_timer;

    DigitalOut user_led(LED1);
#if defined(TEST_LINE_FOLLOWER) || defined(TEST_LINE_FOLLOWER_SLOW) || defined(TEST_LINE_FOLLOWER_FAST)
    // PB_9 = I2C1 SDA for sensor bar — use a different pin to avoid conflict
    DigitalOut led1(PB_10);
#else
    DigitalOut led1(PB_9);
#endif

    const int loops_per_second = static_cast<int>(
        ceilf(1.0f / (0.001f * static_cast<float>(main_task_period_ms))));

    TEST_INIT(loops_per_second);

    main_task_timer.start();

    while (true) {
        main_task_timer.reset();

        if (do_execute_main_task) {
            TEST_TASK(led1);
        } else {
            if (do_reset_all_once) {
                do_reset_all_once = false;
                TEST_RESET(led1);
            }
        }

        user_led = !user_led;
        TEST_PRINT();

        int elapsed_ms = duration_cast<milliseconds>(
            main_task_timer.elapsed_time()).count();
        if (main_task_period_ms - elapsed_ms < 0)
            printf("Warning: Main task took longer than main_task_period_ms\n");
        else
            thread_sleep_for(main_task_period_ms - elapsed_ms);
    }
}

void toggle_do_execute_main_fcn()
{
    do_execute_main_task = !do_execute_main_task;
    if (do_execute_main_task)
        do_reset_all_once = true;
}
