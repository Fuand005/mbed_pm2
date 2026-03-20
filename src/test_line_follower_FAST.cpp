#include "test_line_follower_FAST.h"

#include "LineFollower.h"
#include "DCMotor.h"
#include "PESBoardPinMap.h"

static const bool USE_GEAR_RATIO_78 = false;

static const float VOLTAGE_MAX = 12.0f;
static const float GEAR_RATIO  = USE_GEAR_RATIO_78 ? 78.125f : 100.0f;
static const float KN          = USE_GEAR_RATIO_78 ? 180.0f / 12.0f : 140.0f / 12.0f;

static const float D_WHEEL  = 0.0393f;
static const float B_WHEEL  = 0.179f;
static const float BAR_DIST = 0.1836f;

// Gains and speed - tune for fast operation
static const float KP        = 4.0f;
static const float KP_NL     = 15.0f;
static const float MAX_SPEED = 3.5f;  // RPS - faster than SLOW

static const float VEL_SIGN = -1.0f;

static DCMotor*      g_M1 = nullptr;
static DCMotor*      g_M2 = nullptr;
static DigitalOut*   g_en = nullptr;
static LineFollower* g_lf = nullptr;

static float g_cmd_M1 = 0.0f;
static float g_cmd_M2 = 0.0f;

void line_follower_fast_init(int loops_per_second)
{
    static DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1,
                             GEAR_RATIO, KN, VOLTAGE_MAX);
    static DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2,
                             GEAR_RATIO, KN, VOLTAGE_MAX);
    static DigitalOut enable_motors(PB_ENABLE_DCMOTORS);
    static LineFollower lineFollower(PB_9, PB_8,
                                     BAR_DIST, D_WHEEL, B_WHEEL,
                                     motor_M1.getMaxPhysicalVelocity());

    g_M1 = &motor_M1;
    g_M2 = &motor_M2;
    g_en = &enable_motors;
    g_lf = &lineFollower;

    g_lf->setRotationalVelocityControllerGains(KP, KP_NL);
    g_lf->setMaxWheelVelocity(MAX_SPEED);

    TIM1->BDTR |= TIM_BDTR_MOE;

    *g_en = 0;
    g_M1->setVelocity(0.0f);
    g_M2->setVelocity(0.0f);
}

void line_follower_fast_task(DigitalOut& led)
{
    led = !led;
    *g_en = 1;

    g_cmd_M1 = VEL_SIGN * g_lf->getRightWheelVelocity();
    g_cmd_M2 = VEL_SIGN * g_lf->getLeftWheelVelocity();
    g_M1->setVelocity(g_cmd_M1);
    g_M2->setVelocity(g_cmd_M2);
}

void line_follower_fast_reset(DigitalOut& led)
{
    *g_en = 0;
    g_M1->setVelocity(0.0f);
    g_M2->setVelocity(0.0f);
    g_cmd_M1 = 0.0f;
    g_cmd_M2 = 0.0f;
    led = 0;
}

void line_follower_fast_print()
{
    printf("a=%.3f | RW=%+.2f LW=%+.2f | FB: M1=%+.2f M2=%+.2f | led=%d\n",
           g_lf->getAngleRadians(),
           g_lf->getRightWheelVelocity(),
           g_lf->getLeftWheelVelocity(),
           g_M1->getVelocity(),
           g_M2->getVelocity(),
           g_lf->isLedActive() ? 1 : 0);
}
