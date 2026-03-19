#include "test_config.h"

#ifdef TEST_DC_MOTOR

#include "test_dc_motor.h"
#include "DCMotor.h"
#include "PESBoardPinMap.h"

// ---------------------------------------------------------------------------
// Motor parameters — adjust to your motor's datasheet
// ---------------------------------------------------------------------------
static constexpr float GEAR_RATIO  = 78.125f; // gearbox ratio
static constexpr float KN          = 180.0f;  // motor constant [rpm/V]
static constexpr float VOLTAGE_MAX = 12.0f;   // supply voltage [V]

// Test sequence: ramp up, hold, ramp down, reverse, stop
// Each phase lasts PHASE_DURATION_S seconds
static constexpr float PHASE_DURATION_S = 2.0f;
static constexpr float MAX_TEST_VEL_RPS = 1.5f;  // cap during testing [RPS]

// ---------------------------------------------------------------------------
static DCMotor* mp_motor_left  = nullptr;
static DCMotor* mp_motor_right = nullptr;
static DigitalOut* mp_enable   = nullptr;

static int   m_lps     = 50;
static int   m_counter = 0;

// Simple 5-phase test sequence
enum TestPhase {
    RAMP_UP   = 0,
    HOLD_FWD  = 1,
    RAMP_DOWN = 2,
    HOLD_REV  = 3,
    STOP      = 4,
    NUM_PHASES
};

static const char* phase_names[] = {
    "RAMP_UP", "HOLD_FWD", "RAMP_DOWN", "HOLD_REV", "STOP"
};

static float m_vel_left  = 0.0f;
static float m_vel_right = 0.0f;

// ---------------------------------------------------------------------------

void dc_motor_init(int loops_per_second)
{
    m_lps = loops_per_second;

    static DigitalOut enable(PB_ENABLE_DCMOTORS);
    static DCMotor motor_left (PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1,
                                GEAR_RATIO, KN, VOLTAGE_MAX);
    static DCMotor motor_right(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2,
                                GEAR_RATIO, KN, VOLTAGE_MAX);

    mp_enable      = &enable;
    mp_motor_left  = &motor_left;
    mp_motor_right = &motor_right;

    // Cap velocity for safety during testing
    mp_motor_left->setMaxVelocity(MAX_TEST_VEL_RPS);
    mp_motor_right->setMaxVelocity(MAX_TEST_VEL_RPS);

    *mp_enable = 0;

    mp_motor_left->setVelocity(0.0f);
    mp_motor_right->setVelocity(0.0f);

    m_counter = 0;
}

void dc_motor_task(DigitalOut& led)
{
    *mp_enable = 1;
    led = !led;

    // How many loops fit in one phase
    const int phase_loops = static_cast<int>(PHASE_DURATION_S * m_lps);
    const int phase_index = (m_counter / phase_loops) % NUM_PHASES;
    // Progress within the current phase [0.0 .. 1.0]
    const float t = static_cast<float>(m_counter % phase_loops)
                    / static_cast<float>(phase_loops);

    switch (phase_index) {
        case RAMP_UP:
            // Linearly ramp both motors from 0 to MAX_TEST_VEL_RPS
            m_vel_left  = t * MAX_TEST_VEL_RPS;
            m_vel_right = t * MAX_TEST_VEL_RPS;
            break;

        case HOLD_FWD:
            // Hold at full forward speed
            m_vel_left  = MAX_TEST_VEL_RPS;
            m_vel_right = MAX_TEST_VEL_RPS;
            break;

        case RAMP_DOWN:
            // Linearly ramp back down to 0
            m_vel_left  = (1.0f - t) * MAX_TEST_VEL_RPS;
            m_vel_right = (1.0f - t) * MAX_TEST_VEL_RPS;
            break;

        case HOLD_REV:
            // Hold at full reverse speed
            m_vel_left  = -MAX_TEST_VEL_RPS;
            m_vel_right = -MAX_TEST_VEL_RPS;
            break;

        case STOP:
        default:
            // Coast to zero before the cycle repeats
            m_vel_left  = 0.0f;
            m_vel_right = 0.0f;
            break;
    }

    mp_motor_left->setVelocity(-m_vel_left);
    mp_motor_right->setVelocity(-m_vel_right);

    m_counter++;
}

void dc_motor_reset(DigitalOut& led)
{
    mp_motor_left->setVelocity(0.0f);
    mp_motor_right->setVelocity(0.0f);
    *mp_enable  = 0;
    m_counter   = 0;
    m_vel_left  = 0.0f;
    m_vel_right = 0.0f;
    led = 0;
}

void dc_motor_print()
{
    const int phase_loops = static_cast<int>(PHASE_DURATION_S * m_lps);
    const int phase_index = (m_counter / phase_loops) % NUM_PHASES;

    printf("Phase: %-10s | Cmd (L/R): %+.2f / %+.2f RPS"
           " | Act (L/R): %+.2f / %+.2f RPS\n",
           phase_names[phase_index],
           m_vel_left,
           m_vel_right,
           mp_motor_left->getVelocity(),
           mp_motor_right->getVelocity());
}

#endif // TEST_DC_MOTOR
