#include "test_line_follower.h"

#include "DCMotor.h"
#include "SensorBar.h"
#include "PESBoardPinMap.h"
#include <Eigen/Dense>

// ---------------------------------------------------------------------------
// Robot physical parameters — professor's reference values
// ---------------------------------------------------------------------------
static constexpr float VOLTAGE_MAX = 12.0f;
static constexpr float GEAR_RATIO  = 100.0f;
static constexpr float KN          = 140.0f / 12.0f; // [rpm/V]

static constexpr float D_WHEEL  = 0.039f;           // wheel diameter [m]
static constexpr float B_WHEEL  = 0.175f;             // wheelbase [m]
static constexpr float BAR_DIST = 0.1885f;             // sensor bar to wheelbase centre [m]
static constexpr float R_WHEEL  = D_WHEEL / 2.0f;    // wheel radius [m]

static constexpr float KP = 5.0f;                    // angular P gain

// ---------------------------------------------------------------------------
// Pointers to function-local statics — set in line_follower_init()
// ---------------------------------------------------------------------------
static DCMotor*    mp_motor_M1   = nullptr;
static DCMotor*    mp_motor_M2   = nullptr;
static DigitalOut* mp_enable     = nullptr;
static SensorBar*  mp_sensor_bar = nullptr;

// Kinematics (initialised in line_follower_init)
static Eigen::Matrix2f m_Cwheel2robot;
static float           m_wheel_vel_max = 0.0f; // max wheel speed [rad/s]

// Last computed angle — held when line is lost (professor's approach)
static float m_angle = 0.0f;

// ---------------------------------------------------------------------------

void line_follower_init(int loops_per_second)
{
    // Motors constructed first — FastPWM / TIM1 set up before I2C bus opens
    static DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1,
                             GEAR_RATIO, KN, VOLTAGE_MAX);
    static DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2,
                             GEAR_RATIO, KN, VOLTAGE_MAX);
    static DigitalOut enable(PB_ENABLE_DCMOTORS);
    // Sensor bar: I2C1 on PB_9 (SDA) / PB_8 (SCL), runs its own RTOS thread
    static SensorBar sensor_bar(PB_9, PB_8, BAR_DIST);

    mp_motor_M1   = &motor_M1;
    mp_motor_M2   = &motor_M2;
    mp_enable     = &enable;
    mp_sensor_bar = &sensor_bar;

    // Hardware polarity: white surface = bit HIGH (1), dark line = bit LOW (0).
    // The position accumulator sums SET bits, so without inversion the many
    // symmetric white bits cancel to ~0 and angle stays near 0.
    // setInvertBits() XORs 0xFF so the single dark-line bit becomes 1 and the
    // algorithm correctly computes position from it.
    mp_sensor_bar->setInvertBits();

    // Wheel-to-robot kinematics matrix
    // Maps [w1, w2] (rad/s) -> [v (m/s), omega (rad/s)]
    m_Cwheel2robot << R_WHEEL / 2.0f,  R_WHEEL / 2.0f,
                      R_WHEEL / B_WHEEL, -R_WHEEL / B_WHEEL;

    // Max wheel angular speed [rad/s]
    m_wheel_vel_max = 2.0f * M_PIf * motor_M2.getMaxPhysicalVelocity();

    // Enable TIM1 PWM outputs (MOE bit required for advanced timers)
    TIM1->BDTR |= TIM_BDTR_MOE;

    // Motor driver disabled until the task is active
    *mp_enable = 0;

    mp_motor_M1->setVelocity(0.0f);
    mp_motor_M2->setVelocity(0.0f);
}

void line_follower_task(DigitalOut& led)
{
    *mp_enable = 1;
    led = !led;

    // Update angle only when a line is detected; hold last value otherwise
    if (mp_sensor_bar->isAnyLedActive())
        m_angle = mp_sensor_bar->getAvgAngleRad();

    // Robot-space command: [forward speed (m/s), angular rate (rad/s)]
    const Eigen::Vector2f robot_coord = { 0.5f * m_wheel_vel_max * R_WHEEL,
                                          KP * m_angle };

    // Invert kinematics -> wheel speeds [rad/s], convert to RPS for setVelocity
    const Eigen::Vector2f wheel_speed = m_Cwheel2robot.inverse() * robot_coord;

    mp_motor_M1->setVelocity(wheel_speed(0) / (2.0f * M_PIf));
    mp_motor_M2->setVelocity(wheel_speed(1) / (2.0f * M_PIf));
}

void line_follower_reset(DigitalOut& led)
{
    mp_motor_M1->setVelocity(0.0f);
    mp_motor_M2->setVelocity(0.0f);
    *mp_enable = 0;
    led = 0;
}

void line_follower_print()
{
    printf("Angle: %6.1f deg | M1: %+.2f RPS  M2: %+.2f RPS | LED: %d"
           " | Lft: %.2f  Ctr: %.2f  Rgt: %.2f\n",
           m_angle * (180.0f / M_PIf),
           mp_motor_M1->getVelocity(),
           mp_motor_M2->getVelocity(),
           static_cast<int>(mp_sensor_bar->isAnyLedActive()),
           mp_sensor_bar->getMeanThreeAvgBitsLeft(),
           mp_sensor_bar->getMeanFourAvgBitsCenter(),
           mp_sensor_bar->getMeanThreeAvgBitsRight());
}
