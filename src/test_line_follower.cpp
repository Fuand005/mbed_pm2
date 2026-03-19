#include "test_line_follower.h"

#include "LineFollower.h"
#include "DCMotor.h"
#include "PESBoardPinMap.h"

// ---------------------------------------------------------------------------
// Robot physical parameters — measure your actual robot with a caliper
// ---------------------------------------------------------------------------
static constexpr float BAR_DIST       = 0.1861f;   // sensor bar to wheelbase [m]
static constexpr float WHEEL_DIAMETER = 0.0391f;   // wheel diameter [m]
static constexpr float WHEEL_BASE     = 0.1799f;   // distance between wheels [m]
static constexpr float MAX_VEL_RPS    = 2.0f;    // max motor velocity [RPS]

// ---------------------------------------------------------------------------
// DCMotor parameters — check your motor datasheet
// ---------------------------------------------------------------------------
static constexpr float GEAR_RATIO     = 100f; // gearbox ratio
static constexpr float KN             = 180.0f;  // motor constant [rpm/V]
static constexpr float VOLTAGE_MAX    = 11.1f;   // supply voltage [V]

// ---------------------------------------------------------------------------
// LineFollower uses the IMU I2C bus (PC_9 = SDA, PA_8 = SCL)
// These are defined in PESBoardPinMap.h as PB_IMU_SDA / PB_IMU_SCL
// ---------------------------------------------------------------------------
LineFollower lf(PB_IMU_SDA, PB_IMU_SCL,
                BAR_DIST, WHEEL_DIAMETER, WHEEL_BASE, MAX_VEL_RPS);

// ---------------------------------------------------------------------------
// DCMotors — closed-loop velocity control via encoders
// Motor 1 = left wheel, Motor 2 = right wheel
// Swap if robot turns in the wrong direction on first run
// ---------------------------------------------------------------------------
DCMotor motor_left (PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, GEAR_RATIO, KN, VOLTAGE_MAX);
DCMotor motor_right(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, GEAR_RATIO, KN, VOLTAGE_MAX);

// Motor driver board enable pin (PB_15 on new PES board)
DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

// ---------------------------------------------------------------------------

void line_follower_init(int loops_per_second)
{
    // Start slow — increase once the robot tracks the line reliably
    lf.setMaxWheelVelocity(1.0f);

    // Optional: tune angular controller (lower Kp_nl if oscillating)
    // lf.setRotationalVelocityControllerGains(2.0f, 15.0f);

    // Enable the DC motor power stage
    enable_motors = 1;

    // Ensure motors start from rest
    motor_left.setVelocity(0.0f);
    motor_right.setVelocity(0.0f);
}

void line_follower_task(DigitalOut& led)
{
    led = !led;

    // LineFollower computes targets in its own background RTOS thread.
    // We just read the latest values here every 20 ms.
    const float left_rps  = lf.getLeftWheelVelocity();
    const float right_rps = lf.getRightWheelVelocity();

    if (!lf.isLedActive()) {
        // No line detected — stop safely
        motor_left.setVelocity(0.0f);
        motor_right.setVelocity(0.0f);
    } else {
        // DCMotor.setVelocity() accepts RPS directly — no voltage mapping needed
        motor_left.setVelocity(left_rps);
        motor_right.setVelocity(right_rps);
    }
}

void line_follower_reset(DigitalOut& led)
{
    // Called once when BUTTON1 disables the task
    motor_left.setVelocity(0.0f);
    motor_right.setVelocity(0.0f);
    led = 0;
}

void line_follower_print()
{
    printf("Angle: %6.1f deg | L: %+.2f RPS  R: %+.2f RPS | LED: %d\n",
           lf.getAngleDegrees(),
           lf.getLeftWheelVelocity(),
           lf.getRightWheelVelocity(),
           static_cast<int>(lf.isLedActive()));
}
