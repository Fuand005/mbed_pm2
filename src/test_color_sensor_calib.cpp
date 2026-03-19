#include "test_config.h"

#ifdef TEST_COLOR_SENSOR_CALIB

#include "test_color_sensor_calib.h"

// ANSI escape codes
#define ANSI_RESET   "\033[0m"
#define ANSI_BOLD    "\033[1m"
#define ANSI_RED     "\033[91m"
#define ANSI_GREEN   "\033[92m"
#define ANSI_YELLOW  "\033[93m"
#define ANSI_BLUE    "\033[94m"
#define ANSI_CYAN    "\033[96m"
#define ANSI_WHITE   "\033[97m"
#define ANSI_ERASE   "\033[K"

// Number of printed lines — must stay in sync with color_sensor_calib_print()
#define DISPLAY_LINES 10
// Print every ~500 ms (slow refresh helps reading values during calibration)
#define PRINT_EVERY_N 25

static ColorSensor* mp_sensor         = nullptr;
static float        m_color_raw_Hz[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static float        m_color_avg_Hz[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static float        m_color_cal[4]    = {0.0f, 0.0f, 0.0f, 0.0f};
static int          m_color_num       = -1;
static const char*  m_color_string    = nullptr;
static bool         m_task_active     = false;
static int          m_print_counter   = 0;
static bool         m_first_print     = true;

static const char* get_ansi_color(int color_num)
{
    switch (color_num) {
        case 1:  return "\033[90m";  // BLACK  — dark gray
        case 2:  return ANSI_WHITE;  // WHITE
        case 3:  return ANSI_RED;    // RED
        case 4:  return ANSI_YELLOW; // YELLOW
        case 5:  return ANSI_GREEN;  // GREEN
        case 6:  return ANSI_CYAN;   // CYAN
        case 7:  return ANSI_BLUE;   // BLUE
        case 8:  return "\033[95m";  // MAGENTA
        default: return ANSI_RESET;  // UNKNOWN
    }
}

void color_sensor_calib_init(int loops_per_second)
{
    (void)loops_per_second;

    // Our wiring — 6-pin constructor: freq_out, led_enable, S0, S1, S2, S3
    // PH_0 is our LED pin (differs from library default PB_14!)
    static ColorSensor sensor(PB_3, PC_3, PA_4, PB_0, PC_1, PC_0);
    mp_sensor = &sensor;
    mp_sensor->switchLed(ON); // keep the 4 illumination LEDs permanently on
    m_first_print = true;
}

void color_sensor_calib_task(DigitalOut& led1)
{
    led1 = 1;
    m_task_active = true;

    const float* raw = mp_sensor->readRawColor();
    const float* avg = mp_sensor->readColor();
    const float* cal = mp_sensor->readColorCalib();

    for (int i = 0; i < 4; i++) {
        m_color_raw_Hz[i] = raw[i];
        m_color_avg_Hz[i] = avg[i];
        m_color_cal[i]    = cal[i];
    }

    m_color_num    = mp_sensor->getColor();
    m_color_string = mp_sensor->getColorString(m_color_num);
}

void color_sensor_calib_reset(DigitalOut& led1)
{
    led1 = 0;
    m_task_active = false;

    for (int i = 0; i < 4; i++) {
        m_color_raw_Hz[i] = 0.0f;
        m_color_avg_Hz[i] = 0.0f;
        m_color_cal[i]    = 0.0f;
    }
    m_color_num    = -1;
    m_color_string = nullptr;
    m_first_print  = true;
}

void color_sensor_calib_print()
{
    m_print_counter++;
    if (m_print_counter < PRINT_EVERY_N)
        return;
    m_print_counter = 0;

    if (!m_first_print)
        printf("\033[%dA", DISPLAY_LINES);
    m_first_print = false;

    const char* det_color = (m_color_num >= 0) ? get_ansi_color(m_color_num) : ANSI_RESET;
    const char* det_name  = m_color_string ? m_color_string : "---";
    const char* status    = m_task_active
                            ? ANSI_GREEN  ANSI_BOLD "[ACTIVE] " ANSI_RESET
                            : ANSI_YELLOW ANSI_BOLD "[STOPPED]" ANSI_RESET;

    // ---- DISPLAY_LINES = 10 lines below ----
    printf(ANSI_CYAN ANSI_BOLD "==== Color Sensor Calibration | TCS3200 ===========" ANSI_RESET ANSI_ERASE "\n");
    printf(ANSI_CYAN " Hold a colour in front of the sensor, note the" ANSI_RESET ANSI_ERASE "\n");
    printf(ANSI_CYAN " Avg Hz values for R / G / B / W for each colour." ANSI_RESET ANSI_ERASE "\n");
    printf(ANSI_CYAN "----------------------------------------------------" ANSI_RESET ANSI_ERASE "\n");
    printf(ANSI_BOLD " Raw Hz  " ANSI_RESET
           "| " ANSI_RED   "R:%8.1f" ANSI_RESET
           "  " ANSI_GREEN "G:%8.1f" ANSI_RESET
           "  " ANSI_BLUE  "B:%8.1f" ANSI_RESET
           "  " ANSI_WHITE "W:%8.1f" ANSI_RESET
           ANSI_ERASE "\n",
           m_color_raw_Hz[0], m_color_raw_Hz[1], m_color_raw_Hz[2], m_color_raw_Hz[3]);
    printf(ANSI_BOLD " Avg Hz  " ANSI_RESET
           "| " ANSI_RED   "R:%8.1f" ANSI_RESET
           "  " ANSI_GREEN "G:%8.1f" ANSI_RESET
           "  " ANSI_BLUE  "B:%8.1f" ANSI_RESET
           "  " ANSI_WHITE "W:%8.1f" ANSI_RESET
           ANSI_ERASE "\n",
           m_color_avg_Hz[0], m_color_avg_Hz[1], m_color_avg_Hz[2], m_color_avg_Hz[3]);
    printf(ANSI_BOLD " Calibr. " ANSI_RESET
           "| " ANSI_RED   "R:%8.3f" ANSI_RESET
           "  " ANSI_GREEN "G:%8.3f" ANSI_RESET
           "  " ANSI_BLUE  "B:%8.3f" ANSI_RESET
           "  " ANSI_WHITE "W:%8.3f" ANSI_RESET
           ANSI_ERASE "\n",
           m_color_cal[0], m_color_cal[1], m_color_cal[2], m_color_cal[3]);
    printf(ANSI_CYAN "----------------------------------------------------" ANSI_RESET ANSI_ERASE "\n");
    printf(" Detected: " ANSI_BOLD "%s>>> %-6s <<<" ANSI_RESET "   %s" ANSI_ERASE "\n",
           det_color, det_name, status);
    printf(ANSI_CYAN "====================================================" ANSI_RESET ANSI_ERASE "\n");
    // ---- end of DISPLAY_LINES = 10 lines ----
}

#endif // TEST_COLOR_SENSOR_CALIB
