#include "test_config.h"

#ifdef TEST_COLOR_SENSOR

#include "test_color_sensor.h"

// ANSI escape codes
#define ANSI_RESET   "\033[0m"
#define ANSI_BOLD    "\033[1m"
#define ANSI_RED     "\033[91m"
#define ANSI_GREEN   "\033[92m"
#define ANSI_YELLOW  "\033[93m"
#define ANSI_BLUE    "\033[94m"
#define ANSI_CYAN    "\033[96m"
#define ANSI_WHITE   "\033[97m"
#define ANSI_ERASE   "\033[K"        // erase to end of line

#define DISPLAY_LINES 7              // number of lines printed — must stay in sync with color_sensor_print()
#define PRINT_EVERY_N 12             // print every ~240 ms at 20 ms task period

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

void color_sensor_init(int loops_per_second)
{
    (void)loops_per_second;

    // pin order: freq_out, led_enable, S0, S1, S2, S3
    static ColorSensor sensor(PB_3, PC_3, PA_4,PB_0, PC_1, PC_0);
    mp_sensor = &sensor;
    mp_sensor->switchLed(ON); // keep the 4 illumination LEDs permanently on
    m_first_print = true;
}

void color_sensor_task(DigitalOut& led1)
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

void color_sensor_reset(DigitalOut& led1)
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

void color_sensor_print()
{
    m_print_counter++;
    if (m_print_counter < PRINT_EVERY_N)
        return;
    m_print_counter = 0;

    // after first print, move cursor back up to overwrite previous output
    if (!m_first_print)
        printf("\033[%dA", DISPLAY_LINES);
    m_first_print = false;

    const char* det_color  = (m_color_num >= 0) ? get_ansi_color(m_color_num) : ANSI_RESET;
    const char* det_name   = m_color_string ? m_color_string : "---";
    const char* status     = m_task_active
                             ? ANSI_GREEN  ANSI_BOLD " [ACTIVE]  " ANSI_RESET
                             : ANSI_YELLOW ANSI_BOLD " [STOPPED] " ANSI_RESET;

    // ---- DISPLAY_LINES = 7 lines below ----
    printf(ANSI_CYAN "---- Color Sensor | TCS3200 --------------------------------" ANSI_RESET ANSI_ERASE "\n");
    printf(" Raw Hz  | R:%8.1f  G:%8.1f  B:%8.1f  W:%8.1f" ANSI_ERASE "\n",
           m_color_raw_Hz[0], m_color_raw_Hz[1], m_color_raw_Hz[2], m_color_raw_Hz[3]);
    printf(" Avg Hz  | R:%8.1f  G:%8.1f  B:%8.1f  W:%8.1f" ANSI_ERASE "\n",
           m_color_avg_Hz[0], m_color_avg_Hz[1], m_color_avg_Hz[2], m_color_avg_Hz[3]);
    printf(" Calibr. | R:%8.3f  G:%8.3f  B:%8.3f  W:%8.3f" ANSI_ERASE "\n",
           m_color_cal[0], m_color_cal[1], m_color_cal[2], m_color_cal[3]);
    printf(ANSI_CYAN "------------------------------------------------------------" ANSI_RESET ANSI_ERASE "\n");
    printf(" Detected: " ANSI_BOLD "%s>>> %-6s <<<" ANSI_RESET "  %s  (press blue button to toggle)" ANSI_ERASE "\n",
           det_color, det_name, status);
    printf(ANSI_CYAN "------------------------------------------------------------" ANSI_RESET ANSI_ERASE "\n");
    // ---- end of DISPLAY_LINES = 7 lines ----
}

#endif // TEST_COLOR_SENSOR
