/**
 * V. Hunter Adams (vha3@cornell.edu)
 *
 * Based on the MPU6050 VGA demo. MODIFIED for Lab 3 Week 2:
 *   Top plot    : beam angle (degrees, +/-180)
 *                   WHITE = accel-only angle, raw axes (noisy, no drift)
 *                   RED   = gyro-only integrated angle (smooth, drifts)
 *                   GREEN = complementary filter (fused estimate)
 *   Bottom plot : low-passed motor command (PWM duty counts, 0..WRAPVAL)
 *   P control runs in the 1 kHz ISR. Gain and setpoint are changed live from a
 *   USB serial terminal (PuTTY, any baud). Requires
 *   pico_enable_stdio_usb(<target> 1) in CMake.
 *
 * SERIAL COMMANDS (type a line, press Enter):
 *   kp <v>   set the proportional gain (duty counts per degree)
 *   sp <v>   set the target angle in degrees
 *   on / off enable / disable the controller (starts OFF, kp starts 0)
 *   p        print parameters and current angle
 *   t <n>    VGA time step (store every n-th sample)
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 470 ohm resistor ---> VGA Green
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *  - GND ---> VGA GND
 *  - GPIO 8 ---> MPU6050 SDA
 *  - GPIO 9 ---> MPU6050 SCL
 *  - 3.3v ---> MPU6050 VCC
 *  - GND ---> MPU6050 GND
 *  - GPIO 4 (PWM slice 2, channel A) ---> motor driver  (ASSUMED, see MOTOR_CHAN)
 */


// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/stdio_usb.h"
// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
// Include custom libraries
#include "VGA/vga16_graphics_v3.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1_4.h"

// We will start drawing at column 81
#define LEFT_EDGE 81

// Plot geometry
#define TOP_PLOT_TOP 80
#define TOP_PLOT_BOTTOM 230
#define BOTTOM_PLOT_TOP 280
#define BOTTOM_PLOT_BOTTOM 430
#define NEW_RANGE 150.      // pixels per plot
#define PLOT_WIDTH 530

// Angle plot range (degrees)
#define ANGLE_MIN -180.
#define ANGLE_RANGE 360.

// PWM parameters (1 kHz interrupt, also the sample rate for everything below)
#define WRAPVAL 6000
#define CLKDIV  25.0
uint slice_num ;

// ---- Tunables / conventions (CHECK THESE against your mounting) ----------
// Which accel axes form the tilt angle: angle = atan2(-a[ACC_NUM], a[ACC_DEN])
// (the course site uses x over y). Which gyro axis rotates with the beam:
// Beam rotates about the IMU y axis: gyro y measures it, and the tilt angle
// comes from the two accel axes perpendicular to y (x and z).
#define ACC_NUM 0
#define ACC_DEN 2
#define GYRO_AXIS 1
// +1 or -1. If the green/red traces move OPPOSITE to white when you tilt,
// flip this.
#define GYRO_SIGN -1
// Complementary filter weights (must sum to ~1). Gyro weight / accel weight.
#define GYRO_W zeropt99
#define ACC_W  zeropt01
// Low-pass on raw accel axes (before atan2) and on the motor command.
// alpha 0.1 at 1 kHz is a ~10 ms time constant.
#define LP_ALPHA zeropt1
#define CMD_ALPHA zeropt1
// Which PWM channel drives the motor. GPIO 4 = channel A, GPIO 5 = channel B.
#define MOTOR_CHAN PWM_CHAN_A
// +1 if MORE motor thrust RAISES the angle value (so a positive error,
// setpoint - angle, calls for more thrust). -1 if more thrust LOWERS it.
#define CTRL_SIGN 1
// Max motor command in fix15 duty counts
#define DUTY_MAX ((long long)WRAPVAL << 16)
// -------------------------------------------------------------------------

// Raw measurements
fix15 acceleration[3], gyro[3];

// Filter state (written only by the ISR)
fix15 ax_f = 0, ay_f = 0;            // low-passed accel axes
fix15 accel_angle_raw = 0;           // from raw axes (display, unfiltered)
fix15 accel_angle = 0;               // from low-passed axes (feeds the filter)
fix15 gyro_angle = 0;                // gyro-only integration
fix15 comp_angle = 0;                // complementary filter output

// P-controller parameters, changed from the serial thread. Gain starts at
// zero and the controller starts off so nothing spins until you ask for it.
volatile fix15 kp = 0;               // duty counts per degree
volatile fix15 setpoint = 0;         // target angle, degrees
volatile int ctrl_on = 0;

// Motor command (duty counts, fix15). Written by the ISR.
volatile fix15 motor_cmd_raw = 0;
fix15 motor_cmd_lp = 0;              // low-passed command (display only)

// Circular display buffers
fix15 buf_acc[PLOT_WIDTH] ;
fix15 buf_gyro[PLOT_WIDTH] ;
fix15 buf_comp[PLOT_WIDTH] ;
fix15 buf_cmd[PLOT_WIDTH] ;

// Pointer to head of buffer
volatile int head = 0 ;

// character array
char screentext[40];

// draw speed: store every Nth sample (1 kHz / threshold)
volatile int threshold = 10 ;
volatile int throttle = 10 ;

// semaphore
static struct pt_sem vga_semaphore ;

// Angle in degrees (fix15) from two accel axes
static inline fix15 accel_to_angle(fix15 num_axis, fix15 den_axis) {
    return float2fix15(atan2f(-fix2float15(num_axis), fix2float15(den_axis)) * 57.29578f);
}

// fix15 multiply with a 64-bit result (fix15 only holds +/-32768, and
// gain * error can exceed that before we clamp)
static inline long long mul64(fix15 a, fix15 b) {
    return ((long long)a * (long long)b) >> 16;
}

// Map a value to a screen row inside a plot, clamped to the plot box
static inline int plot_y(int bottom, float value, float vmin, float vrange) {
    int y = bottom - (int)(NEW_RANGE * ((value - vmin) / vrange));
    if (y < bottom - (int)NEW_RANGE) y = bottom - (int)NEW_RANGE;
    if (y > bottom) y = bottom;
    return y;
}

void drawAxes() {
    setTextSize(1) ;
    setTextColor(WHITE);

    // Bottom plot: motor command (low-passed), 0..WRAPVAL
    drawHLine(75, BOTTOM_PLOT_BOTTOM, 5, CYAN) ;
    drawHLine(75, 355, 5, CYAN) ;
    drawHLine(75, BOTTOM_PLOT_TOP, 5, CYAN) ;
    drawVLine(80, BOTTOM_PLOT_TOP, 150, CYAN) ;
    sprintf(screentext, "0") ;
    setCursor(55, 425) ;
    writeString(screentext) ;
    sprintf(screentext, "3000") ;
    setCursor(35, 350) ;
    writeString(screentext) ;
    sprintf(screentext, "6000") ;
    setCursor(35, 275) ;
    writeString(screentext) ;
    sprintf(screentext, "Motor cmd (low-passed)") ;
    setCursor(LEFT_EDGE, 265) ;
    writeString(screentext) ;

    // Top plot: beam angle in degrees
    drawHLine(75, TOP_PLOT_BOTTOM, 5, CYAN) ;
    drawHLine(75, 155, 5, CYAN) ;
    drawHLine(75, TOP_PLOT_TOP, 5, CYAN) ;
    drawVLine(80, TOP_PLOT_TOP, 150, CYAN) ;
    sprintf(screentext, "0") ;
    setCursor(55, 150) ;
    writeString(screentext) ;
    sprintf(screentext, "+180") ;
    setCursor(35, 75) ;
    writeString(screentext) ;
    sprintf(screentext, "-180") ;
    setCursor(35, 225) ;
    writeString(screentext) ;

    // Legend for the angle traces
    setTextColor(WHITE) ;
    sprintf(screentext, "ACCEL") ;
    setCursor(440, 60) ;
    writeString(screentext) ;
    setTextColor(RED) ;
    sprintf(screentext, "GYRO") ;
    setCursor(490, 60) ;
    writeString(screentext) ;
    setTextColor(GREEN) ;
    sprintf(screentext, "COMP") ;
    setCursor(535, 60) ;
    writeString(screentext) ;
    setTextColor(WHITE) ;
}

// Interrupt service routine: 1 kHz
void on_pwm_wrap() {

    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(5));
    gpio_put(14, !gpio_get(14)) ;

    // Read the IMU (15.16 fixed point; accel in g, gyro in deg/s)
    mpu6050_read_raw(acceleration, gyro);

    // Accel angle from RAW axes (unfiltered, for display)
    accel_angle_raw = accel_to_angle(acceleration[ACC_NUM], acceleration[ACC_DEN]);

    // Low-pass the accel axes BEFORE atan2, then angle from filtered axes
    ax_f += multfix15(LP_ALPHA, acceleration[ACC_NUM] - ax_f);
    ay_f += multfix15(LP_ALPHA, acceleration[ACC_DEN] - ay_f);
    accel_angle = accel_to_angle(ax_f, ay_f);

    // Gyro delta = rate * dt (dt = 0.001 s). Gyro gets NO pre-filter.
    fix15 gyro_delta = multfix15(gyro[GYRO_AXIS], zeropt001);
    if (GYRO_SIGN > 0) {
        gyro_angle -= gyro_delta;
    } else {
        gyro_angle += gyro_delta;
    }

    // Complementary filter: high-pass the gyro, low-pass the accel
    fix15 comp_prev = comp_angle;
    if (GYRO_SIGN > 0) {
        comp_prev -= gyro_delta;
    } else {
        comp_prev += gyro_delta;
    }
    comp_angle = multfix15(comp_prev, GYRO_W) + multfix15(accel_angle, ACC_W);

    // ---------------- P control ----------------
    if (ctrl_on) {
        // Positive err means "needs more thrust"
        fix15 err = setpoint - comp_angle;
        if (CTRL_SIGN < 0) err = -err;

        // P term: duty counts = kp * error. 64-bit because gain * error
        // can exceed the fix15 range before we clamp.
        long long out = mul64(kp, err);

        // Clamp to the motor's range (the prop can only push one way)
        if (out > DUTY_MAX) out = DUTY_MAX;
        if (out < 0) out = 0;

        motor_cmd_raw = (fix15)out;
        pwm_set_chan_level(slice_num, MOTOR_CHAN, (uint16_t)(out >> 16));
    } else {
        // Controller off: motor off
        motor_cmd_raw = 0;
        pwm_set_chan_level(slice_num, MOTOR_CHAN, 0);
    }

    // Low-pass the motor command (display only)
    motor_cmd_lp += multfix15(CMD_ALPHA, motor_cmd_raw - motor_cmd_lp);

    // Decimate for display
    throttle += 1 ;
    if (throttle >= threshold) {
        throttle = 0 ;

        buf_acc[head]  = accel_angle_raw ;
        buf_gyro[head] = gyro_angle ;
        buf_comp[head] = comp_angle ;
        buf_cmd[head]  = motor_cmd_lp ;

        head = ((head+1) == PLOT_WIDTH) ? 0 : (head + 1) ;
    }
}

static PT_THREAD (protothread_draw(struct pt *pt))
{
    PT_BEGIN(pt) ;

    static int start ;
    static int i ;

    // Draw axes
    drawAxes() ;
    // Copy to other buffer
    copy_buffer_to_other() ;

    while(1) {
        // Wait for a buffer swap
        PT_YIELD_UNTIL(pt, draw_start_signal()) ;
        gpio_put(15, !gpio_get(15)) ;

        // Clear the buffer, draw the axes
        clearLowFrame(0, BLACK) ;
        drawAxes() ;

        // Plot data oldest-->newest
        start = head ;
        for (i = 0; i < PLOT_WIDTH; i++) {
            drawPixel(LEFT_EDGE + i,
                plot_y(TOP_PLOT_BOTTOM, fix2float15(buf_acc[start]), ANGLE_MIN, ANGLE_RANGE), WHITE) ;
            drawPixel(LEFT_EDGE + i,
                plot_y(TOP_PLOT_BOTTOM, fix2float15(buf_gyro[start]), ANGLE_MIN, ANGLE_RANGE), RED) ;
            drawPixel(LEFT_EDGE + i,
                plot_y(TOP_PLOT_BOTTOM, fix2float15(buf_comp[start]), ANGLE_MIN, ANGLE_RANGE), GREEN) ;
            drawPixel(LEFT_EDGE + i,
                plot_y(BOTTOM_PLOT_BOTTOM, fix2float15(buf_cmd[start]), 0., (float)WRAPVAL), GREEN) ;
            start = ((start+1)==PLOT_WIDTH) ? 0 : (start+1) ;
        }
    }

    PT_END(pt) ;
}

// Print the current parameters and state
static void print_status(void) {
    printf("kp=%.3f sp=%.1f deg | ctrl=%s | angle=%.1f deg | cmd=%.0f/%d\r\n",
           fix2float15(kp), fix2float15(setpoint),
           ctrl_on ? "ON" : "OFF",
           fix2float15(comp_angle), fix2float15(motor_cmd_raw), WRAPVAL) ;
}

// USB serial command thread (PuTTY). Non-blocking line reader, same
// pattern as the working PWM USB demo.
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt) ;
    static char line[40] ;
    static char cmd[16] ;
    static int len, c, n ;
    static float val ;

    // Wait for a terminal to open the port so the banner is not lost.
    // Only this thread waits; the control loop and VGA keep running.
    while (!stdio_usb_connected()) {
        PT_YIELD_usec(100000) ;
    }
    printf("\r\nCommands: kp|sp <value>, on, off, p, t <n>\r\n") ;
    print_status() ;

    while (1) {
        printf("> ") ;
        len = 0 ;

        // Collect one line
        while (1) {
            c = getchar_timeout_us(0) ;
            if (c == PICO_ERROR_TIMEOUT) {
                PT_YIELD_usec(1000) ;
                continue ;
            }
            if (c == '\r' || c == '\n') {
                if (len == 0) continue ;          // ignore stray \n after \r
                printf("\r\n") ;
                break ;
            }
            if ((c == '\b' || c == 127) && len > 0) {
                len-- ;
                printf("\b \b") ;
                continue ;
            }
            if (c >= 32 && c < 127 && len < (int)sizeof(line) - 1) {
                line[len++] = (char)c ;
                putchar(c) ;                       // echo
            }
        }
        line[len] = '\0' ;

        // Parse "<cmd> [value]"
        val = 0 ;
        n = sscanf(line, "%15s %f", cmd, &val) ;
        if (n < 1) continue ;

        if (!strcmp(cmd, "on")) {
            ctrl_on = 1 ;
        } else if (!strcmp(cmd, "off")) {
            ctrl_on = 0 ;
        } else if (!strcmp(cmd, "p")) {
            // just print below
        } else if (n == 2 && val >= 0.0f && val < 30000.0f && !strcmp(cmd, "kp")) {
            kp = float2fix15(val) ;
        } else if (n == 2 && fabsf(val) <= 180.0f && !strcmp(cmd, "sp")) {
            setpoint = float2fix15(val) ;
        } else if (n == 2 && val >= 1.0f && !strcmp(cmd, "t")) {
            threshold = (int)val ;
        } else {
            printf("? (kp|sp <value>, on, off, p, t <n>)\r\n") ;
        }
        print_status() ;
    }
    PT_END(pt) ;
}

// Entry point for core 1
void core1_entry() {
    gpio_init(15) ;
    gpio_set_dir(15, GPIO_OUT) ;
    gpio_put(15, 0) ;
    pt_add_thread(protothread_draw) ;
    pt_schedule_start ;
}

int main() {

    // Overclock
    set_sys_clock_khz(150000, true) ;

    // Initialize stdio
    stdio_init_all();

    // Initialize VGA
    initVGA() ;

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;
    // Required on our setup: without these the bus floats and init hangs
    gpio_pull_up(SDA_PIN) ;
    gpio_pull_up(SCL_PIN) ;

    // MPU6050 initialization
    mpu6050_reset();
    mpu6050_read_raw(acceleration, gyro);

    // Seed every angle estimate from the first accel reading so the
    // filter does not start at 0 and spend seconds converging
    ax_f = acceleration[ACC_NUM] ;
    ay_f = acceleration[ACC_DEN] ;
    accel_angle = accel_to_angle(ax_f, ay_f) ;
    accel_angle_raw = accel_angle ;
    gyro_angle = accel_angle ;
    comp_angle = accel_angle ;

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO's 4,5 that they allocated to the PWM
    gpio_set_function(5, GPIO_FUNC_PWM);
    gpio_set_function(4, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 5 (it's slice 2, same for 4)
    slice_num = pwm_gpio_to_slice_num(5);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;

    // Motor off at start
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));


    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // start core 1
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // start core 0
    gpio_init(14) ;
    gpio_set_dir(14, GPIO_OUT) ;
    gpio_put(14, 0) ;
    pt_add_thread(protothread_serial) ;
    pt_schedule_start ;

}