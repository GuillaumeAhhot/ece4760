/**
 * ECE 4760 Lab 3: beam control and automatic PID experiments
 *
 * Commands:
 *   kp <v>, ki <v>, kd <v>, sp <v>
 *   on, off, p, v, t <n>
 *   tune  - baseline, P/D sweep, then I sweep
 *   range - angle tests using the current gains
 *
 * kd is now in PWM counts per (deg/s), with NO /1024 scaling.
 * Old kd = 29999 corresponds to new kd = 29.2959.
 *
 * Button GPIO 11: abort and latch motor off.
 * Release does not restart the motor.
 *
 * Tests use the complementary angle, not the display-filtered command.
 * CSV results are printed after each observation window.
 *
 * Pins:
 *   VGA 16-21
 *   MPU6050 SDA 8 / SCL 9
 *   Motor PWM GPIO 4
 *   Button GPIO 11
 *   Debug GPIO 14 / 15
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/stdio_usb.h"

#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"

#include "VGA/vga16_graphics_v3.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1_4.h"

/* These calculations follow the 16 fractional bits in the supplied code. */
#define LEFT_EDGE 81
#define TOP_PLOT_TOP 80
#define TOP_PLOT_BOTTOM 230
#define BOTTOM_PLOT_TOP 280
#define BOTTOM_PLOT_BOTTOM 430
#define PLOT_WIDTH 530
#define TERM_RANGE 3000.0f

#define WRAPVAL 6000
#define CLKDIV 25.0f
#define MOTOR_CHAN PWM_CHAN_A

#define ACC_NUM 0
#define ACC_DEN 2
#define GYRO_AXIS 1
#define GYRO_SIGN -1
#define CTRL_SIGN 1

#define GYRO_W zeropt99
#define ACC_W zeropt01
#define LP_ALPHA zeropt1
#define CMD_SHIFT 6

#define DUTY_MAX ((long long)WRAPVAL << 16)
#define INT_LIMIT ((long long)100000 << 16)

#define BUTTON_PIN 11
#define ANGLE_LOW_ABORT -120.0f
#define ANGLE_HIGH_ABORT 200.0f
#define MANUAL_SP_LIMIT 200.0f

#define SETTLE_BAND 5.0f
#define SETTLE_DWELL_MS 2000u
#define PREP_MS 15000u
#define STEP_MS 10000u
#define FINAL_WINDOW_MS 2000u
#define METRIC_INTERVAL_US 10000u
#define TUNE_STEP 15.0f

uint slice_num;

fix15 acceleration[3], gyro[3];
fix15 ax_f = 0, ay_f = 0;
fix15 accel_angle_raw = 0;
fix15 accel_angle = 0;
fix15 gyro_angle = 0;
fix15 comp_angle = 0;

volatile fix15 kp = 0;
volatile fix15 ki = 0;
volatile fix15 kd = 0;
volatile fix15 setpoint = 0;
volatile int ctrl_on = 0;
volatile int plot_view = 0;

long long integral = 0;
fix15 term_p = 0, term_i = 0, term_d = 0;
volatile fix15 motor_cmd_raw = 0;
fix15 cmd_lp1 = 0;
fix15 motor_cmd_lp = 0;

fix15 buf_acc[PLOT_WIDTH];
fix15 buf_gyro[PLOT_WIDTH];
fix15 buf_comp[PLOT_WIDTH];
fix15 buf_cmd[PLOT_WIDTH];
fix15 buf_cmd_raw[PLOT_WIDTH];
fix15 buf_p[PLOT_WIDTH];
fix15 buf_i[PLOT_WIDTH];
fix15 buf_d[PLOT_WIDTH];

volatile int head = 0;
volatile int threshold = 40;
volatile int throttle = 0;

/* Faults stay latched until an explicit valid start command. */
enum {
    FAULT_NONE,
    FAULT_BUTTON,
    FAULT_ANGLE,
    FAULT_USER
};

volatile int fault_code = FAULT_NONE;

/* Core 0 owns experiment state; core 1 only displays it. */
enum {
    MODE_IDLE,
    MODE_TUNE,
    MODE_RANGE
};

enum {
    PHASE_PREP,
    PHASE_STEP
};

volatile int test_mode = MODE_IDLE;
volatile int test_phase = PHASE_PREP;
volatile int candidate_number = 0;

typedef struct {
    float p, i, d;
} Gains;

static const float p_values[] = {
    100.0f, 125.0f, 150.0f, 175.0f, 200.0f
};

static const float d_values[] = {
    15.0f, 22.0f, 29.2959f, 40.0f, 55.0f
};

static const float i_values[] = {
    0.0125f, 0.025f, 0.05f, 0.075f, 0.10f
};

static Gains original_gains;
static Gains active_gains;
static Gains best_gains;
static Gains pd_winner;

static float best_score;
static float candidate_score;
static int tuning_stage;
static int tuning_index;
static int step_index;
static int range_index;
static bool range_positive_failed;
static bool range_negative_failed;
static float range_positive_pass;
static float range_negative_pass;
static bool have_winner;

/*
 * Only the ISR updates an active metric window.
 * Core 0 copies a completed window with interrupts briefly disabled.
 */
typedef struct {
    uint32_t start_us;
    uint32_t duration_ms;
    uint32_t elapsed_ms;
    uint32_t samples;
    uint32_t final_samples;
    uint32_t saturated;
    uint32_t last_outside_ms;
    float target;
    float start_angle;
    float abs_error_sum;
    float final_error_sum;
    float final_angle_sum;
    float final_angle_sq_sum;
    float peak_overshoot;
} Metrics;

static volatile Metrics metrics;
static volatile bool metrics_active = false;
static volatile bool metrics_done = false;
static uint32_t metric_last_us;

/* Convert the selected accelerometer axes to degrees. */
static inline fix15 accel_to_angle(fix15 num, fix15 den) {
    return float2fix15(
        atan2f(-fix2float15(num), fix2float15(den)) * 57.29578f
    );
}

/* Keep gain products in 64 bits. */
static inline long long mul64(fix15 a, fix15 b) {
    return ((long long)a * (long long)b) >> 16;
}

/* Limit only the displayed PID terms. */
static inline fix15 clamp_term(long long v) {
    const long long limit = (long long)30000 << 16;
    if (v > limit) v = limit;
    if (v < -limit) v = -limit;
    return (fix15)v;
}

/* Read a consistent set of gains on core 0. */
static Gains read_gains(void) {
    Gains g;
    g.p = fix2float15(kp);
    g.i = fix2float15(ki);
    g.d = fix2float15(kd);
    return g;
}

/* Change all gains together; reset the integral between candidates only. */
static void apply_gains(Gains g) {
    uint32_t irq = save_and_disable_interrupts();

    kp = float2fix15(g.p);
    ki = float2fix15(g.i);
    kd = float2fix15(g.d);
    integral = 0;

    restore_interrupts(irq);
}

/* Abort immediately and leave the motor disabled. */
static void stop_motor(int reason) {
    uint32_t irq = save_and_disable_interrupts();

    ctrl_on = 0;
    fault_code = reason;
    metrics_active = false;
    metrics_done = false;
    motor_cmd_raw = 0;
    integral = 0;
    pwm_set_chan_level(slice_num, MOTOR_CHAN, 0);

    restore_interrupts(irq);
}

/* An explicit command is required to clear a stop latch. */
static bool enable_motor(void) {
    bool allowed;
    uint32_t irq = save_and_disable_interrupts();
    float angle = fix2float15(comp_angle);

    allowed = gpio_get(BUTTON_PIN)
           && angle > ANGLE_LOW_ABORT
           && angle < ANGLE_HIGH_ABORT;

    if (allowed) {
        fault_code = FAULT_NONE;
        ctrl_on = 1;
    }

    restore_interrupts(irq);
    return allowed;
}

/* Set the target and start an observation in one atomic operation. */
static void begin_window(float target, uint32_t duration_ms) {
    uint32_t irq = save_and_disable_interrupts();

    metrics_active = false;
    metrics_done = false;
    metrics = (Metrics){0};

    setpoint = float2fix15(target);
    metrics.target = target;
    metrics.start_angle = fix2float15(comp_angle);
    metrics.duration_ms = duration_ms;
    metrics.start_us = time_us_32();
    metric_last_us = metrics.start_us;

    metrics_active = true;
    restore_interrupts(irq);
}

/* Copy only after the ISR has finished the observation. */
static bool take_window(Metrics *out) {
    bool ready;
    uint32_t irq = save_and_disable_interrupts();

    ready = metrics_done;
    if (ready) {
        *out = metrics;
        metrics_done = false;
    }

    restore_interrupts(irq);
    return ready;
}

/* Accumulate measurements at approximately 100 Hz without serial output. */
static void update_metrics(void) {
    if (!metrics_active) return;

    uint32_t now = time_us_32();
    if ((uint32_t)(now - metric_last_us) < METRIC_INTERVAL_US) return;
    metric_last_us = now;

    uint32_t elapsed = (now - metrics.start_us) / 1000u;
    float angle = fix2float15(comp_angle);
    float error = metrics.target - angle;
    float delta = metrics.target - metrics.start_angle;

    metrics.elapsed_ms = elapsed;
    metrics.samples++;
    metrics.abs_error_sum += fabsf(error);

    if (fabsf(error) > SETTLE_BAND) {
        metrics.last_outside_ms = elapsed;
    }

    if (motor_cmd_raw <= 0 || motor_cmd_raw >= DUTY_MAX) {
        metrics.saturated++;
    }

    float overshoot = 0.0f;
    if (delta > 0.1f) overshoot = angle - metrics.target;
    if (delta < -0.1f) overshoot = metrics.target - angle;
    if (overshoot > metrics.peak_overshoot) {
        metrics.peak_overshoot = overshoot;
    }

    if (elapsed >= metrics.duration_ms - FINAL_WINDOW_MS) {
        metrics.final_samples++;
        metrics.final_error_sum += error;
        metrics.final_angle_sum += angle;
        metrics.final_angle_sq_sum += angle * angle;
    }

    if (elapsed >= metrics.duration_ms) {
        metrics_active = false;
        metrics_done = true;
    }
}

/* Return an observed settling time, or -1 if it did not settle. */
static int settling_ms(const Metrics *m) {
    if (!m->samples) return -1;

    if (m->elapsed_ms - m->last_outside_ms < SETTLE_DWELL_MS) {
        return -1;
    }

    /*
     * The last outside sample brackets the settling instant to about 10 ms.
     * Add one sample interval for a conservative reported estimate.
     */
    if (m->last_outside_ms == 0) return 0;
    return (int)m->last_outside_ms + 10;
}

/* Lower scores favor smaller tracking error, overshoot, and settling time. */
static float window_score(const Metrics *m) {
    int settle = settling_ms(m);
    if (settle < 0 || !m->samples) return 1.0e30f;

    return m->abs_error_sum / m->samples
         + 0.5f * m->peak_overshoot
         + 0.5f * (settle / 1000.0f);
}

/* Print one CSV record after a window completes. */
static void print_result(const char *kind, const Metrics *m) {
    float final_error = 0.0f;
    float final_sd = 0.0f;

    if (m->final_samples) {
        float mean = m->final_angle_sum / m->final_samples;
        float variance = m->final_angle_sq_sum / m->final_samples
                       - mean * mean;

        final_error = m->final_error_sum / m->final_samples;
        final_sd = sqrtf(fmaxf(variance, 0.0f));
    }

    printf("%s,%d,%d,%.4f,%.5f,%.4f,%.2f,%.2f,%d,"
           "%.3f,%.3f,%.3f,%.3f,%.2f,%s\r\n",
           kind, tuning_stage, candidate_number,
           active_gains.p, active_gains.i, active_gains.d,
           m->start_angle, m->target, settling_ms(m),
           m->samples ? m->abs_error_sum / m->samples : 0.0f,
           m->peak_overshoot, final_error, final_sd,
           m->samples ? 100.0f * m->saturated / m->samples : 0.0f,
           settling_ms(m) >= 0 ? "PASS" : "NO_SETTLE");
}

/* Print the CSV column names once per experiment. */
static void print_csv_header(void) {
    printf("kind,stage,candidate,kp,ki,kd,start_deg,target_deg,"
           "settle_ms,mean_abs_error_deg,overshoot_deg,"
           "final_error_deg,final_sd_deg,saturation_pct,result\r\n");
}

/* Start a fresh horizontal preparation without resetting the integral. */
static void prepare_horizontal(void) {
    test_phase = PHASE_PREP;
    begin_window(0.0f, PREP_MS);
}

/* Load the next tuning candidate. Candidate zero is the user's baseline. */
static void load_candidate(void) {
    if (tuning_stage == 0) {
        if (tuning_index == 0) {
            active_gains = original_gains;
        } else {
            int index = tuning_index - 1;
            active_gains.p = p_values[index / 5];
            active_gains.i = original_gains.i;
            active_gains.d = d_values[index % 5];
        }
    } else {
        active_gains = pd_winner;
        active_gains.i = i_values[tuning_index];
    }

    candidate_number++;
    candidate_score = 0.0f;
    step_index = 0;

    apply_gains(active_gains);
    prepare_horizontal();

    printf("CANDIDATE,%d,stage=%d,kp=%.4f,ki=%.5f,kd=%.4f\r\n",
           candidate_number, tuning_stage,
           active_gains.p, active_gains.i, active_gains.d);
}

/* Finish a candidate and advance to the next sweep stage if needed. */
static void finish_candidate(bool passed) {
    if (passed && candidate_score < best_score) {
        best_score = candidate_score;
        best_gains = active_gains;
        have_winner = true;
    }

    tuning_index++;

    if (tuning_stage == 0 && tuning_index >= 26) {
        if (!have_winner) {
            stop_motor(FAULT_NONE);
            test_mode = MODE_IDLE;
            apply_gains(original_gains);
            printf("TUNE_FAILED: no candidate passed both steps. "
                   "Original gains restored; motor OFF.\r\n");
            return;
        }

        pd_winner = best_gains;
        tuning_stage = 1;
        tuning_index = 0;

        /*
         * Retain the PD winner and its score so the I sweep cannot
         * replace it with a worse candidate.
         */
    } else if (tuning_stage == 1 && tuning_index >= 5) {
        stop_motor(FAULT_NONE);
        test_mode = MODE_IDLE;
        apply_gains(best_gains);

        printf("BEST,kp=%.4f,ki=%.5f,kd=%.4f,score=%.4f\r\n",
               best_gains.p, best_gains.i, best_gains.d, best_score);
        printf("Motor OFF. Use 'range' to test these gains at larger angles.\r\n");
        return;
    }

    load_candidate();
}

/* Measure every target, even if an earlier target did not settle. */
static void advance_range(void) {
    if (range_index < 21) {
        prepare_horizontal();
        return;
    }

    stop_motor(FAULT_NONE);
    test_mode = MODE_IDLE;

    printf("RANGE_DONE,most_negative_pass_deg=%.1f,"
           "most_positive_pass_deg=%.1f\r\n",
           range_negative_pass, range_positive_pass);

    printf("Check individual STEP rows for failures within the tested range. "
           "Motor OFF.\r\n");
}

/* Start a tuning run explicitly from the terminal. */
static void start_tuning(void) {
    if (!enable_motor()) {
        printf("Cannot start: button held or angle outside limits.\r\n");
        return;
    }

    original_gains = read_gains();
    best_gains = original_gains;
    best_score = 1.0e30f;
    have_winner = false;
    tuning_stage = 0;
    tuning_index = 0;
    candidate_number = 0;
    test_mode = MODE_TUNE;

    print_csv_header();
    load_candidate();
}

/* Test progressively larger targets with the currently selected gains. */
static void start_range(void) {
    if (!enable_motor()) {
        printf("Cannot start: button held or angle outside limits.\r\n");
        return;
    }

    active_gains = read_gains();
    tuning_stage = 2;
    candidate_number = 0;
    range_index = 0;
    range_positive_failed = false;
    range_negative_failed = false;
    range_positive_pass = 0.0f;
    range_negative_pass = 0.0f;
    test_mode = MODE_RANGE;

    print_csv_header();
    prepare_horizontal();
}

/* Map a value into a 150-pixel plot. */
static int plot_y(int bottom, float value, float low, float span) {
    int y = bottom - (int)(150.0f * (value - low) / span);
    if (y < bottom - 150) y = bottom - 150;
    if (y > bottom) y = bottom;
    return y;
}

/* Join consecutive samples of the main traces. */
static void draw_seg(int x, int y0, int y1, int color) {
    int top = y0 < y1 ? y0 : y1;
    drawVLine(x, top, abs(y1 - y0) + 1, color);
}

/* Draw a label without a shared formatting buffer. */
static void label(int x, int y, int color, const char *text) {
    setTextColor(color);
    setCursor(x, y);
    writeString((char *)text);
}

/* Draw scales and the live experiment status. */
static void drawAxes(void) {
    char text[120];
    setTextSize(1);

    drawVLine(80, TOP_PLOT_TOP, 150, CYAN);
    drawHLine(75, TOP_PLOT_TOP, 5, CYAN);
    drawHLine(75, 155, 5, CYAN);
    drawHLine(75, TOP_PLOT_BOTTOM, 5, CYAN);

    label(35, 75, WHITE, "+180");
    label(55, 150, WHITE, "0");
    label(35, 225, WHITE, "-180");
    label(415, 60, WHITE, "ACCEL LP");
    label(490, 60, RED, "GYRO");
    label(535, 60, GREEN, "COMP");

    drawVLine(80, BOTTOM_PLOT_TOP, 150, CYAN);
    drawHLine(75, BOTTOM_PLOT_TOP, 5, CYAN);
    drawHLine(75, 355, 5, CYAN);
    drawHLine(75, BOTTOM_PLOT_BOTTOM, 5, CYAN);

    if (plot_view == 0) {
        label(35, 275, WHITE, "6000");
        label(35, 350, WHITE, "3000");
        label(55, 425, WHITE, "0");
        label(LEFT_EDGE, 265, WHITE, "Motor command");
        label(440, 265, RED, "RAW");
        label(475, 265, GREEN, "LOW-PASS");
    } else {
        label(30, 275, WHITE, "+3000");
        label(55, 350, WHITE, "0");
        label(30, 425, WHITE, "-3000");
        label(LEFT_EDGE, 265, WHITE, "PID terms");
        label(440, 265, RED, "P");
        label(460, 265, GREEN, "I");
        label(480, 265, WHITE, "D");
    }

    snprintf(text, sizeof(text),
             "kp %.1f ki %.4f kd %.2f sp %.1f [%s]",
             fix2float15(kp), fix2float15(ki), fix2float15(kd),
             fix2float15(setpoint), ctrl_on ? "ON" : "OFF");
    label(LEFT_EDGE, 10, WHITE, text);

    snprintf(text, sizeof(text),
             "angle %.1f raw %.0f lp %.0f fault %d",
             fix2float15(comp_angle), fix2float15(motor_cmd_raw),
             fix2float15(motor_cmd_lp), fault_code);
    label(LEFT_EDGE, 25, WHITE, text);

    snprintf(text, sizeof(text), "test %s  candidate %d  phase %s",
             test_mode == MODE_TUNE ? "TUNE" :
             test_mode == MODE_RANGE ? "RANGE" : "IDLE",
             candidate_number,
             test_phase == PHASE_PREP ? "PREP" : "STEP");
    label(LEFT_EDGE, 40, WHITE, text);
}

/* Control ISR: retain the supplied estimator and controller structure. */
void on_pwm_wrap(void) {
    pwm_clear_irq(slice_num);
    gpio_put(14, !gpio_get(14));

    /*
     * Check the button before the I2C transaction.
     * A stuck I2C transaction still requires a physical power cutoff.
     */
    if (!gpio_get(BUTTON_PIN)) {
        ctrl_on = 0;
        fault_code = FAULT_BUTTON;
        pwm_set_chan_level(slice_num, MOTOR_CHAN, 0);
    }

    mpu6050_read_raw(acceleration, gyro);

    accel_angle_raw = accel_to_angle(
        acceleration[ACC_NUM], acceleration[ACC_DEN]
    );

    ax_f += multfix15(LP_ALPHA, acceleration[ACC_NUM] - ax_f);
    ay_f += multfix15(LP_ALPHA, acceleration[ACC_DEN] - ay_f);
    accel_angle = accel_to_angle(ax_f, ay_f);

    fix15 gyro_delta = multfix15(gyro[GYRO_AXIS], zeropt001);
    fix15 comp_prev = comp_angle;

    if (GYRO_SIGN > 0) {
        gyro_angle -= gyro_delta;
        comp_prev -= gyro_delta;
    } else {
        gyro_angle += gyro_delta;
        comp_prev += gyro_delta;
    }

    comp_angle = multfix15(comp_prev, GYRO_W)
               + multfix15(accel_angle, ACC_W);

    fix15 angle_rate = GYRO_SIGN > 0
                     ? -gyro[GYRO_AXIS] : gyro[GYRO_AXIS];

    float angle = fix2float15(comp_angle);

    if (ctrl_on &&
        (angle <= ANGLE_LOW_ABORT || angle >= ANGLE_HIGH_ABORT)) {
        ctrl_on = 0;
        fault_code = FAULT_ANGLE;
    }

    if (ctrl_on) {
        fix15 error = setpoint - comp_angle;
        fix15 rate = angle_rate;

        if (CTRL_SIGN < 0) {
            error = -error;
            rate = -rate;
        }

        long long p = mul64(kp, error);
        long long i = ((long long)ki * integral) >> 16;
        long long d = -mul64(kd, rate);
        long long out = p + i + d;

        term_p = clamp_term(p);
        term_i = clamp_term(i);
        term_d = clamp_term(d);

        if (!((out > DUTY_MAX && error > 0) ||
              (out < 0 && error < 0))) {
            integral += error;
            if (integral > INT_LIMIT) integral = INT_LIMIT;
            if (integral < -INT_LIMIT) integral = -INT_LIMIT;
        }

        if (ki == 0) integral = 0;
        if (out > DUTY_MAX) out = DUTY_MAX;
        if (out < 0) out = 0;

        motor_cmd_raw = (fix15)out;
        pwm_set_chan_level(slice_num, MOTOR_CHAN, (uint16_t)(out >> 16));
    } else {
        integral = 0;
        term_p = 0;
        term_i = 0;
        term_d = 0;
        motor_cmd_raw = 0;
        pwm_set_chan_level(slice_num, MOTOR_CHAN, 0);
    }

    /* Two display-only stages, about 64 ms per stage. */
    cmd_lp1 += (motor_cmd_raw - cmd_lp1) >> CMD_SHIFT;
    motor_cmd_lp += (cmd_lp1 - motor_cmd_lp) >> CMD_SHIFT;

    if (ctrl_on) update_metrics();

    throttle++;
    if (throttle >= threshold) {
        throttle = 0;

        buf_acc[head] = accel_angle;
        buf_gyro[head] = gyro_angle;
        buf_comp[head] = comp_angle;
        buf_cmd[head] = motor_cmd_lp;
        buf_cmd_raw[head] = motor_cmd_raw;
        buf_p[head] = term_p;
        buf_i[head] = term_i;
        buf_d[head] = term_d;

        head = (head + 1) % PLOT_WIDTH;
    }
}

/* Core 1 draws the existing angle and command plots. */
static PT_THREAD(protothread_draw(struct pt *pt)) {
    PT_BEGIN(pt);

    static int start, i, view;
    static int y, prev_comp, prev_cmd;

    drawAxes();
    copy_buffer_to_other();

    while (1) {
        PT_YIELD_UNTIL(pt, draw_start_signal());
        gpio_put(15, !gpio_get(15));

        clearLowFrame(0, BLACK);
        drawAxes();

        start = head;
        view = plot_view;

        for (i = 0; i < PLOT_WIDTH; i++) {
            drawPixel(LEFT_EDGE + i,
                      plot_y(TOP_PLOT_BOTTOM,
                             fix2float15(buf_acc[start]), -180, 360),
                      WHITE);

            drawPixel(LEFT_EDGE + i,
                      plot_y(TOP_PLOT_BOTTOM,
                             fix2float15(buf_gyro[start]), -180, 360),
                      RED);

            y = plot_y(TOP_PLOT_BOTTOM,
                       fix2float15(buf_comp[start]), -180, 360);

            if (i == 0) prev_comp = y;
            draw_seg(LEFT_EDGE + i, prev_comp, y, GREEN);
            prev_comp = y;

            if (view == 0) {
                drawPixel(LEFT_EDGE + i,
                          plot_y(BOTTOM_PLOT_BOTTOM,
                                 fix2float15(buf_cmd_raw[start]), 0, WRAPVAL),
                          RED);

                y = plot_y(BOTTOM_PLOT_BOTTOM,
                           fix2float15(buf_cmd[start]), 0, WRAPVAL);

                if (i == 0) prev_cmd = y;
                draw_seg(LEFT_EDGE + i, prev_cmd, y, GREEN);
                prev_cmd = y;
            } else {
                drawPixel(LEFT_EDGE + i,
                          plot_y(BOTTOM_PLOT_BOTTOM,
                                 fix2float15(buf_p[start]),
                                 -TERM_RANGE, 2 * TERM_RANGE), RED);
                drawPixel(LEFT_EDGE + i,
                          plot_y(BOTTOM_PLOT_BOTTOM,
                                 fix2float15(buf_i[start]),
                                 -TERM_RANGE, 2 * TERM_RANGE), GREEN);
                drawPixel(LEFT_EDGE + i,
                          plot_y(BOTTOM_PLOT_BOTTOM,
                                 fix2float15(buf_d[start]),
                                 -TERM_RANGE, 2 * TERM_RANGE), WHITE);
            }

            start = (start + 1) % PLOT_WIDTH;
        }
    }

    PT_END(pt);
}

/* Core 0 advances experiments after the ISR completes each window. */
static PT_THREAD(protothread_test(struct pt *pt)) {
    PT_BEGIN(pt);

    static Metrics result;
    static float target;
    static bool passed;
    static bool positive;

    while (1) {
        PT_YIELD_usec(10000);

        if (test_mode == MODE_IDLE) continue;

        if (fault_code != FAULT_NONE || !ctrl_on) {
            printf("ABORT,fault=%d,candidate=%d,angle=%.2f\r\n",
                   fault_code, candidate_number, fix2float15(comp_angle));
            stop_motor(fault_code);
            test_mode = MODE_IDLE;
            continue;
        }

        if (!take_window(&result)) continue;

        if (test_phase == PHASE_PREP) {
            print_result("PREP", &result);

            if (settling_ms(&result) < 0) {
                if (test_mode == MODE_TUNE) {
                    finish_candidate(false);
                } else {
                    printf("RANGE_ABORT: horizontal preparation failed.\r\n");
                    stop_motor(FAULT_NONE);
                    test_mode = MODE_IDLE;
                }
                continue;
            }

            if (test_mode == MODE_TUNE) {
                target = step_index == 0 ? TUNE_STEP : -TUNE_STEP;
            } else {
                if (range_index < 16) {
                    /* +10, -10, +20, -20, ... +80, -80 */
                    target = 10.0f * (range_index / 2 + 1);
                    if (range_index % 2) target = -target;
                } else {
                    /* +90, +100, +110, +120, +130 */
                    target = 90.0f + 10.0f * (range_index - 16);
                }

                candidate_number++;
            }

            test_phase = PHASE_STEP;
            begin_window(target, STEP_MS);
            continue;
        }

        print_result("STEP", &result);
        passed = settling_ms(&result) >= 0;

        if (test_mode == MODE_TUNE) {
            if (!passed) {
                finish_candidate(false);
                continue;
            }

            candidate_score += window_score(&result);
            step_index++;

            if (step_index < 2) {
                prepare_horizontal();
            } else {
                printf("SCORE,%d,%.4f\r\n",
                       candidate_number, candidate_score);
                finish_candidate(true);
            }
        } else {
            positive = (range_index >= 16 || range_index % 2 == 0);

            if (positive) {
                if (passed) {
                    range_positive_pass = result.target;
                } else {
                    range_positive_failed = true;
                }
            } else {
                if (passed) {
                    range_negative_pass = result.target;
                } else {
                    range_negative_failed = true;
                }
            }

            range_index++;
            advance_range();
        }
    }

    PT_END(pt);
}

/* Print the current controller settings. */
static void print_status(void) {
    printf("kp=%.4f ki=%.5f kd=%.4f sp=%.1f "
           "angle=%.2f cmd=%.0f ctrl=%s test=%d fault=%d\r\n",
           fix2float15(kp), fix2float15(ki), fix2float15(kd),
           fix2float15(setpoint), fix2float15(comp_angle),
           fix2float15(motor_cmd_raw), ctrl_on ? "ON" : "OFF",
           test_mode, fault_code);
}

/* Preserve the working nonblocking serial input loop. */
static PT_THREAD(protothread_serial(struct pt *pt)) {
    PT_BEGIN(pt);

    static char line[64], cmd[16];
    static int len, c, n;
    static float value;

    while (!stdio_usb_connected()) {
        PT_YIELD_usec(100000);
    }

    printf("\r\nCommands: kp/ki/kd/sp <value>, on, off, p, v, t <n>, "
           "tune, range\r\n");
    printf("kd uses DIRECT units. Old 29999 is now 29.2959.\r\n");
    printf("Button aborts; release does not restart.\r\n");
    print_status();

    while (1) {
        printf("> ");
        len = 0;

        while (1) {
            c = getchar_timeout_us(0);

            if (c == PICO_ERROR_TIMEOUT) {
                PT_YIELD_usec(1000);
                continue;
            }

            if (c == '\r' || c == '\n') {
                if (len == 0) continue;
                printf("\r\n");
                break;
            }

            if ((c == '\b' || c == 127) && len > 0) {
                len--;
                printf("\b \b");
                continue;
            }

            if (c >= 32 && c < 127 && len < (int)sizeof(line) - 1) {
                line[len++] = (char)c;
                putchar(c);
            }
        }

        line[len] = '\0';
        value = 0;
        n = sscanf(line, "%15s %f", cmd, &value);
        if (n < 1) continue;

        if (!strcmp(cmd, "off")) {
            stop_motor(FAULT_USER);
            test_mode = MODE_IDLE;
        } else if (!strcmp(cmd, "p")) {
            /* Status printed below. */
        } else if (!strcmp(cmd, "v")) {
            plot_view = !plot_view;
        } else if (test_mode != MODE_IDLE) {
            printf("Experiment active. Use off before changing settings.\r\n");
        } else if (!strcmp(cmd, "tune")) {
            start_tuning();
        } else if (!strcmp(cmd, "range")) {
            start_range();
        } else if (!strcmp(cmd, "on")) {
            if (!enable_motor()) {
                printf("Start blocked: check button and angle.\r\n");
            }
        } else if (n == 2 && isfinite(value)) {
            if (!strcmp(cmd, "kp") && value >= 0 && value < 30000) {
                kp = float2fix15(value);
            } else if (!strcmp(cmd, "ki") && value >= 0 && value < 100) {
                ki = float2fix15(value);
            } else if (!strcmp(cmd, "kd") && value >= 0 && value < 30000) {
                kd = float2fix15(value);
            } else if (!strcmp(cmd, "sp")
                       && fabsf(value) <= MANUAL_SP_LIMIT) {
                setpoint = float2fix15(value);
            } else if (!strcmp(cmd, "t") && value >= 1 && value <= 10000) {
                threshold = (int)value;
            } else {
                printf("Invalid command or value.\r\n");
            }
        } else {
            printf("Unknown command or missing value.\r\n");
        }

        print_status();
    }

    PT_END(pt);
}

/* Core 1 owns the VGA drawing thread. */
void core1_entry(void) {
    pt_add_thread(protothread_draw);
    pt_schedule_start;
}

/* Initialize pins before enabling the control interrupt. */
int main(void) {
    set_sys_clock_khz(150000, true);
    stdio_init_all();

    gpio_init(14);
    gpio_set_dir(14, GPIO_OUT);
    gpio_put(14, 0);

    gpio_init(15);
    gpio_set_dir(15, GPIO_OUT);
    gpio_put(15, 0);

    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);

    initVGA();

    i2c_init(I2C_CHAN, I2C_BAUD_RATE);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    mpu6050_reset();
    mpu6050_read_raw(acceleration, gyro);

    ax_f = acceleration[ACC_NUM];
    ay_f = acceleration[ACC_DEN];
    accel_angle = accel_to_angle(ax_f, ay_f);
    accel_angle_raw = accel_angle;
    gyro_angle = accel_angle;
    comp_angle = accel_angle;

    kp = float2fix15(150.0f);
    ki = float2fix15(0.05f);
    kd = float2fix15(29999.0f / 1024.0f);

    gpio_set_function(4, GPIO_FUNC_PWM);
    gpio_set_function(5, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(4);

    pwm_set_wrap(slice_num, WRAPVAL);
    pwm_set_clkdiv(slice_num, CLKDIV);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);

    pwm_clear_irq(slice_num);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_enabled(PWM_IRQ_WRAP, true);
    pwm_set_mask_enabled(1u << slice_num);

    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    pt_add_thread(protothread_test);
    pt_add_thread(protothread_serial);
    pt_schedule_start;
}