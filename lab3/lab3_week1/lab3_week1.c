/**
 * V. Hunter Adams (vha3@cornell.edu)
 * PWM demo code with serial input, modified for USB serial
 *
 * This demonstration sets a PWM duty cycle to a
 * user-specified value typed into a USB serial terminal.
 *
 * The Cornell protothreads serial_read/serial_write macros only use
 * UART0 (GPIO 0/1), so they are replaced here with a non-blocking
 * USB stdio reader. Requires pico_enable_stdio_usb(<target> 1) in CMake.
 *
 * HARDWARE CONNECTIONS
 *   - GPIO 4 ---> PWM output
 *   - USB    ---> computer (serial terminal, 115200)
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/stdio_usb.h"

#include "hardware/pwm.h"
#include "hardware/irq.h"

#include "pt_cornell_rp2040_v1_4.h"

// PWM wrap value and clock divide value
// For a CPU rate of 150 MHz, this gives
// a PWM frequency of 1 kHz.
#define WRAPVAL 6000
#define CLKDIV 25.0f

// GPIO we're using for PWM
#define PWM_OUT 4

// Input line buffer size
#define LINE_LEN 16

// Variable to hold PWM slice number
uint slice_num ;

// PWM duty cycle (start both at 50% so the ISR doesn't fire a change)
volatile int control = 3000 ;
volatile int old_control = 3000 ;

// PWM interrupt service routine
void on_pwm_wrap() {
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(PWM_OUT));
    // Update duty cycle only when it changed
    if (control != old_control) {
        old_control = control ;
        pwm_set_chan_level(slice_num, PWM_CHAN_A, control);
    }
}

// User input thread: non-blocking line reader over USB stdio
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt) ;
    static char line[LINE_LEN] ;
    static int len ;
    static int c ;
    static int test_in ;

    // Wait until a terminal opens the USB port, otherwise the
    // first prompt is sent to nobody and lost
    while (!stdio_usb_connected()) {
        PT_YIELD_usec(100000) ;
    }
    printf("\r\nPWM duty cycle control (0-6000, 3000 = 50%%)\r\n") ;

    while (1) {
        printf("input a duty cycle (0-6000): ") ;
        len = 0 ;

        // Collect characters until Enter
        while (1) {
            c = getchar_timeout_us(0) ;          // returns immediately
            if (c == PICO_ERROR_TIMEOUT) {
                PT_YIELD_usec(1000) ;            // nothing typed, let others run
                continue ;
            }
            if (c == '\r' || c == '\n') {        // Enter (PuTTY sends \r)
                if (len == 0) continue ;         // ignore stray \n after \r
                printf("\r\n") ;
                break ;
            }
            if ((c == '\b' || c == 127) && len > 0) {  // backspace / delete
                len-- ;
                printf("\b \b") ;
                continue ;
            }
            if (c >= '0' && c <= '9' && len < LINE_LEN - 1) {
                line[len++] = (char)c ;
                putchar(c) ;                     // echo so you see what you type
            }
            // anything else (letters, minus sign) is ignored
        }
        line[len] = '\0' ;

        test_in = atoi(line) ;
        if (test_in > WRAPVAL) {
            printf("out of range, ignored\r\n") ;
        } else {
            control = test_in ;
            printf("duty set to %d (%.1f%%)\r\n", test_in, 100.0f * test_in / WRAPVAL) ;
        }
    }
    PT_END(pt) ;
}

int main() {

    // Initialize stdio (USB and/or UART, per CMakeLists)
    stdio_init_all();

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO PWM_OUT that it is allocated to the PWM
    gpio_set_function(PWM_OUT, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO PWM_OUT (it's slice 2)
    slice_num = pwm_gpio_to_slice_num(PWM_OUT);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;

    // This sets duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 3000);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    pt_add_thread(protothread_serial) ;
    pt_schedule_start ;

}