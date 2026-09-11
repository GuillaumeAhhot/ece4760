/*
 * Birdsong Synthesizer - RP2350
 * ECE 4760, Week 2
 *
 * DDS tone generation on MCP4822 DAC, frequency set by ADC (slide pot).
 * Keypad input to be integrated next.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/spi.h"
#include "hardware/sync.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "pt_cornell_rp2040_v1_4.h"

// ================= Pin / hardware config =================
#define LED_PIN 25
#define ADC_PIN 26
#define ADC_MUX 0

#define ALARM_NUM 0
#define ALARM_IRQ timer_hardware_alarm_get_irq_num(timer_hw, ALARM_NUM)

#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define SPI_PORT spi0

#define ISR_GPIO 2

#define BASE_KEYPAD_PIN 9
#define KEYROWS         4
#define NUMKEYS         12

// ================= Keypad Config ==================
unsigned int keycodes[NUMKEYS] = {      0x57, 0x6E, 0x5E, 0x3E, 0x6D,
                                        0x5D, 0x3D, 0x6B, 0x5B, 0x3B,
                                        0x67, 0x37} ;
unsigned int scancodes[KEYROWS] = {   0xE, 0xD, 0xB, 0x7} ;
unsigned int button = 0x70 ;

#define MAX_RECORDING_LEN 6000
uint16_t recordings[9][MAX_RECORDING_LEN];

volatile int recording_target = -1;
volatile int recording_index = 0;
int recording_length[9] = {0};

char keytext[40];

#define RESET_STATE 0
#define MAYBE_YES_STATE 1
#define CONFIRM_YES_STATE 2
#define MAYBE_NOT_STATE 3 

volatile int key_event = -1;
volatile int key_held = -1;
volatile bool record_mode = false;

volatile int playback_target = -1;
volatile int playback_index = 0;
volatile bool playback_mode = false;

static void debounce_keypad(int key_val, 
                            int *state, 
                            int *prev_val,
                            int *confirmed_press_val, 
                            bool *was_released,
                            int *key_held){
    switch(*state){
        case RESET_STATE:
            if (key_val != -1){
                *prev_val = key_val;
                *state = MAYBE_YES_STATE;
            }
            break;

        case MAYBE_YES_STATE:
            if (key_val == *prev_val){
                *state = CONFIRM_YES_STATE;
                *confirmed_press_val = key_val;
                *key_held = key_val;
            } else {
                *state = RESET_STATE;
            }
            break;

        case CONFIRM_YES_STATE:
            if (key_val != *prev_val){
                *state = MAYBE_NOT_STATE;
            }
            break;

        case MAYBE_NOT_STATE:
            if (key_val != *prev_val){
                *state = RESET_STATE;
                *was_released = true;
                *key_held = -1;
            } else {
                *state = CONFIRM_YES_STATE;
            }
            break;
    }
}

// ================= DDS parameters =================
#define two32 4294967296.0 // 2^32
#define Fs 50000
#define DELAY 20 // 1/Fs, in microseconds

volatile unsigned int phase_accum_main;
volatile unsigned int phase_incr_main = 0;
volatile unsigned int desired_frequency = 800;
volatile bool mute = false;
volatile bool skip_next_event = false;

#define MAX_AMPLITUDE 2048
#define MUTE_AMPLITUDE 0
#define AMPLITUDE_OFFSET 11

volatile int amplitude = MAX_AMPLITUDE;
volatile int amplitude_target = MAX_AMPLITUDE;
#define AMPLITUDE_STEP 8

#define sine_table_size 256
volatile int sin_table[sine_table_size];

// ================= DAC config =================
uint16_t DAC_data;
#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000

// ================= Alarm ISR =================
static void alarm_irq(void) {
    gpio_put(ISR_GPIO, 1);

    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    phase_accum_main += phase_incr_main;

    if (amplitude < amplitude_target){
        amplitude += AMPLITUDE_STEP;
        if (amplitude > amplitude_target){
            amplitude = amplitude_target;
        }
    }
    else if (amplitude > amplitude_target){
        amplitude -= AMPLITUDE_STEP;
        if (amplitude < amplitude_target){
            amplitude = amplitude_target;
        }
    }

    DAC_data = (DAC_config_chan_B | (((sin_table[phase_accum_main >> 24]*amplitude >> AMPLITUDE_OFFSET) + 2048) & 0x0fff));
    spi_write16_blocking(SPI_PORT, &DAC_data, 1);

    gpio_put(ISR_GPIO, 0);
}

static void update_amplitude_target(void){
    if (mute){
        amplitude_target = MUTE_AMPLITUDE;
    }
    else{
        amplitude_target = MAX_AMPLITUDE;
    }
}

static void handle_key_event(int key_event){
    if (key_event == 0){
        mute = !mute;
        update_amplitude_target();
    }
    else if (key_event == 10){
        record_mode = !record_mode;
    }
    else if (key_event == 11){
        //stub
    }
    else if (key_event >= 1 && key_event <= 9){
        if (!record_mode && recording_length[key_event-1] > 0){
            playback_target = key_event;
            playback_index = 0;
            playback_mode = true;
            update_amplitude_target();
        }
    }
}

// ================= ADC read thread =================
static PT_THREAD (protothread_sequencer(struct pt *pt))
{
    PT_BEGIN(pt);
    PT_INTERVAL_INIT();
    static unsigned int adc_val;

    while(1) {
        gpio_put(LED_PIN, !gpio_get(LED_PIN));

        adc_val = adc_read();
        if (playback_mode){
            desired_frequency = recordings[playback_target-1][playback_index];
            phase_incr_main = (desired_frequency * two32) / Fs;
            playback_index++;
            if (playback_index >= recording_length[playback_target-1]){
                playback_mode = false;
            }
        }
        else{
            desired_frequency = (unsigned int)((10000.0 / 4095.0) * adc_val);
            phase_incr_main = (desired_frequency * two32) / Fs;
        }
        
        printf("ADC value: %d\n", adc_val);

        if (key_event != -1){
            if (!skip_next_event){
                handle_key_event(key_event);
            }
            skip_next_event = false;
            key_event = -1;
        }

        if (record_mode && key_held >= 1 && key_held <= 9){
            if (recording_target != key_held){
                recording_target = key_held;
                recording_index = 0;
            }
            if (recording_index < MAX_RECORDING_LEN){
                recordings[key_held-1][recording_index] = (uint16_t) desired_frequency;
                recording_index++;
            }
        }
        else if (recording_target != -1){
            recording_length[recording_target-1] = recording_index;
            recording_target = -1;
            record_mode = false;
            skip_next_event = true;
        }

        PT_YIELD_INTERVAL(10000);
    }
    PT_END(pt);
}

// ================= Keypad thread =================
static PT_THREAD (protothread_keypad(struct pt *pt))
{
    PT_BEGIN(pt) ;

    static int i ;
    static uint32_t keypad ;
    static int state = RESET_STATE;
    static int prev_val = -1;
    static int confirmed_press_val = -1;
    static bool was_released = false;
    static int key_held_local = -1;

    while(1) {

        // drive one row low at a time, read the column pins
        for (i=0; i<KEYROWS; i++) {
            gpio_put_masked((0xF << BASE_KEYPAD_PIN),
                            (scancodes[i] << BASE_KEYPAD_PIN)) ;
            sleep_us(1) ;
            keypad = ((gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F) ;
            if ((~keypad) & button) break ;
        }

        // resolve scancode to key index, -1 for no press or invalid
        if ((~keypad) & button) {
            for (i=0; i<NUMKEYS; i++) {
                if (keypad == keycodes[i]) break ;
            }
            if (i==NUMKEYS) (i = -1) ;
        }
        else (i=-1) ;

        debounce_keypad(i, &state, &prev_val, &confirmed_press_val, &was_released, &key_held_local);
        key_held = key_held_local;

        if (was_released){
            key_event = confirmed_press_val;
            was_released = false;
            confirmed_press_val = -1;
        }

        printf("\n%d", i) ;

        PT_YIELD_usec(30000) ;
    }
    PT_END(pt) ;
}

// ================= main =================
int main(){

    set_sys_clock_khz(150000, true) ;

    stdio_init_all();
    printf("\n\rProtothreads RP2040 v1.4\n\r");

    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(ADC_MUX);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, true);

    spi_init(SPI_PORT, 20000000);
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    gpio_init(ISR_GPIO);
    gpio_set_dir(ISR_GPIO, GPIO_OUT);
    gpio_put(ISR_GPIO, 0);

    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);

    // build sine lookup table, scaled to +/- 2047
    for (int ii = 0; ii < sine_table_size; ii++){
        sin_table[ii] = (int)(2047 * sin((float)ii * 6.283 / (float)sine_table_size));
    }

    ////////////////// KEYPAD INITS ///////////////////////
    // Initialize the keypad GPIO's
    gpio_init_mask((0x7F << BASE_KEYPAD_PIN)) ;
    gpio_set_dir((BASE_KEYPAD_PIN+4), GPIO_IN);
    gpio_set_dir((BASE_KEYPAD_PIN+5), GPIO_IN);
    gpio_set_dir((BASE_KEYPAD_PIN+6), GPIO_IN);
    // Set row-pins to output
    gpio_set_dir_out_masked((0xF << BASE_KEYPAD_PIN)) ;
    // Set all output pins to high
    gpio_put_masked((0xF << BASE_KEYPAD_PIN), (0xF << BASE_KEYPAD_PIN)) ;
    // Turn on pullup resistors for column pins
    gpio_pull_up((BASE_KEYPAD_PIN+4)) ;
    gpio_pull_up((BASE_KEYPAD_PIN+5)) ;
    gpio_pull_up((BASE_KEYPAD_PIN+6)) ;

    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
    irq_set_enabled(ALARM_IRQ, true);
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    pt_add_thread(protothread_keypad) ;
    pt_add_thread(protothread_sequencer);
    pt_schedule_start;
}