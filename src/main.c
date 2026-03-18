#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <sensor/hx711/hx711.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h> 

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

/* --- System Modes --- */
#define MODE_MANUAL 0
#define MODE_AUTO   1

/* --- Tuning Parameters --- */
#define DEADBAND_GRAMS 20   
#define SPEED_MULTIPLIER 300
#define MAX_SPEED_HZ 60000     
#define MANUAL_SPEED_HZ 2000 // Constant speed when holding A1/A2
#define SPEED_SMOOTHING 0.005f // Lower = smoother/slower acceleration, Higher = jerkier

/* --- State & Math --- */
static uint8_t frame_buffer[1024];
static volatile int32_t latest_raw = 0;
static volatile float filtered_raw = 0.0f;
static volatile int current_grams = 0;

static float tare_offset = 8281000.0f; 
static float scale_factor = 200.0f;    

/* State Machine Variables */
static volatile int system_mode = MODE_MANUAL; 
static volatile int manual_cmd = 0; // 1 = UP, -1 = DOWN, 0 = HOLD

/* --- Hardware Handles --- */
const struct device *hx = DEVICE_DT_GET(DT_NODELABEL(hx711));
const struct device *disp = DEVICE_DT_GET(DT_NODELABEL(ssd1306));
const struct device *gpioa = DEVICE_DT_GET(DT_NODELABEL(gpioa)); 
const struct device *gpiob = DEVICE_DT_GET(DT_NODELABEL(gpiob)); 

static const struct gpio_dt_spec btn_tare = GPIO_DT_SPEC_GET(DT_ALIAS(tare_btn), gpios);
static const struct gpio_dt_spec btn_down = GPIO_DT_SPEC_GET(DT_ALIAS(down_btn), gpios);
static const struct gpio_dt_spec btn_up   = GPIO_DT_SPEC_GET(DT_ALIAS(up_btn), gpios);

/* Complete ASCII Font (Space to 'Z') */
static const uint8_t font_5x7[59][5] = {
    {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5f,0x00,0x00}, {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7f,0x14,0x7f,0x14}, 
    {0x24,0x2a,0x7f,0x2a,0x12}, {0x23,0x13,0x08,0x64,0x62}, {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00}, 
    {0x00,0x1c,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1c,0x00}, {0x14,0x08,0x3e,0x08,0x14}, {0x08,0x08,0x3e,0x08,0x08}, 
    {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x60,0x60,0x00,0x00}, {0x20,0x10,0x08,0x04,0x02}, 
    {0x3e,0x51,0x49,0x45,0x3e}, {0x00,0x42,0x7f,0x40,0x00}, {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4b,0x31}, 
    {0x18,0x14,0x12,0x7f,0x10}, {0x27,0x45,0x45,0x45,0x39}, {0x3c,0x4a,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03}, 
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1e}, {0x00,0x36,0x36,0x00,0x00}, {0x00,0x56,0x36,0x00,0x00}, 
    {0x08,0x14,0x22,0x41,0x00}, {0x14,0x14,0x14,0x14,0x14}, {0x00,0x41,0x22,0x14,0x08}, {0x02,0x01,0x51,0x09,0x06}, 
    {0x32,0x49,0x79,0x41,0x3e}, {0x7e,0x11,0x11,0x11,0x7e}, {0x7f,0x49,0x49,0x49,0x36}, {0x3e,0x41,0x41,0x41,0x22}, 
    {0x7f,0x41,0x41,0x22,0x1c}, {0x7f,0x49,0x49,0x49,0x41}, {0x7f,0x09,0x09,0x09,0x01}, {0x3e,0x41,0x49,0x49,0x7a}, 
    {0x7f,0x08,0x08,0x08,0x7f}, {0x00,0x41,0x7f,0x41,0x00}, {0x20,0x40,0x41,0x3f,0x01}, {0x7f,0x08,0x14,0x22,0x41}, 
    {0x7f,0x40,0x40,0x40,0x40}, {0x7f,0x02,0x0c,0x02,0x7f}, {0x7f,0x04,0x08,0x10,0x7f}, {0x3e,0x41,0x41,0x41,0x3e}, 
    {0x7f,0x09,0x09,0x09,0x06}, {0x3e,0x41,0x51,0x21,0x5e}, {0x7f,0x09,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31}, 
    {0x01,0x01,0x7f,0x01,0x01}, {0x3f,0x40,0x40,0x40,0x3f}, {0x1f,0x20,0x40,0x20,0x1f}, {0x3f,0x40,0x38,0x40,0x3f}, 
    {0x63,0x14,0x08,0x14,0x63}, {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43}  
};

static void draw_text(int x, int y, const char *s) {
    while (*s) {
        int idx = 0;
        if (*s >= 32 && *s <= 90) idx = *s - 32; 
        for (int i = 0; i < 5; i++) {
            uint8_t col = font_5x7[idx][i];
            for (int j = 0; j < 7; j++) {
                if (col & (1 << j)) frame_buffer[((y + j) / 8) * SCREEN_WIDTH + (x + i)] |= (1 << ((y + j) % 8));
            }
        }
        x += 6; s++;
    }
}

/* --- THREAD 1: HX711 Sampler (80Hz) --- */
void sensor_thread(void *p1, void *p2, void *p3) {
    struct sensor_value rate = { .val1 = HX711_RATE_80HZ };
    sensor_attr_set(hx, HX711_SENSOR_CHAN_WEIGHT, SENSOR_ATTR_SAMPLING_FREQUENCY, &rate);
    struct sensor_value val;
    
    while (1) {
        if (sensor_sample_fetch(hx) == 0) {
            sensor_channel_get(hx, HX711_SENSOR_CHAN_WEIGHT, &val);
            latest_raw = val.val1;
            
            if (filtered_raw == 0.0f) filtered_raw = (float)latest_raw;
            filtered_raw = (0.15f * (float)latest_raw) + (0.85f * filtered_raw);
            current_grams = (int)((filtered_raw - tare_offset) / scale_factor);
        }
        k_msleep(1); 
    }
}
K_THREAD_DEFINE(s_tid, 1024, sensor_thread, NULL, NULL, NULL, 5, 0, 0);

/* --- THREAD 2: Stepper Controller (Smoothed Physics) --- */
void stepper_thread(void *p1, void *p2, void *p3) {
    if (!device_is_ready(gpioa) || !device_is_ready(gpiob)) return;
    
    gpio_pin_configure(gpioa, 9, GPIO_OUTPUT_ACTIVE);  // PA9: DIR
    gpio_pin_configure(gpioa, 10, GPIO_OUTPUT_ACTIVE); // PA10: STEP
    gpio_pin_configure(gpiob, 5, GPIO_OUTPUT_ACTIVE);  // PB5: ENABLE
    
    gpio_pin_set(gpiob, 5, 0); // PB5 LOW = Motor ENABLED

    float current_speed_hz = 0.0f; // Track actual motor speed
    int current_direction = 0;     // 1 for UP, 0 for DOWN

    while (1) {
        int target_speed_hz = 0;
        int target_direction = current_direction;

        /* Determine target speed and direction based on mode */
        if (system_mode == MODE_MANUAL) {
            if (manual_cmd == 1) {
                target_direction = 0; // Adjust to 1 if motor runs backward
                target_speed_hz = MANUAL_SPEED_HZ;
            } else if (manual_cmd == -1) {
                target_direction = 1; // Adjust to 0 if motor runs backward
                target_speed_hz = MANUAL_SPEED_HZ;
            } else {
                target_speed_hz = 0;
            }
        } else if (system_mode == MODE_AUTO) {
            int err = current_grams; 
            if (abs(err) > DEADBAND_GRAMS) {
                target_direction = (err > 0) ? 1 : 0;
                target_speed_hz = abs(err) * SPEED_MULTIPLIER;
                if (target_speed_hz > MAX_SPEED_HZ) target_speed_hz = MAX_SPEED_HZ;
            } else {
                target_speed_hz = 0;
            }
        }

        /* Prevent instant direction changes while motor is still spinning down */
        if (target_direction != current_direction && current_speed_hz > 500.0f) {
            target_speed_hz = 0; // Force brake before reversing
        } else {
            current_direction = target_direction;
            gpio_pin_set(gpioa, 9, current_direction);
        }

        /* Smoothly ramp the actual speed toward the target speed */
        current_speed_hz = (SPEED_SMOOTHING * (float)target_speed_hz) + ((1.0f - SPEED_SMOOTHING) * current_speed_hz);

        /* Drive the motor if we are above the minimum speed threshold */
        if (current_speed_hz > 100.0f) {
            int delay_us = 1000000 / (int)current_speed_hz;
            
            // Fire the Pulse
            gpio_pin_set(gpioa, 10, 1);
            k_busy_wait(15); 
            gpio_pin_set(gpioa, 10, 0);
            
            // Wait for next step
            k_busy_wait(delay_us); 
        } else {
            // Motor is functionally stopped
            k_msleep(1); 
        }
    }
}
K_THREAD_DEFINE(step_tid, 1024, stepper_thread, NULL, NULL, NULL, 5, 0, 0);

/* --- MAIN THREAD: Display & Inputs --- */
int main(void) {
    gpio_pin_configure_dt(&btn_tare, GPIO_INPUT);
    gpio_pin_configure_dt(&btn_down, GPIO_INPUT);
    gpio_pin_configure_dt(&btn_up,   GPIO_INPUT);
    
    struct display_buffer_descriptor desc = { .buf_size = 1024, .width = 128, .height = 64, .pitch = 128 };
    char buf[32];

    while (1) {
        memset(frame_buffer, 0, 1024);

        /* 1. Poll the Override Buttons (Immediate execution) */
        if (gpio_pin_get_dt(&btn_up)) {
            system_mode = MODE_MANUAL;
            manual_cmd = 1; 
        } else if (gpio_pin_get_dt(&btn_down)) {
            system_mode = MODE_MANUAL;
            manual_cmd = -1;
        } else {
            manual_cmd = 0; // Release hold
        }

        /* 2. Poll Tare (Sets Auto Mode) */
        if (gpio_pin_get_dt(&btn_tare)) {
            tare_offset = filtered_raw;
            system_mode = MODE_AUTO; // Override back to Auto
        }

        /* 3. Render Display */
        snprintf(buf, sizeof(buf), "GRAMS: %d", current_grams);
        draw_text(0, 5, buf);

        if (system_mode == MODE_AUTO) {
            draw_text(0, 20, "MODE: AUTO");
            if (abs(current_grams) <= DEADBAND_GRAMS) draw_text(0, 35, "MOTOR: HOLD");
            else if (current_grams > 0) draw_text(0, 35, "MOTOR: DOWN");
            else draw_text(0, 35, "MOTOR: UP");
        } else {
            draw_text(0, 20, "MODE: MANUAL");
            if (manual_cmd == 1) draw_text(0, 35, "MOTOR: UP");
            else if (manual_cmd == -1) draw_text(0, 35, "MOTOR: DOWN");
            else draw_text(0, 35, "MOTOR: HOLD");
        }

        snprintf(buf, sizeof(buf), "RAW: %d", latest_raw);
        draw_text(0, 50, buf);

        display_write(disp, 0, 0, &desc, frame_buffer);
        k_sleep(K_MSEC(25)); 
    }
}