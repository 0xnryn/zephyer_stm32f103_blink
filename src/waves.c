#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <string.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
static uint8_t frame_buffer[1024];

/* Fast inline pixel drawing - Maps directly to SSD1306 page/column architecture */
static inline void set_pixel(int x, int y) {
    if (x >= 0 && x < SCREEN_WIDTH && y >= 0 && y < SCREEN_HEIGHT) {
        frame_buffer[(y / 8) * SCREEN_WIDTH + x] |= (1 << (y % 8));
    }
}

/* 64-step Sine Wave Lookup Table (Amplitude 255) for ZERO-overhead math */
static const int16_t sin_tbl[64] = {
    0, 25, 50, 74, 98, 120, 142, 162, 180, 197, 212, 225, 236, 244, 250, 254,
    255, 254, 250, 244, 236, 225, 212, 197, 180, 162, 142, 120, 98, 74, 50, 25,
    0, -25, -50, -74, -98, -120, -142, -162, -180, -197, -212, -225, -236, -244, -250, -254,
    -255, -254, -250, -244, -236, -225, -212, -197, -180, -162, -142, -120, -98, -74, -50, -25
};

static inline int fast_sin(int angle) {
    return sin_tbl[(angle & 63)];
}

int main(void)
{
    const struct device *display_dev = DEVICE_DT_GET(DT_NODELABEL(ssd1306));

    if (!device_is_ready(display_dev)) return 0;
    display_blanking_off(display_dev);

    struct display_buffer_descriptor buf_desc = {
        .buf_size = sizeof(frame_buffer), 
        .width = SCREEN_WIDTH,
        .height = SCREEN_HEIGHT, 
        .pitch = SCREEN_WIDTH
    };

    /* Fluid physics states */
    int phase1 = 0, phase2 = 0, phase3 = 0;

    while (1) {
        /* 1. Clear the frame buffer (Fastest way to wipe the screen) */
        memset(frame_buffer, 0, sizeof(frame_buffer));

        /* 2. Compute and Draw the wave surface */
        for (int x = 0; x < SCREEN_WIDTH; x++) {
            
            /* Combine 3 sine waves for organic interference patterns */
            int y1 = (fast_sin((x * 2) + phase1) * 12) / 255;
            int y2 = (fast_sin((x * 1) + phase2) * 8) / 255;
            int y3 = (fast_sin((x * 3) + phase3) * 4) / 255;
            
            int surface_y = 32 + y1 + y2 + y3;
            
            /* Draw solid surface line */
            set_pixel(x, surface_y);
            set_pixel(x, surface_y + 1);
            
            /* Fill the 'underwater' area with a 50% dither pattern */
            for(int fy = surface_y + 2; fy < SCREEN_HEIGHT; fy++) {
                if ((x + fy) % 2 == 0) { 
                    set_pixel(x, fy);
                }
            }
        }
        
        /* 3. Evolve the phases for the next frame */
        phase1 += 2;
        phase2 -= 1;
        phase3 += 3;

        /* 4. Blast the entire buffer to the OLED in one SPI transaction */
        display_write(display_dev, 0, 0, &buf_desc, frame_buffer);

        /* 5. Precise microsecond sleep for 120Hz lock */
        k_usleep(7300); 
    }
}