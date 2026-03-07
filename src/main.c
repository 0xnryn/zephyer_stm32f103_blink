#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/usb/usb_device.h>
#include <stdio.h>

/* Ask the DeviceTree for the LED pin */
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{
    /* 1. Wake up the USB Port */
    if (usb_enable(NULL)) {
        return 0; /* Boot failed if USB fails */
    }

    /* 2. Configure the LED */
    if (!gpio_is_ready_dt(&led)) {
        return 0;
    }
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);

    /* 3. The Main Loop */
    int count = 0;
    while (1) {
        gpio_pin_toggle_dt(&led);
        
        /* Print to the virtual serial port! */
        printf("Hello from Zephyr! Blink count: %d\n", count);
        
        count++;
        k_msleep(100); 
    }
    
    return 0;
}