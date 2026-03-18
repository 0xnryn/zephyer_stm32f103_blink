# 1. Load the compiler toolchain
nix develop

# 2. Download the Zephyr blueprint into this folder
west init -m https://github.com/zephyrproject-rtos/zephyr --mr main .

# 3. Download the actual OS source code and STM32 drivers (This takes a few minutes)
west update


west build -p always -b stm32_min_dev@blue 


Signal Name,Description,STM32 Pin Type
VCC,Power Supply (typically 3.3V),Power
GND,Ground,Power
SCK / CLK,Serial Clock,"SPI Hardware Peripheral (e.g., SPI1_SCK)"
MOSI / SDA,Master Out Slave In (Data),"SPI Hardware Peripheral (e.g., SPI1_MOSI)"
CS / SS,Chip Select (Active Low),Any GPIO
DC / RS,Data/Command Selection,Any GPIO
RES / RST,Hardware Reset (Active Low),Any GPIO
