# GPIO_RPI5 — GPIO Library for Raspberry Pi 5

A lightweight C library for controlling GPIO pins on the **Raspberry Pi 5**, with two backends:

- **Direct register access** (`gpio_rpi5.c`) — maps RP1 registers via `/dev/gpiomem0` using `mmap()`. Extremely fast (~10ns per operation). Supports GPIO 0–53.
- **Pinctrl backend** (`gpio_rpi5_pinctrl.c`) — uses the `pinctrl` command-line tool via `popen()`. Slower (~10ms per operation), but supports all GPIO ranges (0–53, 100–135, 200–237).

## Build

### Linux (Raspberry Pi 5)

```bash
chmod +x ./build.sh

# Default: direct register access (fast)
./build.sh

# Alternative: pinctrl backend
./build.sh pinctrl
```

### Manual compilation

```bash
# Direct register access backend
gcc -c gpio_rpi5.c -o gpio_rpi5.o -Wall -Wextra
ar rcs gpio_rpi5.a gpio_rpi5.o
g++ test/main.cpp -I. -L. -l:gpio_rpi5.a -o test/myprogram
sudo ./test/myprogram

# Pinctrl backend
gcc -c gpio_rpi5_pinctrl.c -o gpio_rpi5.o -Wall -Wextra
ar rcs gpio_rpi5.a gpio_rpi5.o
g++ test/main.cpp -I. -L. -l:gpio_rpi5.a -o test/myprogram
sudo ./test/myprogram
```

### Windows (cross-compile / pinctrl backend only)

```cmd
build_test.bat
```

Requires MinGW (gcc/g++) in PATH. The direct register backend requires Linux on a Raspberry Pi 5.

## API Reference

### Initialization / Cleanup

| Function | Description |
|---|---|
| `int gpio_init(void)` | Initialize the library. Opens `/dev/gpiomem0` and maps RP1 registers (direct backend) or initializes pin state array (pinctrl backend). Returns 0 on success, -1 on error. **Must be called before any other function.** |
| `void gpio_cleanup(void)` | Release all resources. Unmaps memory and closes file descriptors. Call when done using GPIOs. |

### Pin Control

| Function | Description |
|---|---|
| `pin_t pinopen(int pin, int mode)` | Configure a pin as `INPUT` or `OUTPUT`. Sets FUNCSEL to SIO (GPIO mode), configures pad, and sets output enable. Returns `pin_t` with valid mode on success, or `{UNDEF, UNDEF}` on error. |
| `void pinclose(int pin)` | Release a pin. Resets it to input (safe default) and sets FUNCSEL to NULL. |
| `int pinwrite(int pin, int value)` | Set output pin to `HIGH` or `LOW`. Uses atomic SET/CLR registers. Returns 0 on success, -1 on error. |
| `int pinread(int pin)` | Read current pin level. Returns 0 (LOW), 1 (HIGH), or -1 on error. |
| `int pintoggle(int pin)` | Toggle pin output. Uses atomic XOR register (single write). Returns 0 on success, -1 on error. |

### Pin Configuration

| Function | Description |
|---|---|
| `int pinpull(int pin, int pull)` | Set pull resistor: `PULL_UP`, `PULL_DOWN`, or `PULL_NONE`. Returns 0 on success, -1 on error. |
| `int pinalt(int pin, int func)` | Set alternate function (`ALT0`–`ALT8`, `GPIO_FUNC_SIO`, `GPIO_FUNC_NULL`). Used to enable SPI, I2C, UART, PWM on specific pins. Returns 0 on success, -1 on error. |
| `int pin_set_drive(int pin, int strength)` | Set pad drive strength: `DRIVE_2MA`, `DRIVE_4MA`, `DRIVE_8MA`, `DRIVE_12MA`. Returns 0 on success, -1 on error. |

### Testing

| Function | Description |
|---|---|
| `int gpio_pintest(int pin)` | Quick test: sets pin as output, drives HIGH for 1 second, then LOW, then closes. Returns 0 on success, -1 on error. |

## Constants

### Pin Modes
| Constant | Value | Description |
|---|---|---|
| `INPUT` | 0 | Pin configured as input |
| `OUTPUT` | 1 | Pin configured as output |

### Pin States
| Constant | Value | Description |
|---|---|---|
| `LOW` | 0 | Logic low (0V) |
| `HIGH` | 1 | Logic high (3.3V) |
| `UNDEF` | 3 | Unknown / uninitialized state |

### Pull Resistors

A **pull resistor** is a resistor that connects a GPIO pin to a known voltage level (3.3V or GND) so the pin has a defined state when nothing is actively driving it. Without a pull resistor, an input pin left unconnected (floating) picks up electrical noise and reads random HIGH/LOW values — this causes unreliable behavior.

- **Pull-up** ties the pin to 3.3V through a resistor → pin reads **HIGH** by default. When a button (connected between pin and GND) is pressed, it pulls the pin LOW. This is the most common configuration for buttons and switches.
- **Pull-down** ties the pin to GND through a resistor → pin reads **LOW** by default. When a signal drives the pin HIGH, the change is detected cleanly.
- **No pull (floating)** leaves the pin unconnected — only use this when an external circuit always drives the pin.

The Raspberry Pi 5 has built-in (internal) pull resistors that can be enabled via software using `pinpull()`, so in most cases you don't need external resistors.

| Constant | Value | Description |
|---|---|---|
| `PULL_NONE` | 0 | No pull resistor (floating) |
| `PULL_UP` | 1 | Internal pull-up enabled |
| `PULL_DOWN` | 2 | Internal pull-down enabled |

### Alternate Functions (FUNCSEL)
| Constant | Value | Description |
|---|---|---|
| `ALT0`–`ALT8` | 0–8 | Alternate functions (SPI, I2C, UART, PWM, etc.) |
| `GPIO_FUNC_SIO` | 5 | Standard GPIO mode (set automatically by `pinopen`) |
| `GPIO_FUNC_NULL` | 31 | Disconnect pin (set by `pinclose`) |

### Drive Strength
| Constant | Value | Current |
|---|---|---|
| `DRIVE_2MA` | 0 | 2 mA |
| `DRIVE_4MA` | 1 | 4 mA |
| `DRIVE_8MA` | 2 | 8 mA |
| `DRIVE_12MA` | 3 | 12 mA |

## Usage Examples

### Blink an LED on GPIO17

```c
#include "gpio_rpi5.h"
#include <unistd.h>

int main() {
    gpio_init();
    pinopen(GPIO17, OUTPUT);

    for (int i = 0; i < 10; i++) {
        pintoggle(GPIO17);
        usleep(500000);  /* 500ms */
    }

    pinclose(GPIO17);
    gpio_cleanup();
    return 0;
}
```

### Read a button on GPIO22

```c
#include "gpio_rpi5.h"
#include <stdio.h>

int main() {
    gpio_init();
    pinopen(GPIO22, INPUT);
    pinpull(GPIO22, PULL_UP);  /* Pull-up: pressed = LOW */

    int state = pinread(GPIO22);
    printf("Button: %s\n", state ? "released" : "pressed");

    pinclose(GPIO22);
    gpio_cleanup();
    return 0;
}
```

### Set pin to SPI alternate function

```c
#include "gpio_rpi5.h"

int main() {
    gpio_init();
    /* Enable SPI0 on its default pins */
    pinalt(GPIO9,  ALT0);  /* SPI0_MISO */
    pinalt(GPIO10, ALT0);  /* SPI0_MOSI */
    pinalt(GPIO11, ALT0);  /* SPI0_SCLK */
    /* ... use SPI ... */
    gpio_cleanup();
    return 0;
}
```

## Pin Reference

### RP1 GPIO 0–27 (40-Pin Header)

These are the user-accessible pins on the Raspberry Pi 5 40-pin header. Directly supported by the **register access** backend.

| Pin | Define | Default Mode | Default Pull | Name / Function | Header Pin |
|-----|--------|-------------|-------------|-----------------|-----------|
| 0 | `GPIO0` | Input | Pull-up | ID_SDA (HAT EEPROM) | 27 |
| 1 | `GPIO1` | Input | Pull-up | ID_SCL (HAT EEPROM) | 28 |
| 2 | `GPIO2` | ALT3 | Pull-up | SDA1 (I2C1 Data) | 3 |
| 3 | `GPIO3` | ALT3 | Pull-up | SCL1 (I2C1 Clock) | 5 |
| 4 | `GPIO4` | Output | Pull-up | GPIO / GPCLK0 | 7 |
| 5 | `GPIO5` | None | Pull-up | GPIO | 29 |
| 6 | `GPIO6` | None | Pull-up | GPIO | 31 |
| 7 | `GPIO7` | Output | Pull-up | SPI0_CE1 | 26 |
| 8 | `GPIO8` | Output | Pull-up | SPI0_CE0 | 24 |
| 9 | `GPIO9` | ALT0 | No pull | SPI0_MISO | 21 |
| 10 | `GPIO10` | ALT0 | No pull | SPI0_MOSI | 19 |
| 11 | `GPIO11` | ALT0 | No pull | SPI0_SCLK | 23 |
| 12 | `GPIO12` | None | Pull-down | PWM0_CHAN0 / GPIO | 32 |
| 13 | `GPIO13` | None | Pull-down | PWM0_CHAN1 / GPIO | 33 |
| 14 | `GPIO14` | ALT4 | No pull | TXD0 (UART TX) | 8 |
| 15 | `GPIO15` | ALT4 | Pull-up | RXD0 (UART RX) | 10 |
| 16 | `GPIO16` | None | Pull-down | GPIO | 36 |
| 17 | `GPIO17` | None | Pull-down | GPIO | 11 |
| 18 | `GPIO18` | None | Pull-down | PCM_CLK / PWM0_CHAN0 / GPIO | 12 |
| 19 | `GPIO19` | None | Pull-down | PCM_FS / PWM0_CHAN1 / GPIO | 35 |
| 20 | `GPIO20` | None | Pull-down | PCM_DIN / GPIO | 38 |
| 21 | `GPIO21` | None | Pull-down | PCM_DOUT / GPIO | 40 |
| 22 | `GPIO22` | None | Pull-down | GPIO | 15 |
| 23 | `GPIO23` | None | Pull-down | GPIO | 16 |
| 24 | `GPIO24` | None | Pull-down | GPIO | 18 |
| 25 | `GPIO25` | None | Pull-down | GPIO | 22 |
| 26 | `GPIO26` | None | Pull-down | GPIO | 37 |
| 27 | `GPIO27` | None | Pull-down | GPIO | 13 |

### RP1 GPIO 28–53 (Internal / System)

Used internally by the Raspberry Pi 5. Supported by the **register access** backend but **use with caution** — modifying these can affect system functionality.

| Pin | Define | Default Mode | Name / Function |
|-----|--------|-------------|-----------------|
| 28 | `GPIO28` | Input | PCIE_RP1_WAKE |
| 29 | `GPIO29` | None | FAN_TACH (fan tachometer) |
| 30 | `GPIO30` | None | HOST_SDA |
| 31 | `GPIO31` | None | HOST_SCL |
| 32 | `GPIO32` | Output | ETH_RST_N (Ethernet reset) |
| 33 | `GPIO33` | None | — |
| 34 | `GPIO34` | Output | CD0_IO0_MICCLK (Camera 0 clock) |
| 35 | `GPIO35` | None | CD0_IO0_MICDAT0 (Camera 0 data) |
| 36 | `GPIO36` | None | RP1_PCIE_CLKREQ_N |
| 37 | `GPIO37` | None | — |
| 38 | `GPIO38` | Input | CD0_SDA (Camera 0 I2C) |
| 39 | `GPIO39` | Input | CD0_SCL (Camera 0 I2C) |
| 40 | `GPIO40` | Input | CD1_SDA (Camera 1 I2C) |
| 41 | `GPIO41` | Input | CD1_SCL (Camera 1 I2C) |
| 42 | `GPIO42` | ALT2 | USB_VBUS_EN |
| 43 | `GPIO43` | ALT2 | USB_OC_N (USB overcurrent) |
| 44 | `GPIO44` | Output | RP1_STAT_LED (activity LED) |
| 45 | `GPIO45` | ALT0 | FAN_PWM (fan control, PWM1_CHAN3) |
| 46 | `GPIO46` | Output | CD1_IO0_MICCLK (Camera 1 clock) |
| 47 | `GPIO47` | None | 2712_WAKE |
| 48 | `GPIO48` | None | CD1_IO1_MICDAT1 |
| 49 | `GPIO49` | Output | EN_MAX_USB_CUR (USB current limit) |
| 50–53 | `GPIO50`–`GPIO53` | None | Reserved |

### BCM2712 GPIO 100–135 (pinctrl backend only)

These are BCM2712 SoC GPIOs. Only accessible via the **pinctrl backend** (`gpio_rpi5_pinctrl.c`).

| Pin | Define | Default Mode | Name / Function |
|-----|--------|-------------|-----------------|
| 100 | `GPIOx100` | Input | — |
| 101 | `GPIOx101` | Output | 2712_BOOT_CS_N (boot SPI CS) |
| 102 | `GPIOx102` | ALT6 | 2712_BOOT_MISO (VC_SPI0_MISO) |
| 103 | `GPIOx103` | ALT5 | 2712_BOOT_MOSI (VC_SPI0_MOSI) |
| 104 | `GPIOx104` | ALT6 | 2712_BOOT_SCLK (VC_SPI0_SCLK) |
| 105–113 | `GPIOx105`–`GPIOx113` | Input | General purpose |
| 114 | `GPIOx114` | ALT1 | PCIE_SDA |
| 115 | `GPIOx115` | ALT1 | PCIE_SCL |
| 116–123 | `GPIOx116`–`GPIOx123` | Input | General purpose |
| 120 | `GPIOx120` | Input | PWR_GPIO (power button) |
| 121 | `GPIOx121` | Input | 2712_G21_FS |
| 124 | `GPIOx124` | ALT3 | BT_RTS (Bluetooth) |
| 125 | `GPIOx125` | ALT4 | BT_CTS (Bluetooth) |
| 126 | `GPIOx126` | ALT4 | BT_TXD (Bluetooth TX) |
| 127 | `GPIOx127` | ALT4 | BT_RXD (Bluetooth RX) |
| 128 | `GPIOx128` | Output | WL_ON (WiFi enable) |
| 129 | `GPIOx129` | Output | BT_ON (Bluetooth enable) |
| 130 | `GPIOx130` | ALT4 | WIFI_SDIO_CLK |
| 131 | `GPIOx131` | ALT4 | WIFI_SDIO_CMD |
| 132–135 | `GPIOx132`–`GPIOx135` | ALT3/ALT4 | WIFI_SDIO_D0–D3 |

### AON (Always-On) GPIO 200–237 (pinctrl backend only)

System-level always-on GPIOs. Only accessible via the **pinctrl backend** (`gpio_rpi5_pinctrl.c`).

| Pin | Define | Default Mode | Name / Function |
|-----|--------|-------------|-----------------|
| 200 | `GPIOx200` | Input | RP1_SDA (RP1 I2C) |
| 201 | `GPIOx201` | Input | RP1_SCL (RP1 I2C) |
| 202 | `GPIOx202` | Output | RP1_RUN (RP1 reset) |
| 203 | `GPIOx203` | Output | SD_IOVDD_SEL (SD voltage select) |
| 204 | `GPIOx204` | Output | SD_PWR_ON (SD card power) |
| 205 | `GPIOx205` | ALT6 | SD_CDET_N (SD card detect) |
| 206 | `GPIOx206` | Input | SD_FLG_N |
| 207–208 | `GPIOx207`–`GPIOx208` | Input | AON_GPIO7, 2712_WAKE |
| 209 | `GPIOx209` | Output | 2712_STAT_LED (power LED) |
| 210–212 | `GPIOx210`–`GPIOx212` | Input | AON_GPIO10–12, PMIC_INT |
| 213 | `GPIOx213` | ALT3 | UART_TX_FS (VC_TXD0) |
| 214 | `GPIOx214` | ALT3 | UART_RX_FS (VC_RXD0) |
| 215–216 | `GPIOx215`–`GPIOx216` | Input | AON_GPIO15–16 |
| 232 | `GPIOx232` | ALT1 | HDMI0_SCL |
| 233 | `GPIOx233` | ALT1 | HDMI0_SDA |
| 234 | `GPIOx234` | ALT1 | HDMI1_SCL |
| 235 | `GPIOx235` | ALT1 | HDMI1_SDA |
| 236 | `GPIOx236` | ALT2 | PMIC_SCL |
| 237 | `GPIOx237` | ALT2 | PMIC_SDA |

## Tests

All tests are located in the `test/` folder. Each file tests a specific interface, implementing both hardware configuration (pin alternate function) and a bit-banged (software) variant of the respective protocol. Tests must be run with `sudo` because direct register access requires root permissions.

### Building individual tests

```bash
chmod +x test/*.sh
./test/build_spi.sh          # build only test_spi
./test/build_pwm.sh          # build only test_pwm
./test/build_i2c.sh          # build only test_i2c
./test/build_uart.sh         # build only test_uart
./test/build_pcm.sh          # build only test_pcm
./test/build_main.sh         # build only main (basic test)
./test/build_tests.sh        # build ALL tests
```

Each script accepts the optional `pinctrl` argument to use the pinctrl backend instead of direct register access:

```bash
./test/build_spi.sh pinctrl
```

---

### test/main.cpp — Basic GPIO Test

The simplest test that verifies basic library functionality.

| Test | Description |
|---|---|
| **gpio_pintest(GPIO4)** | Sets GPIO4 as output, HIGH for 1 second, then LOW. Verifies initialization and write operations. |
| **pintoggle(GPIO17)** | Opens GPIO17 as output, performs two successive toggles and displays the state after each. Verifies the atomic XOR function. |

**Run:**
```bash
sudo ./test/myprogram
```

**Hardware required:** None (optional LED on GPIO4/GPIO17 for visual confirmation).

---

### test/test_spi.cpp — SPI Test

Tests the SPI (Serial Peripheral Interface) in both hardware and bit-banged modes.

**Pins used:**

| Pin | Header | Function |
|-----|--------|----------|
| GPIO8 | 24 | SPI0_CE0 (Chip Enable 0) |
| GPIO7 | 26 | SPI0_CE1 (Chip Enable 1) |
| GPIO9 | 21 | SPI0_MISO (Master In, Slave Out) |
| GPIO10 | 19 | SPI0_MOSI (Master Out, Slave In) |
| GPIO11 | 23 | SPI0_SCLK (Clock) |

| Test | Description |
|---|---|
| **Test 1: HW SPI Setup** | Configures SPI0 pins to alternate function ALT0, enabling the RP1 hardware SPI peripheral. After configuration, transfers are done through `/dev/spidev0.0`. Restores pins to GPIO mode at the end. |
| **Test 2: BB SPI Write** | Software SPI Mode 0 (CPOL=0, CPHA=0) implementation. Sends 2 bytes (register address + value) via bit-banging: manually controls CS (active low), generates clock, sends data MSB-first. Verifies pin states after transfer. |
| **Test 3: BB SPI Loopback** | Full-duplex loopback test. Sends 4 bytes (0xA5, 0x3C, 0xFF, 0x00) and verifies that data received on MISO matches data sent on MOSI. **Requires a physical wire from MOSI to MISO.** |

**Bit-banged SPI implementation details:**
- **Mode:** SPI Mode 0 — clock idle LOW, data sampled on rising edge
- **Bit order:** MSB first
- **Speed:** ~500 kHz (controlled via `usleep(1)`)
- **CS:** Active low, manually controlled with `pinwrite()`
- **Transfer:** Full-duplex — sends and receives simultaneously, bit by bit

**Run:**
```bash
sudo ./test/test_spi           # tests 1 and 2 (no external hardware)
sudo ./test/test_spi l         # + test 3 loopback (requires MOSI→MISO wire)
```

---

### test/test_pwm.cpp — PWM Test

Tests PWM (Pulse Width Modulation) signal generation in both hardware and software modes.

**Pins used:**

| Pin | Header | Function |
|-----|--------|---------|
| GPIO12 | 32 | PWM0_CHAN0 (ALT0) |
| GPIO13 | 33 | PWM0_CHAN1 (ALT0) |
| GPIO17 | 11 | Software PWM / Toggle speed |

| Test | Description |
|---|---|
| **Test 1: HW PWM Setup** | Configures GPIO12 and GPIO13 to alternate function ALT0 for the RP1 hardware PWM0 peripheral. Displays the commands needed for PWM control via sysfs (`/sys/class/pwm/pwmchip0/`). Restores pins at the end. |
| **Test 2: Software PWM** | Generates PWM signal by manually toggling GPIO17: runs 3 duty cycles (25%, 50%, 75%) at 1 kHz for 500 ms each. Then performs a **servo sweep** at 50 Hz: 1 ms (0°), 1.5 ms (90°), 2 ms (180°) pulses for 1 second per position. |
| **Test 3: Toggle Speed** | Measures maximum pin switching speed: performs 100,000 successive toggles using `pintoggle()` (atomic XOR register). With the direct backend, expect a 10–50 MHz toggle rate. |

**SW PWM implementation details:**
- **Method:** `pinwrite(HIGH)` → `usleep(high_us)` → `pinwrite(LOW)` → `usleep(low_us)`
- **Frequencies tested:** 1 kHz (LED dimming), 50 Hz (servo control)
- **Duty cycle:** Calculated as a percentage of the total period
- **Precision:** Limited by `usleep()` resolution (~1 μs on RPi5)

**Run:**
```bash
sudo ./test/test_pwm
```

**Optional hardware:** LED on GPIO17 (brightness variation will be visible), servo motor on GPIO17, oscilloscope for signal measurement.

---

### test/test_i2c.cpp — I2C Test

Tests the I2C (Inter-Integrated Circuit) interface in both hardware and bit-banged modes.

**Pins used:**

| Pin | Header | Function |
|-----|--------|---------|
| GPIO2 | 3 | SDA1 (I2C1 Data) |
| GPIO3 | 5 | SCL1 (I2C1 Clock) |

| Test | Description |
|---|---|
| **Test 1: HW I2C Setup** | Configures GPIO2 and GPIO3 to alternate function ALT3 for the RP1 I2C1 peripheral. Enables internal pull-ups on both pins. After configuration, transfers are done through `/dev/i2c-1`. Displays useful commands (`i2cdetect`, `i2cget`). |
| **Test 2: BB I2C Device Probe** | Probes 5 common I2C addresses (0x48 TMP102/ADS1115, 0x68 MPU6050/DS3231, 0x76 BME280, 0x3C SSD1306 OLED, 0x27 PCF8574 LCD). For each device found, reads register 0x00. |
| **Test 3: BB I2C Bus Scan** | Full scan of I2C addresses 0x03–0x77 (equivalent to `i2cdetect -y 1`). Displays a 16×8 matrix with found addresses. |

**Bit-banged I2C implementation details:**
- **Open-drain emulation:** SDA HIGH = pin set as INPUT with internal pull-up; SDA LOW = pin set as OUTPUT, driven LOW. This emulation enables correct I2C protocol operation where the line must be bidirectional.
- **Speed:** ~100 kHz (Standard Mode), controlled via `usleep(5)`
- **Full protocol:** START condition (SDA↓ while SCL=HIGH), STOP condition (SDA↑ while SCL=HIGH), write byte (MSB first + ACK read), read byte (MSB first + ACK/NACK send)
- **Implemented functions:** `i2c_probe()`, `i2c_write_reg()`, `i2c_read_reg()` — sufficient for communicating with most I2C sensors
- **Repeated START:** Implemented for register reads (write addr + read data without intermediate STOP)

**Run:**
```bash
sudo ./test/test_i2c           # tests 1 and 2 (probe common devices)
sudo ./test/test_i2c scan      # + test 3 (full I2C bus scan)
```

**Hardware required:** External 4.7 kΩ pull-ups on SDA and SCL (optional, internal pull-ups enabled). I2C device connected to pins 3 and 5 for tests 2 and 3.

---

### test/test_uart.cpp — UART Test

Tests the UART (Universal Asynchronous Receiver/Transmitter) interface in both hardware and bit-banged modes.

**Pins used:**

| Pin | Header | Function |
|-----|--------|---------|
| GPIO14 | 8 | TXD0 (UART0 Transmit) |
| GPIO15 | 10 | RXD0 (UART0 Receive) |

| Test | Description |
|---|---|
| **Test 1: HW UART Setup** | Configures GPIO14 and GPIO15 to alternate function ALT4 for the RP1 UART0 peripheral. Enables pull-up on RX (idle HIGH). After configuration, transfers are done through `/dev/ttyAMA0`. |
| **Test 2: BB UART TX** | Transmits the string "Hello from RPi5 GPIO UART!\r\n" in 8N1 format at 9600 baud via bit-banging. Verifies after transmission that the TX line returns to idle HIGH. |
| **Test 3: BB UART Timing** | Generates the 0x55 pattern 10× (alternating bits 01010101) for timing precision verification with an oscilloscope. Expected bit width: 104 μs (at 9600 baud). |
| **Test 4: BB UART Loopback** | Transmits the string "RPi5" byte by byte and verifies the TX line state after each transmission. **Requires a physical TX→RX wire.** Note: half-duplex loopback (single-thread) verifies TX state, not actual reception. |

**Bit-banged UART implementation details:**
- **Format:** 8N1 — 1 start bit (LOW), 8 data bits (LSB first), 1 stop bit (HIGH)
- **Baud rate:** 9600 (bit time = 104 μs, controlled via `usleep()`)
- **TX:** Idle HIGH. Start bit = LOW → 8 data bits → stop bit = HIGH
- **RX:** Detects start bit (HIGH→LOW transition), centers at mid-bit, reads 8 bits at 104 μs intervals, verifies stop bit
- **RX Timeout:** Configurable in milliseconds, returns -1 on expiration
- **Framing error:** Detected if start bit or stop bit are not at the expected level

**Run:**
```bash
sudo ./test/test_uart          # tests 1, 2, and 3 (no external hardware)
sudo ./test/test_uart loop     # + test 4 loopback (requires TX→RX wire)
```

**Optional hardware:** USB-to-serial adapter connected to TX (pin 8), oscilloscope for timing test.

---

### test/test_pcm.cpp — PCM / I2S Test

Tests the PCM/I2S (Inter-IC Sound) interface for digital audio, in both hardware and bit-banged modes.

**Pins used:**

| Pin | Header | Function |
|-----|--------|---------|
| GPIO18 | 12 | PCM_CLK (Bit Clock / BCLK) |
| GPIO19 | 35 | PCM_FS (Frame Sync / LRCLK / Word Select) |
| GPIO20 | 38 | PCM_DIN (Data In — from ADC) |
| GPIO21 | 40 | PCM_DOUT (Data Out — to DAC) |

| Test | Description |
|---|---|
| **Test 1: HW PCM Setup** | Configures GPIO18–21 to alternate function ALT0 for the RP1 PCM peripheral. Sets drive strength: 8 mA on BCLK (fast signal), 4 mA on FS and DOUT. After configuration, audio is controlled through ALSA (`aplay`/`arecord`). |
| **Test 2: BB I2S Pattern** | Transmits 10 I2S frames with alternating pattern 0x5555 (left channel) / 0xAAAA (right channel). Verifies pin state after transmission. The pattern is easy to identify on an oscilloscope. |
| **Test 3: BB I2S Sine Wave** | Generates 256 frames with a 16-bit stereo sine wave. Left channel contains the sine, right channel is phase-shifted by 90° (cosine). Amplitude: ±16000 (out of ±32767 max). |
| **Test 4: BB I2S Read** | Reads 8 frames from the DIN pin (GPIO20). Displays L/R values in hex and decimal. Useful for testing an I2S ADC (e.g., INMP441 microphone). |

**Bit-banged I2S implementation details:**
- **Standard:** Philips I2S — LRCLK toggles 1 BCLK before MSB
- **Format:** 16-bit per channel, stereo (32 BCLK per frame)
- **LRCLK:** LOW = left channel, HIGH = right channel
- **Bit order:** MSB first
- **Clock speed:** ~500 kHz BCLK (controlled via `usleep(1)`)
- **TX:** On the falling edge of BCLK, data is set on DOUT; the receiver samples on the rising edge
- **RX:** Clock is generated and DIN is read on the rising edge of BCLK

**Run:**
```bash
sudo ./test/test_pcm
```

**Optional hardware:** I2S DAC (e.g., PCM5102A, MAX98357A) on pins 12/35/40 for audio; I2S ADC (e.g., INMP441) on pin 38 for read test; oscilloscope for verifying clock and data signals.

---

### Test Summary

| Test file | Interface | # Tests | External HW required | Optional arg |
|---|---|---|---|---|
| `test/main.cpp` | GPIO basic | 2 | — | — |
| `test/test_spi.cpp` | SPI | 3 | MOSI→MISO wire (loopback) | `l` |
| `test/test_pwm.cpp` | PWM | 3 | LED / servo / oscilloscope | — |
| `test/test_i2c.cpp` | I2C | 3 | I2C device + pull-ups | `scan` |
| `test/test_uart.cpp` | UART | 4 | TX→RX wire (loopback) | `loop` |
| `test/test_pcm.cpp` | PCM/I2S | 4 | I2S DAC/ADC | — |

> **Note:** All tests can run without external hardware (HW Setup tests only configure pins). Loopback and scan tests require physical connections and are activated with command-line arguments.

## Architecture

```
gpio_rpi5.h                 ← Common header (API + pin definitions)
        │
        ├── gpio_rpi5.c              ← Direct register access via mmap (fast)
        │     └── /dev/gpiomem0 → RP1 registers
        │         ├── IO_BANK0/1     (GPIO CTRL/STATUS)
        │         ├── SYS_RIO0/1     (fast read/write/toggle)
        │         └── PADS_BANK0/1   (drive/pull/slew config)
        │
        └── gpio_rpi5_pinctrl.c      ← pinctrl CLI backend (fallback)
              └── popen("pinctrl set/get ...")
```

## Files

| File | Description |
|---|---|
| `gpio_rpi5.h` | Public API header with pin definitions and constants |
| `gpio_rpi5.c` | Direct register access backend (RP1 mmap, GPIO 0–53) |
| `gpio_rpi5_pinctrl.c` | Pinctrl command backend (all GPIOs, slower) |
| `build.sh` | Linux build script (`./build.sh` or `./build.sh pinctrl`) |
| `build_test.bat` | Windows build script (pinctrl backend) |
| `test/main.cpp` | Basic GPIO test (init, pintest, toggle) |
| `test/test_spi.cpp` | SPI test (HW setup + bit-banged Mode 0) |
| `test/test_pwm.cpp` | PWM test (HW setup + SW PWM + toggle speed) |
| `test/test_i2c.cpp` | I2C test (HW setup + bit-banged master) |
| `test/test_uart.cpp` | UART test (HW setup + bit-banged 8N1 TX/RX) |
| `test/test_pcm.cpp` | PCM/I2S test (HW setup + bit-banged I2S) |
| `test/build_*.sh` | Individual build scripts per test |
| `GPIO.txt` | Raw `pinctrl` output reference |

## Compatible Devices

The GPIO_RPI5 library can control a wide range of electronic devices connected to the Raspberry Pi 5's 40-pin header. Below are the main categories with concrete examples and the required pins/interfaces.

### Direct GPIO (pinwrite / pinread / pintoggle)

Simple devices requiring only digital HIGH/LOW signals:

| Device | Description | Connection |
|---|---|---|
| **LED** | Light-emitting diode | GPIO → 220–470Ω resistor → LED → GND |
| **Relay (module)** | High-power AC/DC load switching (lights, motors, locks) | GPIO → relay module IN (5V or 3.3V logic) |
| **Active buzzer** | Emits sound when voltage is applied | GPIO → buzzer → GND |
| **Button / Switch** | Read pressed/released state | GPIO (INPUT + PULL_UP) → button → GND |
| **PIR sensor** (HC-SR501) | Motion detection | Sensor OUT → GPIO (INPUT) |
| **Magnetic sensor** (reed switch) | Door/window open detection | GPIO (INPUT + PULL_UP) → reed → GND |
| **Optocoupler** (PC817, 4N35) | Galvanic isolation between circuits | GPIO → resistor → optocoupler LED |
| **Laser module** (KY-008) | 650nm laser emitter | GPIO → laser module IN |
| **IR LED** | Infrared emitter | GPIO → resistor → IR LED → GND |
| **Phototransistor / Photoresistor** | Light detection | GPIO (INPUT) with comparator or voltage divider |

### Software PWM (pinwrite + usleep)

Devices controlled via PWM pulses (software bit-banged on any GPIO pin):

| Device | Description | Typical frequency |
|---|---|---|
| **Servo motor** (SG90, MG996R) | Angular position control 0–180° | 50 Hz, pulse 1–2 ms |
| **LED dimming** | Brightness control | 500–1000 Hz |
| **DC motor** (via L298N, L293D driver) | Motor speed control | 1–20 kHz |
| **Passive buzzer** | Generate musical tones at different frequencies | 200–5000 Hz |
| **PWM fan** (4-pin) | Fan speed control | 25 kHz (via driver) |
| **RGB LED** | Color control via 3 PWM channels | 500–1000 Hz |
| **ESC** (Electronic Speed Controller) | Brushless motor control (drones, RC) | 50 Hz, pulse 1–2 ms |

### SPI (pinalt ALT0 + /dev/spidev or bit-banged)

Devices with SPI interface (4 wires: MOSI, MISO, SCLK, CS):

| Device | Description | Typical speed |
|---|---|---|
| **MCP3008 / MCP3208** | 10/12-bit ADC, 8 channels (read analog sensors) | 1–3.6 MHz |
| **MCP4921 / MCP4922** | 12-bit DAC, 1/2 channels (generate analog voltage) | 1–20 MHz |
| **MAX7219** | LED matrix 8×8 or 7-segment driver (up to 8 digits) | 10 MHz |
| **Nokia 5110** (PCD8544) | 84×48 monochrome LCD display | 4 MHz |
| **ST7735 / ILI9341** | 1.8" / 2.4" color TFT display (SPI) | 20–40 MHz |
| **SD Card** (SPI module) | Data storage on microSD card | 25 MHz |
| **W25Q32 / W25Q128** | 4–16 MB SPI flash (non-volatile storage) | 50–100 MHz |
| **nRF24L01+** | 2.4 GHz radio transceiver (wireless communication) | 8–10 MHz |
| **RFM95W / RFM69** | LoRa / FSK transceiver (long-range communication) | 10 MHz |
| **ENC28J60** | 10Base-T Ethernet controller (networking via SPI) | 20 MHz |
| **MCP2515** | CAN bus controller (vehicle/industrial communication) | 10 MHz |
| **ADS1256** | 24-bit ADC, 8 channels (precision measurements) | 1.92 MHz |
| **ADXL345** | 3-axis accelerometer ±16g | 5 MHz |
| **BME280 / BMP280** | Temperature, humidity, pressure sensor | 10 MHz |

### I2C (pinalt ALT3 + /dev/i2c or bit-banged)

Devices with I2C interface (2 wires: SDA, SCL):

| Device | Address | Description |
|---|---|---|
| **SSD1306** | 0x3C/0x3D | 128×64 / 128×32 monochrome OLED display |
| **BME280 / BMP280** | 0x76/0x77 | Temperature + humidity + atmospheric pressure sensor |
| **MPU6050** | 0x68/0x69 | 6-axis accelerometer + gyroscope (IMU) |
| **DS3231** | 0x68 | High-precision real-time clock (RTC) |
| **ADS1115 / ADS1015** | 0x48–0x4B | 16/12-bit ADC, 4 channels (read analog sensors) |
| **PCF8574** | 0x20–0x27 | 8-bit I/O expander (extends available GPIOs) |
| **MCP23017** | 0x20–0x27 | 16-bit I/O expander with interrupts |
| **PCA9685** | 0x40–0x7F | 16-channel PWM driver, 12-bit (servos, LEDs) |
| **TCA9548A** | 0x70–0x77 | 8-channel I2C multiplexer (connect devices with same address) |
| **HT16K33** | 0x70–0x77 | LED matrix / 7-segment driver |
| **TMP102 / TMP117** | 0x48–0x4B | Digital temperature sensor ±0.1°C |
| **INA219 / INA226** | 0x40–0x4F | Current and voltage monitor (power monitoring) |
| **VEML7700 / TSL2591** | 0x10/0x29 | Ambient light sensor (lux meter) |
| **VL53L0X / VL53L1X** | 0x29/0x52 | Laser ToF distance sensor (up to 2–4 m) |
| **SGP30 / SGP40** | 0x58/0x59 | Air quality sensor (eCO2, TVOC) |
| **SHT31 / SHT40** | 0x44/0x45 | Precision temperature + humidity sensor |
| **LCD 16×2 / 20×4** (with PCF8574) | 0x27/0x3F | Alphanumeric LCD display (via I2C adapter) |
| **EEPROM AT24C32/256** | 0x50–0x57 | Non-volatile memory 4–32 KB |

### UART (pinalt ALT4 + /dev/ttyAMA0 or bit-banged)

Devices with UART serial interface (2 wires: TX, RX):

| Device | Typical baud rate | Description |
|---|---|---|
| **GPS** (NEO-6M, NEO-M8N) | 9600 | GPS module — receives NMEA coordinates |
| **HC-05 / HC-06** | 9600–115200 | Classic Bluetooth module (SPP) |
| **HM-10 / AT-09** | 9600 | Bluetooth Low Energy (BLE) module |
| **SIM800L / SIM7600** | 115200 | GSM/GPRS/4G module (SMS, calls, mobile data) |
| **ESP8266 / ESP32** | 115200 | WiFi module (AT commands or custom communication) |
| **RFID** (RDM6300) | 9600 | 125 kHz RFID card reader |
| **PM sensor** (PMS5003, SDS011) | 9600 | PM2.5/PM10 fine particle sensor (air quality) |
| **Fingerprint** (R307, AS608) | 57600 | Fingerprint sensor |
| **Thermal printer** (CSN-A2) | 19200 | Mini thermal printer (receipts) |
| **LiDAR** (TFMini, RPLIDAR) | 115200 | Laser distance sensor / 360° scanner |
| **RS-485** (via MAX485) | 9600–115200 | Industrial half-duplex communication |
| **Nextion Display** | 9600 | TFT touchscreen display with serial interface |

### PCM / I2S (pinalt ALT0 — GPIO18–21)

Digital audio devices with I2S interface:

| Device | Description |
|---|---|
| **PCM5102A** | Stereo I2S DAC 32-bit/384 kHz (high-quality audio output) |
| **MAX98357A** | I2S amplifier + DAC 3W (direct speaker, no external amplifier) |
| **UDA1334A** | Stereo I2S DAC with 3.5mm jack output |
| **INMP441** | Digital I2S MEMS microphone (audio recording) |
| **SPH0645** | I2S microphone with integrated amplification |
| **ICS-43434** | High-SNR I2S microphone |
| **WM8960** | Full-duplex audio codec (DAC + ADC + amplifier) |
| **CS4344** | Stereo I2S DAC 24-bit/192 kHz |

### Complete Project Examples

| Project | Required components | Interfaces used |
|---|---|---|
| **Weather station** | BME280 + SSD1306 OLED + status LED | I2C + GPIO |
| **Alarm system** | PIR sensor + reed switch + buzzer + siren relay | GPIO |
| **Access control** | Fingerprint sensor + RFID + lock relay + LEDs | UART + GPIO |
| **Energy monitoring** | INA219 + 16×2 LCD + SD card (logging) | I2C + SPI |
| **Mobile robot** | MPU6050 + 2× DC motor (L298N) + distance sensor | I2C + PWM + GPIO |
| **Audio player** | PCM5102A DAC + control buttons + status LEDs | I2S + GPIO |
| **GPS tracker** | NEO-6M + SIM800L + SD card | UART + SPI |
| **Irrigation system** | Soil moisture sensor (MCP3008) + pump relay + DS3231 RTC | SPI + GPIO + I2C |
| **Digital thermometer** | TMP117 + MAX7219 7-segment display | I2C + SPI |
| **Wireless communication** | nRF24L01+ × 2 (TX + RX on 2 RPi5s) | SPI |

> **Note:** All devices listed above operate at **3.3V logic level** (native RPi5). Devices requiring 5V (relays, motors, some sensors) must be powered separately, with adapted logic level (level shifter) or with modules that accept 3.3V on control pins. Do not exceed **50 mA** total on GPIO pins (12 mA / pin with `DRIVE_12MA`).

## Requirements

- **Raspberry Pi 5** running Raspberry Pi OS (64-bit recommended)
- `/dev/gpiomem0` accessible (default on Raspberry Pi OS) — for direct register backend
- `pinctrl` tool installed (default on Raspberry Pi OS) — for pinctrl backend
- GCC toolchain (`gcc`, `g++`, `ar`)

## License

Free to use and modify.
