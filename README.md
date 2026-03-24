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
| `test/main.cpp` | Test program |
| `GPIO.txt` | Raw `pinctrl` output reference |

## Requirements

- **Raspberry Pi 5** running Raspberry Pi OS (64-bit recommended)
- `/dev/gpiomem0` accessible (default on Raspberry Pi OS) — for direct register backend
- `pinctrl` tool installed (default on Raspberry Pi OS) — for pinctrl backend
- GCC toolchain (`gcc`, `g++`, `ar`)

## License

Free to use and modify.
