/**
 * @file gpio_rpi5.h
 * @brief Public API header for the GPIO_RPI5 library (Raspberry Pi 5).
 *
 * Provides a hardware abstraction layer for controlling GPIO pins on the
 * Raspberry Pi 5 through either direct RP1 register access (mmap) or the
 * pinctrl CLI backend. The API covers digital I/O, pull resistor
 * configuration, alternate function selection, and drive strength control.
 *
 * Two interchangeable backends implement this API:
 * - @ref gpio_rpi5.c — Direct register access via `/dev/gpiomem0` (fast, ~10 ns per operation)
 * - @ref gpio_rpi5_pinctrl.c — `pinctrl` CLI wrapper (slower, ~10 ms per operation, wider GPIO range)
 *
 * @note All GPIO numbers follow the RP1/BCM2712 scheme, not physical header pin numbers.
 * @see https://datasheets.raspberrypi.com/rp1/rp1-peripherals.pdf
 */

#ifndef GPIO_RPI5_H
#define GPIO_RPI5_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum valid GPIO index across all banks.
 *
 * Covers RP1 GPIOs (0–53), BCM2712 GPIOs (100–135), and AON GPIOs (200–237).
 */
#define GPIO_MAX_INDEX 237

/**
 * @defgroup gpio_pins GPIO Pin Definitions
 * @brief Symbolic constants for all Raspberry Pi 5 GPIO pins.
 *
 * Three GPIO banks are defined:
 * - **RP1 Bank** (GPIO0–GPIO53): Main GPIO controller on the RP1 southbridge.
 *   GPIO0–GPIO27 are accessible on the 40-pin header.
 * - **BCM2712 Bank** (GPIOx100–GPIOx135): Processor-side GPIOs.
 * - **AON Bank** (GPIOx200–GPIOx237): Always-On domain GPIOs.
 *
 * Each definition includes a comment showing the default boot state:
 * `mode  pull | level` — e.g., `ip pu | hi` = input, pull-up, reads high.
 * @{
 */
#define GPIO0  0//: ip    pu | hi // ID_SDA/GPIO0 = input
#define GPIO1  1//: ip    pu | hi // ID_SCL/GPIO1 = input
#define GPIO2  2//: a3    pu | hi // GPIO2 = SDA1
#define GPIO3  3//: a3    pu | hi // GPIO3 = SCL1
#define GPIO4  4//: op dh pu | hi // GPIO4 = output
#define GPIO5  5//: no    pu | -- // GPIO5 = none
#define GPIO6  6//: no    pu | -- // GPIO6 = none
#define GPIO7  7//: op dh pu | hi // GPIO7 = output
#define GPIO8  8//: op dh pu | hi // GPIO8 = output
#define GPIO9  9//: a0    pn | lo // GPIO9 = SPI0_MISO
#define GPIO10 10//: a0    pn | hi // GPIO10 = SPI0_MOSI
#define GPIO11 11//: a0    pn | lo // GPIO11 = SPI0_SCLK
#define GPIO12 12//: no    pd | -- // GPIO12 = none
#define GPIO13 13//: no    pd | -- // GPIO13 = none
#define GPIO14 14//: a4    pn | hi // GPIO14 = TXD0
#define GPIO15 15//: a4    pu | hi // GPIO15 = RXD0
#define GPIO16 16//: no    pd | -- // GPIO16 = none
#define GPIO17 17//: no    pd | -- // GPIO17 = none
#define GPIO18 18//: no    pd | -- // GPIO18 = none
#define GPIO19 19//: no    pd | -- // GPIO19 = none
#define GPIO20 20//: no    pd | -- // GPIO20 = none
#define GPIO21 21//: no    pd | -- // GPIO21 = none
#define GPIO22 22//: no    pd | -- // GPIO22 = none
#define GPIO23 23//: no    pd | -- // GPIO23 = none
#define GPIO24 24//: no    pd | -- // GPIO24 = none
#define GPIO25 25//: no    pd | -- // GPIO25 = none
#define GPIO26 26//: no    pd | -- // GPIO26 = none
#define GPIO27 27//: no    pd | -- // GPIO27 = none
#define GPIO28 28//: ip    pd | lo // PCIE_RP1_WAKE/GPIO28 = input
#define GPIO29 29//: no    pu | hi // FAN_TACH/GPIO29 = none
#define GPIO30 30//: no    pu | -- // HOST_SDA/GPIO30 = none
#define GPIO31 31//: no    pu | -- // HOST_SCL/GPIO31 = none
#define GPIO32 32//: op dh pd | hi // ETH_RST_N/GPIO32 = output
#define GPIO33 33//: no    pd | lo // GPIO33 = none
#define GPIO34 34//: op dl pd | lo // CD0_IO0_MICCLK/GPIO34 = output
#define GPIO35 35//: no    pd | lo // CD0_IO0_MICDAT0/GPIO35 = none
#define GPIO36 36//: no    pd | lo // RP1_PCIE_CLKREQ_N/GPIO36 = none
#define GPIO37 37//: no    pd | lo // GPIO37 = none
#define GPIO38 38//: ip    pd | hi // CD0_SDA/GPIO38 = input
#define GPIO39 39//: ip    pd | hi // CD0_SCL/GPIO39 = input
#define GPIO40 40//: ip    pd | hi // CD1_SDA/GPIO40 = input
#define GPIO41 41//: ip    pd | hi // CD1_SCL/GPIO41 = input
#define GPIO42 42//: a2    pd | hi // USB_VBUS_EN/GPIO42 = VBUS_EN1
#define GPIO43 43//: a2    pu | hi // USB_OC_N/GPIO43 = VBUS_OC1
#define GPIO44 44//: op dh pd | hi // RP1_STAT_LED/GPIO44 = output
#define GPIO45 45//: a0    pd | lo // FAN_PWM/GPIO45 = PWM1_CHAN3
#define GPIO46 46//: op dl pd | lo // CD1_IO0_MICCLK/GPIO46 = output
#define GPIO47 47//: no    pd | lo // 2712_WAKE/GPIO47 = none
#define GPIO48 48//: no    pd | lo // CD1_IO1_MICDAT1/GPIO48 = none
#define GPIO49 49//: op dh pd | hi // EN_MAX_USB_CUR/GPIO49 = output
#define GPIO50 50//: no    pd | -- // GPIO50 = none
#define GPIO51 51//: no    pd | -- // GPIO51 = none
#define GPIO52 52//: no    pu | -- // GPIO52 = none
#define GPIO53 53//: no    pu | hi // GPIO53 = none
#define GPIOx100 100//: ip     pd | lo // GPIO0 = input
#define GPIOx101 101//: op dh pu | hi // 2712_BOOT_CS_N/GPIO1 = output
#define GPIOx102 102//: a6    pn | hi // 2712_BOOT_MISO/GPIO2 = VC_SPI0_MISO
#define GPIOx103 103//: a5    pn | hi // 2712_BOOT_MOSI/GPIO3 = VC_SPI0_MOSI
#define GPIOx104 104//: a6    pn | lo // 2712_BOOT_SCLK/GPIO4 = VC_SPI0_SCLK
#define GPIOx105 105//: ip    pd | lo // GPIO5 = input
#define GPIOx106 106//: ip    pd | lo // GPIO6 = input
#define GPIOx107 107//: ip    pd | lo // GPIO7 = input
#define GPIOx108 108//: ip    pd | lo // GPIO8 = input
#define GPIOx109 109//: ip    pd | lo // GPIO9 = input
#define GPIOx110 110//: ip    pd | lo // GPIO10 = input
#define GPIOx111 111//: ip    pd | lo // GPIO11 = input
#define GPIOx112 112//: ip    pd | lo // GPIO12 = input
#define GPIOx113 113//: ip    pd | lo // GPIO13 = input
#define GPIOx114 114//: a1    pd | lo // PCIE_SDA/GPIO14 = SPI_S_MOSI_OR_BSC_S_SDA
#define GPIOx115 115//: a1    pd | lo // PCIE_SCL/GPIO15 = SPI_S_SCK_OR_BSC_S_SCL
#define GPIOx116 116//: ip    pd | lo // GPIO16 = input
#define GPIOx117 117//: ip    pd | lo // GPIO17 = input
#define GPIOx118 118//: ip    pd | lo // GPIO18 = input
#define GPIOx119 119//: ip    pd | lo // GPIO19 = input
#define GPIOx120 120//: ip    pu | hi // PWR_GPIO/GPIO20 = input
#define GPIOx121 121//: ip    pd | lo // 2712_G21_FS/GPIO21 = input
#define GPIOx122 122//: ip    pd | lo // GPIO22 = input
#define GPIOx123 123//: ip    pd | lo // GPIO23 = input
#define GPIOx124 124//: a3    pn | lo // BT_RTS/GPIO24 = UART_RTS_0
#define GPIOx125 125//: a4    pu | lo // BT_CTS/GPIO25 = UART_CTS_0
#define GPIOx126 126//: a4    pn | hi // BT_TXD/GPIO26 = UART_TXD_0
#define GPIOx127 127//: a4    pu | hi // BT_RXD/GPIO27 = UART_RXD_0
#define GPIOx128 128//: op dh pd | hi // WL_ON/GPIO28 = output
#define GPIOx129 129//: op dh pd | hi // BT_ON/GPIO29 = output
#define GPIOx130 130//: a4    pn | hi // WIFI_SDIO_CLK/GPIO30 = SD2_CLK
#define GPIOx131 131//: a4    pu | hi // WIFI_SDIO_CMD/GPIO31 = SD2_CMD
#define GPIOx132 132//: a4    pd | hi // WIFI_SDIO_D0/GPIO32 = SD2_DAT0
#define GPIOx133 133//: a3    pu | hi // WIFI_SDIO_D1/GPIO33 = SD2_DAT1
#define GPIOx134 134//: a4    pn | hi // WIFI_SDIO_D2/GPIO34 = SD2_DAT2
#define GPIOx135 135//: a3    pn | hi // WIFI_SDIO_D3/GPIO35 = SD2_DAT3
#define GPIOx200 200//: ip    pd | hi // RP1_SDA/AON_GPIO0 = input
#define GPIOx201 201//: ip    pd | hi // RP1_SCL/AON_GPIO1 = input
#define GPIOx202 202//: op dh pd | hi // RP1_RUN/AON_GPIO2 = output
#define GPIOx203 203//: op dh pd | hi // SD_IOVDD_SEL/AON_GPIO3 = output
#define GPIOx204 204//: op dh pd | hi // SD_PWR_ON/AON_GPIO4 = output
#define GPIOx205 205//: a6    pu | lo // SD_CDET_N/AON_GPIO5 = SD_CARD_PRES_G
#define GPIOx206 206//: ip    pd | hi // SD_FLG_N/AON_GPIO6 = input
#define GPIOx207 207//: ip    pd | lo // AON_GPIO7 = input
#define GPIOx208 208//: ip    pd | lo // 2712_WAKE/AON_GPIO8 = input
#define GPIOx209 209//: op dh pd | hi // 2712_STAT_LED/AON_GPIO9 = output
#define GPIOx210 210//: ip    pd | lo // AON_GPIO10 = input
#define GPIOx211 211//: ip    pd | lo // AON_GPIO11 = input
#define GPIOx212 212//: ip    pd | lo // PMIC_INT/AON_GPIO12 = input
#define GPIOx213 213//: a3    pu | hi // UART_TX_FS/AON_GPIO13 = VC_TXD0
#define GPIOx214 214//: a3    pu | hi // UART_RX_FS/AON_GPIO14 = VC_RXD0
#define GPIOx215 215//: ip    pd | lo // AON_GPIO15 = input
#define GPIOx216 216//: ip    pu | hi // AON_GPIO16 = input
#define GPIOx232 232//: a1    -- | hi // HDMI0_SCL/AON_SGPIO0 = HDMI_TX0_BSC_SCL
#define GPIOx233 233//: a1    -- | hi // HDMI0_SDA/AON_SGPIO1 = HDMI_TX0_BSC_SDA
#define GPIOx234 234//: a1    -- | hi // HDMI1_SCL/AON_SGPIO2 = HDMI_TX1_BSC_SCL
#define GPIOx235 235//: a1    -- | hi // HDMI1_SDA/AON_SGPIO3 = HDMI_TX1_BSC_SDA
#define GPIOx236 236//: a2    -- | hi // PMIC_SCL/AON_SGPIO4 = BSC_M2_SCL
#define GPIOx237 237//: a2    -- | hi // PMIC_SDA/AON_SGPIO5 = BSC_M2_SDA
/** @} */ /* end of gpio_pins */

/**
 * @defgroup pin_modes Pin Mode Constants
 * @brief Direction modes for GPIO pins.
 * @{
 */
/**
 * Pin modes: */
#define INPUT (0)   /**< @brief Configure pin as digital input. */
#define OUTPUT (1)  /**< @brief Configure pin as digital output. */
/** @} */ /* end of pin_modes */

/**
 * @defgroup pin_states Pin State Constants
 * @brief Logic level values returned by pinread() and used by pinwrite().
 * @{
 */
#define LOW (0)     /**< @brief Logic low (0 V). */
#define HIGH (1)    /**< @brief Logic high (3.3 V on RPi5). */
#define UNDEF (3)   /**< @brief Unknown / uninitialized state. */
/** @} */ /* end of pin_states */

/**
 * @defgroup pull_resistors Pull Resistor Constants
 * @brief Internal pull-up/pull-down resistor configuration values.
 *
 * The RP1 includes programmable internal pull resistors on every GPIO pad.
 * Use these constants with pinpull() to set the desired configuration.
 * @{
 */
#define PULL_NONE (0) /**< @brief No pull resistor (pin is floating). */
#define PULL_UP   (1) /**< @brief Enable internal pull-up resistor (~50 kOhm to 3.3 V). */
#define PULL_DOWN (2) /**< @brief Enable internal pull-down resistor (~50 kOhm to GND). */
/** @} */ /* end of pull_resistors */
/**
 * @defgroup alt_functions Alternate Function Constants
 * @brief FUNCSEL values for the RP1 GPIO controller.
 *
 * Each GPIO pin can be muxed to one of several peripheral functions (SPI,
 * I2C, UART, PWM, PCM, etc.) by writing the corresponding FUNCSEL value
 * to the pin's control register. The mapping varies per pin — consult the
 * RP1 peripherals datasheet for the full pin-function matrix.
 * @{
 */
#define ALT0  (0)  /**< @brief Alternate function 0 (e.g., SPI0, PCM). */
#define ALT1  (1)  /**< @brief Alternate function 1. */
#define ALT2  (2)  /**< @brief Alternate function 2. */
#define ALT3  (3)  /**< @brief Alternate function 3 (e.g., I2C1). */
#define ALT4  (4)  /**< @brief Alternate function 4 (e.g., UART0). */
#define ALT5  (5)  /**< @brief SIO — Standard GPIO mode (default after pinopen). */
#define ALT6  (6)  /**< @brief Alternate function 6. */
#define ALT7  (7)  /**< @brief Alternate function 7. */
#define ALT8  (8)  /**< @brief Alternate function 8. */
#define GPIO_FUNC_SIO  ALT5   /**< @brief Normal GPIO (SIO) function selector. */
#define GPIO_FUNC_NULL (31)   /**< @brief Disconnect pin (set by pinclose). */
/** @} */ /* end of alt_functions */
/**
 * @defgroup drive_strength Drive Strength Constants
 * @brief Output drive current configuration for GPIO pads.
 *
 * Controls the maximum current a GPIO output can source/sink.
 * Higher drive strength is needed for fast signals (SPI clock) or heavy loads.
 * The default after reset is typically DRIVE_4MA.
 * @{
 */
#define DRIVE_2MA  (0)  /**< @brief 2 mA drive strength. */
#define DRIVE_4MA  (1)  /**< @brief 4 mA drive strength (default). */
#define DRIVE_8MA  (2)  /**< @brief 8 mA drive strength. */
#define DRIVE_12MA (3)  /**< @brief 12 mA drive strength (maximum). */
/** @} */ /* end of drive_strength */

/** @brief Default pin used by gpio_pintest(). */
#define TEST_PIN 17

/**
 * @brief Represents the state of a single GPIO pin.
 *
 * Returned by pinopen() and cached internally for each pin.
 * The @c state field holds the last known logic level, and the
 * @c mode field holds the current direction (INPUT or OUTPUT).
 */
typedef struct {
        int state; /**< @brief Logic level: LOW (0), HIGH (1), or UNDEF (3). */
        int mode;  /**< @brief Direction: INPUT (0), OUTPUT (1), or UNDEF (3). */
} pin_t;

/**
 * @defgroup core_api Core API Functions
 * @brief Functions for GPIO initialization, I/O, and configuration.
 * @{
 */

/**
 * @brief Initialize the GPIO subsystem.
 *
 * Must be called once before any other GPIO function. Opens the memory-mapped
 * device (mmap backend) or prepares internal state (pinctrl backend).
 *
 * @return 0 on success, -1 on failure (e.g., `/dev/gpiomem0` not accessible).
 */
int gpio_init(void);

/**
 * @brief Release all GPIO resources.
 *
 * Unmaps memory (mmap backend) and closes file descriptors. Should be called
 * at program exit or when GPIO access is no longer needed.
 */
void gpio_cleanup(void);

/**
 * @brief Quick pin test: briefly pulses a pin HIGH then LOW.
 *
 * Opens the pin as OUTPUT, writes HIGH, waits 1 second, writes LOW, then
 * closes the pin. Useful for verifying that a pin is physically functional.
 *
 * @param pin_indx GPIO number to test (0–53 for mmap backend).
 * @return 0 on success, -1 on error.
 */
int gpio_pintest(int pin_indx);

/**
 * @brief Open a GPIO pin for use in the specified direction.
 *
 * Configures the pin as INPUT or OUTPUT by setting the FUNCSEL to SIO mode
 * and adjusting the output-enable register. The pin must be opened before
 * calling pinwrite(), pinread(), or pintoggle().
 *
 * @param pin  GPIO number (0–53 for mmap backend, 0–237 for pinctrl).
 * @param mode Direction: INPUT (0) or OUTPUT (1).
 * @return A pin_t struct with the configured mode. On error, both fields are UNDEF.
 */
pin_t pinopen(int pin, int mode);

/**
 * @brief Close a GPIO pin and reset it to a safe state.
 *
 * Sets the pin back to input mode and disconnects the function selector
 * (FUNCSEL = NULL). The internal state is reset to UNDEF.
 *
 * @param indx_pin GPIO number to close.
 */
void pinclose(int indx_pin);

/**
 * @brief Write a digital value (HIGH or LOW) to a GPIO output pin.
 *
 * Uses atomic SET/CLR registers (mmap backend) for glitch-free writes.
 *
 * @param indx_pin GPIO number (must have been opened as OUTPUT).
 * @param value    Desired level: HIGH (1) or LOW (0).
 * @return 0 on success, -1 on error.
 */
int pinwrite(int indx_pin, int value);

/**
 * @brief Read the current logic level of a GPIO pin.
 *
 * Reads the input register regardless of whether the pin is configured as
 * INPUT or OUTPUT (output pins can be read back).
 *
 * @param pin GPIO number.
 * @return HIGH (1), LOW (0), or -1 on error.
 */
int pinread(int pin);

/**
 * @brief Toggle the output state of a GPIO pin.
 *
 * Uses the atomic XOR register (mmap backend) for a single-instruction
 * toggle, which is faster and safer than a read-modify-write sequence.
 *
 * @param pin GPIO number (must have been opened as OUTPUT).
 * @return 0 on success, -1 on error.
 */
int pintoggle(int pin);

/**
 * @brief Configure the internal pull resistor for a GPIO pin.
 *
 * Sets pull-up, pull-down, or no-pull on the pin's pad register.
 * Pull resistors ensure a defined logic level when the pin is not actively
 * driven — essential for buttons, open-drain buses, etc.
 *
 * @param pin  GPIO number.
 * @param pull One of PULL_UP, PULL_DOWN, or PULL_NONE.
 * @return 0 on success, -1 on error.
 */
int pinpull(int pin, int pull);

/**
 * @brief Set the alternate function (FUNCSEL) for a GPIO pin.
 *
 * Switches the pin from standard GPIO mode to a peripheral function
 * (e.g., SPI, I2C, UART, PWM, PCM). The function number is chip-specific;
 * refer to the RP1 datasheet for the pin-function mapping table.
 *
 * @param pin  GPIO number.
 * @param func Alternate function number: ALT0–ALT8, or GPIO_FUNC_NULL (31).
 * @return 0 on success, -1 on error.
 */
int pinalt(int pin, int func);

/**
 * @brief Set the output drive strength for a GPIO pin.
 *
 * Configures how much current the pin can source or sink. Higher values
 * are needed for driving long traces, LEDs, or fast digital signals.
 *
 * @param pin      GPIO number.
 * @param strength One of DRIVE_2MA, DRIVE_4MA, DRIVE_8MA, or DRIVE_12MA.
 * @return 0 on success, -1 on error.
 * @note Only supported by the mmap backend. The pinctrl backend returns -1.
 */
int pin_set_drive(int pin, int strength);

/** @} */ /* end of core_api */

#ifdef __cplusplus
}
#endif

#endif // GPIO_RPI5_H
