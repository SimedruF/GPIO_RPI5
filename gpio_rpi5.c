/**
 * @file gpio_rpi5.c
 * @brief Direct register access GPIO backend for Raspberry Pi 5.
 *
 * This backend uses memory-mapped I/O (`mmap`) on `/dev/gpiomem0` to access
 * the RP1 southbridge GPIO registers directly from userspace. It provides
 * extremely fast GPIO operations (~10 ns per read/write) compared to the
 * pinctrl CLI backend (~10 ms).
 *
 * @par Supported GPIO Range
 * Only RP1 GPIOs 0–53 are accessible through this backend. For BCM2712
 * GPIOs (100–135) and AON GPIOs (200–237), use the pinctrl backend instead.
 *
 * @par RP1 Memory Map (via /dev/gpiomem0, 192 KB region)
 * | Block        | Offset   | Description                          |
 * |--------------|----------|--------------------------------------|
 * | IO_BANK0     | 0x00000  | GPIO 0–27 control/status registers   |
 * | IO_BANK1     | 0x04000  | GPIO 28–53 control/status registers  |
 * | SYS_RIO0     | 0x10000  | GPIO 0–27 fast register I/O          |
 * | SYS_RIO1     | 0x14000  | GPIO 28–53 fast register I/O         |
 * | PADS_BANK0   | 0x20000  | GPIO 0–27 pad configuration          |
 * | PADS_BANK1   | 0x24000  | GPIO 28–53 pad configuration         |
 *
 * @par Atomic Register Access
 * The SYS_RIO blocks support atomic SET, CLR, and XOR operations at fixed
 * offsets from the base, allowing glitch-free single-register writes without
 * read-modify-write races.
 *
 * @see https://datasheets.raspberrypi.com/rp1/rp1-peripherals.pdf
 *
 * Compile with: gcc -c gpio_rpi5.c -o gpio_rpi5.o -Wall -Wextra
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

#include "gpio_rpi5.h"

/* ============================================================
 * RP1 Register Map (mapped via /dev/gpiomem0)
 * ============================================================
 * IO_BANK0     0x00000  GPIO 0-27  control registers
 * IO_BANK1     0x04000  GPIO 28-53 control registers
 * SYS_RIO0     0x10000  GPIO 0-27  fast register I/O
 * SYS_RIO1     0x14000  GPIO 28-53 fast register I/O
 * PADS_BANK0   0x20000  GPIO 0-27  pad configuration
 * PADS_BANK1   0x24000  GPIO 28-53 pad configuration
 * ============================================================ */

/**
 * @defgroup rp1_registers RP1 Register Definitions
 * @brief Memory offsets and bit-field masks for the RP1 GPIO controller.
 * @{
 */

/** @brief Device path for the RP1 GPIO memory region. */
#define GPIOMEM_DEVICE  "/dev/gpiomem0"

/** @brief Total size of the memory-mapped region (192 KB). */
#define GPIO_MEM_SIZE   0x30000

/** @name IO Bank Base Offsets
 *  Control and status registers for each GPIO pin.
 * @{ */
#define RP1_IO_BANK0     0x00000  /**< @brief GPIO 0–27 control registers. */
#define RP1_IO_BANK1     0x04000  /**< @brief GPIO 28–53 control registers. */
/** @} */

/** @name RIO (Register I/O) Base Offsets
 *  Fast read/write/toggle registers with atomic access support.
 * @{ */
#define RP1_SYS_RIO0     0x10000  /**< @brief GPIO 0–27 fast register I/O. */
#define RP1_SYS_RIO1     0x14000  /**< @brief GPIO 28–53 fast register I/O. */
/** @} */

/** @name Pad Control Base Offsets
 *  Drive strength, pull resistor, slew rate, and Schmitt trigger configuration.
 * @{ */
#define RP1_PADS_BANK0   0x20000  /**< @brief GPIO 0–27 pad configuration. */
#define RP1_PADS_BANK1   0x24000  /**< @brief GPIO 28–53 pad configuration. */
/** @} */

/** @name Atomic Access Offsets
 *  Added to the RIO base to perform atomic bit operations without read-modify-write.
 * @{ */
#define RP1_XOR_OFFSET   0x1000  /**< @brief XOR (toggle) bits in the register. */
#define RP1_SET_OFFSET   0x2000  /**< @brief SET (write 1) bits in the register. */
#define RP1_CLR_OFFSET   0x3000  /**< @brief CLR (write 0) bits in the register. */
/** @} */

/** @name RIO Register Offsets
 *  Offsets within each SYS_RIOx block for output, output-enable, and input.
 * @{ */
#define RIO_OUT  0x00   /**< @brief Output value register. */
#define RIO_OE   0x04   /**< @brief Output enable register (1 = output, 0 = input). */
#define RIO_IN   0x08   /**< @brief Input value register (read-only). */
/** @} */

/** @name IO Bank Per-Pin Register Macros
 *  Each pin occupies 8 bytes: STATUS (4 bytes) + CTRL (4 bytes).
 * @{ */
#define GPIO_STATUS(pin_in_bank) ((pin_in_bank) * 8)       /**< @brief STATUS register offset for a pin within its bank. */
#define GPIO_CTRL(pin_in_bank)   ((pin_in_bank) * 8 + 4)   /**< @brief CTRL register offset for a pin within its bank. */
/** @} */

/** @name GPIO CTRL Register Fields
 * @{ */
#define CTRL_FUNCSEL_MASK  0x1F          /**< @brief FUNCSEL field mask (bits 4:0). */
#define CTRL_FUNCSEL_LSB   0             /**< @brief FUNCSEL field LSB position. */
#define CTRL_OUTOVER_MASK  (0x3 << 12)   /**< @brief Output override field mask. */
#define CTRL_OEOVER_MASK   (0x3 << 14)   /**< @brief Output-enable override field mask. */
/** @} */

/** @name PADS Register Macros and Bit Fields
 * @{ */
#define PADS_GPIO(pin_in_bank) (0x04 + (pin_in_bank) * 4)  /**< @brief Pad register offset (pin 0 starts at 0x04). */
#define PAD_OD_BIT       (1 << 7)  /**< @brief Output disable bit. */
#define PAD_IE_BIT       (1 << 6)  /**< @brief Input enable bit. */
#define PAD_DRIVE_MASK   (0x3 << 4) /**< @brief Drive strength field mask. */
#define PAD_DRIVE_LSB    4          /**< @brief Drive strength field LSB position. */
#define PAD_PUE_BIT      (1 << 3)  /**< @brief Pull-up enable bit. */
#define PAD_PDE_BIT      (1 << 2)  /**< @brief Pull-down enable bit. */
#define PAD_SCHMITT_BIT  (1 << 1)  /**< @brief Schmitt trigger enable bit. */
#define PAD_SLEWFAST_BIT (1 << 0)  /**< @brief Fast slew rate enable bit. */
/** @} */

/** @} */ /* end of rp1_registers */

/* ============================================================ */

/** @brief Base pointer to the memory-mapped RP1 register region. */
static volatile uint32_t *gpio_mmap_base = NULL;

/** @brief File descriptor for `/dev/gpiomem0`. */
static int gpio_mmap_fd = -1;

/** @brief Initialization flag (1 after successful gpio_init()). */
static int gpio_initialized = 0;

/** @brief Global array caching the state of all GPIO pins. */
pin_t rpi5_gpio[GPIO_MAX_INDEX + 1];

/* ---- Internal helpers ------------------------------------ */

/**
 * @brief Read a 32-bit hardware register.
 * @param byte_offset Byte offset from the mapped base address.
 * @return The 32-bit register value.
 */
static inline uint32_t reg_read(uint32_t byte_offset)
{
  return gpio_mmap_base[byte_offset / 4];
}

/**
 * @brief Write a 32-bit value to a hardware register.
 * @param byte_offset Byte offset from the mapped base address.
 * @param val         Value to write.
 */
static inline void reg_write(uint32_t byte_offset, uint32_t val)
{
  gpio_mmap_base[byte_offset / 4] = val;
}

/**
 * @brief Resolve a GPIO number (0–53) into its bank-specific parameters.
 *
 * The RP1 has two banks: Bank 0 covers GPIO 0–27, Bank 1 covers GPIO 28–53.
 * This function returns the base offsets for IO, RIO, and PADS registers,
 * along with the bit position within the bank.
 *
 * @param[in]  pin        GPIO number (0–53).
 * @param[out] io_base    IO bank base offset.
 * @param[out] rio_base   RIO bank base offset.
 * @param[out] pads_base  PADS bank base offset.
 * @param[out] bit        Bit position of the pin within the bank (0–27).
 * @return 0 on success, -1 if the pin is out of the RP1 range.
 */
static int pin_to_bank(int pin,
                       uint32_t *io_base,
                       uint32_t *rio_base,
                       uint32_t *pads_base,
                       int *bit)
{
  if (pin >= 0 && pin <= 27)
  {
    *io_base   = RP1_IO_BANK0;
    *rio_base  = RP1_SYS_RIO0;
    *pads_base = RP1_PADS_BANK0;
    *bit       = pin;
    return 0;
  }
  if (pin >= 28 && pin <= 53)
  {
    *io_base   = RP1_IO_BANK1;
    *rio_base  = RP1_SYS_RIO1;
    *pads_base = RP1_PADS_BANK1;
    *bit       = pin - 28;
    return 0;
  }
  return -1;
}

/**
 * @brief Validate that a pin number is within the RP1 range.
 * @param pin GPIO number to validate.
 * @return 1 if valid (0–53), 0 otherwise.
 */
static int gpio_valid_pin(int pin)
{
  return (pin >= 0 && pin <= 53);
}

/* ---- Public API (mmap backend) -------------------------- */

/**
 * @brief Initialize the GPIO subsystem (mmap backend).
 *
 * Opens `/dev/gpiomem0` and maps 192 KB of RP1 register space into the
 * process address space. All pin states are reset to UNDEF.
 *
 * Must be called exactly once before any other GPIO function.
 *
 * @return 0 on success, -1 if the device cannot be opened or mmap fails.
 * @note Requires read/write access to `/dev/gpiomem0` (default on Raspberry Pi OS).
 */
int gpio_init(void)
{
  int i;

  gpio_mmap_fd = open(GPIOMEM_DEVICE, O_RDWR | O_SYNC);
  if (gpio_mmap_fd < 0)
  {
    perror("gpio_init: Failed to open " GPIOMEM_DEVICE);
    return -1;
  }

  gpio_mmap_base = (volatile uint32_t *)mmap(
      NULL,
      GPIO_MEM_SIZE,
      PROT_READ | PROT_WRITE,
      MAP_SHARED,
      gpio_mmap_fd,
      0);

  if (gpio_mmap_base == MAP_FAILED)
  {
    perror("gpio_init: mmap failed");
    close(gpio_mmap_fd);
    gpio_mmap_fd = -1;
    gpio_mmap_base = NULL;
    return -1;
  }

  for (i = 0; i <= GPIO_MAX_INDEX; i++)
  {
    rpi5_gpio[i].mode  = UNDEF;
    rpi5_gpio[i].state = UNDEF;
  }

  gpio_initialized = 1;
  return 0;
}

/**
 * @brief Release all GPIO resources (mmap backend).
 *
 * Unmaps the RP1 register region and closes the `/dev/gpiomem0` file
 * descriptor. Safe to call even if gpio_init() was not called or already
 * cleaned up.
 */
void gpio_cleanup(void)
{
  if (gpio_mmap_base && gpio_mmap_base != MAP_FAILED)
  {
    munmap((void *)gpio_mmap_base, GPIO_MEM_SIZE);
    gpio_mmap_base = NULL;
  }
  if (gpio_mmap_fd >= 0)
  {
    close(gpio_mmap_fd);
    gpio_mmap_fd = -1;
  }
  gpio_initialized = 0;
}

/**
 * @brief Open a GPIO pin in the specified direction (mmap backend).
 *
 * Configures the pin for standard GPIO (SIO) operation by:
 * 1. Setting FUNCSEL to SIO (ALT5) in the IO_BANK CTRL register.
 * 2. Clearing output/OE overrides so SIO controls the pin.
 * 3. Enabling input (IE) and clearing output-disable (OD) in the pad register.
 * 4. Setting or clearing the Output Enable bit via atomic RIO registers.
 *
 * @param pin  GPIO number (0–53).
 * @param mode Direction: INPUT (0) or OUTPUT (1).
 * @return pin_t with the configured mode. On error, both fields are UNDEF.
 */
pin_t pinopen(int pin, int mode)
{
  uint32_t io_base, rio_base, pads_base;
  int bit;
  uint32_t ctrl_val, pad_val;
  pin_t err_pin = { UNDEF, UNDEF };

  if (!gpio_valid_pin(pin))
  {
    fprintf(stderr, "pinopen: Invalid pin %d (only 0-53 supported)\n", pin);
    return err_pin;
  }
  if (!gpio_initialized)
  {
    fprintf(stderr, "pinopen: gpio_init() not called\n");
    return err_pin;
  }
  if (pin_to_bank(pin, &io_base, &rio_base, &pads_base, &bit) < 0)
    return err_pin;

  /* Set FUNCSEL to SIO (5) for normal GPIO operation */
  ctrl_val = reg_read(io_base + GPIO_CTRL(bit));
  ctrl_val &= ~CTRL_FUNCSEL_MASK;
  ctrl_val |= GPIO_FUNC_SIO;
  /* Clear overrides: let SIO control output/OE */
  ctrl_val &= ~(CTRL_OUTOVER_MASK | CTRL_OEOVER_MASK);
  reg_write(io_base + GPIO_CTRL(bit), ctrl_val);

  /* Configure pad: enable input, clear output-disable */
  pad_val = reg_read(pads_base + PADS_GPIO(bit));
  pad_val |= PAD_IE_BIT;     /* Input enable (needed for readback) */
  pad_val &= ~PAD_OD_BIT;    /* Clear output disable */
  reg_write(pads_base + PADS_GPIO(bit), pad_val);

  if (mode == OUTPUT)
  {
    /* Set Output Enable bit via atomic SET register */
    reg_write(rio_base + RP1_SET_OFFSET + RIO_OE, (1u << bit));
  }
  else
  {
    /* Clear Output Enable bit via atomic CLR register */
    reg_write(rio_base + RP1_CLR_OFFSET + RIO_OE, (1u << bit));
  }

  rpi5_gpio[pin].mode = mode;
  return (pin_t) { UNDEF, mode };
}

/**
 * @brief Close a GPIO pin and reset it to a safe state (mmap backend).
 *
 * Clears the Output Enable bit (sets pin to input) and sets FUNCSEL to
 * GPIO_FUNC_NULL (31) to disconnect the pin from all peripherals.
 * The cached state is reset to UNDEF.
 *
 * @param indx_pin GPIO number (0–53) to close.
 */
void pinclose(int indx_pin)
{
  uint32_t io_base, rio_base, pads_base;
  int bit;
  uint32_t ctrl_val;

  if (!gpio_valid_pin(indx_pin) || !gpio_initialized) return;
  if (pin_to_bank(indx_pin, &io_base, &rio_base, &pads_base, &bit) < 0) return;

  /* Reset to input (safe default) */
  reg_write(rio_base + RP1_CLR_OFFSET + RIO_OE, (1u << bit));

  /* Set FUNCSEL to NULL (disable) */
  ctrl_val = reg_read(io_base + GPIO_CTRL(bit));
  ctrl_val &= ~CTRL_FUNCSEL_MASK;
  ctrl_val |= GPIO_FUNC_NULL;
  reg_write(io_base + GPIO_CTRL(bit), ctrl_val);

  rpi5_gpio[indx_pin].mode  = UNDEF;
  rpi5_gpio[indx_pin].state = UNDEF;
}

/**
 * @brief Write a digital value to a GPIO output pin (mmap backend).
 *
 * Uses the atomic SET or CLR register on SYS_RIO to change the output
 * level in a single write, avoiding read-modify-write glitches.
 *
 * @param indx_pin GPIO number (0–53, must have been opened as OUTPUT).
 * @param value    Desired logic level: HIGH (1) or LOW (0).
 * @return 0 on success, -1 on error (invalid pin or not initialized).
 */
int pinwrite(int indx_pin, int value)
{
  uint32_t io_base, rio_base, pads_base;
  int bit;

  if (!gpio_valid_pin(indx_pin))
  {
    fprintf(stderr, "pinwrite: Invalid pin %d\n", indx_pin);
    return -1;
  }
  if (!gpio_initialized)
  {
    fprintf(stderr, "pinwrite: gpio_init() not called\n");
    return -1;
  }
  if (pin_to_bank(indx_pin, &io_base, &rio_base, &pads_base, &bit) < 0)
    return -1;

  if (value == LOW)
    reg_write(rio_base + RP1_CLR_OFFSET + RIO_OUT, (1u << bit));
  else
    reg_write(rio_base + RP1_SET_OFFSET + RIO_OUT, (1u << bit));

  rpi5_gpio[indx_pin].state = value;
  return 0;
}

/**
 * @brief Read the current logic level of a GPIO pin (mmap backend).
 *
 * Reads the RIO_IN input register and extracts the bit for the given pin.
 * Works on both INPUT and OUTPUT pins (output readback is supported).
 *
 * @param pin GPIO number (0–53).
 * @return HIGH (1), LOW (0), or -1 on error.
 */
int pinread(int pin)
{
  uint32_t io_base, rio_base, pads_base;
  int bit;
  uint32_t val;
  int result;

  if (!gpio_valid_pin(pin))
  {
    fprintf(stderr, "pinread: Invalid pin %d\n", pin);
    return -1;
  }
  if (!gpio_initialized)
  {
    fprintf(stderr, "pinread: gpio_init() not called\n");
    return -1;
  }
  if (pin_to_bank(pin, &io_base, &rio_base, &pads_base, &bit) < 0)
    return -1;

  val = reg_read(rio_base + RIO_IN);
  result = (val >> bit) & 1;

  rpi5_gpio[pin].state = result;
  return result;
}

/**
 * @brief Toggle the output state of a GPIO pin (mmap backend).
 *
 * Uses the atomic XOR register on SYS_RIO to flip the output bit in a
 * single write. This is faster and race-free compared to read-modify-write.
 *
 * @param pin GPIO number (0–53, must have been opened as OUTPUT).
 * @return 0 on success, -1 on error.
 */
int pintoggle(int pin)
{
  uint32_t io_base, rio_base, pads_base;
  int bit;

  if (!gpio_valid_pin(pin))
  {
    fprintf(stderr, "pintoggle: Invalid pin %d\n", pin);
    return -1;
  }
  if (!gpio_initialized)
  {
    fprintf(stderr, "pintoggle: gpio_init() not called\n");
    return -1;
  }
  if (pin_to_bank(pin, &io_base, &rio_base, &pads_base, &bit) < 0)
    return -1;

  /* XOR the output bit — atomic single-register toggle */
  reg_write(rio_base + RP1_XOR_OFFSET + RIO_OUT, (1u << bit));

  /* Update cached state */
  rpi5_gpio[pin].state = rpi5_gpio[pin].state ? LOW : HIGH;
  return 0;
}

/**
 * @brief Configure the internal pull resistor for a GPIO pin (mmap backend).
 *
 * Modifies the PUE (pull-up enable) and PDE (pull-down enable) bits in the
 * pad register. Both bits are cleared first, then the requested pull is set.
 *
 * @param pin  GPIO number (0–53).
 * @param pull Pull mode: PULL_UP (1), PULL_DOWN (2), or PULL_NONE (0).
 * @return 0 on success, -1 on error (invalid pin, not initialized, or bad pull value).
 */
int pinpull(int pin, int pull)
{
  uint32_t io_base, rio_base, pads_base;
  int bit;
  uint32_t pad_val;

  if (!gpio_valid_pin(pin))
  {
    fprintf(stderr, "pinpull: Invalid pin %d\n", pin);
    return -1;
  }
  if (!gpio_initialized)
  {
    fprintf(stderr, "pinpull: gpio_init() not called\n");
    return -1;
  }
  if (pin_to_bank(pin, &io_base, &rio_base, &pads_base, &bit) < 0)
    return -1;

  pad_val = reg_read(pads_base + PADS_GPIO(bit));

  /* Clear both pull bits first */
  pad_val &= ~(PAD_PUE_BIT | PAD_PDE_BIT);

  switch (pull)
  {
    case PULL_UP:
      pad_val |= PAD_PUE_BIT;
      break;
    case PULL_DOWN:
      pad_val |= PAD_PDE_BIT;
      break;
    case PULL_NONE:
      /* Both bits already cleared */
      break;
    default:
      fprintf(stderr, "pinpull: Invalid pull mode %d\n", pull);
      return -1;
  }

  reg_write(pads_base + PADS_GPIO(bit), pad_val);
  return 0;
}

/**
 * @brief Set the alternate function (FUNCSEL) for a GPIO pin (mmap backend).
 *
 * Switches the pin from standard GPIO mode to a hardware peripheral function
 * by writing the function number into the CTRL register's FUNCSEL field.
 * Also enables input and clears output-disable on the pad.
 *
 * @param pin  GPIO number (0–53).
 * @param func Alternate function: ALT0–ALT8 (0–8) or GPIO_FUNC_NULL (31).
 * @return 0 on success, -1 on error.
 *
 * @par Example
 * @code
 * pinalt(GPIO2, ALT3);   // Set GPIO2 to I2C1 SDA
 * pinalt(GPIO3, ALT3);   // Set GPIO3 to I2C1 SCL
 * @endcode
 */
int pinalt(int pin, int func)
{
  uint32_t io_base, rio_base, pads_base;
  int bit;
  uint32_t ctrl_val, pad_val;

  if (!gpio_valid_pin(pin))
  {
    fprintf(stderr, "pinalt: Invalid pin %d\n", pin);
    return -1;
  }
  if (!gpio_initialized)
  {
    fprintf(stderr, "pinalt: gpio_init() not called\n");
    return -1;
  }
  if (func < 0 || (func > 8 && func != GPIO_FUNC_NULL))
  {
    fprintf(stderr, "pinalt: Invalid function %d for pin %d\n", func, pin);
    return -1;
  }
  if (pin_to_bank(pin, &io_base, &rio_base, &pads_base, &bit) < 0)
    return -1;

  /* Set FUNCSEL in CTRL register */
  ctrl_val = reg_read(io_base + GPIO_CTRL(bit));
  ctrl_val &= ~CTRL_FUNCSEL_MASK;
  ctrl_val |= (func & CTRL_FUNCSEL_MASK);
  reg_write(io_base + GPIO_CTRL(bit), ctrl_val);

  /* Ensure pad input is enabled for the peripheral */
  pad_val = reg_read(pads_base + PADS_GPIO(bit));
  pad_val |= PAD_IE_BIT;
  pad_val &= ~PAD_OD_BIT;
  reg_write(pads_base + PADS_GPIO(bit), pad_val);

  return 0;
}

/**
 * @brief Set the output drive strength for a GPIO pin (mmap backend).
 *
 * Modifies the DRIVE field (bits 5:4) in the pad register to control
 * the maximum source/sink current. Higher drive is useful for fast signals
 * (SPI clock) or pins driving long traces.
 *
 * @param pin      GPIO number (0–53).
 * @param strength One of: DRIVE_2MA (0), DRIVE_4MA (1), DRIVE_8MA (2), DRIVE_12MA (3).
 * @return 0 on success, -1 on error.
 */
int pin_set_drive(int pin, int strength)
{
  uint32_t io_base, rio_base, pads_base;
  int bit;
  uint32_t pad_val;

  if (!gpio_valid_pin(pin))
  {
    fprintf(stderr, "pin_set_drive: Invalid pin %d\n", pin);
    return -1;
  }
  if (!gpio_initialized)
  {
    fprintf(stderr, "pin_set_drive: gpio_init() not called\n");
    return -1;
  }
  if (strength < DRIVE_2MA || strength > DRIVE_12MA)
  {
    fprintf(stderr, "pin_set_drive: Invalid strength %d\n", strength);
    return -1;
  }
  if (pin_to_bank(pin, &io_base, &rio_base, &pads_base, &bit) < 0)
    return -1;

  pad_val = reg_read(pads_base + PADS_GPIO(bit));
  pad_val &= ~PAD_DRIVE_MASK;
  pad_val |= ((strength << PAD_DRIVE_LSB) & PAD_DRIVE_MASK);
  reg_write(pads_base + PADS_GPIO(bit), pad_val);

  return 0;
}

/**
 * @brief Quick pin test: pulse a pin HIGH for 1 second, then LOW (mmap backend).
 *
 * Opens the pin as OUTPUT, drives it HIGH, waits 1 second, drives LOW,
 * then closes the pin. Useful for visual verification with an LED or
 * logic analyzer.
 *
 * @param pin_indx GPIO number (0–53) to test.
 * @return 0 on success, -1 if the pin is invalid or cannot be opened.
 */
int gpio_pintest(int pin_indx)
{
  if (!gpio_valid_pin(pin_indx)) return -1;

  pin_t testpin = pinopen(pin_indx, OUTPUT);
  if (testpin.mode == UNDEF && testpin.state == UNDEF)
    return -1;

  pinwrite(pin_indx, HIGH);
  sleep(1);
  pinwrite(pin_indx, LOW);
  pinclose(pin_indx);
  return 0;
}
