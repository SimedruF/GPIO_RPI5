#ifndef GPIO_RPI5_H
#define GPIO_RPI5_H
#define GPIO_RPI5

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
/**
 * Pin modes: */
#define INPUT (0)
#define OUTPUT (1)
/** 
 * Pin states */
#define LOW (0)
#define HIGH (1)
#define UNDEF (3)

#define TEST_PIN 17

typedef struct {
        int state; /* LOW =0, HIGH=1, UNDEF=3 */
        int mode; /* 0 = input ; 1 = output */
} pin_t;

int gpio_pintest(int pin_indx);
pin_t pinopen(int pin, int mode);
void pinclose(int indx_pin);
void pinwrite(int indx_pin, int value);
int pinread(int pin);

#endif // GPIO_RPI5_H
