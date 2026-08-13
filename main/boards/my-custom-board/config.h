#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

// Audio Settings
#define AUDIO_INPUT_SAMPLE_RATE  24000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

#define AUDIO_INPUT_REFERENCE    true

// I2S Pins
#define AUDIO_I2S_GPIO_MCLK GPIO_NUM_42
#define AUDIO_I2S_GPIO_WS   GPIO_NUM_39
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_40
#define AUDIO_I2S_GPIO_DIN  GPIO_NUM_21
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_41

// I2C Pins (Shared for Audio Codec, TCA9554, and Touch)
#define AUDIO_CODEC_I2C_SDA_PIN  GPIO_NUM_1
#define AUDIO_CODEC_I2C_SCL_PIN  GPIO_NUM_2

// PA_CTRL
#define AUDIO_CODEC_PA_PIN       GPIO_NUM_45

// Codec Addresses
#define AUDIO_CODEC_ES8311_ADDR  ES8311_CODEC_DEFAULT_ADDR
#define AUDIO_CODEC_ES7210_ADDR  ES7210_CODEC_DEFAULT_ADDR

// Buttons
#define BOOT_BUTTON_GPIO        GPIO_NUM_0

// Battery ADC Pin (GPIO8 -> ADC1 Channel 7)
#define BATTERY_ADC_GPIO        GPIO_NUM_8

// IO Expander (TCA9554)
#define I2C_ADDRESS             ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000

// QSPI LCD Pins (CO5300)
#define QSPI_LCD_HOST           SPI2_HOST
#define QSPI_PIN_NUM_LCD_PCLK   GPIO_NUM_12
#define QSPI_PIN_NUM_LCD_DATA0  GPIO_NUM_11
#define QSPI_PIN_NUM_LCD_DATA1  GPIO_NUM_13
#define QSPI_PIN_NUM_LCD_DATA2  GPIO_NUM_14
#define QSPI_PIN_NUM_LCD_DATA3  GPIO_NUM_9
#define QSPI_PIN_NUM_LCD_CS     GPIO_NUM_10
#define QSPI_PIN_NUM_LCD_RST    GPIO_NUM_NC // Reset handled via TCA9554 EXP2

// Display Settings
#define DISPLAY_WIDTH           410
#define DISPLAY_HEIGHT          502
#define DISPLAY_MIRROR_X        false
#define DISPLAY_MIRROR_Y        false
#define DISPLAY_SWAP_XY         false

#define DISPLAY_OFFSET_X        22
#define DISPLAY_OFFSET_Y        0

#define DISPLAY_BACKLIGHT_PIN           GPIO_NUM_NC
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false

// Touch Settings (CST9217)
#define PIN_NUM_TOUCH_RST       GPIO_NUM_NC // Reset handled via TCA9554 EXP3
#define PIN_NUM_TOUCH_INT       GPIO_NUM_NC

// Camera Pins
// Data pins (D0-D7)
#define CAMERA_PIN_D0    GPIO_NUM_7
#define CAMERA_PIN_D1    GPIO_NUM_5
#define CAMERA_PIN_D2    GPIO_NUM_4
#define CAMERA_PIN_D3    GPIO_NUM_6
#define CAMERA_PIN_D4    GPIO_NUM_15
#define CAMERA_PIN_D5    GPIO_NUM_16
#define CAMERA_PIN_D6    GPIO_NUM_17
#define CAMERA_PIN_D7    GPIO_NUM_18
// Timing/control pins
#define CAMERA_PIN_VSYNC GPIO_NUM_47
#define CAMERA_PIN_HREF  GPIO_NUM_48
#define CAMERA_PIN_PCLK  GPIO_NUM_46
#define CAMERA_PIN_XCLK  GPIO_NUM_3   // MCLK
// SCCB (I2C) - reuse existing I2C bus (port 0)
#define CAMERA_PIN_SIOD  GPIO_NUM_1   // DVP_SDA (shared with I2C bus)
#define CAMERA_PIN_SIOC  GPIO_NUM_2   // DVP_SCL (shared with I2C bus)
// Power/reset (not connected)
#define CAMERA_PIN_PWDN  GPIO_NUM_NC
#define CAMERA_PIN_RESET GPIO_NUM_NC
// XCLK frequency
#define XCLK_FREQ_HZ     20000000

#endif // _BOARD_CONFIG_H_

