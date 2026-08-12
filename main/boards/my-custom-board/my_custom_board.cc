#include "wifi_board.h"
#include "display/lcd_display.h"
#include "esp_lcd_co5300.h"

#include "codecs/box_audio_codec.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "i2c_device.h"

#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/i2c_master.h>
#include <driver/spi_master.h>
#include "esp_io_expander_tca9554.h"

#include <esp_lcd_touch_cst9217.h>
#include <esp_lvgl_port.h>
#include <lvgl.h>

#define TAG "MyCustomBoard"

#define LCD_OPCODE_WRITE_CMD (0x02ULL)
#define LCD_OPCODE_READ_CMD (0x03ULL)
#define LCD_OPCODE_WRITE_COLOR (0x32ULL)

// PY206W38B_V2_206_410x502 屏幕初始化序列（完全对齐 lcd_init_qspi_co5300.c）
static const co5300_lcd_init_cmd_t vendor_specific_init[] = {
    {0xFE, (uint8_t []){0x00}, 1, 0},       // Unlock
    {0x11, (uint8_t []){0x00}, 0, 120},     // Sleep Out
    {0x35, (uint8_t []){0x00}, 1, 0},       // Tearing Effect Line ON
    {0xFE, (uint8_t []){0x00}, 1, 0},       // Unlock again
    {0xC4, (uint8_t []){0x80}, 1, 0},       // SPI Mode Control
    {0x53, (uint8_t []){0x20}, 1, 0},       // Write CTRL Display
    {0x63, (uint8_t []){0xFF}, 1, 0},       // HBM Brightness
    {0x2A, (uint8_t []){0x00, 0x00, 0x01, 0x99}, 4, 0}, // Column Address: 410
    {0x2B, (uint8_t []){0x00, 0x00, 0x01, 0xF5}, 4, 0}, // Row Address: 502
    {0x29, (uint8_t []){0x00}, 0, 60},      // Display ON
    {0x51, (uint8_t []){0xFF}, 1, 0},       // Normal Brightness
    {0x58, (uint8_t []){0x07}, 1, 10},      // Brightness Control
};

class CustomLcdDisplay : public SpiLcdDisplay {
public:
    static void rounder_event_cb(lv_event_t* e) {
        lv_area_t* area = (lv_area_t*)lv_event_get_param(e);
        uint16_t x1 = area->x1;
        uint16_t x2 = area->x2;

        uint16_t y1 = area->y1;
        uint16_t y2 = area->y2;

        // round the start of coordinate down to the nearest 2M number
        area->x1 = (x1 >> 1) << 1;
        area->y1 = (y1 >> 1) << 1;
        // round the end of coordinate up to the nearest 2N+1 number
        area->x2 = ((x2 >> 1) << 1) + 1;
        area->y2 = ((y2 >> 1) << 1) + 1;
    }

    CustomLcdDisplay(esp_lcd_panel_io_handle_t io_handle,
                     esp_lcd_panel_handle_t panel_handle,
                     int width, int height,
                     int offset_x, int offset_y,
                     bool mirror_x, bool mirror_y, bool swap_xy)
        : SpiLcdDisplay(io_handle, panel_handle,
                        width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy) {
        if (display_) {
            lv_display_add_event_cb(display_, rounder_event_cb, LV_EVENT_INVALIDATE_AREA, NULL);
        }
    }

    virtual void SetupUI() override {
        SpiLcdDisplay::SetupUI();

        DisplayLockGuard lock(this);
        lv_obj_set_style_pad_left(status_bar_, LV_HOR_RES * 0.1, 0);
        lv_obj_set_style_pad_right(status_bar_, LV_HOR_RES * 0.1, 0);
    }
};

class CustomBacklight : public Backlight {
public:
    CustomBacklight(esp_lcd_panel_handle_t panel) : Backlight(), panel_(panel) {
        if (panel_) {
            esp_lcd_panel_co5300_set_brightness(panel_, 100);
        }
    }

protected:
    esp_lcd_panel_handle_t panel_;

    virtual void SetBrightnessImpl(uint8_t brightness) override {
        if (panel_) {
            if (transition_timer_ != nullptr) {
                esp_timer_stop(transition_timer_);
            }
            brightness_ = target_brightness_;
            esp_lcd_panel_co5300_set_brightness(panel_, target_brightness_);
        }
    }
};

static esp_err_t (*cst9217_read_data_orig)(esp_lcd_touch_handle_t tp) = NULL;

static esp_err_t cst9217_read_data_polling_wrapper(esp_lcd_touch_handle_t tp) {
    esp_err_t err = cst9217_read_data_orig(tp);
    if (err == ESP_ERR_INVALID_RESPONSE) {
        portENTER_CRITICAL(&tp->data.lock);
        tp->data.points = 0;
        portEXIT_CRITICAL(&tp->data.lock);
        return ESP_OK;
    }
    return err;
}

class MyCustomBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Button boot_button_;
    CustomLcdDisplay* display_;
    CustomBacklight* backlight_;
    esp_io_expander_handle_t io_expander = NULL;

    void InitializeI2c() {
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
        ESP_LOGI(TAG, "I2C bus initialized");
    }

    void InitializeTca9554() {
        esp_err_t ret = esp_io_expander_new_i2c_tca9554(i2c_bus_, I2C_ADDRESS, &io_expander);
        if (ret != ESP_OK) {
            ret = esp_io_expander_new_i2c_tca9554(i2c_bus_, ESP_IO_EXPANDER_I2C_TCA9554A_ADDRESS_000, &io_expander);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "TCA9554 create returned error: %d", ret);
                return;
            }
        }

        // EXP0: POWER_CTRL (Active Low via PMOS)
        // EXP2: LCD_RST (CO5300 Reset, Active Low)
        // EXP3: TP_RST (Touch Panel Reset, Active High)
        // EXP4: CAM_RST (Camera Reset, Active Low)
        ESP_ERROR_CHECK(esp_io_expander_set_dir(io_expander,
            IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_2 | IO_EXPANDER_PIN_NUM_3 | IO_EXPANDER_PIN_NUM_4,
            IO_EXPANDER_OUTPUT));

        // Enable Power (EXP0, Active Low via PMOS)
        ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_0, 0));

        // Reset Camera (EXP4=CAM_RST, Active Low)
        ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_4, 1));
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_4, 0));
        vTaskDelay(pdMS_TO_TICKS(50));
        ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_4, 1));
        vTaskDelay(pdMS_TO_TICKS(120));

        // Reset LCD and Touch (EXP2=LCD_RST, EXP3=TP_RST)
        ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_2 | IO_EXPANDER_PIN_NUM_3, 1));
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_2 | IO_EXPANDER_PIN_NUM_3, 0));
        vTaskDelay(pdMS_TO_TICKS(50));
        ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_2 | IO_EXPANDER_PIN_NUM_3, 1));
        vTaskDelay(pdMS_TO_TICKS(120));

        ESP_LOGI(TAG, "TCA9554 Initialized, Power/LCD reset done");
    }

    void InitializeSpi() {
        ESP_LOGI(TAG, "Initialize QSPI bus");
        spi_bus_config_t buscfg = {};
        buscfg.sclk_io_num = QSPI_PIN_NUM_LCD_PCLK;
        buscfg.data0_io_num = QSPI_PIN_NUM_LCD_DATA0;
        buscfg.data1_io_num = QSPI_PIN_NUM_LCD_DATA1;
        buscfg.data2_io_num = QSPI_PIN_NUM_LCD_DATA2;
        buscfg.data3_io_num = QSPI_PIN_NUM_LCD_DATA3;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(QSPI_LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        ESP_LOGI(TAG, "Install CO5300 panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = QSPI_PIN_NUM_LCD_CS;
        io_config.dc_gpio_num = GPIO_NUM_NC;
        io_config.spi_mode = 0;
        io_config.pclk_hz = 60 * 1000 * 1000;
        io_config.trans_queue_depth = 1;
        io_config.lcd_cmd_bits = 32;
        io_config.lcd_param_bits = 8;
        io_config.flags.quad_mode = true;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(QSPI_LCD_HOST, &io_config, &panel_io));

        ESP_LOGI(TAG, "Install CO5300 panel driver");
        co5300_vendor_config_t vendor_config = {};
        vendor_config.init_cmds = &vendor_specific_init[0];
        vendor_config.init_cmds_size = sizeof(vendor_specific_init) / sizeof(co5300_lcd_init_cmd_t);
        vendor_config.flags.use_qspi_interface = 1;

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        panel_config.vendor_config = (void*)&vendor_config;
        ESP_ERROR_CHECK(esp_lcd_new_panel_co5300(panel_io, &panel_config, &panel));

        ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
        ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y));
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel, false));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, false, false));
        esp_lcd_panel_disp_on_off(panel, true);

        // ---------------- 纯硬件直刷红屏 & 100% 亮度测试 ----------------
        ESP_LOGI(TAG, "Testing pure hardware draw (RED SCREEN & 100%% Brightness)...");
        esp_lcd_panel_co5300_set_brightness(panel, 100);
        std::vector<uint16_t> red_buf(DISPLAY_WIDTH, 0xF800); // RGB565 红色
        for (int y = 0; y < DISPLAY_HEIGHT; y++) {
            esp_lcd_panel_draw_bitmap(panel, 0, y, DISPLAY_WIDTH, y + 1, red_buf.data());
        }
        // ----------------------------------------------------------------

        display_ = new CustomLcdDisplay(panel_io, panel,
                                        DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                        0, 0, // Hardware gap already set, logical offset should be 0
                                        DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, false);
        backlight_ = new CustomBacklight(panel);
    }

    void InitializeTouch() {
        esp_lcd_touch_handle_t tp;
        esp_lcd_touch_config_t tp_cfg = {
            .x_max = DISPLAY_WIDTH - 1,
            .y_max = DISPLAY_HEIGHT - 1,
            .rst_gpio_num = PIN_NUM_TOUCH_RST,
            .int_gpio_num = PIN_NUM_TOUCH_INT,
            .levels = {
                .reset = 0,
                .interrupt = 0,
            },
            .flags = {
                .swap_xy = 0,
                .mirror_x = 0,
                .mirror_y = 0,
            },
        };
        esp_lcd_panel_io_handle_t tp_io_handle = NULL;
        esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST9217_CONFIG();
        tp_io_config.scl_speed_hz = 400 * 1000;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus_, &tp_io_config, &tp_io_handle));
        ESP_LOGI(TAG, "Initialize CST9217 touch controller");
        ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_cst9217(tp_io_handle, &tp_cfg, &tp));

        // In polling mode (int_gpio_num = NC), CST9217 returns ESP_ERR_INVALID_RESPONSE
        // when no touch event is active. Wrap read_data to return ESP_OK with 0 points.
        cst9217_read_data_orig = tp->read_data;
        tp->read_data = cst9217_read_data_polling_wrapper;

        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp = lv_display_get_default(),
            .handle = tp,
        };
        lvgl_port_add_touch(&touch_cfg);
        ESP_LOGI(TAG, "Touch panel initialized successfully (polling mode)");
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

    void InitializeTools() {
        auto &mcp_server = McpServer::GetInstance();
        mcp_server.AddTool("self.system.reconfigure_wifi",
            "End this conversation and enter WiFi configuration mode.\n"
            "**CAUTION** You must ask the user to confirm this action.",
            PropertyList(), [this](const PropertyList& properties) {
                EnterWifiConfigMode();
                return true;
            });
    }

public:
    MyCustomBoard() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeI2c();
        InitializeTca9554();
        InitializeSpi();
        InitializeDisplay();
        InitializeTouch();
        InitializeButtons();
        InitializeTools();
        GetBacklight()->RestoreBrightness();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static BoxAudioCodec audio_codec(
            i2c_bus_, 
            AUDIO_INPUT_SAMPLE_RATE, 
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, 
            AUDIO_I2S_GPIO_BCLK, 
            AUDIO_I2S_GPIO_WS, 
            AUDIO_I2S_GPIO_DOUT, 
            AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN, 
            AUDIO_CODEC_ES8311_ADDR, 
            AUDIO_CODEC_ES7210_ADDR, 
            AUDIO_INPUT_REFERENCE);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        return backlight_;
    }
};

DECLARE_BOARD(MyCustomBoard);
