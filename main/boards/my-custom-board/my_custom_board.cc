#include "wifi_board.h"
#include "display/emote_display.h"
#include "display/lcd_display.h"
#include "esp_lcd_co5300.h"

#include "codecs/box_audio_codec.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "i2c_device.h"
#include "esp32_camera.h"

#include <vector>
#include <string>
#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_interface.h>
#include <esp_heap_caps.h>
#include <driver/i2c_master.h>
#include <driver/spi_master.h>
#include "esp_io_expander_tca9554.h"

#include <esp_lcd_touch_cst9217.h>
#include <esp_lvgl_port.h>
#include <lvgl.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>

#define TAG "MyCustomBoard"

#define LCD_OPCODE_WRITE_CMD (0x02ULL)
#define LCD_OPCODE_READ_CMD (0x03ULL)
#define LCD_OPCODE_WRITE_COLOR (0x32ULL)

// 软件旋转分块大小 (Tile size for cache efficiency during software rotation)
#define SW_ROT_TILE (64)

// 270度软件旋转 Hook 钩子函数相关变量
static esp_err_t (*s_orig_draw_bitmap)(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data) = NULL;
static uint16_t *s_rot_buf = NULL;
static size_t s_rot_buf_size = 0;

/* ── 270度软件旋转 Hook 钩子函数 (CO5300 缺乏 MADCTL MV 位) ───── */
static esp_err_t co5300_sw_rotate_270_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    if (!s_orig_draw_bitmap) {
        return ESP_ERR_INVALID_STATE;
    }

    int w = x_end - x_start;
    int h = y_end - y_start;
    size_t needed_bytes = (size_t)w * h * sizeof(uint16_t);

    if (needed_bytes > s_rot_buf_size) {
        ESP_LOGE(TAG, "Rotation buffer too small: need %zu, have %zu", needed_bytes, s_rot_buf_size);
        return ESP_ERR_NO_MEM;
    }

    const uint16_t * __restrict__ src = (const uint16_t *)color_data;
    uint16_t * __restrict__ dst = s_rot_buf;

    // 270 度旋转物理坐标映射
    int x_start_phy = y_start;
    int x_end_phy   = y_end;
    int y_start_phy = DISPLAY_HEIGHT - x_end;
    int y_end_phy   = DISPLAY_HEIGHT - x_start;

    // 分块转置, 提升 Cache 命中率 (完全对齐 lcd_init_qspi_co5300.c)
    for (int by = 0; by < h; by += SW_ROT_TILE) {
        int y_max = (by + SW_ROT_TILE < h) ? (by + SW_ROT_TILE) : h;
        for (int bx = 0; bx < w; bx += SW_ROT_TILE) {
            int x_max = (bx + SW_ROT_TILE < w) ? (bx + SW_ROT_TILE) : w;

            for (int y = by; y < y_max; y++) {
                const uint16_t *src_row = src + (size_t)y * w;
                uint16_t *dst_row_base = dst + y;
                for (int x = bx; x < x_max; x++) {
                    dst_row_base[(size_t)(w - 1 - x) * h] = src_row[x];
                }
            }
        }
    }

    return s_orig_draw_bitmap(panel, x_start_phy, y_start_phy, x_end_phy, y_end_phy, dst);
}

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
    Display* display_;
    CustomBacklight* backlight_;
    Esp32Camera* camera_ = nullptr;
    esp_io_expander_handle_t io_expander = NULL;
    esp_lcd_touch_handle_t touch_handle_ = nullptr;
    TaskHandle_t touch_task_handle_ = nullptr;
    size_t current_emotion_index_ = 0;

    static void TouchGestureTask(void* arg) {
        auto board = static_cast<MyCustomBoard*>(arg);
        uint16_t x[1], y[1];
        uint8_t count = 0;

        bool is_touching = false;
        uint16_t start_x = 0, start_y = 0;
        uint16_t last_x = 0, last_y = 0;
        int64_t start_time = 0;

        const std::vector<std::string> emotions = {
            "neutral", "happy", "laughing", "funny", "sad", "angry", "crying",
            "loving", "embarrassed", "surprised", "shocked", "thinking", "winking",
            "cool", "relaxed", "delicious", "kissy", "confident", "sleepy",
            "silly", "confused"
        };

        while (true) {
            if (board->touch_handle_) {
                esp_lcd_touch_read_data(board->touch_handle_);
                bool touched = esp_lcd_touch_get_coordinates(board->touch_handle_, x, y, NULL, &count, 1);
                int64_t now = esp_timer_get_time() / 1000; // ms

                if (touched && count > 0) {
                    if (!is_touching) {
                        is_touching = true;
                        start_x = x[0];
                        start_y = y[0];
                        start_time = now;
                    }
                    last_x = x[0];
                    last_y = y[0];
                } else if (is_touching) {
                    is_touching = false;
                    int dx = (int)last_x - (int)start_x;
                    int dy = (int)last_y - (int)start_y;
                    int dt = (int)(now - start_time);

                    std::string target_emotion = "";

                    if (abs(dx) < 30 && abs(dy) < 30 && dt < 500) {
                        // 单击 Tap：循环切换表情
                        board->current_emotion_index_ = (board->current_emotion_index_ + 1) % emotions.size();
                        target_emotion = emotions[board->current_emotion_index_];
                        ESP_LOGI(TAG, "Touch Gesture: Tap -> %s", target_emotion.c_str());
                    } else if (dx > 60 && abs(dy) < 50) {
                        // 向右滑动 Swipe Right
                        board->current_emotion_index_ = (board->current_emotion_index_ + 1) % emotions.size();
                        target_emotion = emotions[board->current_emotion_index_];
                        ESP_LOGI(TAG, "Touch Gesture: Swipe Right -> %s", target_emotion.c_str());
                    } else if (dx < -60 && abs(dy) < 50) {
                        // 向左滑动 Swipe Left
                        if (board->current_emotion_index_ == 0) {
                            board->current_emotion_index_ = emotions.size() - 1;
                        } else {
                            board->current_emotion_index_--;
                        }
                        target_emotion = emotions[board->current_emotion_index_];
                        ESP_LOGI(TAG, "Touch Gesture: Swipe Left -> %s", target_emotion.c_str());
                    }

                    if (!target_emotion.empty()) {
                        Application::GetInstance().Schedule([board, target_emotion]() {
                            if (board->GetDisplay()) {
                                board->GetDisplay()->SetEmotion(target_emotion.c_str());
                            }
                        });
                    }
                }
            }
            vTaskDelay(pdMS_TO_TICKS(30));
        }
    }

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

        // ---------------- 纯硬件直刷红屏 & 100% 亮度测试 ----------------
        ESP_LOGI(TAG, "Testing pure hardware draw (RED SCREEN & 100%% Brightness)...");
        esp_lcd_panel_co5300_set_brightness(panel, 100);
        std::vector<uint16_t> red_buf(DISPLAY_WIDTH, 0xF800); // RGB565 红色
        for (int y = 0; y < DISPLAY_HEIGHT; y++) {
            esp_lcd_panel_draw_bitmap(panel, 0, y, DISPLAY_WIDTH, y + 1, red_buf.data());
        }
        // ----------------------------------------------------------------

        // 预分配 270 度软件旋转缓冲区 (完全对齐 lcd_init_qspi_co5300.c)
        size_t max_bytes = (size_t)DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        if (!s_rot_buf) {
            s_rot_buf = (uint16_t *)heap_caps_malloc(max_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (!s_rot_buf) {
                s_rot_buf = (uint16_t *)malloc(max_bytes);
            }
            if (s_rot_buf) {
                s_rot_buf_size = max_bytes;
                ESP_LOGI(TAG, "Software rotation buffer pre-allocated: %zu bytes", max_bytes);
            } else {
                ESP_LOGE(TAG, "Failed to pre-allocate software rotation buffer (%zu bytes)", max_bytes);
            }
        }

        // 安装 270 度软件旋转 Hook 钩子函数 (CO5300 MADCTL 缺乏 MV 位)
        if (panel->draw_bitmap != co5300_sw_rotate_270_draw_bitmap) {
            s_orig_draw_bitmap = panel->draw_bitmap;
            panel->draw_bitmap = co5300_sw_rotate_270_draw_bitmap;
            ESP_LOGI(TAG, "Installed 270 degree software rotation hook for CO5300");
        }

        esp_lcd_panel_disp_on_off(panel, true);

        // 旋转 270 度后，逻辑分辨率变为 DISPLAY_HEIGHT x DISPLAY_WIDTH (502 x 410)
#if CONFIG_USE_EMOTE_MESSAGE_STYLE
        display_ = new emote::EmoteDisplay(panel, panel_io, DISPLAY_HEIGHT, DISPLAY_WIDTH);
#else
        display_ = new CustomLcdDisplay(panel_io, panel,
                                        DISPLAY_HEIGHT, DISPLAY_WIDTH,
                                        0, 0, // Hardware gap already set, logical offset should be 0
                                        DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, false);
#endif
        backlight_ = new CustomBacklight(panel);
    }

    void InitializeTouch() {
        esp_lcd_touch_handle_t tp;
        esp_lcd_touch_config_t tp_cfg = {
            .x_max = DISPLAY_HEIGHT - 1, // 270度旋转后的逻辑 Max X
            .y_max = DISPLAY_WIDTH - 1,  // 270度旋转后的逻辑 Max Y
            .rst_gpio_num = PIN_NUM_TOUCH_RST,
            .int_gpio_num = PIN_NUM_TOUCH_INT,
            .levels = {
                .reset = 0,
                .interrupt = 0,
            },
            .flags = {
                .swap_xy = 1,   // 270度旋转：交换 XY
                .mirror_x = 0,  // 270度旋转：mirror_x = 0
                .mirror_y = 1,  // 270度旋转：mirror_y = 1 (参考 touch_rotation_helper)
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
        touch_handle_ = tp;

        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp = lv_display_get_default(),
            .handle = tp,
        };
        if (touch_cfg.disp != nullptr) {
            lvgl_port_add_touch(&touch_cfg);
            ESP_LOGI(TAG, "Touch panel initialized successfully (polling mode)");
        } else {
            ESP_LOGI(TAG, "Touch panel initialized, skipping LVGL port binding (LVGL display is NULL under Emote style)");
            xTaskCreate(TouchGestureTask, "touch_gesture", 3072, this, 3, &touch_task_handle_);
        }
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

    void InitializeCamera() {
        camera_config_t config = {};
        config.ledc_channel = LEDC_CHANNEL_2;
        config.ledc_timer = LEDC_TIMER_2;
        config.pin_d0 = CAMERA_PIN_D0;
        config.pin_d1 = CAMERA_PIN_D1;
        config.pin_d2 = CAMERA_PIN_D2;
        config.pin_d3 = CAMERA_PIN_D3;
        config.pin_d4 = CAMERA_PIN_D4;
        config.pin_d5 = CAMERA_PIN_D5;
        config.pin_d6 = CAMERA_PIN_D6;
        config.pin_d7 = CAMERA_PIN_D7;
        config.pin_xclk = CAMERA_PIN_XCLK;
        config.pin_pclk = CAMERA_PIN_PCLK;
        config.pin_vsync = CAMERA_PIN_VSYNC;
        config.pin_href = CAMERA_PIN_HREF;
        config.pin_sccb_sda = -1;
        config.pin_sccb_scl = CAMERA_PIN_SIOC;
        config.sccb_i2c_port = 0;
        config.pin_pwdn = CAMERA_PIN_PWDN;
        config.pin_reset = CAMERA_PIN_RESET;
        config.xclk_freq_hz = XCLK_FREQ_HZ;
        config.pixel_format = PIXFORMAT_RGB565;
        config.frame_size = FRAMESIZE_QVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

        camera_ = new Esp32Camera(config);
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

    adc_oneshot_unit_handle_t adc_handle_ = nullptr;
    adc_cali_handle_t adc_cali_handle_ = nullptr;
    bool adc_cali_enabled_ = false;

    void InitializeAdc() {
        if (adc_handle_ != nullptr) {
            return;
        }

        adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = ADC_UNIT_1,
        };
        if (adc_oneshot_new_unit(&init_config, &adc_handle_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize ADC1 unit");
            return;
        }

        adc_oneshot_chan_cfg_t chan_config = {
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        if (adc_oneshot_config_channel(adc_handle_, ADC_CHANNEL_7, &chan_config) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to config ADC1 channel 7 (GPIO8)");
            return;
        }

        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = ADC_UNIT_1,
            .chan = ADC_CHANNEL_7,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        if (adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle_) == ESP_OK) {
            adc_cali_enabled_ = true;
            ESP_LOGI(TAG, "ADC calibration curve fitting enabled for battery measurement");
        } else {
            ESP_LOGW(TAG, "ADC calibration curve fitting unavailable, using raw conversion");
        }
    }

    uint16_t ReadBatteryVoltageMv() {
        InitializeAdc();
        if (!adc_handle_) {
            return 0;
        }

        int sum_raw = 0;
        const int samples = 8;
        for (int i = 0; i < samples; i++) {
            int raw_val = 0;
            adc_oneshot_read(adc_handle_, ADC_CHANNEL_7, &raw_val);
            sum_raw += raw_val;
        }
        int avg_raw = sum_raw / samples;

        int voltage_mv = 0;
        if (adc_cali_enabled_) {
            adc_cali_raw_to_voltage(adc_cali_handle_, avg_raw, &voltage_mv);
        } else {
            voltage_mv = (avg_raw * 3300) / 4095;
        }

        // 两个 100K 1:1 分压电阻：真实电池电压 = ADC 测量电压 * 2
        return (uint16_t)(voltage_mv * 2);
    }

public:
    MyCustomBoard() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeI2c();
        InitializeTca9554();
        InitializeSpi();
        InitializeDisplay();
        InitializeTouch();
        InitializeButtons();
        InitializeCamera();
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

    virtual Camera* GetCamera() override {
        return camera_;
    }

    virtual bool GetBatteryLevel(int &level, bool &charging, bool &discharging) override {
        uint16_t voltage_mv = ReadBatteryVoltageMv();
        if (voltage_mv == 0) {
            return false;
        }

        // 锂电池电压映射 3.3V (0%) ~ 4.2V (100%)
        if (voltage_mv >= 4200) {
            level = 100;
        } else if (voltage_mv <= 3300) {
            level = 0;
        } else {
            level = (voltage_mv - 3300) * 100 / (4200 - 3300);
        }

        charging = false;
        discharging = true;
        return true;
    }
};

DECLARE_BOARD(MyCustomBoard);
