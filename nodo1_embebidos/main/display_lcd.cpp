#include "display_lcd.h"
#include "board_config.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_heap_caps.h"

#include "driver/spi_master.h"

#include "esp_lcd_gc9a01.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"

static const char *TAG = "LCD";

static esp_lcd_panel_handle_t s_panel = NULL;

#define LCD_HOST SPI2_HOST

/**
 * @brief Convierte un color RGB888 al formato RGB565 usado por el panel.
 */
static uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return ((r & 0xF8) << 8) |
           ((g & 0xFC) << 3) |
           ((b & 0xF8) >> 3);
}

/**
 * @brief Rellena toda la pantalla usando bloques DMA.
 */
static void lcd_fill_screen(uint16_t color)
{
    uint16_t *buffer = (uint16_t *)heap_caps_malloc(
        LCD_H_RES * 40 * sizeof(uint16_t),
        MALLOC_CAP_DMA
    );

    if (!buffer)
    {
        ESP_LOGE(TAG, "DMA alloc failed");
        return;
    }

    for (int i = 0; i < LCD_H_RES * 40; i++)
    {
        buffer[i] = color;
    }

    for (int y = 0; y < LCD_V_RES; y += 40)
    {
        esp_lcd_panel_draw_bitmap(
            s_panel,
            0,
            y,
            LCD_H_RES,
            y + 40,
            buffer
        );
    }

    free(buffer);
}

/**
 * @brief Dibuja un rectangulo solido dentro de los limites de la pantalla.
 */
static void lcd_fill_rect(int x1, int y1, int x2, int y2, uint16_t color)
{
    if (!s_panel) return;
    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (x2 > LCD_H_RES) x2 = LCD_H_RES;
    if (y2 > LCD_V_RES) y2 = LCD_V_RES;
    if (x2 <= x1 || y2 <= y1) return;

    int pixels = (x2 - x1) * (y2 - y1);
    uint16_t *buffer = (uint16_t *)heap_caps_malloc(pixels * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!buffer) {
        ESP_LOGE(TAG, "DMA alloc failed");
        return;
    }

    for (int i = 0; i < pixels; ++i) buffer[i] = color;
    esp_lcd_panel_draw_bitmap(s_panel, x1, y1, x2, y2, buffer);
    free(buffer);
}

/**
 * @brief Obtiene el mapa bitmap 5x7 de un caracter soportado.
 */
static bool get_glyph(char c, uint8_t out[7])
{
    memset(out, 0, 7);
    switch (c) {
    case 'A': { uint8_t g[7]={0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}; memcpy(out,g,7); return true; }
    case 'B': { uint8_t g[7]={0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}; memcpy(out,g,7); return true; }
    case 'C': { uint8_t g[7]={0x0E,0x11,0x10,0x10,0x10,0x11,0x0E}; memcpy(out,g,7); return true; }
    case 'D': { uint8_t g[7]={0x1E,0x11,0x11,0x11,0x11,0x11,0x1E}; memcpy(out,g,7); return true; }
    case 'G': { uint8_t g[7]={0x0E,0x11,0x10,0x17,0x11,0x11,0x0E}; memcpy(out,g,7); return true; }
    case 'I': { uint8_t g[7]={0x1F,0x04,0x04,0x04,0x04,0x04,0x1F}; memcpy(out,g,7); return true; }
    case 'M': { uint8_t g[7]={0x11,0x1B,0x15,0x15,0x11,0x11,0x11}; memcpy(out,g,7); return true; }
    case 'N': { uint8_t g[7]={0x11,0x19,0x15,0x13,0x11,0x11,0x11}; memcpy(out,g,7); return true; }
    case 'O': { uint8_t g[7]={0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}; memcpy(out,g,7); return true; }
    case 'P': { uint8_t g[7]={0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}; memcpy(out,g,7); return true; }
    case 'R': { uint8_t g[7]={0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}; memcpy(out,g,7); return true; }
    case 'S': { uint8_t g[7]={0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E}; memcpy(out,g,7); return true; }
    case 'T': { uint8_t g[7]={0x1F,0x04,0x04,0x04,0x04,0x04,0x04}; memcpy(out,g,7); return true; }
    case 'U': { uint8_t g[7]={0x11,0x11,0x11,0x11,0x11,0x11,0x0E}; memcpy(out,g,7); return true; }
    case 'X': { uint8_t g[7]={0x11,0x0A,0x04,0x04,0x04,0x0A,0x11}; memcpy(out,g,7); return true; }
    case 'Y': { uint8_t g[7]={0x11,0x0A,0x04,0x04,0x04,0x04,0x04}; memcpy(out,g,7); return true; }
    case 'Z': { uint8_t g[7]={0x1F,0x01,0x02,0x04,0x08,0x10,0x1F}; memcpy(out,g,7); return true; }
    case 'a': { uint8_t g[7]={0x00,0x00,0x0E,0x01,0x0F,0x11,0x0F}; memcpy(out,g,7); return true; }
    case 'd': { uint8_t g[7]={0x01,0x01,0x0F,0x11,0x11,0x11,0x0F}; memcpy(out,g,7); return true; }
    case 'g': { uint8_t g[7]={0x00,0x0F,0x11,0x11,0x0F,0x01,0x0E}; memcpy(out,g,7); return true; }
    case 'i': { uint8_t g[7]={0x04,0x00,0x0C,0x04,0x04,0x04,0x0E}; memcpy(out,g,7); return true; }
    case 'm': { uint8_t g[7]={0x00,0x00,0x1A,0x15,0x15,0x15,0x15}; memcpy(out,g,7); return true; }
    case 'n': { uint8_t g[7]={0x00,0x00,0x1E,0x11,0x11,0x11,0x11}; memcpy(out,g,7); return true; }
    case 'o': { uint8_t g[7]={0x00,0x00,0x0E,0x11,0x11,0x11,0x0E}; memcpy(out,g,7); return true; }
    case 'r': { uint8_t g[7]={0x00,0x00,0x16,0x18,0x10,0x10,0x10}; memcpy(out,g,7); return true; }
    case 't': { uint8_t g[7]={0x08,0x08,0x1E,0x08,0x08,0x09,0x06}; memcpy(out,g,7); return true; }
    case 'u': { uint8_t g[7]={0x00,0x00,0x11,0x11,0x11,0x13,0x0D}; memcpy(out,g,7); return true; }
    case '0': { uint8_t g[7]={0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}; memcpy(out,g,7); return true; }
    case '1': { uint8_t g[7]={0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}; memcpy(out,g,7); return true; }
    case '2': { uint8_t g[7]={0x0E,0x11,0x01,0x02,0x04,0x08,0x1F}; memcpy(out,g,7); return true; }
    case '3': { uint8_t g[7]={0x1E,0x01,0x01,0x0E,0x01,0x01,0x1E}; memcpy(out,g,7); return true; }
    case '4': { uint8_t g[7]={0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}; memcpy(out,g,7); return true; }
    case '5': { uint8_t g[7]={0x1F,0x10,0x10,0x1E,0x01,0x01,0x1E}; memcpy(out,g,7); return true; }
    case '6': { uint8_t g[7]={0x0E,0x10,0x10,0x1E,0x11,0x11,0x0E}; memcpy(out,g,7); return true; }
    case '7': { uint8_t g[7]={0x1F,0x01,0x02,0x04,0x08,0x08,0x08}; memcpy(out,g,7); return true; }
    case '8': { uint8_t g[7]={0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}; memcpy(out,g,7); return true; }
    case '9': { uint8_t g[7]={0x0E,0x11,0x11,0x0F,0x01,0x01,0x0E}; memcpy(out,g,7); return true; }
    case ':': { uint8_t g[7]={0x00,0x04,0x04,0x00,0x04,0x04,0x00}; memcpy(out,g,7); return true; }
    case '.': { uint8_t g[7]={0x00,0x00,0x00,0x00,0x00,0x06,0x06}; memcpy(out,g,7); return true; }
    case '-': { uint8_t g[7]={0x00,0x00,0x00,0x1F,0x00,0x00,0x00}; memcpy(out,g,7); return true; }
    case '%': { uint8_t g[7]={0x19,0x1A,0x02,0x04,0x08,0x0B,0x13}; memcpy(out,g,7); return true; }
    case ' ': { uint8_t g[7]={0x00,0x00,0x00,0x00,0x00,0x00,0x00}; memcpy(out,g,7); return true; }
    default: return false;
    }
}

/**
 * @brief Dibuja un caracter escalado usando la fuente bitmap interna.
 */
static void lcd_draw_char(int x, int y, char c, uint16_t fg, uint16_t bg, int scale)
{
    uint8_t glyph[7];
    if (!get_glyph(c, glyph)) get_glyph(' ', glyph);

    for (int row = 0; row < 7; ++row) {
        for (int col = 0; col < 5; ++col) {
            bool on = (glyph[row] >> (4 - col)) & 0x01;
            lcd_fill_rect(x + col * scale, y + row * scale,
                          x + (col + 1) * scale, y + (row + 1) * scale,
                          on ? fg : bg);
        }
    }
}

/**
 * @brief Dibuja una cadena de texto con la fuente bitmap interna.
 */
static void lcd_draw_text(int x, int y, const char *text, uint16_t fg, uint16_t bg, int scale)
{
    while (*text) {
        lcd_draw_char(x, y, *text++, fg, bg, scale);
        x += 6 * scale;
    }
}

/**
 * @copydoc lcd_init_display
 */
esp_err_t lcd_init_display(void)
{
    ESP_LOGI(TAG, "Initializing LCD");

    gpio_config_t bk_gpio = {};

    bk_gpio.mode = GPIO_MODE_OUTPUT;
    bk_gpio.pin_bit_mask = (1ULL << LCD_BL);

    ESP_ERROR_CHECK(gpio_config(&bk_gpio));

    gpio_set_level(LCD_BL, 1);

    spi_bus_config_t buscfg = {};

    buscfg.sclk_io_num = LCD_CLK;
    buscfg.mosi_io_num = LCD_MOSI;
    buscfg.miso_io_num = LCD_MISO;

    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;

    buscfg.max_transfer_sz =
        LCD_H_RES * 40 * sizeof(uint16_t);

    ESP_ERROR_CHECK(
        spi_bus_initialize(
            LCD_HOST,
            &buscfg,
            SPI_DMA_CH_AUTO
        )
    );

    esp_lcd_panel_io_handle_t io_handle = NULL;

    esp_lcd_panel_io_spi_config_t io_config = {};

    io_config.dc_gpio_num = LCD_DC;
    io_config.cs_gpio_num = LCD_CS;

    io_config.pclk_hz = 40000000;

    io_config.lcd_cmd_bits = 8;
    io_config.lcd_param_bits = 8;

    io_config.spi_mode = 0;

    io_config.trans_queue_depth = 10;

    ESP_ERROR_CHECK(
        esp_lcd_new_panel_io_spi(
            (esp_lcd_spi_bus_handle_t)LCD_HOST,
            &io_config,
            &io_handle
        )
    );

    esp_lcd_panel_dev_config_t panel_config = {};

    panel_config.reset_gpio_num = LCD_RST;
    panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR;
    panel_config.bits_per_pixel = 16;

    ESP_ERROR_CHECK(
        esp_lcd_new_panel_gc9a01(
            io_handle,
            &panel_config,
            &s_panel
        )
    );

    ESP_ERROR_CHECK(
        esp_lcd_panel_reset(s_panel)
    );

    ESP_ERROR_CHECK(
        esp_lcd_panel_init(s_panel)
    );

    ESP_ERROR_CHECK(
        esp_lcd_panel_swap_xy(s_panel, false)
    );

    ESP_ERROR_CHECK(
        esp_lcd_panel_mirror(s_panel, true, false)
    );

    ESP_ERROR_CHECK(
        esp_lcd_panel_invert_color(s_panel, false)
    );

    ESP_ERROR_CHECK(
        esp_lcd_panel_disp_on_off(s_panel, true)
    );

    lcd_fill_screen(rgb565(18, 52, 86));

    ESP_LOGI(TAG, "LCD OK");

    return ESP_OK;
}

/**
 * @copydoc lcd_show_imu_data
 */
extern "C" void lcd_show_imu_data(float ax, float ay, float az,
                                  float gx, float gy, float gz,
                                  int bpm, int spo2,
                                  bool bpm_valid, bool spo2_valid,
                                  bool finger_detected)
{
    const uint16_t bg = rgb565(18, 52, 86);
    const uint16_t fg = rgb565(206, 232, 242);
    const uint16_t title = rgb565(206, 232, 242);
    char line[32];

    lcd_fill_screen(bg);
    lcd_draw_text(54, 32, "MOTIONGUARD", title, bg, 2.3);
    lcd_fill_rect(52, 52, 188, 55, title);

    lcd_draw_text(38, 76, "ACC", title, bg, 1);
    snprintf(line, sizeof(line), "X:% .2f", ax);
    lcd_draw_text(34, 92, line, fg, bg, 1);
    snprintf(line, sizeof(line), "Y:% .2f", ay);
    lcd_draw_text(34, 106, line, fg, bg, 1);
    snprintf(line, sizeof(line), "Z:% .2f", az);
    lcd_draw_text(34, 120, line, fg, bg, 1);

    lcd_draw_text(134, 76, "GYR", title, bg, 1);
    snprintf(line, sizeof(line), "X:% .2f", gx);
    lcd_draw_text(126, 92, line, fg, bg, 1);
    snprintf(line, sizeof(line), "Y:% .2f", gy);
    lcd_draw_text(126, 106, line, fg, bg, 1);
    snprintf(line, sizeof(line), "Z:% .2f", gz);
    lcd_draw_text(126, 120, line, fg, bg, 1);

    if (finger_detected && bpm_valid) {
        snprintf(line, sizeof(line), "BPM:%3d", bpm);
    } else {
        snprintf(line, sizeof(line), "BPM:%3d", 0);
    }
    lcd_draw_text(54, 154, line, title, bg, 2);

    if (finger_detected && spo2_valid) {
        snprintf(line, sizeof(line), "SPO2:%3d%%", spo2);
    } else {
        snprintf(line, sizeof(line), "SPO2:%3d%%", 0);
    }
    lcd_draw_text(48, 188, line, title, bg, 2);
}
