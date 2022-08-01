/*
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

// This file contains a simple Hello World app which you can base you own
// native Badge apps on.

#include "main.h"
#include "stdlib.h"
#include "string.h"
#include "esp_timer.h"

extern uint8_t fpga_bin_start[] asm("_binary_logix_bin_start");
extern uint8_t fpga_bin_end[]   asm("_binary_logix_bin_end");

static pax_buf_t buf;
xQueueHandle buttonQueue;
ICE40 *fpga;

#include <esp_log.h>
static const char *TAG = "logix";
static bool do_fast_cap;
static uint64_t sample_nanos = 20000000;
static char capbuf[128];

// Updates the screen with the latest buffer.
void disp_flush() {
    ili9341_write(get_ili9341(), buf.buf);
}

// Exits the app, returning to the launcher.
void exit_to_launcher() {
    REG_WRITE(RTC_CNTL_STORE0_REG, 0);
    esp_restart();
}

void app_main() {
    ESP_LOGI(TAG, "Starting...");

    // Initialize the screen, the I2C and the SPI busses.
    bsp_init();

    // Initialize the RP2040 (responsible for buttons, etc).
    bsp_rp2040_init();
    
    // This queue is used to receive button presses.
    buttonQueue = get_rp2040()->queue;
    
    // Initialize graphics for the screen.
    pax_buf_init(&buf, NULL, 320, 240, PAX_BUF_16_565RGB);
    
    // Initialize NVS.
    nvs_flash_init();
    
    // Initialise FPGA.
    ESP_LOGI(TAG, "Starting FPGA...");
    bsp_ice40_init();
    fpga = get_ice40();
    ice40_enable(fpga);
    ice40_load_bitstream(fpga, fpga_bin_start, fpga_bin_end-fpga_bin_start);
    
    
    ESP_LOGI(TAG, "Ready.");
    
    while (1) {
        // Structure used to receive data.
        rp2040_input_message_t message;
        
        pax_background(&buf, 0);
        capture_sample(FPGA_CAP_SIZE, sizeof(capbuf), capbuf);
        draw_ui();
        disp_flush();
        
        // Wait forever for a button press (because of portMAX_DELAY)
        if (xQueueReceive(buttonQueue, &message, pdMS_TO_TICKS(20))) {
            // Which button is currently pressed?
            if (message.input == RP2040_INPUT_BUTTON_HOME && message.state) {
                // If home is pressed, exit to launcher.
                exit_to_launcher();
            } else if (message.input == RP2040_INPUT_BUTTON_ACCEPT && message.state) {
                // Start fast capture.
                pax_draw_text(&buf, -1, pax_font_saira_regular, 18, 5, 5, "Fast capture...");
                disp_flush();
                
                pax_background(&buf, 0);
                capture_fast(sizeof(capbuf), capbuf);
                sample_nanos = 41666;
                draw_ui();
                sample_nanos = 20000000;
                pax_draw_text(&buf, -1, pax_font_saira_regular, 18, 5, 5, "Done!");
                disp_flush();
                
                // Wait for goahead.
                while (!xQueueReceive(buttonQueue, &message, portMAX_DELAY) || !message.state);
            }
        }
    }
}

// Sets the FPGA's mode.
void set_mode(fpgamode to) {
    uint8_t cmd = to;
    ice40_transaction(fpga, NULL, 0, &to, 1);
}

// Does one fast capture, waiting until data changes.
void capture_fast(size_t out_len, char *out_data) {
    // Send fast capture command.
    do_fast_cap = true;
    capture_sample(1, out_len, out_data);
    // Capture some data.
    vTaskDelay(pdMS_TO_TICKS(5));
    capture_sample(FPGA_CAP_SIZE, out_len, out_data);
    // Select the initial state.
    uint8_t initial  = out_data[out_len-1];
    memset(out_data, initial, out_len - 1);
    // Keep 2 old data bits.
    size_t quota = out_len - 2;
    // TODO: Disable WDT on current core.
    // Poll at 1KHz for samples.
    ESP_LOGI(TAG, "Starting fast capture.");
    uint64_t limit = esp_timer_get_time() + 1000000;
    while (quota > 0 && esp_timer_get_time() < limit) {
        // Get new data.
        capture_sample(quota, out_len, out_data);
        // Count initial matching bytes.
        size_t i;
        for (i = 0; i < out_len; i++) {
            if (out_data[i] != initial) {
                break;
            }
        }
        quota = i-2;
    }
    // done:
    ESP_LOGI(TAG, "Finished fast capture.");
    // Return to normal cap.
    do_fast_cap = false;
}

// Captures a sample of the data.
// Len as input: capacity of output buffer.
// Len as output: amount of data read.
size_t capture_sample(size_t cap_len, size_t out_len, char *out_data) {
    static uint8_t last_offs = 0;
    static uint8_t in_buf [FPGA_CAP_SIZE+1];
    static uint8_t out_buf[FPGA_CAP_SIZE+1];
    if (cap_len > FPGA_CAP_SIZE) cap_len = FPGA_CAP_SIZE;
    // Grab data.
    memset(out_buf, 0, FPGA_CAP_SIZE+1);
    out_buf[cap_len] = do_fast_cap;
    ice40_transaction(fpga, out_buf, cap_len+1, in_buf, cap_len+1);
    
    // Shift over out data buffer.
    int shift = in_buf[0]-last_offs;
    if (shift<0) {
        shift += FPGA_CAP_SIZE;
    }
    if (shift > cap_len) shift = cap_len;
    memmove(out_data, out_data+shift, out_len-shift);
    // Copy to out data buffer.
    for (size_t i = 0; i < cap_len; i++) {
        size_t index = ((i-in_buf[0]-1+FPGA_CAP_SIZE)%FPGA_CAP_SIZE);
        if (index >= FPGA_CAP_SIZE-cap_len) {
            out_data[out_len-FPGA_CAP_SIZE+index] = in_buf[i+1];
        }
    }
    
    last_offs = in_buf[0];
    return shift;
}

static void draw_bits_to(char next, float x, float y, float height) {
    static char  prev;
    static float prev_x = 1324783;
    if (x < prev_x) {
        prev   = next;
        prev_x = x;
        return;
    }
    
    const float pitch = height / 8.0;
    const float delta = pitch / 3;
    y += delta;
    for (int i = 0; i < 8; i++) {
        float y0 = ((prev >> i) & 1) ? y : y + delta;
        float y1 = ((next >> i) & 1) ? y : y + delta;
        pax_draw_line(&buf, 0xffffffff, prev_x, y0, x, y1);
        y += pitch;
    }
    
    prev_x = x;
    prev   = next;
}

// Draws a sample of data across the width of the screen.
static void draw_sample(char *data, size_t len, float x, float y, float width, float height) {
    float pitch = width / (float) (len-1);
    for (size_t i = 0; i < len; i ++) {
        draw_bits_to(data[i], x, y, height);
        x += pitch;
    }
}

static void draw_time_grid(float x, float y, float width, float height, char *data, size_t data_len) {
    float samplesPerDiv;
    float divWidth;
    const char *text;
    { // Determine scale.
        if (sample_nanos < 1000) {
            text = "1us / 10^-6";
            samplesPerDiv = 1000.0 / sample_nanos;
        } else if (sample_nanos < 10000) {
            text = "10us / 10^-5";
            samplesPerDiv = 10000.0 / sample_nanos;
        } else if (sample_nanos < 100000) {
            text = "100us / 10^-4";
            samplesPerDiv = 100000.0 / sample_nanos;
        } else if (sample_nanos < 1000000) {
            text = "1ms / 10^-3";
            samplesPerDiv = 1000000.0 / sample_nanos;
        } else if (sample_nanos < 10000000) {
            text = "10ms / 10^-2";
            samplesPerDiv = 10000000.0 / sample_nanos;
        } else if (sample_nanos < 100000000) {
            text = "100ms / 10^-1";
            samplesPerDiv = 100000000.0 / sample_nanos;
        } else {
            text = "1s";
            samplesPerDiv = 1000000000.0 / sample_nanos;
        }
        divWidth = width / (float) data_len * samplesPerDiv;
    }
    
    // Draw vertical lines.
    float lim = x + width;
    while (x <= lim) {
        pax_draw_line(&buf, 0xff7f7f7f, x, y, x, y+height);
        x += divWidth;
    }
    
    // Draw timescale text.
    pax_draw_text(&buf, 0xffffffff, pax_font_sky, 9, 230, 2, text);
}

// Draws all UI.
void draw_ui() {
    float x=0, y=30, width=320, height=210;
    
    // Draw the time grid.
    draw_time_grid(x, y, width, height, capbuf, sizeof(capbuf));
    // Draw the samples.
    draw_sample(capbuf, sizeof(capbuf), x, y, width, height);
    
    // Draw the outline.
    pax_draw_line(&buf, 0xffffffff, x, y, x+width, y);
    // pax_draw_line(&buf, 0xffffffff, x, y, x,       y+height);
}

