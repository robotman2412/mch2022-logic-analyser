/*
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#pragma once

typedef enum {
    MODE_SLOW,
    MODE_FAST,
    MODE_RAMST,
    MODE_RAMLD,
} mode_t;

// For pin mappings.
#include "hardware.h"
// For graphics.
#include "pax_gfx.h"
// For PNG images.
#include "pax_codecs.h"
// The screen driver.
#include "ili9341.h"
// For all system settings and alike.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"
// For WiFi connectivity.
#include "wifi_connect.h"
#include "wifi_connection.h"
// For exiting to the launcher.
#include "soc/rtc.h"
#include "soc/rtc_cntl_reg.h"
// For logic analisys capture.
#include "ice40.h"

#define FPGA_CAP_SIZE 16
// Updates the screen with the last drawing.
void disp_flush();

// Exits the app, returning to the launcher.
void exit_to_launcher();

// Sets the FPGA's mode.
void set_mode(mode_t to);
// Does one fast capture, waiting until data changes.
void capture_fast(size_t out_len, char *out_data);
// Captures a sample of the data.
// Len as input: capacity of output buffer.
// Len as output: amount of data read.
size_t capture_sample(size_t cap_len, size_t out_len, char *out_data);
// Draws all UI.
void draw_ui();
