/*
 * Copyright (c) 2025, Contiki-NG ESP32-C6 Port
 * All rights reserved.
 */

/**
 * \file
 *      ESP32-C6 image header for direct ROM boot
 */

#include <stdint.h>

/* ESP32-C6 image header magic */
#define ESP_IMAGE_HEADER_MAGIC 0xE9

/* Flash modes */
#define ESP_IMAGE_FLASH_MODE_QIO  0x00
#define ESP_IMAGE_FLASH_MODE_QOUT 0x01
#define ESP_IMAGE_FLASH_MODE_DIO  0x02
#define ESP_IMAGE_FLASH_MODE_DOUT 0x03

/* Flash frequencies */
#define ESP_IMAGE_FLASH_FREQ_80M  0x0F
#define ESP_IMAGE_FLASH_FREQ_40M  0x00
#define ESP_IMAGE_FLASH_FREQ_26M  0x01
#define ESP_IMAGE_FLASH_FREQ_20M  0x02

/* Flash sizes */
#define ESP_IMAGE_FLASH_SIZE_1MB  0x00
#define ESP_IMAGE_FLASH_SIZE_2MB  0x10
#define ESP_IMAGE_FLASH_SIZE_4MB  0x20
#define ESP_IMAGE_FLASH_SIZE_8MB  0x30
#define ESP_IMAGE_FLASH_SIZE_16MB 0x40

/* Chip IDs */
#define ESP_CHIP_ID_ESP32C6 0x000D

extern void _start(void);

/* Image header - must be at the very beginning of flash */
__attribute__((section(".esp_header")))
const struct {
    uint8_t magic;
    uint8_t segment_count;
    uint8_t spi_mode;
    uint8_t spi_speed_size;
    uint32_t entry_addr;
    uint8_t wp_pin;
    uint8_t spi_pin_drv[3];
    uint16_t chip_id;
    uint8_t min_chip_rev;
    uint8_t reserved[8];
    uint8_t hash_appended;
} esp_image_header = {
    .magic = ESP_IMAGE_HEADER_MAGIC,
    .segment_count = 1,
    .spi_mode = ESP_IMAGE_FLASH_MODE_DIO,
    .spi_speed_size = ESP_IMAGE_FLASH_FREQ_40M | ESP_IMAGE_FLASH_SIZE_8MB,
    .entry_addr = (uint32_t)&_start,
    .wp_pin = 0xEE,  /* Disabled */
    .spi_pin_drv = {0x0F, 0x0F, 0x0F},
    .chip_id = ESP_CHIP_ID_ESP32C6,
    .min_chip_rev = 0,
    .reserved = {0},
    .hash_appended = 0
};

/* Segment header follows image header */
__attribute__((section(".esp_segment_header")))
const struct {
    uint32_t load_addr;
    uint32_t data_len;
} esp_segment_header = {
    .load_addr = 0x40800000,  /* IRAM base for ESP32-C6 */
    .data_len = 0  /* Will be filled by esptool or linker */
};
