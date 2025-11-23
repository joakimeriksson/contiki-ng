/*
 * ESP32-C6 App Descriptor for ESP-IDF bootloader compatibility
 */

#include <stdint.h>

/* ESP-IDF app descriptor structure */
typedef struct {
    uint32_t magic_word;        /* Magic word ESP_APP_DESC_MAGIC_WORD */
    uint32_t secure_version;    /* Secure version */
    uint32_t reserv1[2];        /* Reserved */
    char version[32];           /* Application version */
    char project_name[32];      /* Project name */
    char time[16];              /* Compile time */
    char date[16];              /* Compile date */
    char idf_ver[32];           /* IDF version */
    uint8_t app_elf_sha256[32]; /* SHA256 of app ELF */
    uint32_t reserv2[20];       /* Reserved */
} esp_app_desc_t;

#define ESP_APP_DESC_MAGIC_WORD 0xABCD5432

/* Place in .rodata.desc section at start of app */
__attribute__((section(".rodata_desc"))) __attribute__((used))
const esp_app_desc_t esp_app_desc = {
    .magic_word = ESP_APP_DESC_MAGIC_WORD,
    .secure_version = 0,
    .version = "1.0.0",
    .project_name = "contiki-ng",
    .time = __TIME__,
    .date = __DATE__,
    .idf_ver = "v5.4",
    .app_elf_sha256 = {0},
};
