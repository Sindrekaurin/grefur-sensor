/**
 * @file main.c
 * @brief Example: ESP32 + ADIN1110 10BASE-T1L Ethernet with lwIP / TCP-IP stack
 *
 * Wiring (adjust GPIOs to your board):
 *
 *  ESP32 Pin   │  ADIN1110 Pin   │  Function
 *  ────────────┼─────────────────┼──────────────────────────
 *  GPIO 18     │  SCLK           │  SPI clock
 *  GPIO 23     │  MOSI / SDI     │  SPI MOSI
 *  GPIO 19     │  MISO / SDO     │  SPI MISO
 *  GPIO  5     │  /CS            │  SPI chip-select (active-low)
 *  GPIO  4     │  /INT           │  Interrupt (active-low)
 *  GPIO  2     │  /RESET         │  Hardware reset (active-low, optional)
 *  3V3         │  VDD            │  Power
 *  GND         │  GND            │  Ground
 *
 *  ADIN1110 SPE port → 100-Ω twisted pair → remote device
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/inet.h"

#include "adin1110.h"

static const char *TAG = "main";

/* ─── Pin definitions – change to match your hardware ─────────────────── */
#define PIN_SCLK   18
#define PIN_MOSI   23
#define PIN_MISO   19
#define PIN_CS      5
#define PIN_INT     4
#define PIN_RST     2   /* -1 if not connected */

/* ─── Our MAC address ──────────────────────────────────────────────────── */
static const uint8_t MY_MAC[6] = { 0x02, 0xAD, 0x1E, 0x10, 0x00, 0x01 };

/* ─── Global driver handle ─────────────────────────────────────────────── */
static adin1110_handle_t g_adin = NULL;
static esp_netif_t      *g_netif = NULL;

/* ─── esp_netif I/O driver interface ───────────────────────────────────── */

static esp_err_t _netif_transmit(void *h, void *buf, size_t len, void *nb)
{
    return adin1110_netif_transmit(h, buf, len, nb);
}

static void _netif_free_rx(void *h, void *buf)
{
    adin1110_netif_free_rx_buf(h, buf);
}

static esp_err_t _netif_attach(esp_netif_t *netif, void *driver_handle)
{
    adin1110_attach_netif((adin1110_handle_t)driver_handle, netif);
    return ESP_OK;
}

static const esp_netif_driver_ifconfig_t s_drv_ifconfig = {
    .driver_free_rx_buffer = _netif_free_rx,
    .transmit              = _netif_transmit,
    .handle                = NULL,  /* set after adin1110_init() */
};

/* ─── IP event handler ─────────────────────────────────────────────────── */
static void _on_ip_event(void *arg, esp_event_base_t base,
                         int32_t event_id, void *data)
{
    if (event_id == IP_EVENT_ETH_GOT_IP) {
        ip_event_got_ip_t *evt = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&evt->ip_info.ip));
    }
}

/* ─── Application entry ────────────────────────────────────────────────── */
void app_main(void)
{
    ESP_LOGI(TAG, "ADIN1110 10BASE-T1L demo starting...");

    /* 1. NVS (required by some components) */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* 2. Event loop */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,
                    ESP_EVENT_ANY_ID, _on_ip_event, NULL));

    /* 3. esp_netif */
    ESP_ERROR_CHECK(esp_netif_init());

    /* Build a custom netif config for our wired SPE interface */
    esp_netif_inherent_config_t base_cfg = ADIN1110_NETIF_DEFAULT_CONFIG();
    esp_netif_config_t netif_cfg = {
        .base   = &base_cfg,
        .driver = NULL,         /* will be set below */
        .stack  = ESP_NETIF_NETSTACK_DEFAULT_ETH,
    };
    g_netif = esp_netif_new(&netif_cfg);
    assert(g_netif);

    /* 4. Initialize the ADIN1110 driver */
    adin1110_config_t adin_cfg = {
        .spi_host     = SPI2_HOST,
        .pin_miso     = PIN_MISO,
        .pin_mosi     = PIN_MOSI,
        .pin_sclk     = PIN_SCLK,
        .pin_cs       = PIN_CS,
        .pin_int      = PIN_INT,
        .pin_rst      = PIN_RST,
        .spi_clock_hz = 12 * 1000 * 1000,
        .tx_level     = ADIN1110_TX_LEVEL_2V4,  /* long cable support */
        .spi_prot     = ADIN1110_SPI_PROT_NONE,
        .append_crc   = true,
    };
    memcpy(adin_cfg.mac_addr, MY_MAC, 6);

    ESP_ERROR_CHECK(adin1110_init(&adin_cfg, &g_adin));

    /* 5. Set the MAC address on the netif */
    esp_netif_set_mac(g_netif, (uint8_t *)MY_MAC);

    /* 6. Attach driver → netif glue */
    esp_netif_driver_ifconfig_t drv_cfg = s_drv_ifconfig;
    drv_cfg.handle = g_adin;
    /* Use the esp_netif_attach() helper */
    ESP_ERROR_CHECK(esp_netif_attach(g_netif, g_adin));
    adin1110_attach_netif(g_adin, g_netif);

    /* 7. Start the driver (enables IRQ, launches RX task, enables PHY) */
    ESP_ERROR_CHECK(adin1110_start(g_adin));

    ESP_LOGI(TAG, "Driver started – waiting for 10BASE-T1L link...");

    /* 8. Main loop – just monitor link state */
    while (1) {
        bool link = false;
        if (adin1110_get_link(g_adin, &link) == ESP_OK) {
            ESP_LOGD(TAG, "Link: %s", link ? "UP" : "DOWN");
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
