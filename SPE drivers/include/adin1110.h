#pragma once

/**
 * @file adin1110.h
 * @brief ESP32 SPI Driver for ADIN1110 10BASE-T1L Single Pair Ethernet
 *
 * Architecture:
 *   ESP32 <──SPI──> ADIN1110 MAC/PHY <──10BASE-T1L──> Twisted Pair
 *
 * Features:
 *   - Full-duplex SPI up to 25 MHz (CPOL=0, CPHA=0)
 *   - Optional CRC on SPI frames (for noisy environments)
 *   - Interrupt-driven RX via INT# GPIO
 *   - lwIP netif integration
 *   - MDIO access to internal 10BASE-T1L PHY
 *   - Configurable TX voltage level (1.0 V or 2.4 V)
 */

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_netif.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ─── Configuration ──────────────────────────────────────────────────────── */

/** SPI host to use (SPI2_HOST or SPI3_HOST on ESP32) */
#ifndef ADIN1110_SPI_HOST
#define ADIN1110_SPI_HOST   SPI2_HOST
#endif

/** Maximum SPI clock speed in Hz */
#define ADIN1110_SPI_CLOCK_HZ       (12 * 1000 * 1000)  /* 12 MHz – safe start */
#define ADIN1110_SPI_CLOCK_MAX_HZ   (25 * 1000 * 1000)

/** FreeRTOS task priorities */
#define ADIN1110_RX_TASK_PRIORITY   (configMAX_PRIORITIES - 1)
#define ADIN1110_RX_TASK_STACK      (4096)

/** Receive FIFO poll interval if interrupt is not used (ms) */
#define ADIN1110_POLL_MS            10

/** Maximum number of frames in the RX processing loop per interrupt */
#define ADIN1110_RX_MAX_BURST       8

/* ─── Types ──────────────────────────────────────────────────────────────── */

/** TX voltage levels for 10BASE-T1L */
typedef enum {
    ADIN1110_TX_LEVEL_1V0 = 0,   /*!< 1.0 Vpp – short cables (<15 m) */
    ADIN1110_TX_LEVEL_2V4 = 1,   /*!< 2.4 Vpp – long cables (up to 1000 m) */
} adin1110_tx_level_t;

/** SPI frame protection mode */
typedef enum {
    ADIN1110_SPI_PROT_NONE = 0,  /*!< No CRC on SPI frames */
    ADIN1110_SPI_PROT_CRC  = 1,  /*!< CRC-8 appended to every SPI transaction */
} adin1110_spi_prot_t;

/**
 * @brief Hardware configuration for the ESP32 ↔ ADIN1110 connection.
 */
typedef struct {
    /* SPI bus (must already be initialized with spi_bus_initialize()) */
    spi_host_device_t spi_host;   /*!< SPI peripheral (SPI2_HOST / SPI3_HOST) */
    int               pin_miso;   /*!< MISO GPIO */
    int               pin_mosi;   /*!< MOSI GPIO */
    int               pin_sclk;   /*!< SCLK GPIO */
    int               pin_cs;     /*!< /CS  GPIO */
    int               pin_int;    /*!< /INT GPIO (interrupt, active-low) */
    int               pin_rst;    /*!< /RST GPIO (-1 = not connected) */
    uint32_t          spi_clock_hz; /*!< SPI clock in Hz */

    /* Driver options */
    adin1110_tx_level_t   tx_level;  /*!< PHY TX voltage level */
    adin1110_spi_prot_t   spi_prot;  /*!< SPI frame CRC protection */
    bool                  append_crc;/*!< Let ADIN1110 MAC append FCS to TX */
    uint8_t               mac_addr[6]; /*!< Local MAC address */
} adin1110_config_t;

/**
 * @brief Driver handle (opaque to the user).
 */
typedef struct adin1110_dev *adin1110_handle_t;

/* ─── Public API ─────────────────────────────────────────────────────────── */

/**
 * @brief  Initialize SPI bus, reset ADIN1110, verify chip ID, configure MAC/PHY.
 *
 * @param  cfg      Pointer to hardware/software configuration.
 * @param  out_dev  Returns the driver handle on success.
 * @return ESP_OK on success, ESP_ERR_* otherwise.
 */
esp_err_t adin1110_init(const adin1110_config_t *cfg, adin1110_handle_t *out_dev);

/**
 * @brief  Attach driver to an esp_netif instance (lwIP glue).
 *
 * Call after esp_netif_new() with ADIN1110_NETIF_DEFAULT_CONFIG.
 *
 * @param  dev     Driver handle.
 * @param  netif   esp_netif handle.
 * @return ESP_OK on success.
 */
esp_err_t adin1110_attach_netif(adin1110_handle_t dev, esp_netif_t *netif);

/**
 * @brief  Start the driver: enable interrupts, launch RX task, bring PHY up.
 *
 * @param  dev  Driver handle.
 * @return ESP_OK on success.
 */
esp_err_t adin1110_start(adin1110_handle_t dev);

/**
 * @brief  Stop driver and disable interrupts.
 *
 * @param  dev  Driver handle.
 * @return ESP_OK on success.
 */
esp_err_t adin1110_stop(adin1110_handle_t dev);

/**
 * @brief  Free all resources and de-initialize the device.
 *
 * @param  dev  Driver handle (invalid after this call).
 * @return ESP_OK on success.
 */
esp_err_t adin1110_deinit(adin1110_handle_t dev);

/**
 * @brief  Send a raw Ethernet frame.
 *
 * @param  dev    Driver handle.
 * @param  buf    Frame buffer (must include Ethernet header, no FCS).
 * @param  len    Frame length in bytes (14 … 1514).
 * @return ESP_OK on success.
 */
esp_err_t adin1110_send(adin1110_handle_t dev, const uint8_t *buf, size_t len);

/**
 * @brief  Read link status.
 *
 * @param  dev        Driver handle.
 * @param  link_up    Set to true if 10BASE-T1L link is established.
 * @return ESP_OK on success.
 */
esp_err_t adin1110_get_link(adin1110_handle_t dev, bool *link_up);

/**
 * @brief  Write a Clause-22 PHY register via internal MDIO bus.
 */
esp_err_t adin1110_mdio_write(adin1110_handle_t dev,
                              uint8_t phy_addr, uint8_t reg,
                              uint16_t data);

/**
 * @brief  Read a Clause-22 PHY register via internal MDIO bus.
 */
esp_err_t adin1110_mdio_read(adin1110_handle_t dev,
                             uint8_t phy_addr, uint8_t reg,
                             uint16_t *out_data);

/**
 * @brief  Read a MAC register (32-bit).
 */
esp_err_t adin1110_reg_read(adin1110_handle_t dev,
                            uint16_t reg, uint32_t *out_val);

/**
 * @brief  Write a MAC register (32-bit).
 */
esp_err_t adin1110_reg_write(adin1110_handle_t dev,
                             uint16_t reg, uint32_t val);

/* ─── esp_netif helper ───────────────────────────────────────────────────── */

/**
 * @brief esp_netif driver I/O glue – transmit callback used internally.
 */
esp_err_t adin1110_netif_transmit(void *h, void *buffer, size_t len, void *netstack_buf);
void      adin1110_netif_free_rx_buf(void *h, void *buffer);

/**
 * @brief  Default esp_netif_inherent_config for a wired ADIN1110 interface.
 */
#define ADIN1110_NETIF_DEFAULT_CONFIG() {                          \
    .flags         = ESP_NETIF_FLAG_AUTOUP,                        \
    .ip_info       = NULL,                                         \
    .get_ip_event  = IP_EVENT_ETH_GOT_IP,                          \
    .lost_ip_event = 0,                                            \
    .if_key        = "ADIN1110",                                   \
    .if_desc       = "adin1110",                                   \
    .route_prio    = 50,                                           \
}

#ifdef __cplusplus
}
#endif
