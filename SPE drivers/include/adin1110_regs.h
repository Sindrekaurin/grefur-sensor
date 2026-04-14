#pragma once

/**
 * @file adin1110_regs.h
 * @brief ADIN1110 Register Map and Bit Definitions
 *
 * ADIN1110 is a 10BASE-T1L Single Pair Ethernet MAC/PHY.
 * Datasheet: https://www.analog.com/en/products/adin1110.html
 */

/* ─── SPI Frame Control ──────────────────────────────────────────────────── */
#define ADIN1110_SPI_CD_DATA       (0 << 7)   /* Data frame */
#define ADIN1110_SPI_CD_CTRL       (1 << 7)   /* Control frame */
#define ADIN1110_SPI_RW_WRITE      (0 << 5)   /* Write operation */
#define ADIN1110_SPI_RW_READ       (1 << 5)   /* Read operation */
#define ADIN1110_SPI_ADDR_MASK     (0x1FFF)   /* 13-bit register address */

/* ─── MAC Registers (32-bit) ─────────────────────────────────────────────── */
#define ADIN1110_IDVER             0x000  /* ID and Revision */
#define ADIN1110_PHYID             0x001  /* PHY ID */
#define ADIN1110_CAPABILITY        0x002  /* Capability */
#define ADIN1110_RESET             0x003  /* Reset Control */
#define ADIN1110_CONFIG0           0x004  /* Configuration 0 */
#define ADIN1110_CONFIG2           0x006  /* Configuration 2 */
#define ADIN1110_STATUS0           0x008  /* Status 0 */
#define ADIN1110_STATUS1           0x009  /* Status 1 */
#define ADIN1110_BUFSTS            0x00B  /* Buffer Status */
#define ADIN1110_IMASK0            0x00C  /* Interrupt Mask 0 */
#define ADIN1110_IMASK1            0x00D  /* Interrupt Mask 1 */
#define ADIN1110_MDIOACCI          0x020  /* MDIO Access */
#define ADIN1110_MDIOACC0          0x020
#define ADIN1110_TX_FSIZE          0x030  /* TX Frame Size */
#define ADIN1110_TX                0x031  /* TX FIFO (write-only) */
#define ADIN1110_TX_SPACE          0x032  /* TX FIFO Space */
#define ADIN1110_RX_FSIZE          0x090  /* RX Frame Size */
#define ADIN1110_RX                0x091  /* RX FIFO (read-only) */
#define ADIN1110_MAC_ADDR_FILTER_UPR  0x050 /* MAC Addr Filter Upper */
#define ADIN1110_MAC_ADDR_FILTER_LWR  0x051 /* MAC Addr Filter Lower */
#define ADIN1110_MAC_ADDR_BCAST_UPR   0x052 /* Broadcast Filter Upper */
#define ADIN1110_MAC_ADDR_BCAST_LWR   0x053 /* Broadcast Filter Lower */

/* ─── IDVER Fields ───────────────────────────────────────────────────────── */
#define ADIN1110_IDVER_REV_MASK    (0x0F)
#define ADIN1110_IDVER_MODEL_MASK  (0xFF0)
#define ADIN1110_IDVER_MODEL_SHIFT (4)
#define ADIN1110_PHYID_OUI         (0x0283BC81UL)

/* ─── RESET Fields ───────────────────────────────────────────────────────── */
#define ADIN1110_RESET_SWRESET     (1 << 0)  /* Software Reset */

/* ─── CONFIG0 Fields ─────────────────────────────────────────────────────── */
#define ADIN1110_CONFIG0_TXCTE     (1 << 9)  /* TX Cut-Through Enable */
#define ADIN1110_CONFIG0_RXCTE     (1 << 8)  /* RX Cut-Through Enable */
#define ADIN1110_CONFIG0_FTSE      (1 << 7)  /* Frame Timestamp Enable */
#define ADIN1110_CONFIG0_FTSS      (1 << 6)  /* Frame Timestamp Select */
#define ADIN1110_CONFIG0_SYNC      (1 << 15) /* Sync bit for config writes */

/* ─── CONFIG2 Fields ─────────────────────────────────────────────────────── */
#define ADIN1110_CONFIG2_P1_FWD_UNK2HOST (1 << 2) /* Forward unknown to host */
#define ADIN1110_CONFIG2_CRC_APPEND      (1 << 5) /* Append CRC to TX */

/* ─── STATUS0 Fields ─────────────────────────────────────────────────────── */
#define ADIN1110_STATUS0_TXPE      (1 << 0)  /* TX Protocol Error */
#define ADIN1110_STATUS0_TXBOE     (1 << 1)  /* TX Buffer Overflow */
#define ADIN1110_STATUS0_TXBUSY    (1 << 2)  /* TX Busy */
#define ADIN1110_STATUS0_RXBOE     (1 << 3)  /* RX Buffer Overflow */
#define ADIN1110_STATUS0_LOFE      (1 << 4)  /* Loss of Frame Error */
#define ADIN1110_STATUS0_HDRE      (1 << 5)  /* Header Error */
#define ADIN1110_STATUS0_RESETC    (1 << 6)  /* Reset Complete */
#define ADIN1110_STATUS0_PHYINT    (1 << 7)  /* PHY Interrupt */
#define ADIN1110_STATUS0_TXRDY     (1 << 8)  /* TX Ready */
#define ADIN1110_STATUS0_P1RXRDY   (1 << 9)  /* Port1 RX Ready */

/* ─── STATUS1 Fields ─────────────────────────────────────────────────────── */
#define ADIN1110_STATUS1_TX_RDY    (1 << 5)
#define ADIN1110_STATUS1_P1_RX_RDY (1 << 4)
#define ADIN1110_STATUS1_SPI_ERR   (1 << 10)

/* ─── IMASK0/IMASK1 Fields ───────────────────────────────────────────────── */
#define ADIN1110_IMASK0_TXPEM      (1 << 0)
#define ADIN1110_IMASK0_TXBOEM     (1 << 1)
#define ADIN1110_IMASK0_RXBOEM     (1 << 3)
#define ADIN1110_IMASK0_LOFEM      (1 << 4)
#define ADIN1110_IMASK0_HDREM      (1 << 5)
#define ADIN1110_IMASK0_RESETCM    (1 << 6)
#define ADIN1110_IMASK0_PHYINTM    (1 << 7)
#define ADIN1110_IMASK0_TXRDYM     (1 << 8)
#define ADIN1110_IMASK0_P1RXRDYM   (1 << 9)

/* ─── MDIO Access ────────────────────────────────────────────────────────── */
#define ADIN1110_MDIO_TRDONE       (1 << 31)  /* Transaction Done */
#define ADIN1110_MDIO_ST           (1 << 30)  /* Clause 45 = 0, Clause 22 = 1 */
#define ADIN1110_MDIO_OP_RD        (0x2 << 26)
#define ADIN1110_MDIO_OP_WR        (0x1 << 26)
#define ADIN1110_MDIO_PRTAD_SHIFT  (21)
#define ADIN1110_MDIO_DEVAD_SHIFT  (16)
#define ADIN1110_MDIO_DATA_MASK    (0xFFFF)

/* ─── PHY Registers (accessed via MDIO/MDIOACCI) ────────────────────────── */
/* Clause 22 – basic regs */
#define ADIN1110_PHY_BMCR          0x0000  /* Basic Mode Control */
#define ADIN1110_PHY_BMSR          0x0001  /* Basic Mode Status */
#define ADIN1110_PHY_ID1           0x0002
#define ADIN1110_PHY_ID2           0x0003

/* 10BASE-T1L specific (Clause 45, MMD 1) */
#define ADIN1110_MMD1_DEV_ID1      0x0002
#define ADIN1110_MMD1_DEV_ID2      0x0003
#define ADIN1110_MMD1_PMA_CTRL1    0x0000
#define ADIN1110_MMD1_PMA_CTRL2    0x0001
#define ADIN1110_MMD1_T1L_PMA_CTRL 0x08F6  /* T1L PMA Control */
#define ADIN1110_MMD1_LINK_STAT    0x0001

/* BMCR bits */
#define ADIN1110_BMCR_RESET        (1 << 15)
#define ADIN1110_BMCR_LOOPBACK     (1 << 14)
#define ADIN1110_BMCR_POWER_DOWN   (1 << 11)
#define ADIN1110_BMCR_ISOLATE      (1 << 10)

/* T1L PMA Control bits */
#define ADIN1110_T1L_PMA_LB        (1 << 0)
#define ADIN1110_T1L_TX_LEVEL_2V4  (1 << 12) /* 2.4V TX level */

/* ─── TX/RX Frame Header ─────────────────────────────────────────────────── */
#define ADIN1110_FRAME_HEADER_LEN  2   /* 2-byte frame header before payload */
#define ADIN1110_FRAME_FCS_LEN     4   /* 4-byte FCS appended by MAC */
#define ADIN1110_SPI_HEADER_LEN    2   /* 2-byte SPI control header */
#define ADIN1110_MAX_FRAMELEN      1518
#define ADIN1110_MIN_FRAMELEN      64
