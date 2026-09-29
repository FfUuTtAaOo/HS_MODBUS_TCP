#include "w5500.h"
#include "spi.h"
#include <string.h>

/* ================================================================
 *  CS Pin Control Macros
 * ================================================================ */
#define W5500_CS_LOW()   HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET)
#define W5500_CS_HIGH()  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET)

/* ================================================================
 *  SPI Communication (VDM mode)
 *  Frame: [16-bit Address MSB] [8-bit Control] [N x 8-bit Data]
 *  Control = (block << 5) | (rwb << 2)  where rwb: 0=read, 1=write
 * ================================================================ */

static void spi_select(void)
{
    W5500_CS_LOW();
}

static void spi_deselect(void)
{
    W5500_CS_HIGH();
}

static void spi_write_byte(uint8_t data)
{
    HAL_SPI_Transmit(&hspi1, &data, 1, HAL_MAX_DELAY);
}

static uint8_t spi_read_byte(void)
{
    uint8_t data;
    HAL_SPI_Receive(&hspi1, &data, 1, HAL_MAX_DELAY);
    return data;
}

static void spi_send_address_control(uint8_t block, uint16_t addr, uint8_t rwb)
{
    uint8_t ctrl = (uint8_t)((block << 3) | (rwb << 2));  /* VDM: BS[7:3], RWB[2], OM[1:0]=00 */
    spi_write_byte((uint8_t)(addr >> 8));    /* Address high byte */
    spi_write_byte((uint8_t)(addr & 0xFF));  /* Address low byte */
    spi_write_byte(ctrl);                     /* Control byte */
}

/* ================================================================
 *  Register Read/Write
 * ================================================================ */

uint8_t w5500_read_byte(uint8_t block, uint16_t addr)
{
    uint8_t data;
    spi_select();
    spi_send_address_control(block, addr, 0);  /* rwb=0: read */
    data = spi_read_byte();
    spi_deselect();
    return data;
}

void w5500_write_byte(uint8_t block, uint16_t addr, uint8_t data)
{
    spi_select();
    spi_send_address_control(block, addr, 1);  /* rwb=1: write */
    spi_write_byte(data);
    spi_deselect();
}

uint16_t w5500_read_word(uint8_t block, uint16_t addr)
{
    uint16_t val;
    spi_select();
    spi_send_address_control(block, addr, 0);
    val = (uint16_t)spi_read_byte() << 8;
    val |= spi_read_byte();
    spi_deselect();
    return val;
}

void w5500_write_word(uint8_t block, uint16_t addr, uint16_t data)
{
    spi_select();
    spi_send_address_control(block, addr, 1);
    spi_write_byte((uint8_t)(data >> 8));
    spi_write_byte((uint8_t)(data & 0xFF));
    spi_deselect();
}

void w5500_read_buf(uint8_t block, uint16_t addr, uint8_t *buf, uint16_t len)
{
    spi_select();
    spi_send_address_control(block, addr, 0);
    while (len--) {
        *buf++ = spi_read_byte();
    }
    spi_deselect();
}

void w5500_write_buf(uint8_t block, uint16_t addr, const uint8_t *buf, uint16_t len)
{
    spi_select();
    spi_send_address_control(block, addr, 1);
    while (len--) {
        spi_write_byte(*buf++);
    }
    spi_deselect();
}

/* ================================================================
 *  Hardware Reset
 * ================================================================ */

void w5500_hw_reset(void)
{
    /* Release HW reset (RST was held LOW by gpio.c from power-up) */
    HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(50);  /* Wait for internal PLL to lock (2ms typ, 50ms safe) */

    /* Software reset via MR register to ensure clean internal state */
    W5500_CS_LOW();
    spi_send_address_control(W5500_BS_COMMON, W5500_MR, 1);
    spi_write_byte(0x80);
    W5500_CS_HIGH();
    HAL_Delay(1);
    /* Poll until MR RST bit auto-clears */
    while (w5500_read_byte(W5500_BS_COMMON, W5500_MR) & 0x80) {
        HAL_Delay(1);
    }
}

/* ================================================================
 *  Initialization
 * ================================================================ */

uint8_t w5500_init(void)
{
    uint8_t ver;

    /* 1. Hardware reset */
    w5500_hw_reset();

    /* 2. Verify chip version */
    ver = w5500_read_byte(W5500_BS_COMMON, W5500_VERSIONR);
    if (ver != 0x04) {
        return 0;  /* W5500 not detected */
    }

    /* 3. Set retry time and count (W5500 default values) */
    w5500_write_word(W5500_BS_COMMON, W5500_RTR, 0x07D0);   /* 2000 * 100us = 200ms */
    w5500_write_byte(W5500_BS_COMMON, W5500_RCR, 8);         /* 8 retries */

    return 1;
}

/**
 * @brief  Wait for PHY link to come up (auto-negotiation ~1-3s after PHYCFGR write)
 * @return 1 = link up, 0 = timeout (no cable / negotiation failed)
 */
uint8_t w5500_phy_wait_link(uint32_t timeout_ms)
{
    uint8_t phy;
    while (timeout_ms > 0) {
        phy = w5500_read_byte(W5500_BS_COMMON, W5500_PHYCFGR);
        if (phy & 0x01) return 1;  /* LNK bit set = link up */
        HAL_Delay(10);
        timeout_ms -= 10;
    }
    return 0;  /* timeout */
}

/* ================================================================
 *  Network Configuration
 * ================================================================ */

void w5500_set_mac(const uint8_t mac[6])
{
    w5500_write_buf(W5500_BS_COMMON, W5500_SHAR, mac, 6);
}

void w5500_set_ip(const uint8_t ip[4])
{
    w5500_write_buf(W5500_BS_COMMON, W5500_SIPR, ip, 4);
}

void w5500_set_gateway(const uint8_t gw[4])
{
    w5500_write_buf(W5500_BS_COMMON, W5500_GAR, gw, 4);
}

void w5500_set_subnet(const uint8_t sn[4])
{
    w5500_write_buf(W5500_BS_COMMON, W5500_SUBR, sn, 4);
}

/* ================================================================
 *  Buffer Configuration — MUST be called BEFORE any socket OPEN
 *  The W5500 allocates TX/RX buffer memory in batch once ALL
 *  Sn_RXBUF_SIZE / Sn_TXBUF_SIZE registers have been written.
 * ================================================================ */
void w5500_configure_buffers(void)
{
    /* Sockets 0-3: 2KB TX + 2KB RX each (4 × 4KB = 16KB total)
     * Sockets 4-7: 0 KB (disabled) */
    for (uint8_t n = 0; n < 8; n++) {
        uint8_t block = W5500_BS_SOCK(n);
        uint8_t size  = (n < W5500_SOCK_COUNT) ? 0x02 : 0x00;
        w5500_write_byte(block, Sn_RXBUF_SIZE, size);
        w5500_write_byte(block, Sn_TXBUF_SIZE, size);
    }
}

/* ================================================================
 *  Socket Operations
 * ================================================================ */

uint8_t w5500_socket_init(uint8_t sock, uint16_t port)
{
    uint8_t block = W5500_BS_SOCK(sock);
    uint8_t status;
    int t;

    /* Set TCP mode */
    w5500_write_byte(block, Sn_MR, Sn_MR_TCP);

    /* Set source port */
    w5500_write_word(block, Sn_PORT, port);

    /* Enable TCP Keep-Alive BEFORE OPEN: 5s unit × 3 = 15s interval */
    w5500_write_byte(block, Sn_KPALVTR, 0x03);

    /* Open socket */
    w5500_write_byte(block, Sn_CR, Sn_CR_OPEN);

    /* Wait 2ms for command to take effect before first read */
    HAL_Delay(2);

    /* Poll until SOCK_INIT (max 100ms) */
    for (t = 0; t < 100; t++) {
        status = w5500_read_byte(block, Sn_SR);
        if (status == SOCK_INIT) break;
        HAL_Delay(1);
    }

    return status;
}

uint8_t w5500_socket_listen(uint8_t sock)
{
    uint8_t block = W5500_BS_SOCK(sock);
    uint8_t status;
    int t;

    w5500_write_byte(block, Sn_CR, Sn_CR_LISTEN);

    /* Poll until SOCK_LISTEN (max 100ms) */
    for (t = 0; t < 100; t++) {
        status = w5500_read_byte(block, Sn_SR);
        if (status == SOCK_LISTEN) break;
        HAL_Delay(1);
    }

    return status;
}

uint8_t w5500_socket_status(uint8_t sock)
{
    return w5500_read_byte(W5500_BS_SOCK(sock), Sn_SR);
}

void w5500_socket_close(uint8_t sock)
{
    uint8_t block = W5500_BS_SOCK(sock);
    uint8_t sr = w5500_read_byte(block, Sn_SR);
    int t;

    /* Only close if not already closed */
    if (sr == SOCK_CLOSED) return;

    w5500_write_byte(block, Sn_CR, Sn_CR_CLOSE);
    /* Wait up to 200ms for CLOSED state */
    for (t = 0; t < 200; t++) {
        sr = w5500_read_byte(block, Sn_SR);
        if (sr == SOCK_CLOSED) break;
        HAL_Delay(1);
    }
}

void w5500_socket_discon(uint8_t sock)
{
    uint8_t block = W5500_BS_SOCK(sock);
    w5500_write_byte(block, Sn_CR, Sn_CR_DISCON);
}

uint8_t w5500_socket_ir(uint8_t sock)
{
    return w5500_read_byte(W5500_BS_SOCK(sock), Sn_IR);
}

void w5500_socket_ir_clear(uint8_t sock, uint8_t mask)
{
    w5500_write_byte(W5500_BS_SOCK(sock), Sn_IR, mask);
}

/* ================================================================
 *  Data Transfer
 * ================================================================ */

uint8_t w5500_socket_send(uint8_t sock, const uint8_t *data, uint16_t len)
{
    uint8_t block = W5500_BS_SOCK(sock);
    uint16_t tx_fsr = 0;
    uint16_t tx_wr;
    uint16_t phy_addr;
    uint32_t timeout;
    uint8_t  sr;

    /* ---- Guard: never touch a socket that is not actually up ----
     * Sn_TX_FSR never becomes ready on a CLOSED socket, so without this check
     * the wait loop below burns its full timeout on every single call. This
     * function is called from a 500 Hz streaming loop, i.e. up to ~2 s of
     * blocking per frame — enough to stall the main loop until the device
     * stops answering Modbus requests altogether. */
    sr = w5500_read_byte(block, Sn_SR);
    if (sr != SOCK_ESTABLISHED && sr != SOCK_CLOSE_WAIT) {
        return 0;
    }

    /* Check free TX buffer space (bounded — see the guard above) */
    timeout = 100;
    do {
        tx_fsr = w5500_read_word(block, Sn_TX_FSR);
        if (tx_fsr >= len) break;
        HAL_Delay(1);
    } while (--timeout);

    if (tx_fsr < len) {
        return 0;  /* TX buffer full */
    }

    /* Read current TX write pointer */
    tx_wr = w5500_read_word(block, Sn_TX_WR);

    /* Calculate physical TX buffer address and write data */
    phy_addr = tx_wr;
    w5500_write_buf(W5500_BS_TX_BUF(sock), phy_addr, data, len);

    /* Update TX write pointer */
    w5500_write_word(block, Sn_TX_WR, tx_wr + len);

    /* Issue SEND command */
    w5500_write_byte(block, Sn_CR, Sn_CR_SEND);

    /* Wait for SEND_OK (bounded — a dying link must not block the caller) */
    timeout = 100;
    while (--timeout) {
        uint8_t ir = w5500_read_byte(block, Sn_IR);
        if (ir & Sn_IR_SEND_OK) {
            w5500_write_byte(block, Sn_IR, Sn_IR_SEND_OK);  /* Clear flag */
            return 1;
        }
        if (ir & Sn_IR_TIMEOUT) {
            w5500_write_byte(block, Sn_IR, Sn_IR_TIMEOUT);
            return 0;
        }
        HAL_Delay(1);
    }

    return 0;
}

uint16_t w5500_socket_recv_size(uint8_t sock)
{
    return w5500_read_word(W5500_BS_SOCK(sock), Sn_RX_RSR);
}

uint16_t w5500_socket_recv(uint8_t sock, uint8_t *buf, uint16_t len)
{
    uint8_t block = W5500_BS_SOCK(sock);
    uint16_t rx_rsr;
    uint16_t rx_rd;
    uint16_t phy_addr;
    uint16_t read_len;

    rx_rsr = w5500_read_word(block, Sn_RX_RSR);
    if (rx_rsr == 0) {
        return 0;
    }

    read_len = (rx_rsr < len) ? rx_rsr : len;

    /* Read current RX read pointer */
    rx_rd = w5500_read_word(block, Sn_RX_RD);

    /* Calculate physical RX buffer address and read data */
    phy_addr = rx_rd;
    w5500_read_buf(W5500_BS_RX_BUF(sock), phy_addr, buf, read_len);

    /* Update RX read pointer */
    w5500_write_word(block, Sn_RX_RD, rx_rd + read_len);

    /* Issue RECV command to free buffer */
    w5500_write_byte(block, Sn_CR, Sn_CR_RECV);

    return read_len;
}
