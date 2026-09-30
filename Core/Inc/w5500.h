#ifndef __W5500_H__
#define __W5500_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ================================================================
 *  W5500 Common Register Addresses
 * ================================================================ */
#define W5500_MR         0x0000  /* Mode Register */
#define W5500_GAR        0x0001  /* Gateway Address (4 bytes) */
#define W5500_SUBR       0x0005  /* Subnet Mask (4 bytes) */
#define W5500_SHAR       0x0009  /* Source MAC (6 bytes) */
#define W5500_SIPR       0x000F  /* Source IP (4 bytes) */
#define W5500_IR         0x0015  /* Interrupt Register */
#define W5500_IMR        0x0016  /* Interrupt Mask Register */
#define W5500_RTR        0x0017  /* Retry Time-value (2 bytes) */
#define W5500_RCR        0x0019  /* Retry Count */
#define W5500_PHYCFGR    0x002E  /* PHY Configuration */
#define W5500_VERSIONR   0x0039  /* Version Register (expect 0x04) */

/* ================================================================
 *  W5500 Socket Register Offsets
 * ================================================================ */
#define Sn_MR            0x0000  /* Socket Mode */
#define Sn_CR            0x0001  /* Socket Command */
#define Sn_IR            0x0002  /* Socket Interrupt */
#define Sn_SR            0x0003  /* Socket Status */
#define Sn_PORT          0x0004  /* Source Port (2 bytes) */
#define Sn_DIPR          0x000C  /* Destination IP (4 bytes) */
#define Sn_DPORT         0x0010  /* Destination Port (2 bytes) */
#define Sn_TX_FSR        0x0020  /* TX Free Size (2 bytes) */
#define Sn_TX_WR         0x0024  /* TX Write Pointer (2 bytes) */
#define Sn_RX_RSR        0x0026  /* RX Received Size (2 bytes) */
#define Sn_RX_RD         0x0028  /* RX Read Pointer (2 bytes) */
#define Sn_RXBUF_SIZE    0x001E  /* RX Buffer Size (1KB unit) */
#define Sn_TXBUF_SIZE    0x001F  /* TX Buffer Size (1KB unit) */
#define Sn_KPALVTR       0x002F  /* Keep-Alive Timer (5s unit) */

/* ================================================================
 *  Sn_MR (Socket Mode) values
 * ================================================================ */
#define Sn_MR_CLOSE      0x00
#define Sn_MR_TCP        0x01
#define Sn_MR_UDP        0x02

/* ================================================================
 *  Sn_CR (Socket Command) values
 * ================================================================ */
#define Sn_CR_OPEN       0x01
#define Sn_CR_LISTEN     0x02
#define Sn_CR_CONNECT    0x04
#define Sn_CR_DISCON     0x08
#define Sn_CR_CLOSE      0x10
#define Sn_CR_SEND       0x20
#define Sn_CR_SEND_MAC   0x21
#define Sn_CR_SEND_KEEP  0x22
#define Sn_CR_RECV       0x40

/* ================================================================
 *  Sn_SR (Socket Status) values
 * ================================================================ */
#define SOCK_CLOSED      0x00
#define SOCK_INIT        0x13
#define SOCK_LISTEN      0x14
#define SOCK_ESTABLISHED 0x17
#define SOCK_CLOSE_WAIT  0x1C

/* ================================================================
 *  Sn_IR (Socket Interrupt) bits
 * ================================================================ */
#define Sn_IR_CON        0x01  /* Connect */
#define Sn_IR_DISCON     0x02  /* Disconnect */
#define Sn_IR_RECV       0x04  /* Data Received */
#define Sn_IR_TIMEOUT    0x08  /* Timeout */
#define Sn_IR_SEND_OK    0x10  /* Send OK */

/* ================================================================
 *  W5500 Control Byte Helpers (VDM mode)
 *  BS values are 5-bit raw (will be shifted <<3 in spi_send_address_control)
 *  Interleaved layout per socket n: Reg=1+4n, TX=2+4n, RX=3+4n
 *  BS=0x00=Common, BS=0x01=Sock0 Reg, BS=0x02=Sock0 TX, BS=0x03=Sock0 RX,
 *  BS=0x04=Sock1 Reg, BS=0x05=Sock1 TX, BS=0x06=Sock1 RX, ...
 * ================================================================ */
#define W5500_BS_COMMON    0x00
#define W5500_BS_SOCK(n)   ((uint8_t)(1 + 4 * (n)))  /* Socket n Reg: BS=1+4n */
#define W5500_BS_TX_BUF(n) ((uint8_t)(2 + 4 * (n)))  /* Socket n TX:  BS=2+4n */
#define W5500_BS_RX_BUF(n) ((uint8_t)(3 + 4 * (n)))  /* Socket n RX:  BS=3+4n */

/* ================================================================
 *  W5500 Socket Count & Buffer Sizing (hardware: 32KB total)
 * ================================================================ */
#define W5500_SOCK_COUNT      4     /* Sockets used: 0-3 */
#define W5500_SOCK_BUF_KB     2     /* TX+RX buffer per socket (KB) */
/* Upper bound of a valid Sn_RX_RSR reading. A larger value can only be a
 * torn read of that register (the W5500 updates it while we shift it out),
 * so w5500_socket_recv() discards it and retries on the next poll. */
#define W5500_SOCK_BUF_MAX    (W5500_SOCK_BUF_KB * 1024)

/* ================================================================
 *  API Functions
 * ================================================================ */

/** Hardware reset and software initialization */
void w5500_hw_reset(void);
uint8_t w5500_init(void);
uint8_t w5500_phy_wait_link(uint32_t timeout_ms);

/** Buffer configuration — MUST be called before any socket OPEN */
void w5500_configure_buffers(void);

/** Network configuration */
void w5500_set_mac(const uint8_t mac[6]);
void w5500_set_ip(const uint8_t ip[4]);
void w5500_set_gateway(const uint8_t gw[4]);
void w5500_set_subnet(const uint8_t sn[4]);

/** Socket operations */
uint8_t w5500_socket_init(uint8_t sock, uint16_t port);
uint8_t w5500_socket_listen(uint8_t sock);
uint8_t w5500_socket_status(uint8_t sock);
void w5500_socket_close(uint8_t sock);
void w5500_socket_discon(uint8_t sock);
uint8_t w5500_socket_ir(uint8_t sock);
void w5500_socket_ir_clear(uint8_t sock, uint8_t mask);

/** Data transfer */
uint8_t w5500_socket_send(uint8_t sock, const uint8_t *data, uint16_t len);
uint16_t w5500_socket_recv_size(uint8_t sock);
uint16_t w5500_socket_recv(uint8_t sock, uint8_t *buf, uint16_t len);

/** Low-level register access (may also be used externally) */
uint8_t w5500_read_byte(uint8_t block, uint16_t addr);
void w5500_write_byte(uint8_t block, uint16_t addr, uint8_t data);
uint16_t w5500_read_word(uint8_t block, uint16_t addr);
void w5500_write_word(uint8_t block, uint16_t addr, uint16_t data);
void w5500_read_buf(uint8_t block, uint16_t addr, uint8_t *buf, uint16_t len);
void w5500_write_buf(uint8_t block, uint16_t addr, const uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __W5500_H__ */
