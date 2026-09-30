#ifndef __MODBUS_TCP_H__
#define __MODBUS_TCP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ================================================================
 *  Modbus-TCP Protocol Engine
 *
 *  Usage flow in main loop per connected socket:
 *    if (ir & Sn_IR_RECV) {
 *        do {
 *            rx_len = w5500_socket_recv_size(sock);
 *            if (rx_len == 0) break;
 *            w5500_socket_recv(sock, rx_buf, rx_len);
 *            modbus_tcp_process_stream(sock, rx_buf, rx_len, send_cb, ctx);
 *        } while (rx_len == sizeof(rx_buf));
 *        w5500_socket_ir_clear(sock, Sn_IR_RECV);
 *    }
 *
 *  NOTE: use modbus_tcp_process_stream(), not modbus_tcp_process().
 *  TCP is a byte stream, so one recv() may deliver 0..N complete requests
 *  glued together. Processing only the first one silently discards the rest.
 * ================================================================ */

#define MB_TCP_RX_BUF_SIZE  260   /* max ADU: 6 MBAP + 1 FC + 253 data */
#define MB_TCP_TX_BUF_SIZE  260

/** Callback type: the user provides a way to send response bytes */
typedef void (*mb_tcp_send_fn)(uint8_t sock, const uint8_t *data,
                               uint16_t len, void *ctx);

/**
 * Process one Modbus-TCP request.
 * @param sock    Socket number (0..3), opaque to Modbus engine
 * @param rx_buf  Raw TCP payload (MBAP header + PDU)
 * @param rx_len  Number of bytes available
 * @param send_fn User's send function
 * @param ctx     Opaque context passed through to send_fn
 * @return        0 on success, <0 on MBAP parse error,
 *                >0 on exception (exception code)
 */
int modbus_tcp_process(uint8_t sock, const uint8_t *rx_buf, uint16_t rx_len,
                       mb_tcp_send_fn send_fn, void *ctx);

/**
 * Process every complete Modbus-TCP request contained in a receive buffer.
 *
 * TCP is a byte stream: a single recv() can return several requests stuck
 * together ("sticky packets"). This function walks the buffer using each
 * frame's MBAP LEN field and calls modbus_tcp_process() once per complete
 * frame, so no request is dropped. A trailing partial frame is left
 * unconsumed (the caller's buffer is not retained — keep buffers sized to
 * MB_TCP_RX_BUF_SIZE so a whole ADU always fits).
 *
 * @return number of requests processed (0 if the buffer held no complete frame)
 */
int modbus_tcp_process_stream(uint8_t sock, const uint8_t *rx_buf, uint16_t rx_len,
                              mb_tcp_send_fn send_fn, void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_TCP_H__ */
