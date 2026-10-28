#ifndef CLEGG_NINA_H
#define CLEGG_NINA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Board-specific NINA-W102 SPI1 driver. Call only from one core/thread.
 * Integer operations return -1 on transport/protocol error, unless noted.
 * No function logs credentials. No API depends on USB being connected. */
bool nina_init(void);
void nina_set_timeout_ms(uint32_t timeout_ms);
const char *nina_error(void);
uint32_t nina_error_count(void);
bool nina_firmware_version(char *out, size_t capacity);
/* Returns final Wi-Fi status, including 3=connected; -1 means driver error.
 * Keeps polling through old cached status values until connected or timeout. */
int nina_join(const char *ssid, const char *password, uint32_t timeout_ms);
int nina_status(void);
int nina_reason_code(void);
/* Legacy command 0x17: 0 requests WIFI_PS_NONE, 1 requests modem sleep.
 * ACK confirms command handling, not an independent power-state readback. */
bool nina_set_power_mode(uint8_t mode);
bool nina_disconnect(void);
bool nina_local_ip(uint8_t ip[4], uint8_t subnet[4]);
/* Raw IPv4 octets; configure before join. Verifies IP/mask readback because
 * NINA 1.4.8 does not define the SET_IP_CONFIG acknowledgement value. */
bool nina_config(const uint8_t ip[4], const uint8_t gateway[4], const uint8_t subnet[4]);
/* Allocates/listens and returns server socket, or -1. Port is host order. */
int nina_server_begin(uint16_t port);
/* Returns a client socket, 255 when none is ready, -1 on driver error.
 * Legacy 1.4.8 AVAIL_DATA command; caller sends INFO to an accepted client. */
int nina_server_client(uint8_t server_socket);
int nina_client_connected(uint8_t socket);
int nina_available(uint8_t socket);
/* recv is nonblocking with respect to TCP data: 0 when no bytes are ready.
 * Every SPI/ACK operation has a deadline. Read at most 1536 bytes per call. */
int nina_recv(uint8_t socket, void *data, size_t capacity);
/* Sends one block of at most 1536 bytes; returns accepted byte count or -1.
 * A short/failed write must terminate this TCP session: do not blindly retry. */
int nina_send(uint8_t socket, const void *data, size_t length);
bool nina_stop(uint8_t socket);
/* Legacy UDP APIs; local_port=0 requests an ephemeral local port. */
int nina_udp_begin(uint16_t local_port);
bool nina_udp_send(uint8_t socket, const uint8_t destination[4], uint16_t port,
                   const void *data, size_t length);
/* Scan result is bounded to 10 SSIDs, matching WiFiNINA's public limit. */
int nina_scan(char ssids[][33], size_t capacity);

#ifdef __cplusplus
}
#endif
#endif
