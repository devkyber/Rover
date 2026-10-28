#ifndef SCOUT_USB_TX_H
#define SCOUT_USB_TX_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum { SCOUT_USB_TX_MAX = 1536 + 16 };
typedef struct {
    uint8_t bytes[SCOUT_USB_TX_MAX];
    size_t size, offset;
    unsigned samples;
} scout_usb_tx_t;
typedef int (*scout_usb_write_t)(const uint8_t *data,size_t size,void *context);
typedef enum {
    SCOUT_USB_TX_IDLE, SCOUT_USB_TX_BUSY, SCOUT_USB_TX_PROGRESS,
    SCOUT_USB_TX_DONE, SCOUT_USB_TX_ERROR
} scout_usb_tx_result_t;

void scout_usb_tx_reset(scout_usb_tx_t *tx);
bool scout_usb_tx_pending(const scout_usb_tx_t *tx);
bool scout_usb_tx_stage(scout_usb_tx_t *tx,const void *frame,size_t size,unsigned samples);
/* Only entirely unsent telemetry may yield to a newly arrived control reply. */
unsigned scout_usb_tx_discard_samples(scout_usb_tx_t *tx);
/* Exactly one write attempt; 0 means backpressure and preserves every pending byte. */
scout_usb_tx_result_t scout_usb_tx_step(scout_usb_tx_t *tx,scout_usb_write_t write,void *context);
#endif
