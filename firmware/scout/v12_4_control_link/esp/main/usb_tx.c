#include "usb_tx.h"
#include <string.h>

void scout_usb_tx_reset(scout_usb_tx_t *tx)
{ tx->size=tx->offset=0; tx->samples=0; }
bool scout_usb_tx_pending(const scout_usb_tx_t *tx)
{ return tx->offset<tx->size; }
bool scout_usb_tx_stage(scout_usb_tx_t *tx,const void *frame,size_t size,unsigned samples)
{
    if (!frame || !size || size>sizeof(tx->bytes) || scout_usb_tx_pending(tx)) return false;
    memcpy(tx->bytes,frame,size); tx->size=size; tx->offset=0; tx->samples=samples;
    return true;
}
unsigned scout_usb_tx_discard_samples(scout_usb_tx_t *tx)
{
    if (!scout_usb_tx_pending(tx) || tx->offset || !tx->samples) return 0;
    unsigned samples=tx->samples; scout_usb_tx_reset(tx); return samples;
}
scout_usb_tx_result_t scout_usb_tx_step(scout_usb_tx_t *tx,scout_usb_write_t write,void *context)
{
    if (!scout_usb_tx_pending(tx)) return SCOUT_USB_TX_IDLE;
    if (!write) return SCOUT_USB_TX_ERROR;
    size_t remaining=tx->size-tx->offset;
    int count=write(tx->bytes+tx->offset,remaining,context);
    if (!count) return SCOUT_USB_TX_BUSY;
    if (count<0 || (size_t)count>remaining) return SCOUT_USB_TX_ERROR;
    tx->offset+=(size_t)count;
    if (tx->offset<tx->size) return SCOUT_USB_TX_PROGRESS;
    scout_usb_tx_reset(tx); return SCOUT_USB_TX_DONE;
}
