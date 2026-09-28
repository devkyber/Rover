# Local changes to the vendored DYNAMIXEL SDK

The SDK in `dynamixel_sdk/` is a copy of **DynamixelSDK 4.0.5**
(`python/src/dynamixel_sdk`, Apache 2.0). It is NOT pristine.

**If you ever replace this folder with a newer SDK, re-apply these.**

---

## 1. `port_handler.py` — `getCFlagBaud()` accepts any baud rate

**Upstream:**

```python
def getCFlagBaud(self, baudrate):
    if baudrate in [9600, ..., 3500000, 4000000]:
        return baudrate
    else:
        return -1
```

**Problem.** `4500000` is missing from that list, even though the XH430
control table supports it as Baud Rate(8) value 7. The call chain is:

```
setBaudRate(4500000) -> getCFlagBaud() -> -1 -> return False
```

`setBaudRate()` returns **False and does nothing else** — the port keeps
its previous speed. Callers that ignore the return value (which is easy,
because failure is silent) then transmit at the wrong rate and get zero
responses from motors that are perfectly healthy.

This cost us a debugging session: DYNAMIXEL Wizard saw both motors at
4.5 Mbps while our Python saw nothing at any rate.

**Fix.** Accept any positive rate and let pyserial/the driver decide:

```python
def getCFlagBaud(self, baudrate):
    return baudrate if baudrate > 0 else -1
```

The FTDI chip in the U2D2 synthesises non-standard rates from a divisor,
which is exactly how Wizard reaches 4.5 Mbps. Upstream is aware the
handling is incomplete — the caller carries a `# TODO: setCustomBaudrate`
comment.

**Regardless of this patch, always check what `setBaudRate()` returns.**
