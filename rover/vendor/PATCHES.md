# Local changes to the vendored DYNAMIXEL SDK

`dynamixel_sdk/` is **DynamixelSDK 4.0.5**
(`python/src/dynamixel_sdk`, Apache 2.0) with one local change.
Reapply or verify this change whenever replacing the SDK.

In `port_handler.py`, `getCFlagBaud()` accepts any positive baud rate so
pyserial/the driver can select the vehicle's 4.5 Mbps (`4500000`).
The upstream whitelist omitted that rate, although the XH430 supports it.

```python
def getCFlagBaud(self, baudrate):
    return baudrate if baudrate > 0 else -1
```

Always check the return value of `setBaudRate()`. A rejected rate returns
`False` and leaves the previous port speed unchanged; accepting a positive
value here does not guarantee the driver can set it.
