# Clegg final firmware

The Nano RP2040 Connect streams ADXL375 samples through its NINA Wi-Fi module.
The four C/CMake files in this directory are the maintained firmware source.
Host settings are owned by [rover/config.py](../../rover/config.py).
Installed component identities are recorded in [the final release](../../CONTEST_RELEASE.json).

The last recorded installed image has source ID
`1787a6485cea4a617bd7d116f405a1edad19667d77ad2944da0507e758b4d2be`
and UF2 SHA256 `49a3dd044def0855ad8b29c82d2af6075df160bf17dcfc9a1036c82ffb4c09c4`.
The current four source files and `CLEGG_*` values matched that installation snapshot;
other `config.py` contents have since changed, so a rebuild receives a new source ID.

## Build

Use Pico SDK 2.2.0 at commit `a1438dff1d38bd9c65dbd693f0e5db4b9ae91779`
with its required submodules, Arduino ARM GCC `7-2017q4`,
CMake 4.4.4 and Ninja 1.13.2. The recorded board is
`arduino_nano_rp2040_connect`; `PICO_NO_PICOTOOL=1` is selected by the builder.
The SDK and compiler are external installations, independent of the Python venv.
The original compiler location was Arduino15's
`packages/arduino/tools/arm-none-eabi-gcc/7-2017q4`.

Create the [laptop environment](../../docs/11_jetson_bringup.md#laptop-environment), then run
from the repository root with actual SDK/compiler paths:

```powershell
.venv/Scripts/python.exe tools/build_clegg_wireless_sdk.py --sdk C:/Tools/pico-sdk --toolchain C:/Tools/arm-none-eabi-gcc/7-2017q4 --build-dir .venv/clegg-firmware --credentials clegg_wifi.local.md
```

The credential file contains `SSID: ...` and `Password: ...` lines.
The builder generates configuration and `source_manifest.json`, then builds without
uploading. Generated headers, ELF/BIN/UF2 and images containing Wi-Fi credentials
must stay in ignored local storage. Record their SHA256 and source/configuration
identity without publishing credentials. A new build is not assumed byte-identical.

## Restore the Nano

A firmware write requires an explicit hardware task. Use the verified private UF2
whose hash matches the final installation record, or build and identify a new image.
If only ELF is available, the recorded conversion tool was Arduino `rp2040tools`
1.0.6 `elf2uf2`. Enter the Nano's RP2040 USB bootloader, independently identify the
`RPI-RP2` volume using its `INFO_UF2.TXT`, and copy the UF2 to that volume. Do not
assume the old drive letter. Keep a receipt with the source ID and image SHA256,
then confirm firmware identity/status before any mechanical operation.

Operating commands are in [README](../../README.md); wiring and receiver setup
are in [hardware](../../docs/06_하드웨어.md#clegg-가속도-무선-계측).
