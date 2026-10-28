# Final Scout v12.4 Wi-Fi firmware

This is the final application built from [v12.4 source](../../v12_4_control_link/esp/).
Both status paths identify `scout-process-v12.4-control-link-j5`; the API is
`scout-process-v12`. The application SHA256 is
`293dac4d98e688be1bb56284b71e7195b4cdc073f4c518859e15455426df79d7`.
Installed per-board identities belong to [the final release](../../../../CONTEST_RELEASE.json).

Actuator, protocol and recording contracts are in the [Scout guide](../../../../docs/13_scout_integration.md).

## Rebuild

Use WSL Ubuntu 24.04, ESP-IDF v5.5.3 at commit
`2c211b236707889e8400c4dc5644dd5c4ee071e0`, and
`xtensa-esp-elf-gcc` 14.2.0 (`esp-14.2.0_20251107`). ESP-IDF's required submodules,
Python environment and compiler are installed separately from this repository.
The original SDK directory was `~/.cache/rover-scout-sdk-v5.5.3`.
With that SDK or an activated matching `IDF_PATH`, run from the repository root:

```bash
bash firmware/scout/releases/v12_4_control_link_wifi_20261024/build_wifi.sh
```

`SCOUT_SDK_DIR` selects another installed SDK. The script copies and hashes the
source in native WSL storage before compiling with Wi-Fi defaults and target
`esp32s3`. Build cache defaults to
`~/.cache/rover-scout-builds/v12_4_control_link_wifi/`; generated products go to
ignored `output/scout_firmware_build/v12_4_control_link_wifi/`.

`SOURCE.json`, `compiled_source.json` and `BUILD.json` retain the original
build-time source, compiler/configuration and application identities. Their README
hashes describe that snapshot; documentation was shortened during cleanup while
compiled C/H sources and application bytes stayed unchanged. `SHA256.json` indexes
the retained package files; current handbook hashes are in the
[closure source manifest](../../../../data/final_vehicle_2026-10-28/source_manifest.json).
Verbose compiler commands and build logs are disposable. The retained `sdkconfig`,
partition table and app binary describe this exact build; a new build gets its own identity.

## Restore

Board writing requires a hardware task. Confirm the board MAC/identity and existing
partition table, save the existing app, then write only the application at
`0x10000` (partition size `0x200000`). Preserve NVS, bootloader and sensor Flash.
Read the app bytes back and compare SHA256 with the selected image. This build
package alone does not prove installation or mechanical behavior.
