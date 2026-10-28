# Scout firmware

The actual-vehicle source is [v12.4](v12_4_control_link/esp/), with its
[Wi-Fi application and rebuild recipe](releases/v12_4_control_link_wifi_20261024/README.md).
Firmware identity is `scout-process-v12.4-control-link-j5`; the API is `scout-process-v12`.
The final per-board installation identities are in [the release record](../../CONTEST_RELEASE.json).

The current firmware source owns physical output behavior; host settings belong to
[config.py](../../rover/config.py). Operation is in [README](../../README.md),
protocol/recording semantics in the [Scout guide](../../docs/13_scout_integration.md),
and limitations in [JUMPER-01](../../docs/05_한계와_로드맵.md#jumper-01).
Jetson source deployment does not flash a Scout.
