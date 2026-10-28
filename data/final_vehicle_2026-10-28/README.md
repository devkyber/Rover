# Vehicle handover records

Use [README](../../README.md) to operate the vehicle and
[CONTEST_RELEASE.json](../../CONTEST_RELEASE.json) to identify this retained source.

| Record | Purpose |
|---|---|
| `source_manifest.json` | Core source, firmware and handbook hashes |
| `installation_identity.json` | Last recorded Jetson/board receipts, settings provenance and private Nano image identity |
| `baseline_sources.json` | Known file versions used by the deployment helper to preserve unrelated remote edits |
| `recording_manifest.json` | Original recording hashes and Scout archive member hashes |
| `closure_receipt.json` | Cleanup scope, protected-byte verification and hardware-access outcome |

Installation receipts describe their original dates. The Jetson was unavailable
at closure; this cleanup does not claim a fresh installation or restart.
Original parsed receipts are combined in one JSON with their source hashes and
historical Git paths. These paths identify earlier evidence; old Git history is
not part of the clean upload snapshot. The receipt facts are embedded here.
Runtime settings remain owned by `rover/config.py`.

The credential-bearing Nano UF2 stays in ignored `private/`; credentials stay in
ignored `*.local.md`. A clone needs these separately or a rebuild from retained
firmware source. Recording interpretation is in [the data guide](../../docs/13_scout_integration.md).
