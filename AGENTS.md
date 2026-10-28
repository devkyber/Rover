# Working on Rover

Read README.md for operation, docs/README.md for the compact vehicle handbook,
and docs/05_한계와_로드맵.md for frozen limits. Inspect the working tree before
editing and preserve unrelated changes. Keep Korean operator instructions short,
use English UI labels, and keep the existing driving dashboard.

One owner per fact: runtime constants in rover/config.py; equations/update order
in rover/eskf.py and rover/odometry.py; task IDs/status in the roadmap; navigation
in docs/README.md. Keep installation/network recovery in docs/11_jetson_bringup.md,
hardware/wiring in docs/06_하드웨어.md, data/mission contracts in
docs/13_scout_integration.md, and useful derivations/worked examples in docs/01.
Do not recreate separate generic Python lessons or duplicate subject guides.
Keep rover/learn_turn_angle.py as the hardware-free production-predictor example.
The sibling PCB project owns CAD/BOM/pinout; enter through the hardware guide.

Keep only material needed to resume or expand the actual vehicle: current source,
firmware/build/setup, selected actual-motion originals with labels/metadata and
source/config identity, final settings and minimal installation provenance.
Remove unused history, bench/synthetic captures and redundant reports instead of
moving them to another archive. A command or rotating wheel alone does not prove
chassis motion. Never overwrite retained originals or relabel tuning/synthetic
results as independent field validation. Keep originals in logs/ and necessary
results/methods in data/. Passwords and credential-bearing firmware belong only
in ignored *.local.md, private/, or the credential store.

Run python tools/check_docs.py after documentation/layout edits. Other software
checks should match the changed behavior; historical suites are not field startup.
Hardware operation/deployment needs task authorization. Rover software changes
include deployment to Jetson, installed-file verification, and restarting the
existing runtime when needed. Preserve its mode and unrelated remote changes;
keep the final backup/receipt and report restart success. Deploy directly without
a separate validation run unless requested. Actuator motion tests require a
specific task request. Historical receipts do not prove the current hardware state.
