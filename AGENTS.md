# Working on Rover

Read README.md for operation and docs/05_한계와_로드맵.md for status.
Inspect the working tree before editing; preserve unrelated changes.
Keep Korean operator instructions concise in the root README, using English UI
labels. Keep the driving dashboard; do not restore the separate manual website.

One owner per fact: runtime constants in rover/config.py; equations/update order
in rover/eskf.py and rover/odometry.py; task status/IDs in the roadmap; navigation
in docs/README.md; essential setup in the existing subject guide. The sibling
PCB project owns CAD/BOM/pinout; enter through docs/electronics/PDB_Design_Handover.md.

Keep useful records INSIDE this repository. logs/ contains original measurements
and labels; data/ contains results/methods; data/archive/ and docs/archive/ retain
older evidence with SHA256 manifests. Keep current reference text concise. Learning docs 01/02/08/09 and
rover/learn_turn_angle.py are intentionally retained; preserve useful derivations
and worked examples, while daily operating commands stay in the root README.
Never bulk-delete recordings or archived evidence. Preserve CSV metadata and
source/configuration identity; current code may not reproduce historical results.
Do not relabel tuning data as unseen validation or synthetic passes as field proof.
Passwords belong only in ignored *.local.md or the credential store.

Run python tools/check_docs.py after documentation/layout edits. Use relevant
hardware-free checks; historical unit-test suites are archived, not part of field
startup. Hardware operation/deployment needs task authorization. Do not run old
archive scripts against live hardware or overwrite old outputs when investigating.
