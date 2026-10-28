#!/usr/bin/env bash
set -euo pipefail
TASK_REPOSITORY_DIR="$(cd "$(dirname "$0")/../../../.." && pwd)"
TASK_PROJECT_DIR="$TASK_REPOSITORY_DIR/firmware/scout/v12_4_control_link/esp"
TASK_BUILD_DIR="$TASK_REPOSITORY_DIR/output/scout_firmware_build/v12_4_control_link_wifi"
TASK_NATIVE_ROOT="${SCOUT_NATIVE_BUILD_ROOT:-$HOME/.cache/rover-scout-builds/v12_4_control_link_wifi}"
TASK_NATIVE_SOURCE_DIR="$TASK_NATIVE_ROOT/source"
TASK_NATIVE_BUILD_DIR="$TASK_NATIVE_ROOT/build"
if [[ -z "${IDF_PATH:-}" ]]; then
    TASK_SDK_DIR="${SCOUT_SDK_DIR:-$HOME/.cache/rover-scout-sdk-v5.5.3}"
    export IDF_TOOLS_PATH="${IDF_TOOLS_PATH:-$TASK_SDK_DIR/idf-tools}"
    export IDF_PYTHON_ENV_PATH="${IDF_PYTHON_ENV_PATH:-$TASK_SDK_DIR/python-env}"
    source "$TASK_SDK_DIR/export.sh"
fi
mkdir -p "$TASK_BUILD_DIR" "$TASK_NATIVE_SOURCE_DIR" "$TASK_NATIVE_BUILD_DIR"
cp -a "$TASK_PROJECT_DIR/." "$TASK_NATIVE_SOURCE_DIR/"
python - "$TASK_PROJECT_DIR" "$TASK_NATIVE_SOURCE_DIR" "$TASK_BUILD_DIR/compiled_source.json" <<'PY'
import hashlib, json, pathlib, sys
repository, compiled, output = map(pathlib.Path, sys.argv[1:])
def files(root):
    return {p.relative_to(root).as_posix(): {'sha256': hashlib.sha256(p.read_bytes()).hexdigest(), 'bytes': p.stat().st_size}
            for p in sorted(root.rglob('*')) if p.is_file()}
source_files, compiled_files = files(repository), files(compiled)
if source_files != compiled_files:
    raise SystemExit('Native compiled snapshot does not match repository source')
output.write_text(json.dumps({'schema': 'scout_compiled_source_snapshot_v1',
    'repository_source_directory': str(repository), 'compiled_source_directory': str(compiled),
    'repository_and_compiled_files_match': True, 'file_count': len(source_files),
    'files': compiled_files}, indent=2) + '\n')
print(f'Native compiled source snapshot verified: {len(source_files)} files')
PY
cd "$TASK_NATIVE_SOURCE_DIR"
idf.py -B "$TASK_NATIVE_BUILD_DIR" -D IDF_TARGET=esp32s3 \
    -D "SDKCONFIG=$TASK_NATIVE_BUILD_DIR/sdkconfig" \
    -D "SDKCONFIG_DEFAULTS=$TASK_NATIVE_SOURCE_DIR/sdkconfig.defaults;$TASK_NATIVE_SOURCE_DIR/sdkconfig.defaults.wifi" build
cp "$TASK_NATIVE_BUILD_DIR/jumping_scout.bin" "$TASK_BUILD_DIR/"
cp "$TASK_NATIVE_BUILD_DIR/sdkconfig" "$TASK_BUILD_DIR/"
cp "$TASK_NATIVE_BUILD_DIR/compile_commands.json" "$TASK_BUILD_DIR/"
cp "$TASK_NATIVE_BUILD_DIR/partition_table/partition-table.bin" "$TASK_BUILD_DIR/"
