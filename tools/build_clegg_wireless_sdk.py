"""Build standalone Pico SDK Clegg Wi-Fi firmware; credentials stay ignored.

Pico SDK and an ARM GCC toolchain are supplied explicitly; no upload occurs.
"""
import argparse
import hashlib
import json
import re
import shutil
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'rover'))
import config


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--sdk', required=True)
    ap.add_argument('--toolchain', required=True)
    ap.add_argument('--build-dir', required=True)
    ap.add_argument('--cmake', default='cmake')
    ap.add_argument('--ninja', default='ninja')
    ap.add_argument('--credentials', type=Path, required=True)
    args = ap.parse_args()
    build = Path(args.build_dir).resolve()
    assert build.is_relative_to((ROOT / '.venv').resolve()), 'Private firmware must remain in ignored .venv'
    assert args.credentials.name.endswith('.local.md')
    build.mkdir(parents=True, exist_ok=True)
    credentials=args.credentials.read_text(encoding='utf-8-sig')
    ssid=re.search(r'(?m)^SSID: (.+)$',credentials)[1].strip()
    password=re.search(r'(?m)^Password: (.+)$',credentials)[1].strip()
    (build/'wifi_secrets.h').write_text('#define CLEGG_WIFI_SSID '+json.dumps(ssid)+'\n#define CLEGG_WIFI_PASSWORD '+json.dumps(password)+'\n',encoding='utf8')
    source = ROOT / 'firmware/clegg_wireless_sdk'
    values = {k: v for k, v in vars(config).items() if k.startswith('CLEGG_')}
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in (source / 'clegg_wireless_sdk.c', source / 'nina.c', source / 'nina.h', source / 'CMakeLists.txt', ROOT / 'rover/config.py')}
    identity = hashlib.sha256(json.dumps({'values': values, 'sources': hashes}, sort_keys=True).encode()).hexdigest()
    header = ''.join(f'#define {k} {json.dumps(v)}\n' for k, v in sorted(values.items()))
    (build / 'clegg_config.h').write_text(header + f'#define CLEGG_SOURCE_ID "{identity}"\n', encoding='utf-8')
    (build / 'source_manifest.json').write_text(json.dumps(dict(source_id=identity, sources=hashes, config=values), indent=2), encoding='utf-8')
    cmake = shutil.which(args.cmake) or args.cmake
    ninja = shutil.which(args.ninja) or args.ninja
    subprocess.run([cmake, '-S', str(source), '-B', str(build), '-G', 'Ninja',
                    '-DCMAKE_MAKE_PROGRAM=' + str(Path(ninja).resolve()),
                    '-DPICO_SDK_PATH=' + str(Path(args.sdk).resolve()),
                    '-DPICO_TOOLCHAIN_PATH=' + str(Path(args.toolchain).resolve()),
                    '-DPICO_BOARD=arduino_nano_rp2040_connect', '-DPICO_NO_PICOTOOL=1',
                    '-DCLEGG_GENERATED_DIR=' + str(build)], check=True)
    subprocess.run([cmake, '--build', str(build), '--parallel', '4'], check=True)


if __name__ == '__main__':
    main()
