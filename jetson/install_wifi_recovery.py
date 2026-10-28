#!/usr/bin/env python3
"""Install the authorized Rover Wi-Fi helper upgrade as root.

The saved NetworkManager profile is inspected and backed up, never modified.
Credential backups stay root-only on Jetson. The installer checks the helper
without recovery, then enables the timer; its first check is delayed after
activation. Later timer checks may reconnect a failed field link. Explicit
network-maintenance scope is required before running this installer.
"""
from __future__ import annotations

import argparse
import ast
import configparser
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import socket
import stat
import subprocess
import tempfile


SETTINGS_PATH = Path("/etc/rover/wifi-recovery.json")
ENVIRONMENT_PATH = Path("/etc/default/rover-wifi-recover")
UNITS = ("rover-wifi-recover.service", "rover-wifi-recover.timer")
POLICY_KEYS = {
    "grace_s", "cooldown_s", "failed_checks", "ping_count", "ping_timeout_s",
    "tcp_timeout_s", "router_tcp_port", "activation_timeout_s",
    "deactivation_timeout_s", "diagnostic_timeout_s", "timer_boot_s",
    "timer_interval_s", "service_timeout_s",
}


def command(*args: str) -> str:
    result = subprocess.run(args, check=True, text=True, capture_output=True,
                            timeout=100, env={**os.environ, "LC_ALL": "C"})
    return result.stdout.strip()


def sha(path: Path) -> str | None:
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def atomic_write(path: Path, content: bytes, mode: int = 0o644) -> None:
    """A failed single-file write leaves its previous bytes intact."""
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.is_symlink() or any(parent.is_symlink() for parent in path.parents):
        raise RuntimeError(f"Refusing a symlink installation target: {path}")
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary_path = Path(temporary)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(content)
            stream.flush()
            os.fchmod(stream.fileno(), mode)
            os.fchown(stream.fileno(), 0, 0)
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
        directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        temporary_path.unlink(missing_ok=True)


def load_settings(repo_root: Path) -> tuple[dict, Path]:
    """Read the literal policy without importing any hardware/runtime code."""
    tree = ast.parse((repo_root / "rover/config.py").read_text(encoding="utf-8"))
    values = [node.value for node in tree.body if isinstance(node, ast.Assign)
              and any(isinstance(target, ast.Name) and target.id == "WIFI_RECOVERY"
                      for target in node.targets)]
    if len(values) != 1:
        raise RuntimeError("Expected one literal WIFI_RECOVERY policy in rover/config.py")
    policy = ast.literal_eval(values[0])
    if not isinstance(policy, dict) or set(policy) != POLICY_KEYS | {"record_relative_dir"}:
        raise RuntimeError("WIFI_RECOVERY policy has unexpected or missing keys")
    for key in POLICY_KEYS:
        value = policy[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0 < value < 100000:
            raise RuntimeError(f"Invalid positive policy value: {key}")
    for key in {"failed_checks", "ping_count", "ping_timeout_s", "router_tcp_port"}:
        if not isinstance(policy[key], int):
            raise RuntimeError(f"Policy value must be an integer: {key}")
    if policy["router_tcp_port"] > 65535:
        raise RuntimeError("Router TCP port exceeds 65535")
    if policy["service_timeout_s"] <= policy["activation_timeout_s"] + policy["deactivation_timeout_s"]:
        raise RuntimeError("Service timeout must leave time for probes and reconnect")
    relative = policy.pop("record_relative_dir")
    if not isinstance(relative, str) or not relative or Path(relative).is_absolute() or ".." in Path(relative).parts:
        raise RuntimeError("Recovery records must use a repository-relative path")
    record_dir = (repo_root / relative).resolve()
    if not record_dir.is_relative_to(repo_root.resolve()):
        raise RuntimeError("Recovery record directory escapes the repository")
    return {"schema_version": 1, **policy, "record_dir": str(record_dir)}, record_dir


def unit_path(path: Path) -> str:
    """Quote systemd path tokens while preserving literal percent characters."""
    value = str(path)
    if "\n" in value or "\r" in value:
        raise RuntimeError("Newlines are not supported in service paths")
    return '"' + value.replace("\\", "\\\\").replace('"', '\\"').replace("%", "%%") + '"'


def render_units(source: Path, settings: dict, record_dir: Path) -> dict[str, bytes]:
    replacements = {
        "@SERVICE_TIMEOUT_S@": str(settings["service_timeout_s"]),
        "@TIMER_BOOT_S@": str(settings["timer_boot_s"]),
        "@TIMER_INTERVAL_S@": str(settings["timer_interval_s"]),
        "@RECORD_DIR@": unit_path(record_dir),
    }
    result = {}
    for name in UNITS:
        content = (source / name).read_text(encoding="utf-8")
        for marker, value in replacements.items():
            content = content.replace(marker, value)
        if re.search(r"@[A-Z_]+@", content):
            raise RuntimeError(f"Unresolved service template marker: {name}")
        result[name] = content.encode("utf-8")
    return result


def inspect_profile(profile_name: str) -> tuple[str, Path]:
    uuid = command("nmcli", "-g", "connection.uuid", "connection", "show", "id", profile_name)
    candidates = []
    for path in Path("/etc/NetworkManager/system-connections").iterdir():
        if not path.is_file() or path.is_symlink():
            continue
        profile = configparser.ConfigParser(interpolation=None)
        try:
            profile.read(path)
        except configparser.Error:
            continue
        if profile.get("connection", "uuid", fallback="") == uuid:
            candidates.append((path, profile))
    if len(candidates) != 1:
        raise RuntimeError("Expected one system keyfile for the selected profile")
    keyfile, profile = candidates[0]
    if not profile.get("wifi-security", "psk", fallback=""):
        raise RuntimeError("A saved system PSK is required; no credential was changed")
    return uuid, keyfile


# This root-only rollback verifies every touched file before restoring any file.
# It never reads or restores the PSK keyfile: installation never modified it.
ROLLBACK = r'''#!/usr/bin/env python3
import argparse,hashlib,json,os,subprocess,tempfile
from pathlib import Path
parser=argparse.ArgumentParser()
parser.add_argument('--restore-timer-on-refusal',action='store_true')
args=parser.parse_args()
base=Path(__file__).resolve().parent
m=json.loads((base/'manifest.json').read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else None
def guard(r):
 p=Path(r['path'])
 if p.is_symlink() or sha(p) not in (r['sha256'],r['desired_sha256']):
  raise RuntimeError('Concurrent file change preserved; rollback refused: '+str(p))
automation_touched=False
try:
 # Reject unknown bytes before stopping automation, then guard each replacement.
 for r in m['files']:guard(r)
 automation_touched=True
 subprocess.run(['systemctl','stop','rover-wifi-recover.timer'],check=True)
 subprocess.run(['systemctl','stop','rover-wifi-recover.service'],check=True)
 for r in m['files']:
  guard(r);p=Path(r['path'])
  if not r['existed']:
   p.unlink(missing_ok=True);continue
  fd,tmp=tempfile.mkstemp(prefix='.'+p.name+'.rollback-',dir=p.parent)
  try:
   with os.fdopen(fd,'wb') as f:
    f.write(Path(r['backup']).read_bytes());f.flush()
    os.fchmod(f.fileno(),r['mode']);os.fchown(f.fileno(),r['uid'],r['gid']);os.fsync(f.fileno())
   guard(r);os.replace(tmp,p)
  finally:
   if os.path.exists(tmp):os.unlink(tmp)
finally:
 # Installation may already have stopped the previous timer before a guard
 # noticed an unrelated edit. Only that explicit install-failure path restores
 # timer state on preflight refusal. A refused manual rollback touches nothing.
 if automation_touched or args.restore_timer_on_refusal:
  subprocess.run(['systemctl','daemon-reload'],check=True)
  enable=['systemctl','enable'] if m['timer_enabled'] in ('enabled','enabled-runtime') else ['systemctl','disable']
  if m['timer_enabled']=='enabled-runtime':enable.append('--runtime')
  subprocess.run(enable+['rover-wifi-recover.timer'],check=True)
  subprocess.run(['systemctl','start' if m['timer_active']=='active' else 'stop','rover-wifi-recover.timer'],check=True)
print('Helper files restored; saved Wi-Fi profile was unchanged.')
'''


def main() -> None:
    import grp  # Linux installer dependency; pure policy/render checks remain portable.

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True)
    parser.add_argument("--profile", default="rover-field")
    parser.add_argument("--backup-name", required=True, help="unique laptop UTC identifier")
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parent.parent)
    parser.add_argument("--receipt-dir", type=Path, required=True)
    parser.add_argument("--expected-installed-manifest", type=Path, required=True)
    args = parser.parse_args()
    if os.geteuid() != 0:
        parser.error("run as root")
    for prerequisite in ("ping", "nmcli", "systemctl", "systemd-analyze", "python3"):
        if shutil.which(prerequisite) is None:
            parser.error(f"Required executable is unavailable: {prerequisite}")
    if not hasattr(socket, "SO_BINDTODEVICE"):
        parser.error("Python SO_BINDTODEVICE support is required for interface-bound probes")
    for value in (args.interface, args.profile, args.backup_name):
        if not re.fullmatch(r"[A-Za-z0-9_.-]+", value):
            parser.error("interface/profile/backup-name must use simple names")
    repo_root = args.repo_root.resolve(strict=True)
    receipt = args.receipt_dir.resolve()
    if not receipt.is_relative_to(repo_root) or receipt.exists():
        parser.error("receipt-dir must be a new directory inside the repository")
    source = Path(__file__).resolve().parent
    settings, record_dir = load_settings(repo_root)
    rendered = render_units(source, settings, record_dir)
    helper_source = (source / "wifi_recover.py").read_bytes()
    compile(helper_source, str(source / "wifi_recover.py"), "exec")
    desired = {Path("/usr/local/lib/rover/wifi_recover.py"): helper_source,
               SETTINGS_PATH: (json.dumps(settings, indent=2) + "\n").encode(),
               ENVIRONMENT_PATH: (f"ROVER_WIFI_INTERFACE={args.interface}\n"
                                  f"ROVER_WIFI_PROFILE={args.profile}\n").encode()}
    desired.update({Path("/etc/systemd/system") / name: content for name, content in rendered.items()})
    expected = json.loads(args.expected_installed_manifest.read_text(encoding="utf-8"))
    if set(expected) != {str(path) for path in desired}:
        raise RuntimeError("Installed-file guard must name every installation target exactly")
    for path in desired:
        if path.is_symlink() or any(parent.is_symlink() for parent in path.parents):
            raise RuntimeError(f"Refusing a symlink installation target: {path}")
        if sha(path) != expected[str(path)]:
            raise RuntimeError(f"Installed file differs from approved baseline: {path}")
    uuid, keyfile = inspect_profile(args.profile)
    owner = repo_root.stat()
    if owner.st_uid == 0:
        raise RuntimeError("Repository must have its normal operator owner")
    group_name = grp.getgrgid(owner.st_gid).gr_name
    backup_root = Path("/var/backups/rover-wifi")
    if backup_root.is_symlink():
        raise RuntimeError("Refusing a symlink backup root")
    backup_root.mkdir(mode=0o700, exist_ok=True)
    os.chmod(backup_root, 0o700)
    backup = backup_root / args.backup_name
    backup.mkdir(mode=0o700)  # Never overwrite earlier credential backups.
    shutil.copy2(keyfile, backup / "saved-profile.keyfile")
    os.chmod(backup / "saved-profile.keyfile", 0o600)
    saved = []
    for index, (path, content) in enumerate(desired.items()):
        record = {"path": str(path), "existed": path.exists(), "sha256": sha(path),
                  "desired_sha256": hashlib.sha256(content).hexdigest()}
        if path.exists():
            record.update(mode=stat.S_IMODE(path.stat().st_mode), uid=path.stat().st_uid,
                          gid=path.stat().st_gid, backup=str(backup / f"original-{index}"))
            shutil.copy2(path, record["backup"])
            os.chmod(record["backup"], 0o600)
            if sha(Path(record["backup"])) != expected[str(path)]:
                raise RuntimeError(f"Installed file changed while backing up: {path}")
        saved.append(record)
    timer_enabled = subprocess.run(["systemctl", "is-enabled", UNITS[1]], capture_output=True, text=True).stdout.strip()
    timer_active = subprocess.run(["systemctl", "is-active", UNITS[1]], capture_output=True, text=True).stdout.strip()
    if timer_enabled not in {"enabled", "enabled-runtime", "disabled"}:
        raise RuntimeError(f"Unsupported original timer enable state: {timer_enabled!r}")
    if timer_active not in {"active", "inactive", "failed"}:
        raise RuntimeError(f"Unsupported original timer active state: {timer_active!r}")
    manifest = {"profile_uuid": uuid, "files": saved, "timer_enabled": timer_enabled,
                "timer_active": timer_active, "profile_modified": False}
    (backup / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    os.chmod(backup / "manifest.json", 0o600)
    (backup / "rollback.py").write_text(ROLLBACK)
    os.chmod(backup / "rollback.py", 0o700)
    receipt.mkdir(parents=True, mode=0o750)
    os.chown(receipt, owner.st_uid, owner.st_gid)
    originals = receipt / "before"
    originals.mkdir(mode=0o750)
    os.chown(originals, owner.st_uid, owner.st_gid)
    for index, record in enumerate(saved):
        if record["existed"]:
            copy = originals / f"original-{index}"
            shutil.copy2(record["backup"], copy)
            os.chmod(copy, 0o640)
            os.chown(copy, owner.st_uid, owner.st_gid)
    record_dir.mkdir(parents=True, mode=0o2750, exist_ok=True)
    os.chown(record_dir, owner.st_uid, owner.st_gid)
    os.chmod(record_dir, 0o2750)
    outcome = {"backup_directory": str(backup), "profile_uuid": uuid,
               "profile_modified": False, "active_recovery_invoked_by_installer": False,
               "record_directory": {"path": str(record_dir), "uid": owner.st_uid,
                                    "group": group_name, "mode": "2750"},
               "before": saved, "installation_complete": False, "rollback_attempted": False}
    try:
        command("systemctl", "stop", UNITS[1])
        command("systemctl", "stop", UNITS[0])
        for path in desired:
            if sha(path) != expected[str(path)]:
                raise RuntimeError(f"Installed file changed during preparation: {path}")
        for path, content in desired.items():
            atomic_write(path, content)
        command("systemd-analyze", "verify", *(str(Path("/etc/systemd/system") / name) for name in UNITS))
        command("systemctl", "daemon-reload")
        inspection = command("python3", "/usr/local/lib/rover/wifi_recover.py", "--settings", str(SETTINGS_PATH),
                             "--interface", args.interface, "--profile", args.profile, "--check")
        outcome["inspection"] = json.loads(inspection)
        outcome["installed_sha256"] = {str(path): sha(path) for path in desired}
        if any(outcome["installed_sha256"][str(path)] != hashlib.sha256(content).hexdigest()
               for path, content in desired.items()):
            raise RuntimeError("Installed file readback differs from desired bytes")
        command("systemctl", "enable", "--now", UNITS[1])
        outcome.update(installation_complete=True,
                       timer_enabled=command("systemctl", "is-enabled", UNITS[1]),
                       timer_active=command("systemctl", "is-active", UNITS[1]))
    except Exception as exc:
        outcome.update(error=f"{type(exc).__name__}: {exc}", rollback_attempted=True)
        result = subprocess.run(["python3", str(backup / "rollback.py"), "--restore-timer-on-refusal"], capture_output=True,
                                text=True, timeout=100)
        outcome["rollback_exit_code"] = result.returncode
        outcome["rollback_stdout"] = result.stdout
        outcome["rollback_stderr"] = result.stderr
        outcome["rollback_files_verified"] = all(sha(Path(r["path"])) == r["sha256"] for r in saved)
        raise
    finally:
        result_path = receipt / "receipt.json"
        result_path.write_text(json.dumps(outcome, indent=2) + "\n")
        os.chmod(result_path, 0o640)
        os.chown(result_path, owner.st_uid, owner.st_gid)
    print(json.dumps(outcome, indent=2))


if __name__ == "__main__":
    main()
