"""Bundle by default; --apply installs, verifies and preserves the current mode.

No firmware is flashed and no motion command is sent. Existing remote changes
must match a known deployment or the captured local baseline before replacement.
All receipts and replaced bytes are retained inside this repository.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tarfile
import uuid

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, value):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write('\n')


def release_paths():
    # One maintained source tree on both hosts. Development checks, historical
    # evidence and build caches stay on the laptop. Board images are reference
    # files only and are never flashed by this tool.
    for folder in ('rover', 'station', 'jetson', 'firmware', 'docs', 'tools'):
        for path in sorted((ROOT / folder).rglob('*')):
            relative = path.relative_to(ROOT)
            if (not path.is_file() or (path.suffix not in ('.py', '.js', '.css', '.html', '.sh', '.ps1', '.md', '.rules', '.service', '.timer', '.c', '.h', '.txt', '.json', '.csv', '.bin', '.defaults', '.wifi', '.usb', '.projbuild')
                    and path.name not in ('sdkconfig', 'CMakeLists.txt'))
                    or any(part in ('__pycache__', 'upstream', 'upstream_v4', 'node_modules', 'archive', 'checks') for part in relative.parts)
                    or folder == 'tools' and path.name.startswith('check_')
                    or path.name.endswith('.local.md')):
                continue
            yield path
    for name in ('README.md', 'AGENTS.md', 'requirements.txt'):
        yield ROOT / name
    for name in ('CONTEST_RELEASE.json', 'data/README.md', 'logs/README.md'):
        if (ROOT / name).is_file():
            yield ROOT / name


# Runs on the Jetson using only its standard library. stdin is JSON, never shell
# interpolation of filenames or runtime arguments.
REMOTE = r'''
import ast,base64,hashlib,json,os,signal,socket,stat,struct,subprocess,tarfile,time,uuid
from pathlib import Path

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def environment(pid):
 values={}
 for item in (Path('/proc')/str(pid)/'environ').read_bytes().split(b'\0'):
  if item:
   key,separator,value=item.partition(b'=')
   if not separator:raise RuntimeError('Invalid process environment')
   values[os.fsdecode(key)]=os.fsdecode(value)
 return values

def environment_sha(values):
 return hashlib.sha256(json.dumps(values,sort_keys=True,ensure_ascii=True).encode()).hexdigest()

def processes():
 result=[]
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try:
   argv=(p/'cmdline').read_bytes().decode().split('\0')[:-1]
   if any(Path(a).name=='rover_main.py' for a in argv):
    result.append(dict(pid=int(p.name),argv=argv,cwd=str((p/'cwd').resolve()),
      executable=str((p/'exe').resolve()),environment_sha256=environment_sha(environment(int(p.name))),
      start_stat=(p/'stat').read_text().rsplit(')',1)[1].split()[19]))
  except (FileNotFoundError,PermissionError,ProcessLookupError):pass
 return sorted(result,key=lambda value:value['pid'])

def target_path(name):
 target=root/name
 if Path(name).is_absolute() or '..' in Path(name).parts or Path(name).as_posix()!=name or not name:
  raise RuntimeError('Unsafe target name')
 if not target.resolve().is_relative_to(root) or target.is_symlink():raise RuntimeError('Target escapes root or is a symlink')
 for parent in target.parents:
  if parent==root:break
  if parent.is_symlink():raise RuntimeError('Target has a symlink parent')
 return target

def identities():
 return {n:sha(target_path(n)) if target_path(n).is_file() else None for n in request['manifest']}

def record(name,value):
 with (receipt/name).open('x') as f:json.dump(value,f,indent=2)

def telemetry(port):
 # A monitor receives telemetry only; it sends no controller/actuator message.
 with socket.create_connection(('127.0.0.1',port),timeout=3) as s:
  key=base64.b64encode(os.urandom(16)).decode()
  s.sendall(('GET /control HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Version: 13\r\nSec-WebSocket-Key: '+key+'\r\n\r\n').encode())
  data=b''
  while b'\r\n\r\n' not in data:
   part=s.recv(4096)
   if not part or len(data)>65536:raise RuntimeError('Incomplete telemetry handshake')
   data+=part
  header,buffer=data.split(b'\r\n\r\n',1)
  if not header.startswith(b'HTTP/1.1 101'):raise RuntimeError('Telemetry handshake failed')
  def read(n):
   nonlocal buffer
   while len(buffer)<n:
    part=s.recv(max(4096,n-len(buffer)))
    if not part:raise RuntimeError('Telemetry socket closed')
    buffer+=part
   result,buffer=buffer[:n],buffer[n:];return result
  for _ in range(20):
   a,b=read(2);size=b&127
   if size==126:size=struct.unpack('!H',read(2))[0]
   elif size==127:size=struct.unpack('!Q',read(8))[0]
   if size>4000000 or b&128:raise RuntimeError('Unexpected telemetry frame')
   body=read(size)
   if a&15==1:
    value=json.loads(body)
    if value.get('type')=='telemetry':return value
  raise RuntimeError('Fresh telemetry unavailable')

def require_idle(t):
 cmd=t.get('cmd')
 if not isinstance(cmd,dict) or any(cmd.get(k) is None or float(cmd[k])!=0 for k in ('forward','turn','lift','pan')):
  raise RuntimeError('Runtime is moving or command state is unknown')
 hammer=t.get('hammer',{})
 if hammer.get('lift_holding') or hammer.get('busy') or hammer.get('active') not in (None,'idle') or t.get('video_recording',{}).get('requested'):
  raise RuntimeError('Finish the active hold, mechanism operation or recording before deployment')
 pico=t.get('pico',{})
 if pico.get('servo_us') and any(float(x)!=0 for x in pico['servo_us']):raise RuntimeError('Pico outputs active')
 for scout in t.get('jumper',{}).get('scouts',{}).values():
  if not isinstance(scout,dict):raise RuntimeError('Scout state unavailable')
  if scout.get('winding') or scout.get('busy') or scout.get('pending_action') or scout.get('wind_input_active'):raise RuntimeError('Scout operation active')
  status=scout.get('manager',{}).get('status',{})
  if status.get('state') not in (None,'idle','stopped','disarmed','estop','fault') or status.get('winding'):
   raise RuntimeError('Scout mechanism is active or loaded')
  if scout.get('connection_status')=='ok' and status.get('state') is None:raise RuntimeError('Scout state unknown')

def stop_process(identity):
 matches=[p for p in processes() if p['pid']==identity['pid']]
 if not matches:return
 if matches!=[identity]:raise RuntimeError('Process identity changed; refusing signal')
 os.kill(identity['pid'],signal.SIGTERM)
 deadline=time.monotonic()+40
 while any(p['pid']==identity['pid'] for p in processes()) and time.monotonic()<deadline:time.sleep(.2)
 if any(p['pid']==identity['pid'] for p in processes()):raise RuntimeError('Graceful stop unconfirmed; no relaunch allowed')

def atomic_write(name,data,mode):
 target=target_path(name);target.parent.mkdir(parents=True,exist_ok=True)
 temporary=target.with_name(target.name+'.'+uuid.uuid4().hex+'.deploy')
 try:
  with temporary.open('xb') as f:f.write(data);f.flush();os.fsync(f.fileno())
  os.chmod(temporary,mode)
  os.replace(temporary,target)
 finally:
  if temporary.exists():temporary.unlink()

def restore_files(before,attempted):
 # Atomic replacement means every touched target must still be old or new.
 # Any third-party change is preserved rather than overwritten during rollback.
 with tarfile.open(receipt/'before.tar') as archive:
  for name in attempted:
   target=target_path(name);digest=sha(target) if target.is_file() else None
   if digest not in (before[name],request['manifest'][name]['sha256']):raise RuntimeError('Remote edit encountered during rollback: '+name)
   if before[name] is None:
    if target.exists():target.unlink()
   else:
    member=archive.getmember(name);data=archive.extractfile(member).read()
    if hashlib.sha256(data).hexdigest()!=before[name]:raise RuntimeError('Backup hash mismatch: '+name)
    atomic_write(name,data,member.mode)
 if identities()!=before:raise RuntimeError('Rollback identity verification failed')

def start_runtime(old,values,console_name):
 # Environment values remain in memory only. PATH preserves a venv selected by
 # a relative argv[0]; process receipts contain only the environment digest.
 with (receipt/console_name).open('ab',buffering=0) as console:
  child=subprocess.Popen(old['argv'],cwd=old['cwd'],env=values,stdin=subprocess.DEVNULL,
                         stdout=console,stderr=subprocess.STDOUT,start_new_session=True)
 time.sleep(5)
 matches=[p for p in processes() if p['pid']==child.pid]
 same=bool(matches) and all(matches[0][key]==old[key] for key in ('argv','cwd','executable','environment_sha256'))
 if child.poll() is None and same:return matches[0]
 # Confirm the failed candidate has exited before restoring/relaunching.
 if child.poll() is None:
  if not matches:raise RuntimeError('Candidate process identity unavailable; cleanup unconfirmed')
  stop_process(matches[0])
 raise RuntimeError('Runtime restart did not preserve executable/environment/arguments or exited')

def apply_release():
 manifest=request['manifest'];before=None;current=[];attempted=[];old=None;values=None;stopped=False
 result=dict(deployment_complete=False,installed_files=0,changed_files=[],firmware_flashed=False,
             extra_remote_files_preserved=True,restart_succeeded=None,rollback_attempted=False,rollback_verified=False)
 try:
  before=identities();current=processes()
  if current!=request['expected_processes'] or before!=request['expected_files']:
   raise RuntimeError('Remote files or runtime changed after inspection; nothing installed')
  if len(current)>1:raise RuntimeError('More than one Rover process; nothing installed')
  stage=receipt/'stage';stage.mkdir()
  with tarfile.open(root/'logs'/'deployments'/(request['release_id']+'.tar')) as archive:
   members=archive.getmembers()
   if len(members)!=len(manifest) or {m.name for m in members}!=set(manifest):raise RuntimeError('Bundle members differ from manifest')
   for member in members:
    if not member.isfile() or not (stage/member.name).resolve().is_relative_to(stage):raise RuntimeError('Unsafe bundle member')
    destination=stage/member.name;destination.parent.mkdir(parents=True,exist_ok=True)
    with destination.open('xb') as stream:stream.write(archive.extractfile(member).read())
    os.chmod(destination,member.mode & 0o777)
  for name,value in manifest.items():
   if sha(stage/name)!=value['sha256']:raise RuntimeError('Bundle hash mismatch: '+name)
  record('before.json',before)
  with tarfile.open(receipt/'before.tar','x') as archive:
   for name,digest in before.items():
    if digest is not None:archive.add(target_path(name),arcname=name,recursive=False)
  old=current[0] if current else None
  result['previous_process']=old
  changed=[name for name in manifest if before[name]!=manifest[name]['sha256']]
  result['changed_files']=changed
  if old and changed:
   values=environment(old['pid'])
   if environment_sha(values)!=old['environment_sha256']:raise RuntimeError('Environment changed before stop')
   port=None
   for node in ast.parse((root/'rover/config.py').read_text()).body:
    if isinstance(node,ast.Assign) and any(isinstance(n,ast.Name) and n.id=='STATION_PORT' for n in node.targets):port=ast.literal_eval(node.value)
   if '--port' in old['argv']:port=int(old['argv'][old['argv'].index('--port')+1])
   if port is None:raise RuntimeError('Station port unavailable in config')
   t=telemetry(port);require_idle(t);record('restart_precondition.json',t)
   if processes()!=current or identities()!=before:raise RuntimeError('Remote state changed before stop')
   stop_process(old);stopped=True
  elif processes()!=current or identities()!=before:raise RuntimeError('Remote state changed before install')
  for name in changed:
   target=target_path(name)
   mode=stat.S_IMODE(target.stat().st_mode) if target.exists() else stat.S_IMODE((stage/name).stat().st_mode)
   attempted.append(name)
   atomic_write(name,(stage/name).read_bytes(),mode)
  after=identities();mismatches=[name for name,value in manifest.items() if after[name]!=value['sha256']]
  result.update(installed_files=len(manifest),mismatches=mismatches)
  record('installed.json',result)
  if mismatches:raise RuntimeError('Installed hash mismatch')
  if stopped:
   result['new_process']=start_runtime(old,values,'runtime.console.log');result['restart_succeeded']=True
  else:result['note']='Existing runtime unchanged' if old else 'Runtime was stopped and remains stopped'
  result['deployment_complete']=True
 except BaseException as error:
  result['error']=type(error).__name__+': '+str(error)
  # Never restore files underneath a still-running candidate or an unknown PID.
  try:active=processes()
  except BaseException:active=['unknown process state']
  if (attempted or stopped) and not active:
   result['rollback_attempted']=True
   try:
    restore_files(before,attempted);result['rollback_verified']=True
    if stopped:
     result['restored_process']=start_runtime(old,values,'restored_runtime.console.log')
     result['previous_runtime_restored']=True
   except BaseException as rollback_error:
    result['rollback_error']=type(rollback_error).__name__+': '+str(rollback_error)
  elif attempted or stopped:result['rollback_error']='Active or unknown runtime present; no files restored or process relaunched'
 finally:
  record('receipt.json',result)
 return result

request=json.load(__import__('sys').stdin)
root=Path(request['root']).resolve()
if root!=Path('/home/taeholee/Rover'):raise RuntimeError('Unexpected deployment root')
if request['phase']=='inspect':
 print(json.dumps(dict(processes=processes(),files=identities())));raise SystemExit
receipt=root/'logs'/'deployments'/request['release_id']
receipt.mkdir(parents=True,exist_ok=False)
record('request.json',request)
result=apply_release()
print(json.dumps(result))
raise SystemExit(0 if result['deployment_complete'] else 1)
'''


def known_hashes(baseline):
    """Previously installed manifests identify bytes; unknown remote edits stop us."""
    known = {}
    def add(name, digest):
        if isinstance(digest, str) and re.fullmatch(r'[a-f0-9]{64}', digest):
            known.setdefault(name, set()).add(digest)
    if baseline and baseline.is_file():
        for row in json.loads(baseline.read_text(encoding='utf-8'))['files']:
            add(row['path'], row['sha256'])
    for path in (ROOT / 'logs').rglob('*manifest*.json'):
        if path.stat().st_size > 3_000_000:
            continue
        try:
            data = json.loads(path.read_text(encoding='utf-8-sig'))
        except (ValueError, UnicodeError):
            continue
        if isinstance(data, dict):
            for name, row in data.items():
                add(name, row.get('sha256') if isinstance(row, dict) else row)
    return known


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true', help='install bundle over SSH and restart an already running Rover in the same mode')
    parser.add_argument('--bundle', type=Path, help='existing bundle directory; default freezes current source')
    parser.add_argument('--host', default='taeholee@192.168.0.200')
    parser.add_argument('--key', type=Path, default=Path.home() / '.ssh/id_ed25519_jetson')
    parser.add_argument('--baseline', type=Path, default=ROOT / 'data/final_vehicle_2026-10-28/baseline_sources.json')
    args = parser.parse_args(argv)
    if not re.fullmatch(r'[a-zA-Z0-9_.-]+@[a-zA-Z0-9_.-]+', args.host):
        parser.error('host must be user@host')
    release_id = 'release_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S') + '_' + uuid.uuid4().hex[:8]
    bundle = args.bundle.resolve() if args.bundle else ROOT / 'data/deployments/releases' / release_id
    if not args.bundle:
        bundle.mkdir(parents=True, exist_ok=False)
        manifest = {}
        with tarfile.open(bundle / 'release.tar', 'x') as archive:
            for path in release_paths():
                name = path.relative_to(ROOT).as_posix()
                manifest[name] = dict(sha256=sha(path), bytes=path.stat().st_size)
                archive.add(path, arcname=name, recursive=False)
        save(bundle / 'manifest.json', manifest)
        save(bundle / 'bundle.json', dict(created_utc=datetime.now(timezone.utc).isoformat(), archive_sha256=sha(bundle / 'release.tar'), files=len(manifest), firmware_flashed=False))
    manifest = json.loads((bundle / 'manifest.json').read_text(encoding='utf-8'))
    identity = json.loads((bundle / 'bundle.json').read_text(encoding='utf-8'))
    if sha(bundle / 'release.tar') != identity['archive_sha256']:
        raise RuntimeError('Release archive changed since freezing')
    changed = [n for n, v in manifest.items() if not (ROOT / n).is_file() or sha(ROOT / n) != v['sha256']]
    if changed:
        raise RuntimeError('Local files changed after freezing; create a new bundle: ' + ', '.join(changed))
    print(f'Bundle: {bundle}\nFiles: {len(manifest)}; firmware flashing: disabled')
    if not args.apply:
        print('Prepared only. Install with: python tools/deploy_jetson.py --apply --bundle "' + str(bundle) + '"')
        return 0
    options = ['-i', str(args.key), '-o', 'IdentitiesOnly=yes', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8']
    attempts = bundle / ('attempt_' + release_id)
    attempts.mkdir()
    remote_root = '/home/taeholee/Rover'
    payload = dict(root=remote_root, release_id=release_id, manifest=manifest, phase='inspect')
    def ssh(command, source, phase, timeout=90):
        result = subprocess.run(['ssh', *options, args.host, command], input=source, capture_output=True, timeout=timeout)
        (attempts / (phase + '.stdout.txt')).write_bytes(result.stdout)
        (attempts / (phase + '.stderr.txt')).write_bytes(result.stderr)
        if result.returncode:
            raise RuntimeError(f'{phase} failed ({result.returncode}); see {attempts}')
        return result.stdout
    # JSON goes through stdin; the constant program is quoted as one shell token.
    command = '/home/taeholee/rover-venv/bin/python -c ' + shlex.quote(REMOTE)
    try:
        current = json.loads(ssh(command, json.dumps(payload).encode(), 'inspect'))
        save(attempts / 'inspection.json', current)
        known = known_hashes(args.baseline)
        drift = [n for n, digest in current['files'].items()
                 if digest is not None and digest != manifest[n]['sha256'] and digest not in known.get(n, set())]
        if drift:
            save(attempts / 'remote_drift.json', drift)
            raise RuntimeError('Unknown remote changes preserved; review remote_drift.json before installing')
        ssh('mkdir -p ' + shlex.quote(remote_root + '/logs/deployments'), None, 'mkdir')
        result = subprocess.run(['scp', *options, str(bundle / 'release.tar'), args.host + ':' + remote_root + '/logs/deployments/' + release_id + '.tar'], capture_output=True, timeout=90)
        if result.returncode:
            raise RuntimeError('Bundle upload failed: ' + result.stderr.decode(errors='replace'))
        payload.update(phase='apply', expected_processes=current['processes'], expected_files=current['files'])
        try:
            result = json.loads(ssh(command, json.dumps(payload).encode(), 'apply', timeout=240))
            save(attempts / 'receipt.json', result)
        finally:
            # Copy backups and failure diagnostics even if activation did not finish.
            copied = subprocess.run(['scp', *options, '-r', args.host + ':' + remote_root + '/logs/deployments/' + release_id, str(attempts / 'remote')], capture_output=True, timeout=90)
            (attempts / 'backup_copy.stderr.txt').write_bytes(copied.stderr)
            if copied.returncode:
                raise RuntimeError('Remote backup copy incomplete; installed state may have changed, and the remote receipt is retained')
        print(json.dumps(result, indent=2))
        return 0
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
        save(attempts / 'failure.json', dict(error=str(exc), notebook_utc=datetime.now(timezone.utc).isoformat(), deployment_complete=False))
        print(str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
