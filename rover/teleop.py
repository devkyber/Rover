"""
Phone joystick teleop, with the odometry pipeline running underneath.

    python rover/teleop.py                  # drive + log + live odometry
    python rover/teleop.py --no-motors      # UI/IMU only, motors untouched
    python rover/teleop.py --tag square     # name the log logs/<date>_<time>_square.csv

Open the printed http://<ip>:5000 on your phone. Left stick = forward,
right stick = turn.

This is the joystick script reworked to share the rover's own modules
instead of redefining the hardware. Every cycle it:

    read wheels + IMU  ->  ESKF predict/update  ->  log  ->  send velocity

so the log you get from a drive is the same format offline tuning eats.
"""
import argparse
import socket
import threading
import time

import numpy as np
import pygame
from flask import Flask, render_template_string
from flask_socketio import SocketIO

import config
import live_panel
import logger
import odometry

# Joystick state, written by the web thread, read by the main loop.
# Two floats assigned atomically -- no lock needed in CPython.
joy_forward = 0.0
joy_turn = 0.0


def get_local_ip():
    """Find the LAN IP the phone should connect to."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


# ── Web UI (unchanged in spirit from the original) ─────────────────

HTML = """
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0,
        maximum-scale=1.0, user-scalable=no">
  <title>Rover Controller</title>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/nipplejs/0.9.1/nipplejs.min.js"></script>
  <style>
    body { margin:0; padding:0; background:#222; touch-action:none; overflow:hidden; }
    .zone { position:absolute; top:0; width:50%; height:100%; }
    #zone_left  { left:0;  background:rgba(50,150,250,0.05); }
    #zone_right { right:0; background:rgba(250,150,50,0.05); }
    .title { position:absolute; width:100%; text-align:center; color:#fff;
             font-family:sans-serif; top:20px; pointer-events:none; z-index:10; }
    .hint { position:absolute; width:100%; text-align:center; color:#666;
            font-family:sans-serif; bottom:20px; font-size:13px;
            pointer-events:none; z-index:10; }
  </style>
</head>
<body>
  <div class="title">Rover Control</div>
  <div class="hint">left = forward/back &nbsp;|&nbsp; right = turn</div>
  <div id="zone_left" class="zone"></div>
  <div id="zone_right" class="zone"></div>
  <script>
    const socket = io();
    let fwd = 0.0, turn = 0.0;
    function send() { socket.emit('joy', { forward: fwd, turn: turn }); }

    // Heartbeat: the rover stops if these stop arriving, so a phone that
    // walks out of Wi-Fi range does not leave the wheels running.
    setInterval(send, 100);

    nipplejs.create({ zone: document.getElementById('zone_left'),
                      mode:'static', position:{left:'50%',top:'70%'},
                      color:'#3296fa' })
      .on('move', (e,d) => {
          fwd = Math.max(-1, Math.min(1, Math.sin(d.angle.radian)*(d.distance/50)));
          send(); })
      .on('end', () => { fwd = 0.0; send(); });

    nipplejs.create({ zone: document.getElementById('zone_right'),
                      mode:'static', position:{left:'50%',top:'70%'},
                      color:'#fa9632' })
      .on('move', (e,d) => {
          turn = Math.max(-1, Math.min(1, Math.cos(d.angle.radian)*(d.distance/50)));
          send(); })
      .on('end', () => { turn = 0.0; send(); });
  </script>
</body>
</html>
"""

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")
last_command_time = time.monotonic()


@app.route('/')
def index():
    return render_template_string(HTML)


@app.route('/panel')
def panel():
    """Live telemetry dashboard -- see live_panel.py."""
    return render_template_string(live_panel.PANEL_HTML)


@socketio.on('joy')
def on_joy(data):
    global joy_forward, joy_turn, last_command_time
    joy_forward = float(data.get('forward', 0.0))
    joy_turn = float(data.get('turn', 0.0))
    last_command_time = time.monotonic()


WEB_PORT = 5000


def check_port_free():
    """
    Refuse to start if something is already serving on WEB_PORT.

    This is worth a hard stop rather than a warning. socketio.run() lives
    in a daemon thread, so when the port is taken its OSError kills only
    that thread and the main loop carries on -- with no web server. The
    phone then loads the joystick page from the OTHER process, its
    commands go there, and THIS rover sits at zero forever while looking
    completely healthy: torque on, 100 Hz, no errors, wheels still.

    That is a genuinely confusing failure, and the fix (kill the other
    process) is nothing like what the symptom suggests. So we say it.
    """
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    probe.settimeout(0.3)
    busy = probe.connect_ex(("127.0.0.1", WEB_PORT)) == 0
    probe.close()
    if busy:
        raise SystemExit(
            "\nPort %d is already in use -- another teleop.py is still\n"
            "running. The phone would talk to THAT one, not to this.\n\n"
            "Stop every stray copy, then run this again:\n"
            "  taskkill /F /IM python.exe\n" % WEB_PORT)


def run_server():
    socketio.run(app, host='0.0.0.0', port=WEB_PORT,
                 allow_unsafe_werkzeug=True)


# ── Main loop ──────────────────────────────────────────────────────

def main(use_motors, log_file, imu_kind, show_window=True):
    import imu_reader

    imu = imu_reader.create_imu(imu_kind)

    port_h = pkt_h = reader = writer = None
    if use_motors:
        import dxl_reader
        import dxl_control
        port_h, pkt_h = dxl_reader.open_port()
        reader = dxl_reader.create_sync_reader(port_h, pkt_h)
        dxl_control.setup_wheels(pkt_h, port_h)
        writer = dxl_control.create_sync_writer(port_h, pkt_h)

    log = logger.Logger(log_file) if log_file else None
    odo = odometry.Odometry()
    telemetry = live_panel.Telemetry(socketio)

    screen = font = None
    if show_window:
        pygame.init()
        screen = pygame.display.set_mode((520, 300))
        pygame.display.set_caption("Rover teleop + odometry")
        font = pygame.font.SysFont("consolas", 17)

    ip = get_local_ip()
    print("=" * 60)
    print(f"  Phone  ->  http://{ip}:5000")
    print(f"  Panel  ->  http://{ip}:5000/panel")
    print(f"  Log    ->  {log_file if log_file else 'NONE (--no-log)'}")
    print("=" * 60)

    dt = 1.0 / config.EKF_RATE_HZ
    raw_sent = [0] * len(config.MOTOR_IDS)
    t_start = next_tick = time.monotonic()
    n = dropped = consec_drops = 0
    t_prev = 0.0

    def pace():
        """Hold the beat. Never skip this, not even on a dropped cycle."""
        nonlocal next_tick
        next_tick += dt
        sleep = next_tick - time.monotonic()
        if sleep > 0:
            time.sleep(sleep)
        else:
            next_tick = time.monotonic()

    try:
        while True:
            if show_window:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        raise KeyboardInterrupt
            t = time.monotonic() - t_start

            # ── 1. what the phone wants ────────────────────────────
            # Worked out BEFORE the sensor read, so that every path
            # below -- including the failure path -- can act on it.
            # Deadman: if the phone goes quiet for 0.5 s, stop.
            stale = time.monotonic() - last_command_time > 0.5
            fwd = 0.0 if stale else joy_forward
            turn = 0.0 if stale else joy_turn
            if not odo.ready:
                # Startup bias/tilt estimation is valid only while still.
                # Never let a phone command move the rover during it.
                fwd = turn = 0.0

            # ── 2. sensors ─────────────────────────────────────────
            if use_motors:
                import dxl_control
                import dxl_reader
                data = dxl_reader.read_wheels(reader)
                if data is None:
                    dropped += 1
                    consec_drops += 1
                    # A lost READ must not skip the WRITE. Goal Velocity
                    # latches in the motor: if we simply `continue`d
                    # here, a bus that stopped answering would leave the
                    # wheels spinning at the last commanded speed with
                    # the deadman never running again. So keep
                    # commanding, and after half a second of silence
                    # command a stop regardless of the joystick.
                    if consec_drops > 50:
                        fwd = turn = 0.0
                    raw_sent = dxl_control.drive(writer, fwd, turn)
                    pace()
                    continue
                consec_drops = 0
                wheels = data['wheels']
            else:
                wheels = [{'id': m, 'position_rad': 0.0, 'velocity_rads': 0.0,
                           'current_A': 0.0} for m in config.MOTOR_IDS]

            imu_data = imu.read()
            accel = np.asarray(imu_data['accel'])
            gyro = np.asarray(imu_data['gyro'])

            # ── 3. odometry ────────────────────────────────────────
            # Use the dt that actually elapsed, not the nominal one, so
            # a stalled cycle gets the Q it deserves. replay.py does the
            # same, which is what makes live and replay agree.
            step = t - t_prev if n else dt
            t_prev = t
            odo.step(wheels, gyro=gyro, accel=accel,
                     dt=step if 0 < step < 0.5 else dt)

            if log:
                log.log(t, wheels, imu_data)
            n += 1

            # ── 4. drive ───────────────────────────────────────────
            if use_motors:
                raw_sent = dxl_control.drive(writer, fwd, turn)

            # ── 5. show ────────────────────────────────────────────
            stopped = odo.zupt_active
            telemetry.maybe_emit(
                t=t, odo=odo, wheels=wheels, fwd=fwd, turn=turn,
                raw_sent=raw_sent, stale=stale, stopped=stopped,
                ready=odo.ready, alignment_progress=odo.alignment_progress,
                n=n, dropped=dropped, hz=n / max(t, 1e-6))

            if show_window:
                p, v = odo.position, odo.velocity
                roll, pitch, yaw = odo.rpy_deg
                if not odo.ready:
                    state_text = f"ALIGNING {100*odo.alignment_progress:.0f}%"
                else:
                    state_text = 'STOPPED (ZUPT)' if stopped else 'moving'
                screen.fill((18, 22, 28))
                lines = [
                    (f"phone   http://{ip}:5000", (150, 200, 255)),
                    (f"panel   http://{ip}:5000/panel", (150, 200, 255)),
                    (f"stick   fwd {fwd:+.2f}   turn {turn:+.2f}"
                     + ("   [NO SIGNAL]" if stale else ""),
                     (255, 120, 120) if stale else (200, 255, 120)),
                    (f"sent    {raw_sent}", (200, 255, 120)),
                    (f"pos     {p[0]:+7.3f} {p[1]:+7.3f} {p[2]:+7.3f}  m", (255, 255, 255)),
                    (f"vel     {v[0]:+7.3f} {v[1]:+7.3f} {v[2]:+7.3f}  m/s", (255, 255, 255)),
                    (f"rpy     {roll:+7.1f} {pitch:+7.1f} {yaw:+7.1f}  deg", (255, 255, 255)),
                    (f"state   {state_text}",
                     (255, 220, 120)),
                    (f"cycles  {n}   dropped {dropped}   {n/max(t,1e-6):.1f} Hz",
                     (140, 150, 160)),
                ]
                for i, (text, color) in enumerate(lines):
                    screen.blit(font.render(text, True, color), (18, 16 + i * 28))
                pygame.display.flip()

            pace()

    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        if use_motors:
            import dxl_control
            import dxl_reader
            dxl_control.stop(writer, pkt_h, port_h)
            dxl_reader.close_port(port_h)
        if log:
            log.close()
        if hasattr(imu, 'stats'):
            print(f"IMU stats: {imu.stats()}")
        imu.close()
        if show_window:
            pygame.quit()
        elapsed = time.monotonic() - t_start
        print(f"{n} cycles in {elapsed:.1f} s -> {n/max(elapsed,1e-6):.1f} Hz")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-motors", action="store_true",
                    help="do not touch the motors (UI + IMU only)")
    ap.add_argument("--imu", default="arduino",
                    choices=["fake", "arduino", "bmi088"])
    ap.add_argument("--tag", default="drive",
                    help="name fragment for the auto-named log")
    ap.add_argument("--log", default=None,
                    help="explicit CSV path, overriding the auto name")
    ap.add_argument("--no-log", action="store_true", help="do not write a log")
    ap.add_argument("--headless", action="store_true",
                    help="no pygame window -- use http://<ip>:5000/panel instead")
    args = ap.parse_args()

    log_path = None if args.no_log else (args.log or logger.new_log_path(args.tag))

    check_port_free()
    threading.Thread(target=run_server, daemon=True).start()
    main(use_motors=not args.no_motors, log_file=log_path,
         imu_kind=args.imu, show_window=not args.headless)
