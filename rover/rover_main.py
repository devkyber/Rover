"""
The rover program. Runs on the Jetson, needs no screen.

    python rover_main.py                          # motors + IMU + camera + log
    python rover_main.py --tag square             # log named logs/<date>_<time>_square.csv
    python rover_main.py --no-motors --imu fake   # nothing plugged in (camera still on)
    python rover_main.py --pico fake              # no Pico: simulate the servos and battery

Then on the station laptop:  python station/station.py --rover <this machine's address>

Every 10 ms the loop does, in this order:

    station command -> read wheels + IMU -> odometry -> log -> drive -> telemetry

The camera runs beside the loop in its own thread (camera.py), the Pico's
servos and battery monitor in theirs (pico_link.py), and the station
connections in theirs (station_link.py). The loop itself never
waits on the network: a stalled Wi-Fi link cannot delay a motor command.

A RUN is one log file together with one fresh filter. The station's
NEW RUN button ends the current run and starts another, so a new test
needs no SSH and no restart. Keep the rover still for the first 5 s of
every run: the filter measures gyro bias and tilt then, and the wheels
are held at zero until it has.
"""
import argparse
import os
import signal
import socket
import time

import numpy as np

import camera
import config
import imu_reader
import logger
import motors
import odometry
import pico_link
import station_link

# The live NIS mean is taken over this many recent samples (3 s at 100 Hz):
# long enough to be steady, short enough to react while still driving.
NIS_WINDOW = 300


class Run:
    """One recording: a fresh filter, a fresh log, time starting at zero."""

    def __init__(self, log_path, imu_metadata=None):
        # A fresh Odometry on purpose. replay.py starts every log with a new
        # filter and the 5 s startup alignment; if the live filter carried
        # its state across logs, replay would no longer reproduce what the
        # rover computed live.
        self.odo = odometry.Odometry()
        self.log = logger.Logger(log_path, metadata=imu_metadata) if log_path else None
        self.t_start = time.monotonic()
        self.t_prev = 0.0
        self.cycles = 0
        self.dropped = 0            # cycles with no answer from the motor bus

    def stop_log(self):
        if self.log:
            self.log.close()
            self.log = None


def describe_rover(args, camera_on):
    """Sent once to each station when it connects: what this rover is."""
    return {
        # Changes when this program restarts, so a station that reconnects
        # can tell "same rover, link came back" from "rover was restarted".
        "boot": round(time.time(), 3),
        "loop_hz": config.EKF_RATE_HZ,
        "wheels": [{"id": i, "name": name, "side": side}
                   for i, name, side in config.WHEELS],
        "max_speed_mps": config.COMMAND_WHEEL_SPEED_LIMIT_MPS,
        "track_m": config.WHEELBASE_M,
        "wheel_radius_m": config.WHEEL_RADIUS_M,
        "telemetry_hz": config.TELEMETRY_HZ,
        "command_timeout_s": config.COMMAND_TIMEOUT_S,
        "alignment_s": config.STARTUP_ALIGNMENT_SECONDS,
        "motors": not args.no_motors,
        "imu": args.imu,
        "camera": ({"width": config.CAMERA_WIDTH, "height": config.CAMERA_HEIGHT,
                    "fps": config.CAMERA_FPS} if camera_on else None),
        "pico": args.pico,                                       # "usb" | "fake" | "off"
        "battery": {"cells": config.BATTERY_CELLS,
                    "low_cell_v": config.BATTERY_LOW_V_PER_CELL,
                    "empty_cell_v": config.BATTERY_EMPTY_V_PER_CELL},
    }


def build_telemetry(run, t, wheels, command, forward, turn, lift, pan, raw_sent,
                    motor_state, imu, cam, pico, temp_c, wifi_dbm):
    """Everything the station shows, as plain numbers."""
    odo = run.odo
    r3 = lambda v: [round(float(x), 3) for x in v]

    nis = {}
    for name, samples in odo.nis.items():
        window = samples[-NIS_WINDOW:]
        if window:
            nis[name] = [round(float(np.mean(window)), 3),
                         config.NIS_DOF.get(name, 1), len(samples)]

    if not odo.ready:
        state = "aligning"
    elif odo.zupt_active:
        state = "stopped"
    else:
        state = "moving"

    return {
        "type": "telemetry",
        "t": round(t, 3),
        "state": state,
        "align": round(float(odo.alignment_progress), 3),
        "pos": [round(float(x), 4) for x in odo.position],
        "vel": r3(odo.velocity),
        "rpy": [round(float(x), 2) for x in odo.rpy_deg],
        "gyro_bias": r3(np.degrees(odo.gyro_bias)),              # deg/s
        "wheel_speed": r3(odometry.wheel_speeds_mps(wheels)),    # m/s, + = forward
        "wheel_current": r3(w['current_A'] for w in wheels),     # A
        "cmd": {"forward": round(forward, 3), "turn": round(turn, 3),
                "lift": round(lift, 3), "pan": round(pan, 3),
                "sent": list(raw_sent),
                "stale": command.stale, "estop": command.estop},
        "loop": {"hz": round(run.cycles / max(t, 1e-6), 1),
                 "cycles": run.cycles, "dropped": run.dropped},
        "nis": nis,
        "motors": motor_state,                                   # "ok" | "lost" | "off"
        "imu_stale": imu.stats()['stale'] if hasattr(imu, 'stats') else 0,
        "camera": ({"fps": round(cam.fps, 1), "mbps": round(cam.mbps, 2)}
                   if cam else None),
        # status "ok" | "lost" | "off"; battery None = reading unavailable
        "pico": pico.state(),
        "temp_c": temp_c,
        "wifi_dbm": wifi_dbm,                                    # None on a wired link
        "log": ({"file": os.path.basename(run.log.filename), "rows": run.log.count}
                if run.log else None),
    }


def read_cpu_temp():
    try:
        with open(config.CPU_TEMP_FILE) as f:
            return round(int(f.read()) / 1000.0, 1)
    except (OSError, ValueError):
        return None


def read_wifi_dbm():
    """
    How strongly the rover hears the router, in dBm (-50 strong, -80 weak).
    None if there is no Wi-Fi link.
    """
    try:
        with open(config.WIFI_STATUS_FILE) as f:
            lines = f.read().splitlines()[2:]        # two header lines, then one per radio
        return int(float(lines[0].split()[3]))
    except (OSError, ValueError, IndexError):
        return None


def local_ip():
    """The address a station on the same network should connect to."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))       # no packet is sent; this only picks a route
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return socket.gethostname()


def stop_on_signal(signum, frame):
    """
    Turn `kill`, `systemctl stop` and a closed SSH window into Ctrl+C.

    By default those end Python at once, skipping the `finally` below.
    Goal Velocity latches inside the motor, so the wheels would keep
    turning at the last commanded speed with nothing left to stop them.
    """
    raise KeyboardInterrupt


def main(args):
    signal.signal(signal.SIGTERM, stop_on_signal)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, stop_on_signal)

    camera_on = not args.no_camera and camera.available()
    if not args.no_camera and not camera_on:
        print("Camera off: GStreamer Python bindings not found on this machine.")

    # The link binds its port first. If another copy of this program holds
    # it, we stop here -- before torque goes on -- instead of running a
    # second rover that no station can reach.
    try:
        link = station_link.StationLink(port=args.port,
                                        hello=describe_rover(args, camera_on))
    except OSError as e:
        raise SystemExit(
            f"\nCannot listen on port {args.port}: {e}\n"
            "Another rover_main.py is probably still running. Stop it first:\n"
            f"  {'taskkill /F /IM python.exe' if config.ON_WINDOWS else 'pkill -f [r]over_main.py'}\n")

    imu = imu_reader.create_imu(args.imu)
    wheels_hw = motors.open_motors(not args.no_motors)
    pico = pico_link.open_pico(args.pico, on_event=link.event)
    pico.start()
    cam = None
    if camera_on:
        cam = camera.Camera(on_frame=link.publish_video, on_event=link.event)
        cam.start()
    link.start()

    log_path = None if args.no_log else (args.log or logger.new_log_path(args.tag))
    imu_metadata = imu.metadata() if hasattr(imu, 'metadata') else None
    run = Run(log_path, imu_metadata)

    print("=" * 60)
    print(f"  Station ->  python station/station.py --rover {local_ip()}")
    print(f"  Port    ->  {args.port}  (/control, /video)")
    print(f"  Log     ->  {log_path or 'NONE (--no-log)'}")
    print("=" * 60)

    dt = 1.0 / config.EKF_RATE_HZ
    next_tick = time.monotonic()
    next_telemetry = next_temp = 0.0
    temp_c = wifi_dbm = None
    silent_cycles = 0                    # consecutive cycles the motor bus did not answer
    wheels = motors.NoMotors().read()    # shown until the first real reading
    raw_sent = [0] * len(config.MOTOR_IDS)

    try:
        while True:
            # ── 0. one-off requests from the station ───────────────
            for request in link.take_requests():
                if request["type"] == "new_run":
                    run.stop_log()
                    path = logger.new_log_path(str(request.get("tag") or "run"))
                    run = Run(path, imu_metadata)
                    link.event(f"new run: {os.path.basename(path)} -- hold still, aligning")
                elif request["type"] == "stop_log" and run.log:
                    link.event(f"log closed: {os.path.basename(run.log.filename)}")
                    run.stop_log()
                elif request["type"] == "zero_battery":
                    pico.zero_charge()
                    link.event("battery mAh count set to zero")

            now = time.monotonic()
            t = now - run.t_start

            # ── 1. what the operator wants ─────────────────────────
            # Worked out BEFORE the sensor read, so that every path below
            # -- including the failure path -- can act on it.
            command = link.command()
            forward, turn = command.forward, command.turn
            lift, pan = command.lift, command.pan
            if not run.odo.ready:
                # Startup bias/tilt estimation is valid only while still.
                # Never let a station command move the rover during it.
                forward = turn = lift = pan = 0.0

            # ── 2. sensors -> 3. odometry -> log ───────────────────
            reading = wheels_hw.read()
            if reading is not None:
                silent_cycles = 0
                wheels = reading
                imu_data = imu.read()

                # Use the dt that actually elapsed, not the nominal one, so
                # a stalled cycle gets the Q it deserves. replay.py does the
                # same, which is what makes live and replay agree.
                step = t - run.t_prev if run.cycles else dt
                run.t_prev = t
                run.odo.step(wheels, gyro=np.asarray(imu_data['gyro']),
                             accel=np.asarray(imu_data['accel']),
                             dt=imu_data.get('dt', step if 0 < step < 0.5 else dt),
                             gyro_samples=imu_data.get('gyro_samples'))
                if run.log:
                    run.log.log(t, wheels, imu_data)
                run.cycles += 1
            else:
                # A lost READ must not skip the WRITE below. Goal Velocity
                # latches in the motor: skipping the write on a bus that has
                # stopped answering would leave the wheels at their last
                # speed. So keep commanding, and once the bus has been
                # silent as long as the deadman allows, command a stop
                # whatever the stick says.
                run.dropped += 1
                silent_cycles += 1
                if silent_cycles * dt > config.COMMAND_TIMEOUT_S:
                    forward = turn = 0.0

            # ── 4. drive ───────────────────────────────────────────
            raw_sent = wheels_hw.drive(forward, turn)
            # Called every cycle on purpose: pico_link sends a servo command
            # only while these calls keep coming, so a loop that hangs
            # stops the servos as well.
            pico.set_servos(lift, pan)

            # ── 5. tell the station ────────────────────────────────
            if now >= next_telemetry:
                # Step from the schedule, not from "now". The loop only looks
                # every 10 ms, so restarting the wait from now would stretch
                # each period by up to one tick: 18 Hz instead of 20.
                next_telemetry = max(next_telemetry + 1.0 / config.TELEMETRY_HZ, now)
                if now >= next_temp:
                    next_temp = now + 1.0
                    temp_c = read_cpu_temp()
                    wifi_dbm = read_wifi_dbm()
                if not wheels_hw.connected:
                    motor_state = "off"
                elif silent_cycles * dt > config.COMMAND_TIMEOUT_S:
                    motor_state = "lost"
                else:
                    motor_state = "ok"
                link.publish(build_telemetry(run, t, wheels, command, forward, turn,
                                             lift, pan, raw_sent, motor_state, imu,
                                             cam, pico, temp_c, wifi_dbm))

            # ── hold the beat ──────────────────────────────────────
            # Never skipped, not even on a dropped cycle: free-running to
            # catch up turns one lost packet into a burst of jitter.
            next_tick += dt
            pause = next_tick - time.monotonic()
            if pause > 0:
                time.sleep(pause)
            else:
                next_tick = time.monotonic()

    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        wheels_hw.close()                # zero the wheels and release torque FIRST
        pico.close()                     # stop pulses to the servos
        if cam:
            cam.stop()
        run.stop_log()
        if hasattr(imu, 'stats'):
            print(f"IMU stats: {imu.stats()}")
        imu.close()
        elapsed = time.monotonic() - run.t_start
        print(f"{run.cycles} cycles in {elapsed:.1f} s -> "
              f"{run.cycles / max(elapsed, 1e-6):.1f} Hz")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Rover program: drive, log, camera, station link.")
    ap.add_argument("--no-motors", action="store_true",
                    help="do not touch the motors (wheels read zero)")
    ap.add_argument("--imu", default="arduino",
                    choices=["fake", "arduino", "bmi088"])
    ap.add_argument("--no-camera", action="store_true", help="do not open the camera")
    ap.add_argument("--pico", default="usb", choices=["usb", "fake", "off"],
                    help="servo + battery board: on USB (keeps looking if absent), "
                         "simulated, or not used")
    ap.add_argument("--tag", default="drive",
                    help="name fragment for the auto-named log")
    ap.add_argument("--log", default=None,
                    help="explicit CSV path, overriding the auto name")
    ap.add_argument("--no-log", action="store_true",
                    help="do not start a log (NEW RUN from the station still does)")
    ap.add_argument("--port", type=int, default=config.STATION_PORT)
    main(ap.parse_args())
