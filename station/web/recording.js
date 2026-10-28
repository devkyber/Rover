// Display only: telemetry recording, IMU CSV, and Jumper raw capture are separate.
export function recordingDisplay(telemetry, live) {
  const t = telemetry || {};
  const recording = t.recording;
  const imu = t.log ? `recording ${t.log.file} (${t.log.rows} rows)`
    : t.logging_requested ? 'waiting for IMU' : 'stopped';
  const separate = 'Jumper raw capture is separate; see SCOUT. STOP LOG does not stop raw capture.';
  const canStop = !!(live && (recording?.active || t.logging_requested || t.log));
  if (!recording || typeof recording !== 'object' || Array.isArray(recording)) {
    return {
      text: live && t.log ? `REC ${t.log.file}  ${t.log.rows} rows`
        : live && t.logging_requested ? 'REC WAITING FOR IMU' : 'NOT LOGGING',
      title: `IMU CSV: ${imu}. Full telemetry recording status is unavailable on this runtime.\n${separate}`,
      tone: live && t.log ? 'good' : '',
      canStop,
    };
  }

  const count = value => Number.isSafeInteger(value) && value >= 0 ? value : null;
  const number = value => count(value) ?? '--';
  const status = typeof recording.status === 'string' ? recording.status : 'unknown';
  const written = count(recording.written);
  const dropped = count(recording.dropped);
  const errorMessages = Array.isArray(recording.errors) ? recording.errors.filter(value => typeof value === 'string') : [];
  const errors = count(recording.error_count) ?? count(recording.errors)
    ?? (Array.isArray(recording.errors) ? recording.errors.length : null);
  const session = typeof recording.session_dir === 'string' ? recording.session_dir : '';
  const name = session.replace(/[\\/]+$/, '').split(/[\\/]/).pop()
    || recording.run_id || 'session unavailable';
  const paths = Array.isArray(recording.paths) ? recording.paths.map(path => `File: ${String(path)}`)
    : recording.paths && typeof recording.paths === 'object'
    ? Object.entries(recording.paths).map(([key, value]) => `${key}: ${String(value)}`)
    : recording.paths ? [`Files: ${String(recording.paths)}`] : [];
  const title = [
    `Rover + Jumper session: ${status}; ${recording.active ? 'active' : 'inactive'}.`,
    `Session: ${session || 'unavailable'}`,
    `Run: ${recording.run_id || 'unavailable'}`,
    `Records: ${number(recording.accepted)} accepted; ${number(recording.queued)} queued; ${number(recording.written)} written; ${number(recording.flushed)} flushed; ${number(recording.fsynced)} fsynced.`,
    `Dropped: ${number(recording.dropped)}; errors: ${errors ?? '--'}; nonfinite values: ${number(recording.nonfinite_values)}.`,
    `Close complete: ${recording.close_complete === true ? 'yes' : recording.close_complete === false ? 'no' : 'unknown'}.`,
    ...(recording.error ? [`Error: ${String(recording.error)}`] : errorMessages.map(error => `Error: ${error}`)),
    ...paths,
    `IMU CSV: ${imu}.`,
    separate,
  ].join('\n');
  if (!live) return { text: 'REC STATUS UNAVAILABLE', title: `Last received status only.\n${title}`, tone: 'warn', canStop: false };

  let label, tone;
  if (status === 'error' || errors > 0 || errorMessages.length > 0 || recording.error) {
    label = 'REC ERROR'; tone = 'bad';
  } else if (status === 'degraded' || dropped > 0) {
    label = 'REC GAPS'; tone = 'warn';
  } else if (status === 'stopping' || status === 'closing') {
    label = 'REC STOPPING'; tone = 'warn';
  } else if (status === 'closed') {
    label = 'REC STOPPED'; tone = '';
  } else if (status === 'starting') {
    label = 'REC STARTING'; tone = 'warn';
  } else if (recording.active === false) {
    label = 'REC STOPPED'; tone = '';
  } else if (status === 'recording' && written > 0 && recording.active === true) {
    label = 'REC'; tone = 'good';
  } else {
    label = 'REC WAITING'; tone = 'warn';
  }
  // A starting recorder is not evidence that any data reached the file.
  const progress = status === 'starting' ? '' : ` · ${written ?? '--'} written`;
  const gaps = dropped > 0 ? ` · ${dropped} dropped` : '';
  const scope = label === 'REC' ? 'REC Rover + Jumper' : `${label} · Rover + Jumper`;
  return { text: `${scope} · ${name}${progress}${gaps}`, title, tone, canStop };
}
