// Voltage warnings never block operator controls.
export function batteryLevel(reading, limits) {
  if (!Number.isFinite(reading?.v) || !Number.isFinite(limits?.low_v)) return 'unavailable';
  return reading.v <= limits.low_v ? 'low' : 'ok';
}

export function batteryNotice(state, reading, limits) {
  const volts = state?.voltage_v ?? reading?.v;
  const low = state?.level === 'low' || batteryLevel(reading, limits) === 'low';
  return { text: low && Number.isFinite(volts)
    ? `${state?.simulated ? 'SIMULATED · ' : ''}LOW BATTERY · ${volts.toFixed(2)} V · Replace or recharge battery`
    : '', tone: 'warn' };
}
