// Readback only. Motor shaft position is never converted to a cylinder angle.
export function cylinderReadout(lift, live, telemetryAgeS, maxAgeS) {
  const age = Number.isFinite(lift?.sample_age_s) && lift.sample_age_s >= 0
    ? lift.sample_age_s + Math.max(0, telemetryAgeS) : null;
  const current = !!(live && lift?.connected === true && age !== null && age <= maxAgeS
    && Number.isInteger(lift.position_raw) && Number.isFinite(lift.motor_shaft_deg));
  const torque = lift?.torque_enabled === true ? 'ON' : lift?.torque_enabled === false ? 'OFF' : '--';
  return {
    current,
    status: current ? (lift?.control_mode === 'manual' ? 'Manual' : 'Read only') : live && lift?.connected ? 'Stale readback' : 'No readback',
    raw: current ? String(lift.position_raw) : '--',
    shaft: current ? lift.motor_shaft_deg.toFixed(2) + '°' : '--',
    note: 'Reference only · ' + (age === null ? 'Sample age unknown' : age.toFixed(2) + ' s old')
      + ' · Torque ' + torque + (lift?.control_mode === 'manual' ? ' · Hold UP / DOWN' : ' · Read only'),
  };
}

export class CylinderView {
  constructor(root, maxAgeS) {
    this.root = root;
    this.maxAgeS = maxAgeS;
  }

  update(lift, live, telemetryAgeS = 0) {
    const value = cylinderReadout(lift, live, telemetryAgeS, this.maxAgeS);
    this.root.dataset.fresh = String(value.current);
    for (const name of ['status', 'raw', 'shaft', 'note']) {
      const element = this.root.querySelector('#cylinder-' + name);
      if (element.textContent !== value[name]) element.textContent = value[name];
    }
  }
}
