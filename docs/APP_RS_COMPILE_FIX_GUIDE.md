# app.rs Compile-Fix Guide (Decision #026)

All structural changes to types, protocol, parser, serial, steering, reader, and main.rs
are complete. The app.rs file has the old imports fixed but still references removed
fields and changed APIs. Here are the remaining compile errors to fix:

## Struct Fields to Remove
- `heading_filter: HeadingFilter` → DELETE field and all references
- `latest_was: Option<WasReading>` → DELETE (WAS now comes via MotorStatus)
- `latest_imu: Option<ImuData>` → DELETE (no IMU)
- `sensor_uptime_ms: Option<u64>` → DELETE (no sensor ESP32)

## Struct Fields to Add (if not present)
- None needed — `latest_motor: Option<MotorStatus>` already exists and now carries WAS data

## Constructor (`new()`) Changes
- Remove: `heading_filter: HeadingFilter::new()`
- Remove: `latest_was: None`, `latest_imu: None`, `sensor_uptime_ms: None`
- Remove: `s.kp_angle = steer_kp_angle` (field no longer on SteeringController)
- Keep loading `steer_kp_angle` from DB but send to ESP32 via $FINNCFG instead

## FINN Message Processing (in `update()`)
Replace the match block:
```rust
// OLD:
FinnMessage::Was(was) => { self.latest_was = Some(was); }
FinnMessage::Imu(imu) => { self.heading_filter.update_imu(&imu); self.latest_imu = Some(imu); }
FinnMessage::SensorHeartbeat(hb) => { self.sensor_uptime_ms = Some(hb.uptime_ms); }
FinnMessage::MotorStatus(mtr) => { self.latest_motor = Some(mtr); }

// NEW:
FinnMessage::MotorStatus(mtr) => {
    self.steering.update_motor_feedback(mtr.current_pwm, mtr.actual_angle);
    self.latest_motor = Some(mtr);
}
FinnMessage::ConfigAck(ack) => {
    let status = if ack.success { "OK" } else { "FAILED" };
    self.was_cal_msg = Some((format!("ESP32 {} config: {}", ack.param, status), 180));
}
```

## Remove heading_filter usage
- Delete: `self.heading_filter.update_gps_fix(&fix);`
- Delete: `let fused_heading = self.heading_filter.current_heading();`
- Replace: `self.interpolator.interpolate(fused_heading)` → `self.interpolator.interpolate(None)`
  (or pass GPS heading directly if interpolator supports it)
- Delete: `self.steering.notify_was_reading()` call (no longer exists)

## Auto-steer compute() call
```rust
// OLD (4 args, returns i16 PWM):
let (raw_pwm, disengaged) = self.steering.compute(
    error.distance_m, error.heading_error, interp_fix.speed, actual_angle);
// ... then apply_motor_direction and send_steer(pwm)

// NEW (3 args, returns f64 desired angle):
let (desired_angle, disengaged) = self.steering.compute(
    error.distance_m, error.heading_error, interp_fix.speed);
if disengaged {
    let _ = self.motor_handle.send_steer_angle(0.0);
    // ... disengage message
} else if should_send {
    let _ = self.motor_handle.send_steer_angle(desired_angle);
    self.last_steer_send = now;
}
```

## send_steer → send_steer_angle
All calls to `self.motor_handle.send_steer(pwm)` become `self.motor_handle.send_steer_angle(angle)`
- Disengage: `send_steer_angle(0.0)` (zero angle = centre wheels)
- Motor test section: needs rethinking — test buttons could send fixed angles
  (e.g. +5°, -5°, +10°) instead of raw PWM, OR we could add a `send_raw_pwm()`
  method on MotorHandle for test-only use (separate from the steering protocol)

## SENSORS section in setup page
- Remove IMU display block (no `latest_imu`)
- Remove fused heading display (no `heading_filter`)
- WAS display: get from `latest_motor` instead of `latest_was`:
```rust
if let Some(mtr) = &self.latest_motor {
    ui.label(format!("WAS: {} raw  Angle: {:.1}°", mtr.was_raw, mtr.actual_angle));
    let en_label = if mtr.enabled { "ON" } else { "OFF" };
    ui.label(format!("Motor: PWM {} [{}]", mtr.current_pwm, en_label));
    let secs = mtr.uptime_ms / 1000;
    ui.label(format!("ESP32 uptime: {}m {}s", secs / 60, secs % 60));
}
```
- Remove separate heartbeat/uptime display

## WAS Calibration section
- `has_was_data` check: `self.latest_motor.is_some()` instead of `self.latest_was.is_some()`
- Live readout: `self.latest_motor.as_ref().map(|m| m.was_raw)` instead of `self.latest_was`
- On calibration button clicks: additionally call `self.motor_handle.send_was_config(c, l, r)`
  after saving to local DB (send config to ESP32 NVS)

## Motor Direction section
- On invert toggle: additionally call `self.motor_handle.send_invert_config(self.motor_invert)`

## Inner loop sliders (Kp_angle, min_pwm, max_pwm, angle_deadband)
These fields no longer exist on SteeringController. Options:
1. Keep as local app state, send to ESP32 via $FINNCFG,PID on change
2. Remove sliders entirely (tune via reflashing or serial terminal)

Recommended: keep sliders, store values locally, send $FINNCFG,PID on change.

## Methods to remove from GuidanceApp
- `was_calibrated_angle()` — calibration now done on ESP32
- `apply_motor_direction()` — motor invert now applied on ESP32
