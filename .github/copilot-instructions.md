# AI Agent Working Guide — ESP32 Self-Balancing Robot (Arduino)

This repo is a single-file Arduino sketch for an ESP32 self-balancing robot with:
- MPU6050 (DMP) for orientation
- PID loop driving two motors via LEDC PWM
- WiFi setup via WiFiManager + captive portal
- Web UI + JSON API using ESPAsyncWebServer
- EEPROM persistence for PID and setpoint

Keep edits safe (motors), non-blocking (web/UI), and consistent with the current hardware assumptions.

## Key file and hardware assumptions
- Sketch: `PID_Auto_Tuner_Robot.ino` (single compilation unit)
- Pins: motors A (A_IN1=27, A_IN2=14), motors B (B_IN3=26, B_IN4=12), MPU INT=2, LED=13, I2C SDA=21, SCL=22
- PWM (LEDC): 5 kHz, 8-bit. Channels: CH_A1=0, CH_A2=1, CH_B1=2, CH_B2=3 (see `applyMotor()`)
- IMU: `MPU6050_6Axis_MotionApps20` (DMP enabled); uses I2Cdevlib

## Runtime architecture
- Startup
  - WiFiManager `autoConnect()` may open AP “ESP32-Robot-Setup” (password `configure123`) and blocks until configured; on failure the device restarts.
  - Motor pins are OUTPUT+LOW; LEDC channels are set; motors are explicitly OFF.
- Main loop
  - Reads a DMP packet; if not ready, motors stay OFF and loop returns.
  - Computes PID (with first-cycle guard to avoid dt spike), applies signed output to both motors.
  - Tuning mode optionally perturbs Kp/Ki/Kd and evaluates a 2s error metric (position + small gyro penalty), persisting best to EEPROM.
- Web server (Async)
  - Routes: `/` (UI), `/status` (JSON), `/setPID`, `/startTune`, `/tuneStep`

## Safety and motor control
- Motors remain OFF until DMP is ready and first valid time base is established.
- `applyMotor(m, spd, dir)` writes to LEDC channels, not pin numbers. `spd` is 0–255; `dir` > 0 forward, else reverse.
- When changing PID or starting a tuning step, PID state and timing are reset to avoid spikes.
- Keep any added code from re-enabling motors during faults; prefer “cut to zero” on sensor/timeouts.

## PID, IMU, and timing
- PID variables: `kp, ki, kd, setpoint`; state: `integral, previousError, prevTimePID`.
- First loop after reset sets `prevTimePID` and keeps motors OFF (prevents large derivative term).
- IMU readings: pitch from DMP (`ypr[1]` as degrees), gyro pitch rate from `gy`/131.
- Tuning: `tuneParams[]`, `deltas[]`, 2s windows; best error shrinks deltas up, otherwise reduces and flips.

## Web API and UI
- `/status` returns: `{ pitch, speed, kp, ki, kd, setpoint }` (numeric values as strings built with `String`).
- `/setPID` accepts `kp, ki, kd, setpoint` as query params; after update, PID state is reset and values are saved to EEPROM.
- UI is a small inline HTML/JS page stored in PROGMEM; uses `fetch` and `setInterval` at 1s.
- Keep handlers small and non-blocking; avoid dynamic allocations that bloat heap.

## Persistence (EEPROM)
- Layout: `ADDR_KP=0`, `ADDR_KI=4`, `ADDR_KD=8`, `ADDR_SETPOINT=12` (floats; `EEPROM_SIZE=16`).
- After writes, `EEPROM.commit()` is required; values are loaded on boot. If `kp` is NaN, defaults are applied.

## Build/flash prerequisites (Arduino-ESP32)
- Board: an ESP32 Dev Module (Arduino core). Baud: 115200.
- Libraries required:
  - WiFiManager (tzapu/WiFiManager)
  - ESPAsyncWebServer and AsyncTCP (me-no-dev)
  - I2Cdevlib (MPU6050 with MotionApps20 header)
  - EEPROM (bundled with ESP32 core)
- I2C: `Wire.begin(21, 22)`; ensure MPU6050 is on those pins and INT on pin 2.

## Patterns to follow when extending
- New web endpoints: `server.on("/path", HTTP_GET, [](auto*r){ /* read params, do work, r->send(...) */ });`
- When changing control parameters: reset `integral`, `previousError`, and `prevTimePID`.
- If writing to EEPROM: `EEPROM.put(...); EEPROM.commit();`
- For motor logic: call `applyMotor("A", spd, dir)` and `applyMotor("B", spd, dir)`; keep direction symmetric unless intentionally differential.
- For safety: gate on IMU readiness and add timeouts to cut motors to zero.

## Common pitfalls
- Don’t call `ledcWrite` with pin numbers—use channels defined at top.
- Avoid blocking code in `loop()` or HTTP handlers (WiFiManager is the only acceptable blocker at boot).
- Handle the “no DMP packet” case by keeping motors OFF.
- Keep JSON small and avoid heavy string concatenation in hot paths.

Questions or gaps? If you need exact board, library versions, or want to add an arm/estop endpoint and IMU timeout, ask and we’ll clarify before changing behavior.