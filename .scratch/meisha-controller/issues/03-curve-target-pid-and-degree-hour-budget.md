# 03: Heating Day Reset, Curve Target, I-Only PID, Power Target, and Degree-Hour Budget

**What to build:** Compute and publish all derived heating targets, budgets, and runtime credits across the **Heating Day**: the daily reset at `heating_start_hour:00` with a `0.1 °C` resolution room temperature snapshot, **Curve Target** with `±2 K` indoor offset clamping, the 10-second I-only PID controller (`Ki = 5e-5 /s`, `[-1, +1]` clamp), **Power Target** (`max(27, curve_target - pid_output)` with `±0.1 K` integer hysteresis), `compressor_running` hysteresis binary state, **Degree-Hours Day** integration, **Degree-Hour Budget**, `dhw_time`, `hours_target`, and `hours_time_left`.

**Blocked by:** 02: User Tunables, Rolling Averages, and Two-Tier Persistence (`RTC_NOINIT_ATTR` + 32-Byte NVS Checkpoint)

**Status:** closed

- [x] At `local_hour == heating_start_hour && local_minute == 0` (once per day), the controller snapshots `room_temp_snapshot = round1(room_temp_6h)` (`0.1 °C` resolution), resets **Degree-Hours Day** (`degree_hours_day = 0.0`) and `dhw_time = 0.0`, and writes the daily 32-byte NVS checkpoint.
- [x] **Curve Target** is computed as `round2(round2(room_target + (heating_curve_start - ambient_24h) * heating_curve_angle) + clamp(-2.0, room_target - room_temp_snapshot, +2.0) * heating_curve_angle)`.
- [x] Every 10 seconds, the I-only PID controller samples `curve_error = (pump_flow > 0 && valve == Room) ? (curve_target - main_outlet_temp) : 0.0` with `pid_error = -curve_error`, updates `pid_output = clamp(pid_output + 5e-5 * 10.0 * pid_error, -1.0, +1.0)` with anti-windup, and computes **Power Target** as `clamp(round(max(27.0, curve_target - pid_output) - 0.1), prev_power_target, round(max(27.0, curve_target - pid_output) + 0.1))`.
- [x] `compressor_running` binary state applies `10.0 Hz` upper threshold with `2.0 Hz` hysteresis (turning `on` above `12.0 Hz` and `off` below `8.0 Hz`).
- [x] `room_delta = (pump_flow > 0 && valve == Room) ? (main_outlet_temp - room_target) : 0.0` is integrated over time in Kelvin-hours into **Degree-Hours Day** (`degree_hours_day`), and compared against **Degree-Hour Budget** (`degree_hours_target = 24.0 * (curve_target - room_target)`).
- [x] `dhw_time` accumulates hours spent with `valve == Tank` since `heating_start_hour:00`, `hours_target` is computed as `clamp(0.0, (curve_target - room_target) / (30.0 - room_target) * 24.0, 24.0)`, and `hours_time_left` is computed as `max(0.0, min(24.0, hours_target + dhw_time) - hours_elapsed_since_start_hour)`.
- [x] All derived sensors (`room_temp_snapshot`, `curve_target`, `pid_output`, `power_target`, `room_delta`, `degree_hours_day`, `degree_hours_target`, `dhw_time`, `hours_target`, `hours_time_left`, `compressor_running`) are published via MQTT Auto-Discovery and verified by host C++ unit tests.
