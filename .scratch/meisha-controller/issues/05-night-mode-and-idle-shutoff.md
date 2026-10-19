# 05: Space Heating Budget Cutoff (`Night Mode`) and 4-Hour Idle Shutoff (`Turn Off`)

**What to build:** Implement **Night Mode** setback (`25 °C` when **Degree-Hours Day** exceeds **Degree-Hour Budget** outside the 2-hour pre-start blackout window) and the 4-hour idle shutoff when **Heat Pump Setpoint** remains `< 25.1 °C` continuously for 4 hours while heating the room loop.

**Blocked by:** 03: Heating Day Reset, Curve Target, I-Only PID, Power Target, and Degree-Hour Budget

**Status:** ready-for-agent

- [ ] **Night Mode** triggers whenever `cooling_mode == off`, the heat pump is on, `degree_hours_day > degree_hours_target`, the current local hour is not within the 2 hours preceding `heating_start_hour` (`((heating_start_hour - local_hour + 24) % 24) not in {1, 2}`), and `heat_pump_setpoint != 25`, setting **Heat Pump Setpoint** (`SetZ1HeatRequestTemperature`) to `25 °C`.
- [ ] `Turn Off` (heat mode only) triggers when `cooling_mode == off`, the heat pump is on, `valve == Room`, operating mode is not `DHW only`, and `heat_pump_setpoint < 25.1 °C` has held continuously for `4 hours`: sets operating mode to `Heat only`, waits `10 seconds` non-blockingly, re-verifies `valve == Room` and operating mode != `DHW only`, and turns the heat pump off (`SetHeatpump = 0`).
- [ ] Host C++ tests verify **Night Mode** budget cutoff, the dynamic 2-hour pre-start blackout window relative to `heating_start_hour`, the 4-hour idle timer, non-blocking 10-second turn-off sequence, and DHW cycle safety guards.
