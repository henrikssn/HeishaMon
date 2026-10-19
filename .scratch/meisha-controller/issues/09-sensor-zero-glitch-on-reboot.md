# 09: Bug: Controller Publishes Zero Values for Telemetry and Rolling Averages on Boot

**What to fix:** Eliminate transient zero (`0`, `0.0`, `0.00`) state publications occurring on HeishaMon boot, reset, and MQTT connection before heat pump telemetry is received, by gating rolling average and derived sensor emissions on `rtc_state.seeded` in [`HeishaMon/meisha.cpp`](file:///agy/esphome-aquarea/HeishaMon/meisha.cpp).

**Type:** Bug / Reliability

**Status:** closed

## Summary

When HeishaMon reboots or reconnects to MQTT, `meisha_mqtt_connected()` immediately calls `publish_rolling_average_states()` and `publish_derived_states()` with MQTT `retain: true`. On a cold boot or whenever RTC memory has been cleared, `rtc_state` contains uninitialized `0.0` values because **Live Seeding** has not yet occurred. As a result, the controller publishes retained `0.0` / `0` values across all rolling averages and derived sensors.

When the first serial datagram from the heat pump arrives approximately 1–2 seconds later, real values are computed and published, creating a momentary zero spike in Home Assistant history and the recorder database.

---

## Evidence from Home Assistant Recorder (MariaDB)

During HeishaMon reboots (e.g. `2026-10-19 14:23:34`), the following transient zero values were recorded before real telemetry arrived 1 second later:

| Timestamp | Entity ID | State Transition |
| :--- | :--- | :--- |
| `14:23:34.244` -> `14:23:35.179` | `sensor.meisha_ambient_1h_2` | `0.0` -> `11.0` |
| `14:23:34.245` -> `14:23:35.179` | `sensor.meisha_ambient_24h_2` | `0.0` -> `11.0` |
| `14:23:34.245` -> `14:23:35.179` | `sensor.meisha_room_temp_6h_2` | `0.0` -> `23.0` |
| `14:23:34.245` -> `14:23:35.180` | `sensor.meisha_room_temp_snapshot` | `0.0` -> `23.0` |
| `14:23:34.252` -> `14:23:35.180` | `sensor.meisha_curve_target` | `0.00` -> `25.40` |
| `14:23:34.253` -> `14:23:35.181` | `sensor.meisha_pid_output` | `0.0000` -> real value |
| `14:23:34.254` -> `14:23:35.181` | `sensor.meisha_power_target` | `0` -> `27` |
| `14:23:34.261` -> `14:23:35.181` | `sensor.meisha_degree_hours_target` | `0.00` -> `57.60` |
| `14:23:34.264` -> `14:23:35.182` | `sensor.meisha_pipe_ambient_delta_2` | `0.0` -> `3.0` |

---

## Root Cause

In `HeishaMon/meisha.cpp`:
```cpp
void meisha_mqtt_connected(void) {
  char sub_topic[160];
  snprintf(sub_topic, sizeof(sub_topic), "%s/meisha/set/#",
           heishamonSettings.mqtt_topic_base);
  mqtt_client.subscribe(sub_topic);

  publish_ha_discovery();
  publish_mode_state();
  publish_all_tunables_state();
  publish_rolling_average_states(); // <-- Unconditionally called before seeded
  publish_derived_states();         // <-- Unconditionally called before seeded
}
```
If `rtc_state.seeded == 0` (cold start or pending SNTP/telemetry), `ambient_1h_mean`, `ambient_24h_mean`, `curve_target`, `power_target`, etc., are all `0.0` / `0`.

---

## Agreed Solution

1. In `meisha_mqtt_connected()`, guard the calls to `publish_rolling_average_states()` and `publish_derived_states()` with `if (rtc_state.seeded)`.
   - If `rtc_state.seeded == 1` (warm reboot / OTA update where RTC SRAM survived with valid CRC32, or warm NVS recovery), publish immediately.
   - If `rtc_state.seeded == 0`, suppress publication.
2. In `meisha_process_heatpump_data()`, once live seeding or deferred SNTP recovery sets `rtc_state.seeded = 1`, the populated non-zero states are published as normal.
3. Keep `retain: true` on all valid seeded telemetry publications.

---

## Acceptance Criteria

- [x] On cold boot or when `rtc_state.seeded == 0`, `meisha_mqtt_connected()` publishes HA discovery, control mode, and user tunables, but does NOT publish rolling averages or derived metrics.
- [x] On warm boot where `rtc_state.seeded == 1` (valid RTC SRAM or warm NVS checkpoint), `meisha_mqtt_connected()` publishes rolling averages and derived metrics immediately.
- [x] In `meisha_process_heatpump_data()`, once `rtc_state.seeded` becomes `1`, populated non-zero rolling averages and derived metrics are published via MQTT.
- [x] Host C++ tests in `tests/test_ticket_09.cpp` verify that unseeded MQTT connection events emit no zero values for rolling averages or derived metrics.
