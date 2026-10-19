# 08: Evaporator-Icing Protection (`Quiet Mode FM Off / FM On`) and ESP32-S3 Firmware Build Verification

**What to build:** Implement evaporator-icing protection (`Quiet Mode` `FM Off` to level `0` under heavy or freezing load, and `FM On` recovery to level `1` after 2 hours without defrost at light load), publish `pipe_ambient_delta` via MQTT Auto-Discovery, and verify that the complete ESP32-S3 (`HeishaMon Large`) firmware binary compiles cleanly with the `min_spiffs.csv` partition layout for Web UI `/firmware` OTA deployment.

**Blocked by:** 01: HeishaMon Fork, Host Test Harness, Mode Control (`Off` / `Shadow` / `Active`), and Crash-Loop Safeguard, 04: Space Heating Start (`Day Mode #1`) and Flow Temperature Modulation (`Bonusgrad` & `Malusgrad`), 05: Space Heating Budget Cutoff (`Night Mode`) and 4-Hour Idle Shutoff (`Turn Off`), 06: Domestic Hot Water (`DHW Start` & `DHW End`) Cycle Control, 07: Summer Cooling Season Automation (`Cool Turn On` & `Cool Turn Off`)

**Status:** ready-for-agent

- [ ] `pipe_ambient_delta = outside_temp - outside_pipe_temp` is computed from decoded telemetry and published via MQTT Auto-Discovery.
- [ ] `Quiet Mode FM Off` sets Quiet Mode to `0` (`SetQuietMode = 0`) whenever `quiet_mode_level != 0`, (`pipe_ambient_delta > 4.0` or (`pipe_ambient_delta > 3.0` and `outside_temp < 1.1`)), and any trigger duration holds (`compressor_freq > 25.0 Hz` for `1 min`, `pipe_ambient_delta > 4.0 K` for `5 min`, `pipe_ambient_delta > 3.0 K` for `5 min`, or `outside_temp < 1.1 °C` for `5 min`).
- [ ] `Quiet Mode FM On` sets Quiet Mode to `1` (`SetQuietMode = 1`) whenever `quiet_mode_level == 0`, `1.0 < compressor_freq < 23.5 Hz`, `defrosting_state == 0` continuously for `2 hours`, `pipe_ambient_delta < 3.1 K`, (`pipe_ambient_delta < 2.1 K` or `outside_temp > 1.0 °C`), and either (`1.0 < compressor_freq < 23.5 Hz` for `1 min`) or (`pipe_ambient_delta` drops below `4.0 K`).
- [ ] Host C++ tests verify `FM Off` and `FM On` triggers, conditions, and the 2-hour post-defrost guard.
- [ ] The full HeishaMon + `MeishaController` firmware compiles cleanly for `esp32:esp32:esp32s3` (`HeishaMon Large`, `min_spiffs.csv` OTA layout) producing a `.bin` image ready for Web UI `/firmware` OTA upload in **Shadow Mode**.
