# 01: HeishaMon Fork, Host Test Harness, Mode Control (`Off` / `Shadow` / `Active`), and Crash-Loop Safeguard

**What to build:** Bring the HeishaMon firmware codebase into the repository, wire the `MeishaController` lifecycle hooks (`setup`, `loop`, `mqtt_reconnect`, `mqtt_callback`) into HeishaMon, stand up a Linux host C++ test harness mocking the HeishaMon/Arduino externs, expose the controller operating mode selector (`Off`, **Shadow Mode**, `Active`) over MQTT Auto-Discovery with NVS persistence, enforce the crash-loop anti-brick safeguard (`> 3` abnormal resets within 5 minutes forces `Off`), and configure a Home Assistant notification when the controller transitions to `Off`.

**Blocked by:** None (can start immediately)

**Status:** resolved

- [x] HeishaMon firmware source is added to the repository with `MeishaController` hooked into `setup()`, `loop()`, `mqtt_reconnect()`, and `mqtt_callback()`, gated so controller ticks wait until the first valid `0x10` heat pump frame (`actData[0] == 0x71`) has been decoded.
- [x] A Linux host C++ test suite compiles and runs `MeishaController` against mocked HeishaMon/Arduino primitives (`getDataValue`, `send_heatpump_command`, `mqtt_client`, `Preferences`, `millis`, `localtime`, `esp_reset_reason`, and `RTC_NOINIT_ATTR` SRAM).
- [x] `select.meisha_esp_control_mode` (`Off`, `Shadow`, `Active`) is registered via Home Assistant MQTT Auto-Discovery (using the configurable `meisha_esp_` prefix), defaults to `Shadow` on first boot, persists changes to NVS immediately when changed over MQTT, and gates `send_heatpump_command()` so that only `Active` mode emits heat pump commands while **Shadow Mode** still runs calculations and publishes telemetry.
- [x] Crash-loop anti-brick safeguard tracks abnormal resets (`ESP_RST_PANIC`, `ESP_RST_INT_WDT`, `ESP_RST_TASK_WDT`, `ESP_RST_WDT`) in `RTC_NOINIT_ATTR` SRAM: 1–3 crashes within 5 minutes or normal/brownout resets (`ESP_RST_BROWNOUT`, `ESP_RST_POWERON`, `ESP_RST_SW`) preserve the saved `control_mode` and clear the crash counter after 5 minutes of continuous uptime (`millis() >= 300000`), whereas `> 3` crashes within 5 minutes force `control_mode` to `Off` in NVS and MQTT and skip loop execution.
- [x] A Home Assistant automation alerts `notify.all_devices` whenever the controller's mode entity transitions to `Off`.
