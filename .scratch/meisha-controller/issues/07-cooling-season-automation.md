# 07: Summer Cooling Season Automation (`Cool Turn On` & `Cool Turn Off`)

**What to build:** Implement autonomous summer cooling startup at `08:00` (when `cooling_mode == on` and `ambient_24h > 24 °C`) and cooling shutoff at `20:00` or when `ambient_24h < 22 °C` continuously for 1 hour, while ensuring all space-heating rules remain suppressed whenever `cooling_mode == on`.

**Blocked by:** 02: User Tunables, Rolling Averages, and Two-Tier Persistence (`RTC_NOINIT_ATTR` + 32-Byte NVS Checkpoint)

**Status:** ready-for-agent

- [ ] `Cool Turn On` triggers at `08:00` local time when `cooling_mode == on` and `ambient_24h > 24.0 °C`: sets operating mode to `Cool only` (`SetOperationMode = 1`), waits `10 seconds` non-blockingly, and turns the heat pump on (`SetHeatpump = 1`).
- [ ] `Cool Turn Off` triggers at `20:00` local time or when `ambient_24h < 22.0 °C` has held continuously for `1 hour`, turning the heat pump off (`SetHeatpump = 0`) if `cooling_mode == on` and the heat pump is on.
- [ ] When `cooling_mode == on`, all winter space-heating automations (`Day Mode #1`, **Bonusgrad**, **Malusgrad**, **Bonusgrad** high-temp, **Night Mode**, and 4-hour heating shutoff) are suppressed.
- [ ] Host C++ tests verify `08:00` cooling start, non-blocking 10-second mode-to-power sequencing, `20:00` and `< 22 °C` (1h) shutoffs, and heating rule suppression during cooling mode.
