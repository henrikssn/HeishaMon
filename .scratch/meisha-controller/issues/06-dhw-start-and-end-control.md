# 06: Domestic Hot Water (`DHW Start` & `DHW End`) Cycle Control

**What to build:** Implement autonomous Domestic Hot Water (DHW) cycle start (`14:00` daily schedule or `DHW_Temp < 38 °C` for 1 minute) and DHW completion (`valve` `Tank -> Room` for 15 minutes or compressor off for 15 minutes while in `DHW only`), restoring the appropriate seasonal mode (`Heat only` or `Cool only`) and conditionally powering down the heat pump if no space heating or cooling is needed.

**Blocked by:** 03: Heating Day Reset, Curve Target, I-Only PID, Power Target, and Degree-Hour Budget

**Status:** closed

- [x] `DHW Start` triggers at `14:00` local time or when `dhw_temp < 38.0 °C` continuously for `1 minute`, provided `valve != Tank` and `dhw_temp < 45.0 °C`: turns the heat pump on (`SetHeatpump = 1`) if currently off, waits `10 seconds` non-blockingly, and sets operating mode to `DHW only` (`SetOperationMode = 3`).
- [x] `DHW End` triggers while operating mode is `DHW only` whenever (`valve` transitions from `Tank` to `Room` and remains `Room` continuously for `15 minutes`) or (`compressor_running == off` continuously for `15 minutes`).
- [x] When `DHW End` fires with `cooling_mode == on`, the controller sets operating mode to `Cool only` (`SetOperationMode = 1`), waits `10 seconds` non-blockingly, and turns the heat pump off if `ambient_24h < 22.0 °C`.
- [x] When `DHW End` fires with `cooling_mode == off`, the controller sets operating mode to `Heat only` (`SetOperationMode = 0`) and turns the heat pump off if `hours_time_left < 0.1`.
- [x] Host C++ tests verify scheduled and low-temperature DHW starts, non-blocking 10-second sequencing, both 15-minute completion triggers, and winter vs. summer mode restoration.
