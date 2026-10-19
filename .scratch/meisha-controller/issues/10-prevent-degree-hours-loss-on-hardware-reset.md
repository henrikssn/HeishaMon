# 10: Prevent Degree-Hours and Runtime State Loss on Hardware Reset / Power Cycle

**What to build:** Ensure runtime degree-hour accumulation (`degree_hours_day`), DHW run credits, and rolling average snapshots are reliably preserved against hardware button resets and sudden power cuts by checkpointing immediately on heat pump shutoff and shortening periodic in-flight checkpoints to 15 minutes.

**Blocked by:** None (can start immediately)

**Status:** ready-for-agent

## Background & Problem

1. **Hardware Reset Wipes RTC SRAM**:
   Pressing the physical reset button pulls the ESP32 `EN` pin to ground, cutting power to internal regulators and clearing the volatile `RTC_NOINIT_ATTR` fast memory. On reboot, `rtc_valid` evaluates to false, forcing the controller to recover its state from the NVS flash checkpoint (`MeishaNvsCheckpoint`).

2. **Checkpoint Cycling Gap Bug**:
   In `HeishaMon/meisha.cpp`, the 2-hour checkpoint timer is implemented as:
   ```cpp
   int hp_state = getDataValue(actData, 0).toInt();
   if (hp_state == 1) {
     if (s_last_checkpoint_ms == 0) {
       s_last_checkpoint_ms = millis();
     } else if ((millis() - s_last_checkpoint_ms) >= 7200000UL) {
       s_last_checkpoint_ms = millis();
       write_nvs_checkpoint();
     }
   } else {
     s_last_checkpoint_ms = millis();
   }
   ```
   Whenever `hp_state == 0`, `s_last_checkpoint_ms` is continually reset to `millis()`. When the heat pump cycles off before reaching 2 continuous hours of compressor runtime, the timer is wiped and no checkpoint is ever written during normal cycling. Furthermore, no checkpoint is saved when the heat pump stops.

3. **Consequence**:
   A hardware reset or power cycle causes `degree_hours_day` to roll back to a checkpoint that is multiple hours old (or to 0 if none was written today), causing the heat pump to re-deliver heating degree-hours and overheat the house.

---

## Acceptance Criteria

- [ ] **Checkpoint on Heat Pump Stop**: Automatically invoke `write_nvs_checkpoint()` whenever `Heatpump_State` transitions from `1` (running) to `0` (off) or changes to DHW operation.
- [ ] **15-Minute Periodic Checkpoint**: Update the in-flight checkpoint interval from 2 hours (`7,200,000 ms`) to 15 minutes (`900,000 ms`) while `Heatpump_State == 1`.
- [ ] **Persistent Cycle Accumulator**: Ensure periodic checkpoint timing tracks cumulative running time across cycles rather than resetting to `millis()` on short off-cycles.
- [ ] **Zero-Data-Loss Idle Reset**: Verify that resetting the ESP32 while the heat pump is idle preserves 100% of the accumulated `degree_hours_day` from the previous heating cycle.
- [ ] **Active-Cycle Reset Cap**: Verify that resetting the ESP32 while the heat pump is running rolls back at most 15 minutes of accumulated degree-hours.
- [ ] **Automated Test Coverage**: Add unit tests in `tests/test_ticket_02.cpp` or a new test suite verifying that:
  - Checkpoint writes trigger on $1 \to 0$ `Heatpump_State` transitions.
  - A 15-minute running interval triggers an NVS write.
  - Simulating an invalid RTC CRC after an on-stop or 15-min checkpoint correctly restores the near-current `degree_hours_day` accumulator.
