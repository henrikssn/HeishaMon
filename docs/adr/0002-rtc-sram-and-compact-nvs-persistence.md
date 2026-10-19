# Two-Tier State Persistence Using `RTC_NOINIT_ATTR` SRAM and Compact NVS Checkpoints

We will store the rolling-average ring buffers (`ambient_24h`, `ambient_1h`, `room_temp_6h`) and high-frequency accumulators in CRC32-guarded `RTC_NOINIT_ATTR` SRAM, and only write a compact 32-byte scalar summary to ESP32 NVS flash at the daily Heating Day reset and once every 2 hours while the heat pump is running (plus tunable helpers on user change).

## Consequences

- Software reboots, OTA firmware updates, and watchdog resets retain 100% of ring-buffer samples, timers, and integrals in RTC SRAM with zero flash writes.
- Hard power cuts (where RTC SRAM loses power) seed the ring buffers from the 32-byte NVS scalar summary (`ambient_24h`, `ambient_1h`, `room_temp_6h` means) if the checkpoint is under 6 hours old, or from live heat pump sensors otherwise, keeping NVS flash wear under 9 sector erases per year (>10,000 years endurance on the 20 KB NVS partition).
