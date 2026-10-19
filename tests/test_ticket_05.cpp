#include <cassert>
#include <iostream>
#include <cmath>
#include "mock_heishamon.h"
#include "../HeishaMon/meisha.h"

void test_night_mode_budget_cutoff() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  set_mock_reset_reason(ESP_RST_POWERON);
  // Start at 09:00:00 on Day 1 (heating_start_hour:00)
  set_mock_time(1761987600);
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);

  actData[0] = 0x71;
  set_mock_topic_value(0, "1");     // Heat pump ON
  set_mock_topic_value(14, "10.0"); // Outside_Temp = 10.0
  set_mock_topic_value(33, "23.0"); // Room_Temp = 23.0
  set_mock_topic_value(1, "10.0");  // Flow > 0
  set_mock_topic_value(20, "Room"); // Valve = Room
  set_mock_topic_value(6, "33.0");  // Outlet = 33.0 (room_delta = 33 - 23 = 10.0 K)
  set_mock_topic_value(7, "28.0");  // Target = 28.0
  set_mock_topic_value(27, "28.0"); // Heat Pump Setpoint = 28.0
  set_mock_topic_value(4, "0");     // OpMode = Heat only

  meisha_loop();
  g_mock_commands.clear();

  // Verify initially degree_hours_day == 0, degree_hours_target > 0, so Night Mode must NOT trigger
  const MeishaRtcState* rtc = meisha_get_rtc_state();
  assert(rtc->degree_hours_day <= rtc->degree_hours_target);
  assert(g_mock_commands.empty());

  // Advance 10 hours to 19:00:00 UTC (10h * 10 K = 100 Kh, exceeding target ~76.8 Kh)
  for (int i = 0; i < 10; i++) {
    advance_mock_time(3600);
    advance_mock_millis(3600000UL);
    meisha_loop();
  }

  // Verify degree_hours_day > degree_hours_target
  assert(rtc->degree_hours_day > rtc->degree_hours_target);

  // Night Mode MUST trigger and emit SetZ1HeatRequestTemperature = 25
  bool found_setpoint_25 = false;
  for (const auto& cmd : g_mock_commands) {
    if (cmd.cmd == "SetZ1HeatRequestTemperature" && cmd.value == "25") {
      found_setpoint_25 = true;
    }
  }
  assert(found_setpoint_25);
  assert(std::string(meisha_get_last_action_reason()) == "night_mode");

  // Deduplication check: on the next loop iteration before actData[27] updates to 25,
  // it must NOT re-emit the command repeatedly!
  g_mock_commands.clear();
  advance_mock_millis(1000);
  advance_mock_time(1);
  meisha_loop();
  assert(g_mock_commands.empty());

  // Now simulate heat pump confirming setpoint is 25.0
  set_mock_topic_value(27, "25.0");
  advance_mock_millis(1000);
  advance_mock_time(1);
  meisha_loop();
  assert(g_mock_commands.empty());

  // If setpoint is manually changed back to 28 while budget is still exceeded:
  set_mock_topic_value(27, "28.0");
  advance_mock_millis(1000);
  advance_mock_time(1);
  meisha_loop();

  found_setpoint_25 = false;
  for (const auto& cmd : g_mock_commands) {
    if (cmd.cmd == "SetZ1HeatRequestTemperature" && cmd.value == "25") {
      found_setpoint_25 = true;
    }
  }
  assert(found_setpoint_25);

  std::cout << "[PASS] test_night_mode_budget_cutoff" << std::endl;
}

void test_night_mode_pre_start_blackout_window() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  set_mock_reset_reason(ESP_RST_POWERON);
  // Default heating_start_hour is 9.
  // Start at 09:00:00 on Day 1 (1761987600 = 2025-11-01 09:00:00 UTC)
  set_mock_time(1761987600);
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);

  actData[0] = 0x71;
  set_mock_topic_value(0, "1");     // HP ON
  set_mock_topic_value(14, "10.0");
  set_mock_topic_value(33, "23.0");
  set_mock_topic_value(1, "10.0");
  set_mock_topic_value(20, "Room");
  set_mock_topic_value(6, "33.0");  // room_delta = 10.0 K
  set_mock_topic_value(7, "28.0");
  set_mock_topic_value(27, "28.0"); // HP Setpoint = 28
  set_mock_topic_value(4, "0");

  meisha_loop();

  // Advance 21 hours forward to 06:00:00 UTC next morning (1762063200)
  // During these 21 hours, degree-hours accumulate (10.0 K * 21h = 210 Kh > target ~76.8 Kh)
  for (int i = 0; i < 21; i++) {
    advance_mock_time(3600);
    advance_mock_millis(3600000UL);
    meisha_loop();
  }

  // 1. At 06:00 UTC: hour 6 is 3 hours before 9 -> OUTSIDE blackout window
  // Night Mode MUST trigger and emit SetZ1HeatRequestTemperature = 25
  const MeishaRtcState* rtc = meisha_get_rtc_state();
  assert(rtc->degree_hours_day > rtc->degree_hours_target);
  bool found_sp25 = false;
  for (const auto& cmd : g_mock_commands) {
    if (cmd.cmd == "SetZ1HeatRequestTemperature" && cmd.value == "25") found_sp25 = true;
  }
  assert(found_sp25);

  // 2. Advance 1 hour forward to 07:00 UTC: hour 7 is 2 hours before 9 -> IN BLACKOUT WINDOW
  // Simulate user changing setpoint back to 28.0
  advance_mock_time(3600);
  advance_mock_millis(3600000UL);
  set_mock_topic_value(27, "28.0");
  g_mock_commands.clear();
  meisha_loop();
  // In blackout window, Night Mode must be inhibited!
  assert(g_mock_commands.empty());

  // 3. Advance 1 hour forward to 08:00 UTC: hour 8 is 1 hour before 9 -> IN BLACKOUT WINDOW
  advance_mock_time(3600);
  advance_mock_millis(3600000UL);
  set_mock_topic_value(27, "28.0");
  g_mock_commands.clear();
  meisha_loop();
  // Still in blackout window, Night Mode must be inhibited!
  assert(g_mock_commands.empty());

  // 4. Test dynamic change of heating_start_hour:
  // While at 08:00 UTC, change heating_start_hour to 12.
  // Now hour 8 is 4 hours before 12 -> OUTSIDE blackout window!
  meisha_set_heating_start_hour(12);
  g_mock_commands.clear();
  meisha_loop();
  found_sp25 = false;
  for (const auto& cmd : g_mock_commands) {
    if (cmd.cmd == "SetZ1HeatRequestTemperature" && cmd.value == "25") found_sp25 = true;
  }
  assert(found_sp25);

  // With heating_start_hour = 12, blackout window is now hours 10 and 11.
  // Advance to 10:30 UTC: in blackout window for 12!
  // From 08:00 to 10:30 = 2.5 hours
  advance_mock_time(9000);
  advance_mock_millis(9000000UL);
  set_mock_topic_value(27, "28.0");
  g_mock_commands.clear();
  meisha_loop();
  assert(g_mock_commands.empty());

  // 5. Test wrap-around: heating_start_hour = 0 (midnight)
  meisha_set_heating_start_hour(0);
  // Blackout window for 0: hours 22 and 23.
  // Advance to 21:00 UTC: hour 21 is 3 hours before 0 -> OUTSIDE blackout
  // 10:30 to 21:00 = 10.5 hours = 37,800 sec
  advance_mock_time(37800);
  advance_mock_millis(37800000UL);
  set_mock_topic_value(27, "28.0");
  g_mock_commands.clear();
  meisha_loop();
  found_sp25 = false;
  for (const auto& cmd : g_mock_commands) {
    if (cmd.cmd == "SetZ1HeatRequestTemperature" && cmd.value == "25") found_sp25 = true;
  }
  assert(found_sp25);

  // Advance to 22:30 UTC: hour 22 is 2 hours before 0 -> IN BLACKOUT WINDOW
  advance_mock_time(5400); // 1.5 hours
  advance_mock_millis(5400000UL);
  set_mock_topic_value(27, "28.0");
  g_mock_commands.clear();
  meisha_loop();
  assert(g_mock_commands.empty());

  std::cout << "[PASS] test_night_mode_pre_start_blackout_window" << std::endl;
}

void test_idle_shutoff_4h_and_10s_sequence() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(1762027200); // 20:00:00 UTC
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);

  actData[0] = 0x71;
  set_mock_topic_value(0, "1");     // HP ON
  set_mock_topic_value(14, "10.0");
  set_mock_topic_value(33, "23.0");
  set_mock_topic_value(1, "10.0");
  set_mock_topic_value(20, "Room"); // Valve = Room
  set_mock_topic_value(4, "4");     // OpMode = Heat+DHW
  set_mock_topic_value(27, "25.0"); // HP Setpoint = 25.0 (< 25.1)

  meisha_loop();
  g_mock_commands.clear();

  // Advance 3 hours 59 minutes (14,340,000 ms) -> not yet 4 hours
  advance_mock_millis(14340000UL);
  advance_mock_time(14340);
  meisha_loop();
  assert(g_mock_commands.empty());

  // Advance another 61 seconds (total > 14,400,000 ms = 4 hours)
  advance_mock_millis(61000UL);
  advance_mock_time(61);
  meisha_loop();

  // Step 1: Sets operating mode to Heat only ("0"), arms 10-second wait
  bool found_opmode_0 = false;
  for (const auto& cmd : g_mock_commands) {
    if (cmd.cmd == "SetOperationMode" && cmd.value == "0") found_opmode_0 = true;
    // SetHeatpump MUST NOT be emitted yet!
    assert(cmd.cmd != "SetHeatpump");
  }
  assert(found_opmode_0);
  assert(std::string(meisha_get_last_action_reason()) == "idle_shutoff");

  // Advance 5 seconds: still pending
  g_mock_commands.clear();
  advance_mock_millis(5000);
  advance_mock_time(5);
  meisha_loop();
  assert(g_mock_commands.empty());

  // Advance another 5 seconds (total 10 seconds since Step 1)
  advance_mock_millis(5000);
  advance_mock_time(5);
  meisha_loop();

  // Step 2: Emits SetHeatpump = "0"
  bool found_hp_off = false;
  for (const auto& cmd : g_mock_commands) {
    if (cmd.cmd == "SetHeatpump" && cmd.value == "0") found_hp_off = true;
  }
  assert(found_hp_off);
  assert(std::string(meisha_get_last_action_reason()) == "idle_shutoff");

  // Now heat pump is reported off (TOP0 = "0")
  set_mock_topic_value(0, "0");
  g_mock_commands.clear();
  advance_mock_millis(5000);
  advance_mock_time(5);
  meisha_loop();
  assert(g_mock_commands.empty()); // No commands while HP is off

  std::cout << "[PASS] test_idle_shutoff_4h_and_10s_sequence" << std::endl;
}

void test_idle_shutoff_dhw_safety_guards() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(1762027200);
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);

  actData[0] = 0x71;
  set_mock_topic_value(0, "1");     // HP ON
  set_mock_topic_value(14, "10.0");
  set_mock_topic_value(33, "23.0");
  set_mock_topic_value(1, "10.0");
  set_mock_topic_value(20, "Room");
  set_mock_topic_value(4, "0");
  set_mock_topic_value(27, "25.0"); // HP Setpoint = 25.0

  meisha_loop();

  // 1. Guard A: Valve changes to Tank at 2 hours -> breaks continuous 4-hour condition
  advance_mock_millis(7200000UL); // 2 hours
  advance_mock_time(7200);
  meisha_loop();

  set_mock_topic_value(20, "Tank"); // DHW cycle running
  advance_mock_millis(1800000UL);  // 30 minutes in DHW
  advance_mock_time(1800);
  meisha_loop();

  // Return to Room
  set_mock_topic_value(20, "Room");
  meisha_loop();

  // Advance 3 hours after returning to Room (total time 5.5h, but only 3h continuous in Room)
  advance_mock_millis(10800000UL);
  advance_mock_time(10800);
  g_mock_commands.clear();
  meisha_loop();
  // Must NOT trigger because continuous duration was interrupted
  assert(g_mock_commands.empty());

  // Advance 1 more hour (now 4 hours continuous in Room)
  advance_mock_millis(3600000UL);
  advance_mock_time(3600);
  meisha_loop();
  bool found_opmode_0 = false;
  for (const auto& cmd : g_mock_commands) {
    if (cmd.cmd == "SetOperationMode" && cmd.value == "0") found_opmode_0 = true;
  }
  assert(found_opmode_0);

  // 2. Guard B: DHW starts DURING the 10-second wait between Step 1 and Step 2!
  // At 5 seconds, valve switches to Tank for a DHW run
  advance_mock_millis(5000);
  advance_mock_time(5);
  set_mock_topic_value(20, "Tank");
  set_mock_topic_value(4, "3"); // DHW only
  meisha_loop();

  // At 10 seconds: Step 2 evaluates re-verification guard.
  // Because valve == Tank and op_mode == DHW_ONLY, SetHeatpump("0") MUST BE ABORTED!
  g_mock_commands.clear();
  advance_mock_millis(5000);
  advance_mock_time(5);
  meisha_loop();

  for (const auto& cmd : g_mock_commands) {
    assert(cmd.cmd != "SetHeatpump"); // MUST NOT turn off HP while heating water tank!
  }

  // 3. Guard C: Cooling mode inhibits Idle Shutoff
  set_mock_topic_value(20, "Room");
  set_mock_topic_value(4, "0");
  meisha_set_cooling_mode(true);
  advance_mock_millis(15000000UL); // > 4 hours
  advance_mock_time(15000);
  g_mock_commands.clear();
  meisha_loop();
  assert(g_mock_commands.empty()); // Must not trigger in cooling mode

  std::cout << "[PASS] test_idle_shutoff_dhw_safety_guards" << std::endl;
}

void test_night_mode_and_idle_shutoff_in_shadow_mode() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(1761987600); // 09:00:00 UTC
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_SHADOW);

  actData[0] = 0x71;
  set_mock_topic_value(0, "1");
  set_mock_topic_value(14, "10.0");
  set_mock_topic_value(33, "23.0");
  set_mock_topic_value(1, "10.0");
  set_mock_topic_value(20, "Room");
  set_mock_topic_value(6, "33.0");  // room_delta = 10.0 K
  set_mock_topic_value(7, "28.0");
  set_mock_topic_value(27, "28.0");
  set_mock_topic_value(4, "0");

  for (int i = 0; i < 10; i++) {
    advance_mock_time(3600);
    advance_mock_millis(3600000UL);
    meisha_loop();
  }

  g_mock_commands.clear();
  meisha_loop();

  // In Shadow Mode, calculations run and last_action_reason updates to "night_mode",
  // but g_mock_commands MUST BE EMPTY!
  assert(g_mock_commands.empty());
  assert(std::string(meisha_get_last_action_reason()) == "night_mode");

  // Now set setpoint to 25.0 and advance 4 hours
  set_mock_topic_value(27, "25.0");
  meisha_loop(); // Arms low_setpoint_since_ms
  advance_mock_millis(14500000UL); // > 4 hours
  advance_mock_time(14500);

  g_mock_commands.clear();
  meisha_loop();

  // In Shadow Mode, idle shutoff triggers state update but no command is sent
  assert(g_mock_commands.empty());
  assert(std::string(meisha_get_last_action_reason()) == "idle_shutoff");

  // Advance 10 seconds: Step 2 also sends no commands
  advance_mock_millis(10000);
  advance_mock_time(10);
  meisha_loop();
  assert(g_mock_commands.empty());

  std::cout << "[PASS] test_night_mode_and_idle_shutoff_in_shadow_mode" << std::endl;
}

int main() {
  test_night_mode_budget_cutoff();
  test_night_mode_pre_start_blackout_window();
  test_idle_shutoff_4h_and_10s_sequence();
  test_idle_shutoff_dhw_safety_guards();
  test_night_mode_and_idle_shutoff_in_shadow_mode();

  std::cout << "\nAll Ticket 05 tests passed successfully!" << std::endl;
  return 0;
}
