#include <cassert>
#include <iostream>
#include <cmath>
#include "mock_heishamon.h"
#include "../HeishaMon/meisha.h"

void test_dhw_start_scheduled_1400_and_guards() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  set_mock_reset_reason(ESP_RST_POWERON);
  // 1762005540 = 2025-11-01 13:59:00 UTC
  set_mock_time(1762005540);
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);

  actData[0] = 0x71;
  set_mock_topic_value(0, "0");     // HP OFF
  set_mock_topic_value(14, "10.0");
  set_mock_topic_value(33, "23.0");
  set_mock_topic_value(1, "0.0");
  set_mock_topic_value(20, "Room"); // Valve = Room
  set_mock_topic_value(4, "0");     // OpMode = Heat only
  set_mock_topic_value(10, "40.0"); // DHW_Temp = 40.0 C (< 45.0 C)
  set_mock_topic_value(27, "28.0");

  meisha_loop();
  g_mock_commands.clear();

  // Advance 60s to 14:00:00 UTC (1762005600)
  advance_mock_time(60);
  advance_mock_millis(60000UL);
  meisha_loop();

  // 1. DHW Start triggers!
  // Since HP was OFF, SetHeatpump = "1" must be emitted immediately
  bool found_hp_on = false;
  for (const auto& cmd : g_mock_commands) {
    if (cmd.cmd == "SetHeatpump" && cmd.value == "1") found_hp_on = true;
    assert(cmd.cmd != "SetOperationMode"); // Mode change must wait 10s non-blockingly
  }
  assert(found_hp_on);
  assert(std::string(meisha_get_last_action_reason()) == "dhw_start");

  // Advance 5s (total 5s): still pending
  g_mock_commands.clear();
  advance_mock_time(5);
  advance_mock_millis(5000);
  meisha_loop();
  assert(g_mock_commands.empty());

  // Advance 5s (total 10s since start):
  // Emits SetOperationMode = "3" (DHW only)
  advance_mock_time(5);
  advance_mock_millis(5000);
  meisha_loop();

  bool found_mode_dhw = false;
  for (const auto& cmd : g_mock_commands) {
    if (cmd.cmd == "SetOperationMode" && cmd.value == "3") found_mode_dhw = true;
  }
  assert(found_mode_dhw);
  assert(std::string(meisha_get_last_action_reason()) == "dhw_start");

  // Test Guard A: If dhw_temp >= 45.0 C, 14:00 schedule is inhibited!
  // Set time to next day 14:00:00 (1762005600 + 86400) with dhw_temp = 48.0
  set_mock_time(1762005600 + 86400);
  advance_mock_millis(86400000UL);
  set_mock_topic_value(10, "48.0"); // DHW_Temp >= 45.0 C
  set_mock_topic_value(0, "0");     // HP OFF
  set_mock_topic_value(4, "0");     // Mode Heat only
  set_mock_topic_value(20, "Room");
  g_mock_commands.clear();
  meisha_loop();
  assert(g_mock_commands.empty()); // Inhibited by temperature guard!

  // Test Guard B: If valve == Tank, 14:00 schedule is inhibited!
  set_mock_time(1762005600 + 2 * 86400);
  advance_mock_millis(86400000UL);
  set_mock_topic_value(10, "40.0"); // Temp ok
  set_mock_topic_value(20, "Tank"); // Valve is Tank
  g_mock_commands.clear();
  meisha_loop();
  assert(g_mock_commands.empty()); // Inhibited by valve guard!

  // Test HP already ON:
  // When HP is already ON, SetHeatpump("1") is not emitted, but 10s timer runs and SetOperationMode("3") is emitted
  set_mock_time(1762005600 + 3 * 86400);
  advance_mock_millis(86400000UL);
  set_mock_topic_value(0, "1");     // HP ON
  set_mock_topic_value(20, "Room"); // Valve Room
  set_mock_topic_value(10, "40.0"); // Temp ok
  g_mock_commands.clear();
  meisha_loop();

  // No SetHeatpump("1") emitted since HP was already ON
  assert(g_mock_commands.empty());

  // Wait 10s:
  advance_mock_time(10);
  advance_mock_millis(10000UL);
  meisha_loop();

  found_mode_dhw = false;
  for (const auto& cmd : g_mock_commands) {
    if (cmd.cmd == "SetOperationMode" && cmd.value == "3") found_mode_dhw = true;
  }
  assert(found_mode_dhw);

  std::cout << "[PASS] test_dhw_start_scheduled_1400_and_guards" << std::endl;
}

void test_dhw_start_low_temp_1min() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(1762000000); // 12:26:40 UTC (not 14:00)
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);

  actData[0] = 0x71;
  set_mock_topic_value(0, "0");     // HP OFF
  set_mock_topic_value(14, "10.0");
  set_mock_topic_value(33, "23.0");
  set_mock_topic_value(20, "Room");
  set_mock_topic_value(4, "0");
  set_mock_topic_value(10, "50.0"); // Normal temp

  meisha_loop();
  g_mock_commands.clear();

  // Drop temperature to 37.0 C (< 38.0 C)
  set_mock_topic_value(10, "37.0");
  meisha_loop(); // Arms low temp timer
  assert(g_mock_commands.empty());

  // Advance 30 seconds: still pending
  advance_mock_time(30);
  advance_mock_millis(30000UL);
  meisha_loop();
  assert(g_mock_commands.empty());

  // Temperature recovers to 39.0 C: timer resets
  set_mock_topic_value(10, "39.0");
  meisha_loop();
  assert(g_mock_commands.empty());

  // Advance 40 seconds: total 70s, but recovered in between so no trigger
  advance_mock_time(40);
  advance_mock_millis(40000UL);
  meisha_loop();
  assert(g_mock_commands.empty());

  // Drop again to 36.5 C: timer starts afresh
  set_mock_topic_value(10, "36.5");
  meisha_loop();
  assert(g_mock_commands.empty());

  // Advance 61 seconds: triggers!
  advance_mock_time(61);
  advance_mock_millis(61000UL);
  meisha_loop();

  bool found_hp_on = false;
  for (const auto& cmd : g_mock_commands) {
    if (cmd.cmd == "SetHeatpump" && cmd.value == "1") found_hp_on = true;
  }
  assert(found_hp_on);
  assert(std::string(meisha_get_last_action_reason()) == "dhw_start");

  // Advance 10s: mode set to 3
  g_mock_commands.clear();
  advance_mock_time(10);
  advance_mock_millis(10000UL);
  meisha_loop();

  bool found_mode_dhw = false;
  for (const auto& cmd : g_mock_commands) {
    if (cmd.cmd == "SetOperationMode" && cmd.value == "3") found_mode_dhw = true;
  }
  assert(found_mode_dhw);

  std::cout << "[PASS] test_dhw_start_low_temp_1min" << std::endl;
}

void test_dhw_end_valve_tank_to_room_15min_winter() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  set_mock_reset_reason(ESP_RST_POWERON);
  // Start at 09:00:00 UTC with heating_start_hour = 9
  set_mock_time(1761987600);
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);

  actData[0] = 0x71;
  set_mock_topic_value(0, "1");     // HP ON
  set_mock_topic_value(14, "10.0");
  set_mock_topic_value(33, "23.0");
  set_mock_topic_value(1, "10.0");
  set_mock_topic_value(4, "3");     // OpMode = DHW only
  set_mock_topic_value(20, "Room"); // Initial valve state before switching to Tank
  set_mock_topic_value(8, "30.0");  // Compressor running
  set_mock_topic_value(10, "48.0");

  meisha_loop();
  g_mock_commands.clear();

  // Initially in Room before Tank has been seen:
  // Advance 20 minutes in Room. Must NOT trigger DHW End because Tank was never seen!
  advance_mock_time(1200);
  advance_mock_millis(1200000UL);
  meisha_loop();
  assert(g_mock_commands.empty());

  // Now heat pump switches valve to Tank
  set_mock_topic_value(20, "Tank");
  advance_mock_time(1800); // 30 minutes in DHW heating
  advance_mock_millis(1800000UL);
  meisha_loop();
  assert(g_mock_commands.empty());

  // Heat pump finishes DHW heating and transitions valve back to Room
  set_mock_topic_value(20, "Room");
  meisha_loop(); // Arms dhw_valve_room_since_ms

  // Advance 14 minutes in Room (< 15m)
  advance_mock_time(840);
  advance_mock_millis(840000UL);
  g_mock_commands.clear();
  meisha_loop();
  assert(g_mock_commands.empty());

  // Advance 2 more minutes (total 16m > 15m in Room)
  advance_mock_time(120);
  advance_mock_millis(120000UL);
  meisha_loop();

  // DHW End triggers!
  // In winter mode, sets SetOperationMode = "0" (Heat only).
  // Because hours_time_left > 0.1, SetHeatpump = "0" is NOT sent (HP stays on for space heating).
  bool found_mode_heat = false;
  bool found_hp_off = false;
  for (const auto& cmd : g_mock_commands) {
    if (cmd.cmd == "SetOperationMode" && cmd.value == "0") found_mode_heat = true;
    if (cmd.cmd == "SetHeatpump" && cmd.value == "0") found_hp_off = true;
  }
  assert(found_mode_heat);
  assert(!found_hp_off); // Heat pump must stay ON when hours_time_left > 0.1
  assert(std::string(meisha_get_last_action_reason()) == "dhw_end");

  // Now test DHW End with hours_time_left < 0.1:
  // Advance 12 hours with valve in Room to 21:00 UTC (hours_elapsed = 12 > hours_target ~11, so hours_time_left = 0)
  set_mock_topic_value(4, "0");
  set_mock_topic_value(20, "Room");
  advance_mock_time(43200);
  advance_mock_millis(43200000UL);
  meisha_loop();

  // Now start DHW only mode with valve in Tank for 10 minutes (adds only 0.16h, so hours_time_left remains 0.0)
  set_mock_topic_value(4, "3");
  set_mock_topic_value(20, "Tank");
  advance_mock_time(600);
  advance_mock_millis(600000UL);
  meisha_loop();

  // Transition Tank -> Room
  set_mock_topic_value(20, "Room");
  meisha_loop();

  // Advance 16 minutes in Room
  advance_mock_time(960);
  advance_mock_millis(960000UL);
  g_mock_commands.clear();
  meisha_loop();

  // Now hours_time_left < 0.1, so both SetOperationMode = "0" AND SetHeatpump = "0" must be emitted!
  found_mode_heat = false;
  found_hp_off = false;
  for (const auto& cmd : g_mock_commands) {
    if (cmd.cmd == "SetOperationMode" && cmd.value == "0") found_mode_heat = true;
    if (cmd.cmd == "SetHeatpump" && cmd.value == "0") found_hp_off = true;
  }
  assert(found_mode_heat);
  assert(found_hp_off);

  std::cout << "[PASS] test_dhw_end_valve_tank_to_room_15min_winter" << std::endl;
}

void test_dhw_end_compressor_off_15min_summer() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(1762027200);
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);
  meisha_set_cooling_mode(true); // Summer mode!

  actData[0] = 0x71;
  set_mock_topic_value(0, "1");     // HP ON
  set_mock_topic_value(14, "20.0"); // Outside temp 20.0 C (ambient_24h < 22.0 C)
  set_mock_topic_value(33, "24.0");
  set_mock_topic_value(4, "3");     // OpMode = DHW only
  set_mock_topic_value(20, "Tank");
  set_mock_topic_value(8, "0.0");   // Compressor OFF

  meisha_loop(); // Arms compressor off timer
  g_mock_commands.clear();

  // Advance 14 minutes with compressor off: no trigger
  advance_mock_time(840);
  advance_mock_millis(840000UL);
  meisha_loop();
  assert(g_mock_commands.empty());

  // Advance 2 more minutes (total 16m > 15m compressor off)
  advance_mock_time(120);
  advance_mock_millis(120000UL);
  meisha_loop();

  // Step 1: Emits SetOperationMode = "1" (Cool only) and arms 10-second timer
  bool found_mode_cool = false;
  for (const auto& cmd : g_mock_commands) {
    if (cmd.cmd == "SetOperationMode" && cmd.value == "1") found_mode_cool = true;
    assert(cmd.cmd != "SetHeatpump"); // 10s wait required
  }
  assert(found_mode_cool);
  assert(std::string(meisha_get_last_action_reason()) == "dhw_end");

  // Advance 5 seconds: still pending
  g_mock_commands.clear();
  advance_mock_time(5);
  advance_mock_millis(5000);
  meisha_loop();
  assert(g_mock_commands.empty());

  // Advance 5 seconds (total 10 seconds):
  // Since ambient_24h is 20.0 C (< 22.0 C), emits SetHeatpump = "0"!
  advance_mock_time(5);
  advance_mock_millis(5000);
  meisha_loop();

  bool found_hp_off = false;
  for (const auto& cmd : g_mock_commands) {
    if (cmd.cmd == "SetHeatpump" && cmd.value == "0") found_hp_off = true;
  }
  assert(found_hp_off);

  // Now test Case B: ambient_24h >= 22.0 C in summer:
  // Heat pump must remain ON in Cool only mode
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(1762027200);
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);
  meisha_set_cooling_mode(true);

  actData[0] = 0x71;
  set_mock_topic_value(0, "1");
  set_mock_topic_value(14, "26.0"); // Warm summer outside temp (seeds ambient_24h = 26.0 C)
  set_mock_topic_value(33, "24.0");
  set_mock_topic_value(4, "3");   // DHW only
  set_mock_topic_value(20, "Tank");
  set_mock_topic_value(8, "0.0"); // Compressor OFF

  meisha_loop();
  const MeishaRtcState* rtc = meisha_get_rtc_state();
  assert(rtc->ambient_24h_mean >= 22.0f);

  // Advance 16 minutes compressor off
  advance_mock_time(960);
  advance_mock_millis(960000UL);
  g_mock_commands.clear();
  meisha_loop();

  found_mode_cool = false;
  for (const auto& cmd : g_mock_commands) {
    if (cmd.cmd == "SetOperationMode" && cmd.value == "1") found_mode_cool = true;
  }
  assert(found_mode_cool);

  // Advance 10 seconds:
  g_mock_commands.clear();
  advance_mock_time(10);
  advance_mock_millis(10000UL);
  meisha_loop();

  // Since ambient_24h >= 22.0 C, SetHeatpump = "0" MUST NOT BE EMITTED (HP stays on cooling)!
  for (const auto& cmd : g_mock_commands) {
    assert(cmd.cmd != "SetHeatpump");
  }

  std::cout << "[PASS] test_dhw_end_compressor_off_15min_summer" << std::endl;
}

void test_dhw_shadow_mode() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(1762005540); // 13:59:00 UTC
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_SHADOW);

  actData[0] = 0x71;
  set_mock_topic_value(0, "0");
  set_mock_topic_value(14, "10.0");
  set_mock_topic_value(33, "23.0");
  set_mock_topic_value(20, "Room");
  set_mock_topic_value(4, "0");
  set_mock_topic_value(10, "40.0");

  meisha_loop();

  // Advance to 14:00:00 UTC
  advance_mock_time(60);
  advance_mock_millis(60000UL);
  g_mock_commands.clear();
  meisha_loop();

  // In Shadow Mode, calculations run and last_action_reason updates to "dhw_start",
  // but g_mock_commands MUST BE EMPTY!
  assert(g_mock_commands.empty());
  assert(std::string(meisha_get_last_action_reason()) == "dhw_start");

  // Advance 10s: Step 2 also sends no commands
  advance_mock_time(10);
  advance_mock_millis(10000UL);
  meisha_loop();
  assert(g_mock_commands.empty());

  // Now simulate in DHW only mode with compressor off for 16 minutes
  set_mock_topic_value(4, "3");
  set_mock_topic_value(8, "0.0");
  meisha_loop();

  advance_mock_time(960);
  advance_mock_millis(960000UL);
  g_mock_commands.clear();
  meisha_loop();

  // In Shadow Mode, DHW End triggers telemetry update to "dhw_end", but NO COMMANDS!
  assert(g_mock_commands.empty());
  assert(std::string(meisha_get_last_action_reason()) == "dhw_end");

  std::cout << "[PASS] test_dhw_shadow_mode" << std::endl;
}

int main() {
  test_dhw_start_scheduled_1400_and_guards();
  test_dhw_start_low_temp_1min();
  test_dhw_end_valve_tank_to_room_15min_winter();
  test_dhw_end_compressor_off_15min_summer();
  test_dhw_shadow_mode();

  std::cout << "\nAll Ticket 06 tests passed successfully!" << std::endl;
  return 0;
}
