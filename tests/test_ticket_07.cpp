#include "../HeishaMon/meisha.h"
#include "mock_heishamon.h"
#include <cassert>
#include <cmath>
#include <iostream>

void test_cool_turn_on_0800_and_10s_sequence() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  set_mock_reset_reason(ESP_RST_POWERON);
  // 1752566340 = 2025-07-15 07:59:00 UTC
  time_t epoch_0759 = 1752566340;

  set_mock_time(epoch_0759);
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);
  meisha_set_cooling_mode(true); // Cooling mode ON

  actData[0] = 0x71;
  set_mock_topic_value(0, "0"); // HP OFF
  set_mock_topic_value(
      14, "26.0"); // Outside temp 26.0 C (seeds ambient_24h > 24.0 C)
  set_mock_topic_value(33, "24.0"); // Room temp
  set_mock_topic_value(1, "0.0");
  set_mock_topic_value(20, "Room");
  set_mock_topic_value(4, "1"); // Cool only

  meisha_loop();
  const MeishaRtcState *rtc = meisha_get_rtc_state();
  assert(rtc->ambient_24h_mean > 24.0f);
  g_mock_commands.clear();

  // Advance 60s to 08:00:00
  advance_mock_time(60);
  advance_mock_millis(60000UL);
  meisha_loop();

  // Step 1: SetOperationMode = "1" must be emitted immediately
  bool found_mode_cool = false;
  for (const auto &cmd : g_mock_commands) {
    if (cmd.cmd == "SetOperationMode" && cmd.value == "1")
      found_mode_cool = true;
    assert(cmd.cmd !=
           "SetHeatpump"); // Heat pump on must wait 10s non-blockingly
  }
  assert(found_mode_cool);
  assert(std::string(meisha_get_last_action_reason()) == "cool_turn_on");

  // Advance 5s (total 5s): still pending
  g_mock_commands.clear();
  advance_mock_time(5);
  advance_mock_millis(5000);
  meisha_loop();
  assert(g_mock_commands.empty());

  // Advance 5s (total 10s):
  // Emits SetHeatpump = "1"
  advance_mock_time(5);
  advance_mock_millis(5000);
  meisha_loop();

  bool found_hp_on = false;
  for (const auto &cmd : g_mock_commands) {
    if (cmd.cmd == "SetHeatpump" && cmd.value == "1")
      found_hp_on = true;
  }
  assert(found_hp_on);
  assert(std::string(meisha_get_last_action_reason()) == "cool_turn_on");

  // Test Guard A: If ambient_24h <= 24.0 C, 08:00 schedule does not trigger
  // Advance to next day 08:00:00
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();
  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(epoch_0759 + 86400);
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);
  meisha_set_cooling_mode(true);

  actData[0] = 0x71;
  set_mock_topic_value(0, "0");
  set_mock_topic_value(14,
                       "23.5"); // Outside temp 23.5 C (ambient_24h <= 24.0 C)
  set_mock_topic_value(33, "24.0");
  set_mock_topic_value(4, "1");

  meisha_loop();
  g_mock_commands.clear();

  // Advance to 08:00:00
  advance_mock_time(60);
  advance_mock_millis(60000UL);
  meisha_loop();
  assert(g_mock_commands.empty());

  // Test Guard B: If cooling_mode == false, 08:00 schedule does not trigger
  // even if ambient > 24.0
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();
  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(epoch_0759 + 2 * 86400);
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);
  meisha_set_cooling_mode(false); // Cooling mode OFF

  actData[0] = 0x71;
  set_mock_topic_value(0, "0");
  set_mock_topic_value(14, "27.0"); // Warm
  set_mock_topic_value(33, "24.0");

  meisha_loop();
  g_mock_commands.clear();

  advance_mock_time(60);
  advance_mock_millis(60000UL);
  meisha_loop();
  assert(g_mock_commands.empty());

  std::cout << "[PASS] test_cool_turn_on_0800_and_10s_sequence" << std::endl;
}

void test_cool_turn_off_2000() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  // 1752609540 = 2025-07-15 19:59:00 UTC
  time_t epoch_1959 = 1752609540;

  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(epoch_1959);
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);
  meisha_set_cooling_mode(true);

  actData[0] = 0x71;
  set_mock_topic_value(0, "1"); // HP ON
  set_mock_topic_value(14, "25.0");
  set_mock_topic_value(33, "23.0");
  set_mock_topic_value(4, "1"); // Cool only

  meisha_loop();
  g_mock_commands.clear();

  // Advance 60s to 20:00:00
  advance_mock_time(60);
  advance_mock_millis(60000UL);
  meisha_loop();

  // Cool Turn Off triggers! Emits SetHeatpump = "0"
  bool found_hp_off = false;
  for (const auto &cmd : g_mock_commands) {
    if (cmd.cmd == "SetHeatpump" && cmd.value == "0")
      found_hp_off = true;
  }
  assert(found_hp_off);
  assert(std::string(meisha_get_last_action_reason()) == "cool_turn_off");

  // If HP was already OFF at 20:00, no command emitted
  set_mock_time(epoch_1959 + 86400);
  advance_mock_millis(86400000UL);
  set_mock_topic_value(0, "0"); // HP OFF
  meisha_loop();
  g_mock_commands.clear();

  advance_mock_time(60);
  advance_mock_millis(60000UL);
  meisha_loop();
  assert(g_mock_commands.empty());

  std::cout << "[PASS] test_cool_turn_off_2000" << std::endl;
}

void test_cool_turn_off_ambient_below_22_for_1h() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(1762000000); // 12:26:40
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);
  meisha_set_cooling_mode(true);

  actData[0] = 0x71;
  set_mock_topic_value(0, "1"); // HP ON
  set_mock_topic_value(
      14, "21.0"); // Outside temp 21.0 C (seeds ambient_24h = 21.0 C < 22.0 C)
  set_mock_topic_value(33, "23.0");
  set_mock_topic_value(4, "1");

  meisha_loop(); // Arms cool_low_ambient_since_ms
  const MeishaRtcState *rtc = meisha_get_rtc_state();
  assert(rtc->ambient_24h_mean < 22.0f);
  g_mock_commands.clear();

  // Advance 30 minutes (1800s): no trigger yet
  advance_mock_time(1800);
  advance_mock_millis(1800000UL);
  meisha_loop();
  assert(g_mock_commands.empty());

  // If ambient rises above 22.0 C (reboot with warm checkpoint > 22 C), timer
  // resets
  MeishaNvsCheckpoint warm_chkpt = {};
  warm_chkpt.ambient_24h_mean = 25.0f;
  warm_chkpt.ambient_1h_mean = 25.0f;
  warm_chkpt.room_temp_6h_mean = 23.0f;
  warm_chkpt.saved_at_epoch = 1762000000 + 1800;
  Preferences::storage["meisha_cfg"]["chkpt"] = std::vector<uint8_t>(
      reinterpret_cast<uint8_t *>(&warm_chkpt),
      reinterpret_cast<uint8_t *>(&warm_chkpt) + sizeof(warm_chkpt));
  mock_corrupt_rtc_state();
  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(1762000000 + 1800);
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);
  meisha_set_cooling_mode(true);

  actData[0] = 0x71;
  set_mock_topic_value(0, "1");
  set_mock_topic_value(14, "25.0");
  set_mock_topic_value(33, "23.0");
  set_mock_topic_value(4, "1");

  meisha_loop();
  assert(rtc->ambient_24h_mean >= 22.0f);
  assert(rtc->cool_low_ambient_timer_active == 0);
  g_mock_commands.clear();

  // Now drop ambient_24h back below 22.0 C with a cool checkpoint (21.0 C)
  MeishaNvsCheckpoint cool_chkpt = {};
  cool_chkpt.ambient_24h_mean = 21.0f;
  cool_chkpt.ambient_1h_mean = 21.0f;
  cool_chkpt.room_temp_6h_mean = 23.0f;
  cool_chkpt.saved_at_epoch = 1762000000 + 2000;
  Preferences::storage["meisha_cfg"]["chkpt"] = std::vector<uint8_t>(
      reinterpret_cast<uint8_t *>(&cool_chkpt),
      reinterpret_cast<uint8_t *>(&cool_chkpt) + sizeof(cool_chkpt));
  mock_corrupt_rtc_state();
  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(1762000000 + 2000);
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);
  meisha_set_cooling_mode(true);

  set_mock_topic_value(14, "21.0");
  meisha_loop(); // Arms cool_low_ambient_since_ms
  assert(rtc->ambient_24h_mean < 22.0f);
  assert(rtc->cool_low_ambient_timer_active == 1);
  g_mock_commands.clear();

  // Advance 59 minutes: still under 1 hour, no command
  advance_mock_time(3540);
  advance_mock_millis(3540000UL);
  meisha_loop();
  assert(g_mock_commands.empty());

  // Advance 2 more minutes (total 61 minutes): triggers!
  advance_mock_time(120);
  advance_mock_millis(120000UL);
  meisha_loop();

  bool found_hp_off = false;
  for (const auto &cmd : g_mock_commands) {
    if (cmd.cmd == "SetHeatpump" && cmd.value == "0")
      found_hp_off = true;
  }
  assert(found_hp_off);
  assert(std::string(meisha_get_last_action_reason()) == "cool_turn_off");

  std::cout << "[PASS] test_cool_turn_off_ambient_below_22_for_1h" << std::endl;
}

void test_winter_heating_rules_suppressed_when_cooling_mode_on() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  struct tm tm_0859 = {};
  tm_0859.tm_year = 125;
  tm_0859.tm_mon = 0;
  tm_0859.tm_mday = 10;
  tm_0859.tm_hour = 8;
  tm_0859.tm_min = 59;
  tm_0859.tm_sec = 0;
  time_t epoch_0859 = mktime(&tm_0859);

  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(epoch_0859);
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);
  meisha_set_cooling_mode(true); // Cooling mode ON

  actData[0] = 0x71;
  set_mock_topic_value(0, "1"); // HP ON
  set_mock_topic_value(
      14, "25.0"); // Warm outside temp (prevents ambient < 22 C cool shutoff)
  set_mock_topic_value(33, "20.0"); // Cold room
  set_mock_topic_value(1, "12.0");  // Flow active
  set_mock_topic_value(6, "40.0");  // Outlet temp
  set_mock_topic_value(7, "30.0");  // Target temp
  set_mock_topic_value(8, "35.0");  // Compressor running
  set_mock_topic_value(20, "Room"); // Valve Room
  set_mock_topic_value(4, "0");     // Heat only mode reported
  set_mock_topic_value(27, "25.0"); // Setpoint 25 (idle condition)
  set_mock_topic_value(56, "35.0"); // Z1 temp

  meisha_loop();
  g_mock_commands.clear();

  // Advance 60s to 09:00:00 (heating_start_hour)
  // Day Mode #1 must be SUPPRESSED because cooling_mode == on
  advance_mock_time(60);
  advance_mock_millis(60000UL);
  meisha_loop();

  // Advance 11s (past Day Mode 10s wait):
  advance_mock_time(11);
  advance_mock_millis(11000UL);
  meisha_loop();

  // No Day Mode #1 commands emitted!
  for (const auto &cmd : g_mock_commands) {
    assert(cmd.cmd != "SetOperationMode" || cmd.value != "0");
    assert(cmd.cmd != "SetZ1HeatRequestTemperature");
  }

  // Advance 5 hours with setpoint == 25 and hp_on == 1
  // 4-Hour Idle Shutoff must be SUPPRESSED because cooling_mode == on
  g_mock_commands.clear();
  advance_mock_time(18000);
  advance_mock_millis(18000000UL);
  meisha_loop();
  for (const auto &cmd : g_mock_commands) {
    assert(cmd.cmd != "SetHeatpump" || cmd.value != "0");
  }

  // Check Bonusgrad curve, Malusgrad, Bonusgrad high-temp, Night mode:
  // All are suppressed while cooling_mode == on
  const MeishaRtcState *rtc = meisha_get_rtc_state();
  assert(rtc->overshoot_timer_active == 0);
  assert(rtc->low_setpoint_timer_active == 0);

  std::cout
      << "[PASS] test_winter_heating_rules_suppressed_when_cooling_mode_on"
      << std::endl;
}

void test_cooling_shadow_mode() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  time_t epoch_0759 = 1752566340;

  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(epoch_0759);
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_SHADOW); // SHADOW MODE
  meisha_set_cooling_mode(true);

  actData[0] = 0x71;
  set_mock_topic_value(0, "0");     // HP OFF
  set_mock_topic_value(14, "26.0"); // Warm summer outside temp
  set_mock_topic_value(33, "24.0");
  set_mock_topic_value(4, "1");

  meisha_loop();
  g_mock_commands.clear();

  // Advance to 08:00:00
  advance_mock_time(60);
  advance_mock_millis(60000UL);
  meisha_loop();

  // In Shadow Mode, calculations run and last_action_reason updates to
  // "cool_turn_on", but g_mock_commands MUST BE EMPTY!
  assert(g_mock_commands.empty());
  assert(std::string(meisha_get_last_action_reason()) == "cool_turn_on");

  // Advance 10s: Step 2 also sends no commands
  advance_mock_time(10);
  advance_mock_millis(10000UL);
  meisha_loop();
  assert(g_mock_commands.empty());

  // Test 20:00 shutoff in Shadow Mode:
  set_mock_topic_value(0, "1"); // HP ON
  set_mock_time(1752609540);    // 19:59:00 UTC
  advance_mock_millis(43200000UL);
  meisha_loop();

  advance_mock_time(60);
  advance_mock_millis(60000UL);
  g_mock_commands.clear();
  meisha_loop();

  assert(g_mock_commands.empty());
  assert(std::string(meisha_get_last_action_reason()) == "cool_turn_off");

  std::cout << "[PASS] test_cooling_shadow_mode" << std::endl;
}

int main() {
  test_cool_turn_on_0800_and_10s_sequence();
  test_cool_turn_off_2000();
  test_cool_turn_off_ambient_below_22_for_1h();
  test_winter_heating_rules_suppressed_when_cooling_mode_on();
  test_cooling_shadow_mode();

  std::cout << "\nAll Ticket 07 tests passed successfully!" << std::endl;
  return 0;
}
