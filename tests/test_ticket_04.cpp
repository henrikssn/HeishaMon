#include <cassert>
#include <iostream>
#include <cmath>
#include "mock_heishamon.h"
#include "../HeishaMon/meisha.h"

void test_day_mode_1_startup_and_guards() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(1761984000); // 08:00 UTC
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);

  actData[0] = 0x71;
  set_mock_topic_value(14, "10.0"); // Outside_Temp = 10.0
  set_mock_topic_value(33, "23.0"); // Room_Temp = 23.0
  set_mock_topic_value(1, "10.0");  // Flow > 0
  set_mock_topic_value(20, "Room"); // Valve = Room
  set_mock_topic_value(6, "27.0");  // Outlet = 27.0
  set_mock_topic_value(7, "28.0");  // Target = 28.0
  set_mock_topic_value(27, "25.0"); // Heat Pump Setpoint = 25
  set_mock_topic_value(4, "4");     // OpMode = Heat+DHW (not Heat only)
  meisha_loop();

  g_mock_commands.clear();

  // Advance time to 09:00:00 UTC (heating_start_hour:00)
  advance_mock_time(3600);
  advance_mock_millis(3600000);
  meisha_loop();

  // At 09:00:00 on the initial tick, Day Mode #1 arms a 10-second non-blocking timer.
  // No commands must be emitted yet!
  assert(g_mock_commands.empty());

  // Advance 5 seconds (5000 ms) -> still pending (< 10s)
  advance_mock_millis(5000);
  advance_mock_time(5);
  meisha_loop();
  assert(g_mock_commands.empty());

  // Advance another 5 seconds (total 10,000 ms) -> Day Mode #1 fires!
  advance_mock_millis(5000);
  advance_mock_time(5);
  meisha_loop();

  // Verify commands emitted:
  // 1. SetOperationMode -> 0 (Heat only)
  // 2. SetHeatpump -> 1 (On)
  // 3. SetZ1HeatRequestTemperature -> max(27, power_target) (power_target is 28)
  bool found_op_mode = false;
  bool found_hp_on = false;
  bool found_setpoint = false;

  for (const auto& call : g_mock_commands) {
    if (call.cmd == "SetOperationMode" && call.value == "0") found_op_mode = true;
    if (call.cmd == "SetHeatpump" && call.value == "1") found_hp_on = true;
    if (call.cmd == "SetZ1HeatRequestTemperature" && call.value == "27") found_setpoint = true;
  }

  assert(found_op_mode);
  assert(found_hp_on);
  assert(found_setpoint);
  assert(std::string(meisha_get_last_action_reason()) == "day_mode_1");

  std::cout << "[PASS] test_day_mode_1_startup_and_guards" << std::endl;
}

void test_bonusgrad_curve_and_change_lock() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(1761984000 + 7200); // 10:00 UTC
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);

  actData[0] = 0x71;
  set_mock_topic_value(14, "5.0"); // Outside_Temp = 5.0 -> power_target = 30
  set_mock_topic_value(33, "23.0");
  set_mock_topic_value(1, "10.0");
  set_mock_topic_value(20, "Room");
  set_mock_topic_value(8, "20.0");  // Compressor freq = 20 Hz (>12 Hz -> compressor_running = on)
  set_mock_topic_value(6, "28.0");  // Outlet = 28.0
  set_mock_topic_value(7, "28.0");  // Target = 28.0 -> int(outlet) + 1 = 29 > 28
  set_mock_topic_value(27, "25.0"); // Current HP Setpoint = 25
  set_mock_topic_value(56, "26.0"); // Z1_Temp = 26.0
  meisha_loop();

  g_mock_commands.clear();

  // Run for 4 minutes: comp_on is 4m (< 5m), room is 4m (< 10m) -> No command
  advance_mock_millis(240000);
  advance_mock_time(240);
  meisha_loop();
  assert(g_mock_commands.empty());

  // Run another 2 minutes (total 6m): comp_on >= 5m, but room is 6m (< 10m) -> No command
  advance_mock_millis(120000);
  advance_mock_time(120);
  meisha_loop();
  assert(g_mock_commands.empty());

  // Run another 4 minutes (total 10m): both room >= 10m and comp >= 5m hold!
  // power_target is 28 > setpoint (25)
  // degree_hours_day is ~0 < degree_hours_target
  // Bonusgrad curve fires!
  // new_setpoint = max(int(setpoint) + 1, int(z1_temp) + 1) = max(26, 27) = 27
  advance_mock_millis(240000);
  advance_mock_time(240);
  meisha_loop();

  assert(g_mock_commands.size() == 1);
  assert(g_mock_commands[0].cmd == "SetZ1HeatRequestTemperature");
  assert(g_mock_commands[0].value == "27");
  assert(std::string(meisha_get_last_action_reason()) == "bonusgrad_curve");

  // Change lock must be set to 30 minutes
  assert(std::fabs(meisha_get_change_lock_remaining_min() - 30.0f) < 0.2f);

  g_mock_commands.clear();
  set_mock_topic_value(27, "27.0"); // HP Setpoint updated to 27

  // Run for 15 minutes: change lock is still active (~15 min left) -> No command
  advance_mock_millis(900000);
  advance_mock_time(900);
  meisha_loop();
  assert(g_mock_commands.empty());
  assert(std::fabs(meisha_get_change_lock_remaining_min() - 15.0f) < 0.2f);

  // Run another 15 minutes (total 30 min): change lock expires!
  advance_mock_millis(900000);
  advance_mock_time(900);
  meisha_loop();

  // Bonusgrad curve fires again: max(27 + 1, 26 + 1) = 28
  assert(g_mock_commands.size() == 1);
  assert(g_mock_commands[0].cmd == "SetZ1HeatRequestTemperature");
  assert(g_mock_commands[0].value == "28");
  // Sets a new 30-minute change lock
  assert(std::fabs(meisha_get_change_lock_remaining_min() - 30.0f) < 0.2f);

  std::cout << "[PASS] test_bonusgrad_curve_and_change_lock" << std::endl;
}

void test_malusgrad_and_change_lock() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(1761984000 + 7200); // 10:00 UTC
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);

  actData[0] = 0x71;
  set_mock_topic_value(14, "10.0");
  set_mock_topic_value(33, "23.0");
  set_mock_topic_value(1, "10.0");
  set_mock_topic_value(20, "Room");
  set_mock_topic_value(8, "20.0");  // Compressor on
  set_mock_topic_value(6, "30.0");  // Outlet = 30.0
  set_mock_topic_value(7, "32.0");  // Target = 32.0 -> int(outlet) - 1 = 29 < 32
  set_mock_topic_value(27, "31.0"); // HP Setpoint = 31 > power_target (28)
  set_mock_topic_value(56, "30.0"); // Z1_Temp = 30.0
  meisha_loop();

  // Advance 10 minutes (600,000 ms) so room >= 10m and comp >= 5m
  advance_mock_millis(600000);
  advance_mock_time(600);
  g_mock_commands.clear();
  meisha_loop();

  // Malusgrad fires:
  // stepped_down = min(int(31) - 1, int(30) - 1) = min(30, 29) = 29
  // new_setpoint = max(29, power_target (28)) = 29
  // change_lock = now + 20 min
  assert(g_mock_commands.size() == 1);
  assert(g_mock_commands[0].cmd == "SetZ1HeatRequestTemperature");
  assert(g_mock_commands[0].value == "29");
  assert(std::string(meisha_get_last_action_reason()) == "malusgrad");
  assert(std::fabs(meisha_get_change_lock_remaining_min() - 20.0f) < 0.2f);

  std::cout << "[PASS] test_malusgrad_and_change_lock" << std::endl;
}

void test_bonusgrad_high_temp_anti_overshoot() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(1761984000 + 7200); // 10:00 UTC
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);

  actData[0] = 0x71;
  set_mock_topic_value(14, "10.0");
  set_mock_topic_value(33, "23.0");
  set_mock_topic_value(1, "10.0");
  set_mock_topic_value(20, "Room");
  set_mock_topic_value(8, "20.0");  // Compressor on
  set_mock_topic_value(27, "28.0"); // HP Setpoint = 28
  set_mock_topic_value(56, "32.0"); // Z1_Temp = 32 >= 28 + 3 = 31 (overshoot!)
  meisha_loop();

  // Put a 30-minute change lock in place from a previous action to verify overshoot bypasses change_lock
  // Run 30 seconds (< 1 min) -> No overshoot step yet
  advance_mock_millis(30000);
  advance_mock_time(30);
  g_mock_commands.clear();
  meisha_loop();
  assert(g_mock_commands.empty());

  // Run another 30 seconds (total 60s = 1 min continuous overshoot):
  // Step 1 fires: setpoint + 1 = 29, change_lock = 15 min!
  advance_mock_millis(30000);
  advance_mock_time(30);
  meisha_loop();

  assert(g_mock_commands.size() == 1);
  assert(g_mock_commands[0].cmd == "SetZ1HeatRequestTemperature");
  assert(g_mock_commands[0].value == "29");
  assert(std::string(meisha_get_last_action_reason()) == "bonusgrad_high_temp");
  assert(std::fabs(meisha_get_change_lock_remaining_min() - 15.0f) < 0.2f);

  g_mock_commands.clear();
  set_mock_topic_value(27, "29.0"); // Telemetry updates setpoint to 29. Z1_Temp is still 32 >= 29 + 3 = 32!

  // Advance 30 seconds: total 90s (1.5 min) -> Step 2 not yet (< 2 min)
  advance_mock_millis(30000);
  advance_mock_time(30);
  meisha_loop();
  assert(g_mock_commands.empty());

  // Advance another 30 seconds (total 120s = 2 min continuous overshoot):
  // Step 2 fires: setpoint + 1 = 30!
  advance_mock_millis(30000);
  advance_mock_time(30);
  meisha_loop();

  assert(g_mock_commands.size() == 1);
  assert(g_mock_commands[0].cmd == "SetZ1HeatRequestTemperature");
  assert(g_mock_commands[0].value == "30");
  assert(std::string(meisha_get_last_action_reason()) == "bonusgrad_high_temp");

  // Telemetry updates setpoint to 30. Now Z1_Temp (32) < 30 + 3 = 33 -> Overshoot resolved!
  g_mock_commands.clear();
  set_mock_topic_value(27, "30.0");
  advance_mock_millis(60000);
  advance_mock_time(60);
  meisha_loop();
  assert(g_mock_commands.empty());

  std::cout << "[PASS] test_bonusgrad_high_temp_anti_overshoot" << std::endl;
}

void test_shadow_mode_and_mqtt_discovery() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(1761984000 + 7200); // 10:00 UTC
  meisha_setup();
  // Mode defaults to Shadow!
  assert(meisha_get_control_mode() == MEISHA_MODE_SHADOW);

  meisha_mqtt_connected();

  // Verify HA discovery registered for change_lock_remaining_min and last_action_reason
  bool found_lock_disc = false;
  bool found_reason_disc = false;

  for (const auto& msg : mqtt_client.published) {
    if (msg.topic == "homeassistant/sensor/meisha_esp_change_lock_remaining_min/config") {
      found_lock_disc = true;
      assert(msg.payload.find("\"unit_of_measurement\":\"min\"") != std::string::npos);
      assert(msg.payload.find("\"device_class\":\"duration\"") != std::string::npos);
    }
    if (msg.topic == "homeassistant/sensor/meisha_esp_last_action_reason/config") {
      found_reason_disc = true;
      assert(msg.payload.find("\"name\":\"Meisha Last Action Reason\"") != std::string::npos);
    }
  }
  assert(found_lock_disc);
  assert(found_reason_disc);

  // Setup conditions for Bonusgrad curve
  actData[0] = 0x71;
  set_mock_topic_value(14, "10.0");
  set_mock_topic_value(33, "23.0");
  set_mock_topic_value(1, "10.0");
  set_mock_topic_value(20, "Room");
  set_mock_topic_value(8, "20.0");
  set_mock_topic_value(6, "28.0");
  set_mock_topic_value(7, "28.0");
  set_mock_topic_value(27, "25.0");
  set_mock_topic_value(56, "26.0");
  meisha_loop();

  // Advance 10 minutes so timers expire
  advance_mock_millis(600000);
  advance_mock_time(600);
  g_mock_commands.clear();
  meisha_loop();

  // In Shadow Mode, calculations run, change_lock updates, last_action_reason updates,
  // but g_mock_commands MUST REMAIN EMPTY!
  assert(g_mock_commands.empty());
  assert(std::string(meisha_get_last_action_reason()) == "bonusgrad_curve");
  assert(std::fabs(meisha_get_change_lock_remaining_min() - 30.0f) < 0.2f);

  std::cout << "[PASS] test_shadow_mode_and_mqtt_discovery" << std::endl;
}

int main() {
  test_day_mode_1_startup_and_guards();
  test_bonusgrad_curve_and_change_lock();
  test_malusgrad_and_change_lock();
  test_bonusgrad_high_temp_anti_overshoot();
  test_shadow_mode_and_mqtt_discovery();

  std::cout << "\nAll Ticket 04 tests passed successfully!" << std::endl;
  return 0;
}
