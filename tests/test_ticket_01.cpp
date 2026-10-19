#include <cassert>
#include <iostream>
#include <string>
#include "mock_heishamon.h"
#include "../HeishaMon/meisha.h"

void test_initial_boot_and_mqtt_discovery() {
  reset_mock_environment();
  Preferences::reset();

  set_mock_reset_reason(ESP_RST_POWERON);
  meisha_setup();

  assert(meisha_get_control_mode() == MEISHA_MODE_SHADOW);
  assert(meisha_get_rtc_state()->crash_count == 0);
  assert(meisha_get_rtc_state()->crash_tripped == 0);

  meisha_mqtt_connected();

  // Verify subscription to <base>/meisha/set/#
  bool found_sub = false;
  for (const auto& sub : mqtt_client.subscriptions) {
    if (sub == "panasonic_heat_pump/meisha/set/#") {
      found_sub = true;
      break;
    }
  }
  assert(found_sub);

  // Verify HA discovery payload
  bool found_disc = false;
  bool found_state = false;
  for (const auto& msg : mqtt_client.published) {
    if (msg.topic == "homeassistant/select/meisha_esp_control_mode/config") {
      found_disc = true;
      assert(msg.retained == true);
      assert(msg.payload.find("\"name\":\"Meisha ESP Control Mode\"") != std::string::npos);
      assert(msg.payload.find("\"unique_id\":\"meisha_esp_control_mode\"") != std::string::npos);
    }
    if (msg.topic == "panasonic_heat_pump/meisha/state/control_mode") {
      found_state = true;
      assert(msg.payload == "Shadow");
      assert(msg.retained == true);
    }
  }
  assert(found_disc);
  assert(found_state);

  std::cout << "[PASS] test_initial_boot_and_mqtt_discovery" << std::endl;
}

void test_frame_gate_and_command_emission() {
  reset_mock_environment();
  Preferences::reset();

  set_mock_reset_reason(ESP_RST_POWERON);
  meisha_setup();

  // In Shadow mode, sending a command must be suppressed
  bool sent = meisha_send_command("SetHeatpump", "1");
  assert(!sent);
  assert(g_mock_commands.empty());

  // Switch to Active mode
  meisha_mqtt_callback("panasonic_heat_pump/meisha/set/control_mode", "meisha/set/control_mode", "Active", 6);
  assert(meisha_get_control_mode() == MEISHA_MODE_ACTIVE);

  // In Active mode, sending command succeeds
  sent = meisha_send_command("SetHeatpump", "1");
  assert(sent);
  assert(g_mock_commands.size() == 1);
  assert(g_mock_commands[0].cmd == "SetHeatpump");
  assert(g_mock_commands[0].value == "1");

  // In Off mode, sending command is suppressed
  meisha_mqtt_callback("panasonic_heat_pump/meisha/set/control_mode", "meisha/set/control_mode", "Off", 3);
  assert(meisha_get_control_mode() == MEISHA_MODE_OFF);
  sent = meisha_send_command("SetHeatpump", "0");
  assert(!sent);
  assert(g_mock_commands.size() == 1); // no new command

  std::cout << "[PASS] test_frame_gate_and_command_emission" << std::endl;
}

void test_mode_persistence_and_deduplication() {
  reset_mock_environment();
  Preferences::reset();

  set_mock_reset_reason(ESP_RST_POWERON);
  meisha_setup();

  int initial_writes = Preferences::write_counts["meisha_cfg/ctrl_mode"];

  // Set Active
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);
  assert(Preferences::write_counts["meisha_cfg/ctrl_mode"] == initial_writes + 1);

  // Set Active again (identical value) -> no extra NVS write
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);
  assert(Preferences::write_counts["meisha_cfg/ctrl_mode"] == initial_writes + 1);

  // Next boot restores Active from NVS
  set_mock_reset_reason(ESP_RST_SW);
  meisha_setup();
  assert(meisha_get_control_mode() == MEISHA_MODE_ACTIVE);

  std::cout << "[PASS] test_mode_persistence_and_deduplication" << std::endl;
}

void test_crash_loop_safeguard() {
  reset_mock_environment();
  Preferences::reset();

  // Initial boot: set to Active
  set_mock_reset_reason(ESP_RST_POWERON);
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);
  assert(meisha_get_control_mode() == MEISHA_MODE_ACTIVE);

  // Brownouts do not count as abnormal crash
  for (int i = 0; i < 5; i++) {
    set_mock_reset_reason(ESP_RST_BROWNOUT);
    meisha_setup();
    assert(meisha_get_control_mode() == MEISHA_MODE_ACTIVE);
    assert(meisha_get_rtc_state()->crash_count == 0);
  }

  // Crash 1 within 5m
  set_mock_reset_reason(ESP_RST_PANIC);
  meisha_setup();
  assert(meisha_get_control_mode() == MEISHA_MODE_ACTIVE);
  assert(meisha_get_rtc_state()->crash_count == 1);

  // Crash 2 within 5m
  set_mock_reset_reason(ESP_RST_INT_WDT);
  meisha_setup();
  assert(meisha_get_control_mode() == MEISHA_MODE_ACTIVE);
  assert(meisha_get_rtc_state()->crash_count == 2);

  // Crash 3 within 5m
  set_mock_reset_reason(ESP_RST_TASK_WDT);
  meisha_setup();
  assert(meisha_get_control_mode() == MEISHA_MODE_ACTIVE);
  assert(meisha_get_rtc_state()->crash_count == 3);
  assert(meisha_get_rtc_state()->crash_tripped == 0);

  // 5 minutes of continuous uptime (300,000 ms) clears the crash counter
  set_mock_millis(300000UL);
  meisha_loop();
  assert(meisha_get_rtc_state()->crash_count == 0);

  // Crash 1 again
  set_mock_millis(0);
  set_mock_reset_reason(ESP_RST_PANIC);
  meisha_setup();
  assert(meisha_get_rtc_state()->crash_count == 1);

  // Crash 2
  set_mock_reset_reason(ESP_RST_WDT);
  meisha_setup();
  assert(meisha_get_rtc_state()->crash_count == 2);

  // Crash 3
  set_mock_reset_reason(ESP_RST_PANIC);
  meisha_setup();
  assert(meisha_get_rtc_state()->crash_count == 3);

  // Crash 4 (> 3 crashes within 5m) -> trips to Off!
  set_mock_reset_reason(ESP_RST_PANIC);
  meisha_setup();
  assert(meisha_get_control_mode() == MEISHA_MODE_OFF);
  assert(meisha_get_rtc_state()->crash_count == 4);
  assert(meisha_get_rtc_state()->crash_tripped == 1);

  // Subsequent normal boot maintains Off
  set_mock_reset_reason(ESP_RST_POWERON);
  meisha_setup();
  assert(meisha_get_control_mode() == MEISHA_MODE_OFF);

  // User manually re-enables Active via MQTT -> clears crash_tripped and crash_count
  meisha_mqtt_callback("panasonic_heat_pump/meisha/set/control_mode", "meisha/set/control_mode", "Active", 6);
  assert(meisha_get_control_mode() == MEISHA_MODE_ACTIVE);
  assert(meisha_get_rtc_state()->crash_tripped == 0);
  assert(meisha_get_rtc_state()->crash_count == 0);

  std::cout << "[PASS] test_crash_loop_safeguard" << std::endl;
}

int main() {
  test_initial_boot_and_mqtt_discovery();
  test_frame_gate_and_command_emission();
  test_mode_persistence_and_deduplication();
  test_crash_loop_safeguard();

  std::cout << "\nAll Ticket 01 tests passed successfully!" << std::endl;
  return 0;
}
