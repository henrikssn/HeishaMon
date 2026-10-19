#include <cassert>
#include <iostream>
#include <cmath>
#include "mock_heishamon.h"
#include "../HeishaMon/meisha.h"

void test_tunables_defaults_discovery_and_bounds() {
  reset_mock_environment();
  Preferences::reset();

  set_mock_reset_reason(ESP_RST_POWERON);
  meisha_setup();

  MeishaTunables t = meisha_get_tunables();
  assert(std::fabs(t.room_target - 23.0f) < 1e-4f);
  assert(std::fabs(t.heating_curve_start - 14.0f) < 1e-4f);
  assert(std::fabs(t.heating_curve_angle - 0.8f) < 1e-4f);
  assert(t.heating_start_hour == 9);
  assert(t.cooling_mode == false);

  meisha_mqtt_connected();

  // Verify HA discovery published for tunables & rolling sensors
  bool found_rt = false;
  bool found_cs = false;
  bool found_ca = false;
  bool found_sh = false;
  bool found_cm = false;
  bool found_a1h = false;
  bool found_a24h = false;
  bool found_r6h = false;
  bool found_rsnap = false;

  for (const auto& msg : mqtt_client.published) {
    if (msg.topic == "homeassistant/number/meisha_esp_room_target/config") found_rt = true;
    if (msg.topic == "homeassistant/number/meisha_esp_heating_curve_start/config") found_cs = true;
    if (msg.topic == "homeassistant/number/meisha_esp_heating_curve_angle/config") found_ca = true;
    if (msg.topic == "homeassistant/number/meisha_esp_heating_start_hour/config") found_sh = true;
    if (msg.topic == "homeassistant/switch/meisha_esp_cooling_mode/config") found_cm = true;
    if (msg.topic == "homeassistant/sensor/meisha_esp_ambient_1h/config") found_a1h = true;
    if (msg.topic == "homeassistant/sensor/meisha_esp_ambient_24h/config") found_a24h = true;
    if (msg.topic == "homeassistant/sensor/meisha_esp_room_temp_6h/config") found_r6h = true;
    if (msg.topic == "homeassistant/sensor/meisha_esp_room_temp_snapshot/config") found_rsnap = true;
  }

  assert(found_rt && found_cs && found_ca && found_sh && found_cm &&
         found_a1h && found_a24h && found_r6h && found_rsnap);

  // Test bounds validation
  assert(!meisha_set_room_target(17.9f));
  assert(!meisha_set_room_target(25.1f));
  assert(!meisha_set_heating_curve_start(11.9f));
  assert(!meisha_set_heating_curve_start(20.1f));
  assert(!meisha_set_heating_curve_angle(0.19f));
  assert(!meisha_set_heating_curve_angle(1.51f));
  assert(!meisha_set_heating_start_hour(24));

  // Test valid updates & NVS write deduplication
  int writes_before = Preferences::write_counts["meisha_cfg/room_tgt"];
  assert(meisha_set_room_target(22.5f));
  assert(Preferences::write_counts["meisha_cfg/room_tgt"] == writes_before + 1);
  assert(std::fabs(meisha_get_tunables().room_target - 22.5f) < 1e-4f);

  // Identical value -> 0 extra NVS writes
  assert(meisha_set_room_target(22.5f));
  assert(Preferences::write_counts["meisha_cfg/room_tgt"] == writes_before + 1);

  // MQTT updates
  meisha_mqtt_callback("panasonic_heat_pump/meisha/set/heating_curve_start",
                       "meisha/set/heating_curve_start", "15.0", 4);
  assert(std::fabs(meisha_get_tunables().heating_curve_start - 15.0f) < 1e-4f);

  meisha_mqtt_callback("panasonic_heat_pump/meisha/set/heating_curve_angle",
                       "meisha/set/heating_curve_angle", "0.9", 3);
  assert(std::fabs(meisha_get_tunables().heating_curve_angle - 0.9f) < 1e-4f);

  meisha_mqtt_callback("panasonic_heat_pump/meisha/set/heating_start_hour",
                       "meisha/set/heating_start_hour", "8", 1);
  assert(meisha_get_tunables().heating_start_hour == 8);

  meisha_mqtt_callback("panasonic_heat_pump/meisha/set/cooling_mode",
                       "meisha/set/cooling_mode", "ON", 2);
  assert(meisha_get_tunables().cooling_mode == true);

  std::cout << "[PASS] test_tunables_defaults_discovery_and_bounds" << std::endl;
}

void test_rolling_averages_and_hot_reboot() {
  reset_mock_environment();
  Preferences::reset();

  set_mock_reset_reason(ESP_RST_POWERON);
  meisha_setup();

  // Set simulated live readings
  actData[0] = 0x71; // valid frame
  set_mock_topic_value(14, "10.0"); // Outside_Temp = 10.0 °C
  set_mock_topic_value(33, "21.0"); // Room_Thermostat_Temp = 21.0 °C
  set_mock_topic_value(0, "0");    // HP off

  // First loop tick seeds all ring buffers
  meisha_loop();
  assert(std::fabs(meisha_get_ambient_1h() - 10.0f) < 0.1f);
  assert(std::fabs(meisha_get_ambient_24h() - 10.0f) < 0.1f);
  assert(std::fabs(meisha_get_room_temp_6h() - 21.0f) < 0.1f);
  assert(std::fabs(meisha_get_room_temp_snapshot() - 21.0f) < 0.1f);

  // Advance time by 60s and change outside temp to 12.0
  advance_mock_millis(60000);
  set_mock_topic_value(14, "12.0");
  meisha_loop();

  // ambient_1h is mean of 59 slots of 10.0 and 1 slot of 12.0 -> (59*10 + 12)/60 = 10.033 -> ~10.0
  float a1h = meisha_get_ambient_1h();
  assert(a1h >= 10.0f && a1h <= 10.1f);

  // Perform a hot reboot (ESP_RST_SW) with valid RTC SRAM
  set_mock_reset_reason(ESP_RST_SW);
  meisha_setup();

  // Assert 100% sample retention across hot reboot
  assert(std::fabs(meisha_get_ambient_1h() - a1h) < 1e-4f);
  assert(std::fabs(meisha_get_room_temp_snapshot() - 21.0f) < 1e-4f);
  // Zero NVS checkpoint writes occurred during this hot reboot
  assert(Preferences::write_counts.find("meisha_cfg/chkpt") == Preferences::write_counts.end());

  std::cout << "[PASS] test_rolling_averages_and_hot_reboot" << std::endl;
}

void test_nvs_checkpoint_throttling_and_warm_recovery() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(1761984000); // 08:00 UTC
  meisha_setup();

  actData[0] = 0x71;
  set_mock_topic_value(14, "8.0");
  set_mock_topic_value(33, "20.5");
  set_mock_topic_value(0, "0"); // HP off
  meisha_loop();

  // Running for 3 hours with HP off -> 0 periodic checkpoints
  for (int m = 0; m < 180; m++) {
    advance_mock_millis(60000);
    advance_mock_time(60);
    meisha_loop();
  }
  assert(Preferences::write_counts.find("meisha_cfg/chkpt") == Preferences::write_counts.end());

  // Turn HP on (TOP0 == 1)
  set_mock_topic_value(0, "1");
  meisha_loop();

  // Run 1 hour active -> still 0 checkpoints
  for (int m = 0; m < 60; m++) {
    advance_mock_millis(60000);
    advance_mock_time(60);
    meisha_loop();
  }
  assert(Preferences::write_counts.find("meisha_cfg/chkpt") == Preferences::write_counts.end());

  // Run another 1 hour active (total 2 hours active) -> 1 checkpoint written!
  for (int m = 0; m < 60; m++) {
    advance_mock_millis(60000);
    advance_mock_time(60);
    meisha_loop();
  }
  assert(Preferences::write_counts["meisha_cfg/chkpt"] == 1);

  // Now simulate a hard power cut (corrupt RTC memory) 1 hour later (within same Heating Day, < 6h old checkpoint)
  advance_mock_time(3600);
  mock_corrupt_rtc_state();
  set_mock_reset_reason(ESP_RST_POWERON);
  meisha_setup();

  // Verify ring buffers were pre-filled with checkpoint values
  assert(std::fabs(meisha_get_ambient_1h() - 8.0f) < 0.1f);
  assert(std::fabs(meisha_get_ambient_24h() - 8.0f) < 0.1f);
  assert(std::fabs(meisha_get_room_temp_6h() - 20.5f) < 0.1f);
  assert(meisha_get_rtc_state()->seeded == 1);

  std::cout << "[PASS] test_nvs_checkpoint_throttling_and_warm_recovery" << std::endl;
}

void test_cold_start_recovery() {
  reset_mock_environment();
  Preferences::reset();

  // Create an old checkpoint from 10 hours ago
  set_mock_time(1761984000);
  MeishaNvsCheckpoint old_chkpt;
  old_chkpt.ambient_24h_mean = 5.0f;
  old_chkpt.ambient_1h_mean = 5.0f;
  old_chkpt.room_temp_6h_mean = 19.0f;
  old_chkpt.room_temp_snapshot = 19.0f;
  old_chkpt.degree_hours_day = 50.0f;
  old_chkpt.dhw_time = 1.0f;
  old_chkpt.pid_output = 0.35f;
  old_chkpt.saved_at_epoch = 1761984000 - (10 * 3600); // 10h old

  Preferences prefs;
  prefs.begin("meisha_cfg");
  prefs.putBytes("chkpt", &old_chkpt, sizeof(old_chkpt));

  mock_corrupt_rtc_state();
  set_mock_reset_reason(ESP_RST_POWERON);
  meisha_setup();

  // Cold start restores pid_output from NVS, but leaves seeded = 0
  assert(std::fabs(meisha_get_rtc_state()->pid_output - 0.35f) < 1e-4f);
  assert(meisha_get_rtc_state()->seeded == 0);

  // First live frame with Outside_Temp = 14.0 and Room_Temp = 22.0 seeds buffers immediately
  actData[0] = 0x71;
  set_mock_topic_value(14, "14.0");
  set_mock_topic_value(33, "22.0");
  meisha_loop();

  assert(meisha_get_rtc_state()->seeded == 1);
  assert(std::fabs(meisha_get_ambient_1h() - 14.0f) < 0.1f);
  assert(std::fabs(meisha_get_ambient_24h() - 14.0f) < 0.1f);
  assert(std::fabs(meisha_get_room_temp_6h() - 22.0f) < 0.1f);
  assert(std::fabs(meisha_get_room_temp_snapshot() - 22.0f) < 0.1f);

  std::cout << "[PASS] test_cold_start_recovery" << std::endl;
}

int main() {
  test_tunables_defaults_discovery_and_bounds();
  test_rolling_averages_and_hot_reboot();
  test_nvs_checkpoint_throttling_and_warm_recovery();
  test_cold_start_recovery();

  std::cout << "\nAll Ticket 02 tests passed successfully!" << std::endl;
  return 0;
}
