#include <cassert>
#include <iostream>
#include <string>
#include <cmath>
#include "mock_heishamon.h"
#include "../HeishaMon/meisha.h"

static bool has_published_topic(const std::string& topic, std::string* out_payload = nullptr) {
  for (const auto& msg : mqtt_client.published) {
    if (msg.topic == topic) {
      if (out_payload) *out_payload = msg.payload;
      return true;
    }
  }
  return false;
}

void test_cold_boot_unseeded_mqtt_connect_suppresses_zeroes() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  set_mock_reset_reason(ESP_RST_POWERON);
  meisha_setup();

  // On cold boot before telemetry, seeded must be 0
  assert(meisha_get_rtc_state()->seeded == 0);

  mqtt_client.clear();
  meisha_mqtt_connected();

  // Verify HA discovery, mode, and tunables ARE published
  assert(has_published_topic("homeassistant/number/meisha_esp_room_target/config"));
  assert(has_published_topic("homeassistant/sensor/meisha_esp_ambient_1h/config"));
  assert(has_published_topic("homeassistant/sensor/meisha_esp_curve_target/config"));

  std::string mode_val;
  assert(has_published_topic("panasonic_heat_pump/meisha/state/control_mode", &mode_val));
  assert(mode_val == "Shadow");

  std::string rt_val;
  assert(has_published_topic("panasonic_heat_pump/meisha/state/room_target", &rt_val));
  assert(rt_val == "23.0");

  // Verify that rolling averages and derived sensor states are SUPPRESSED (not published at all)
  const char* suppressed_topics[] = {
    "panasonic_heat_pump/meisha/state/ambient_1h",
    "panasonic_heat_pump/meisha/state/ambient_24h",
    "panasonic_heat_pump/meisha/state/room_temp_6h",
    "panasonic_heat_pump/meisha/state/room_temp_snapshot",
    "panasonic_heat_pump/meisha/state/curve_target",
    "panasonic_heat_pump/meisha/state/pid_output",
    "panasonic_heat_pump/meisha/state/power_target",
    "panasonic_heat_pump/meisha/state/room_delta",
    "panasonic_heat_pump/meisha/state/degree_hours_day",
    "panasonic_heat_pump/meisha/state/degree_hours_target",
    "panasonic_heat_pump/meisha/state/dhw_time",
    "panasonic_heat_pump/meisha/state/hours_target",
    "panasonic_heat_pump/meisha/state/hours_time_left",
    "panasonic_heat_pump/meisha/state/pipe_ambient_delta",
    "panasonic_heat_pump/meisha/state/compressor_running"
  };

  for (const char* topic : suppressed_topics) {
    std::string val;
    bool found = has_published_topic(topic, &val);
    if (found) {
      std::cerr << "Topic should have been suppressed on unseeded connect: "
                << topic << " with value " << val << std::endl;
      assert(!found);
    }
  }

  // Ensure no message payload anywhere is a zero telemetry spike
  for (const auto& msg : mqtt_client.published) {
    if (msg.topic.find("meisha/state/") != std::string::npos) {
      assert(msg.payload != "0.0");
      assert(msg.payload != "0.00");
      assert(msg.payload != "0.0000");
    }
  }

  std::cout << "[PASS] test_cold_boot_unseeded_mqtt_connect_suppresses_zeroes" << std::endl;
}

void test_first_telemetry_live_seeding_publishes_populated_states() {
  // Continuing from cold boot, feed first valid UART telemetry frame
  mqtt_client.clear();

  actData[0] = 0x71; // Valid frame
  set_mock_topic_value(14, "11.0"); // Outside_Temp = 11.0 °C
  set_mock_topic_value(21, "8.0");  // Outside_Pipe_Temp = 8.0 °C -> pipe_ambient_delta = 3.0
  set_mock_topic_value(33, "23.0"); // Room_Thermostat_Temp = 23.0 °C
  set_mock_topic_value(0, "0");     // HP off

  meisha_loop();

  // Seeding must now be complete
  assert(meisha_get_rtc_state()->seeded == 1);

  // Rolling averages and derived states must now be published with real values
  std::string val;
  assert(has_published_topic("panasonic_heat_pump/meisha/state/ambient_1h", &val));
  assert(val == "11.0");

  assert(has_published_topic("panasonic_heat_pump/meisha/state/ambient_24h", &val));
  assert(val == "11.0");

  assert(has_published_topic("panasonic_heat_pump/meisha/state/room_temp_6h", &val));
  assert(val == "23.0");

  assert(has_published_topic("panasonic_heat_pump/meisha/state/room_temp_snapshot", &val));
  assert(val == "23.0");

  assert(has_published_topic("panasonic_heat_pump/meisha/state/curve_target", &val));
  assert(val == "25.40");

  assert(has_published_topic("panasonic_heat_pump/meisha/state/power_target", &val));
  assert(val == "27");

  assert(has_published_topic("panasonic_heat_pump/meisha/state/pipe_ambient_delta", &val));
  assert(val == "3.0");

  assert(has_published_topic("panasonic_heat_pump/meisha/state/degree_hours_target", &val));
  assert(val == "57.60");

  std::cout << "[PASS] test_first_telemetry_live_seeding_publishes_populated_states" << std::endl;
}

void test_warm_reboot_seeded_mqtt_connect_publishes_immediately() {
  // Preserve valid RTC state from previous run and simulate soft reboot (ESP_RST_SW)
  set_mock_reset_reason(ESP_RST_SW);
  meisha_setup();

  // rtc_state was valid, so seeded must remain 1
  assert(meisha_get_rtc_state()->seeded == 1);

  mqtt_client.clear();
  meisha_mqtt_connected();

  // On warm reboot, rolling averages and derived states must be published immediately
  std::string val;
  assert(has_published_topic("panasonic_heat_pump/meisha/state/ambient_1h", &val));
  assert(val == "11.0");

  assert(has_published_topic("panasonic_heat_pump/meisha/state/ambient_24h", &val));
  assert(val == "11.0");

  assert(has_published_topic("panasonic_heat_pump/meisha/state/curve_target", &val));
  assert(val == "25.40");

  assert(has_published_topic("panasonic_heat_pump/meisha/state/power_target", &val));
  assert(val == "27");

  std::cout << "[PASS] test_warm_reboot_seeded_mqtt_connect_publishes_immediately" << std::endl;
}

void test_deferred_sntp_recovery_publishes_when_seeded() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  // Create an NVS checkpoint saved at 10:00 UTC (heating day after 9:00 start)
  uint32_t saved_epoch = 1761984000 + 7200;
  MeishaNvsCheckpoint chkpt;
  chkpt.ambient_1h_mean = 9.5f;
  chkpt.ambient_24h_mean = 9.5f;
  chkpt.room_temp_6h_mean = 22.0f;
  chkpt.room_temp_snapshot = 22.0f;
  chkpt.degree_hours_day = 12.5f;
  chkpt.dhw_time = 1.0f;
  chkpt.pid_output = 0.0f;
  chkpt.saved_at_epoch = saved_epoch;

  Preferences prefs;
  prefs.begin("meisha_cfg");
  prefs.putBytes("chkpt", &chkpt, sizeof(chkpt));

  // Boot before SNTP sync (now_epoch = 0 < 1000000)
  set_mock_time(0);
  set_mock_reset_reason(ESP_RST_POWERON);
  meisha_setup();

  // While waiting for SNTP, seeded must be 0
  assert(meisha_get_rtc_state()->seeded == 0);
  assert(meisha_get_rtc_state()->nvs_decision_pending == 1);

  mqtt_client.clear();
  meisha_mqtt_connected();

  // Rolling averages must be suppressed while unseeded
  assert(!has_published_topic("panasonic_heat_pump/meisha/state/ambient_1h"));
  assert(!has_published_topic("panasonic_heat_pump/meisha/state/curve_target"));

  // Now simulate SNTP syncing time to 1 hour after checkpoint (11:00 UTC, same heating day)
  set_mock_time(saved_epoch + 3600);
  mqtt_client.clear();

  actData[0] = 0x71;
  set_mock_topic_value(14, "9.5");
  set_mock_topic_value(33, "22.0");
  set_mock_topic_value(0, "0");

  meisha_loop();

  // SNTP recovery must have succeeded and seeded rtc_state
  assert(meisha_get_rtc_state()->seeded == 1);
  assert(meisha_get_rtc_state()->nvs_decision_pending == 0);

  // States must now be published with the restored checkpoint values
  std::string val;
  assert(has_published_topic("panasonic_heat_pump/meisha/state/ambient_1h", &val));
  assert(val == "9.5");

  assert(has_published_topic("panasonic_heat_pump/meisha/state/degree_hours_day", &val));
  assert(val == "12.50");

  std::cout << "[PASS] test_deferred_sntp_recovery_publishes_when_seeded" << std::endl;
}

int main() {
  test_cold_boot_unseeded_mqtt_connect_suppresses_zeroes();
  test_first_telemetry_live_seeding_publishes_populated_states();
  test_warm_reboot_seeded_mqtt_connect_publishes_immediately();
  test_deferred_sntp_recovery_publishes_when_seeded();

  std::cout << "\nAll Ticket 09 tests passed successfully!" << std::endl;
  return 0;
}
