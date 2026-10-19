#include <cassert>
#include <iostream>
#include <cmath>
#include "mock_heishamon.h"
#include "../HeishaMon/meisha.h"

void test_curve_target_and_offset_clamping() {
  reset_mock_environment();
  Preferences::reset();

  set_mock_reset_reason(ESP_RST_POWERON);
  meisha_setup();

  actData[0] = 0x71; // valid frame
  set_mock_topic_value(14, "10.0"); // Outside_Temp = 10.0
  set_mock_topic_value(33, "23.0"); // Room_Thermostat_Temp = 23.0
  meisha_loop();

  // Baseline: room_target=23.0, heating_curve_start=14.0, heating_curve_angle=0.8, ambient_24h=10.0
  // curve_ambient = round2(23.0 + (14.0 - 10.0) * 0.8) = round2(23.0 + 3.2) = 26.2
  // With room_temp_snapshot=23.0 (offset = 0.0):
  // curve_target = round2(26.2 + 0.0 * 0.8) = 26.20
  float ct = meisha_get_curve_target();
  assert(std::fabs(ct - 26.20f) < 0.02f);
  assert(std::fabs(meisha_get_degree_hours_target() - (24.0f * (26.20f - 23.0f))) < 0.1f);

  // Set ambient_24h_mean = 5.0 in rtc_state to test colder ambient
  auto* rtc = const_cast<MeishaRtcState*>(meisha_get_rtc_state());
  rtc->ambient_24h_mean = 5.0f;
  // curve_ambient = round2(23.0 + (14.0 - 5.0) * 0.8) = round2(23.0 + 7.2) = 30.2

  // 1. Snapshot = 23.0 (offset = 0.0) -> curve_target = 30.20
  rtc->room_temp_snapshot = 23.0f;
  meisha_loop();
  assert(std::fabs(meisha_get_curve_target() - 30.20f) < 0.02f);

  // 2. Snapshot = 21.5 (offset = +1.5 K) -> curve_target = round2(30.2 + 1.5 * 0.8) = 31.40
  rtc->room_temp_snapshot = 21.5f;
  meisha_loop();
  assert(std::fabs(meisha_get_curve_target() - 31.40f) < 0.02f);

  // 3. Snapshot = 19.0 (offset = +4.0 K, clamped to +2.0 K) -> curve_target = round2(30.2 + 2.0 * 0.8) = 31.80
  rtc->room_temp_snapshot = 19.0f;
  meisha_loop();
  assert(std::fabs(meisha_get_curve_target() - 31.80f) < 0.02f);

  // 4. Snapshot = 24.0 (offset = -1.0 K) -> curve_target = round2(30.2 - 1.0 * 0.8) = 29.40
  rtc->room_temp_snapshot = 24.0f;
  meisha_loop();
  assert(std::fabs(meisha_get_curve_target() - 29.40f) < 0.02f);

  // 5. Snapshot = 26.5 (offset = -3.5 K, clamped to -2.0 K) -> curve_target = round2(30.2 - 2.0 * 0.8) = 28.60
  rtc->room_temp_snapshot = 26.5f;
  meisha_loop();
  assert(std::fabs(meisha_get_curve_target() - 28.60f) < 0.02f);

  std::cout << "[PASS] test_curve_target_and_offset_clamping" << std::endl;
}

void test_pid_integration_clamping_and_power_target_hysteresis() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  set_mock_reset_reason(ESP_RST_POWERON);
  meisha_setup();

  actData[0] = 0x71;
  set_mock_topic_value(14, "10.0");
  set_mock_topic_value(33, "23.0");
  set_mock_topic_value(1, "15.0");   // Pump_Flow = 15.0 l/min
  set_mock_topic_value(20, "Room");  // ThreeWay_Valve_State = Room
  set_mock_topic_value(6, "26.2");   // Main_Outlet_Temp = 26.2 (matches curve_target initially)

  meisha_loop();
  assert(std::fabs(meisha_get_pid_output()) < 1e-4f);

  // 1. Outlet temp below curve target (error: outlet=24.2 vs target=26.2 -> curve_error = +2.0, pid_error = -2.0)
  // dt = 10s: pid_output += 5e-5 * 10 * (-2.0) = -0.0010
  set_mock_topic_value(6, "24.2");
  advance_mock_millis(10000);
  meisha_loop();
  assert(std::fabs(meisha_get_pid_output() - (-0.0010f)) < 1e-4f);

  // 2. No pump flow -> error is 0, pid_output unchanged
  set_mock_topic_value(1, "0.0");
  advance_mock_millis(10000);
  meisha_loop();
  assert(std::fabs(meisha_get_pid_output() - (-0.0010f)) < 1e-4f);

  // 3. Valve = Tank -> error is 0, pid_output unchanged
  set_mock_topic_value(1, "15.0");
  set_mock_topic_value(20, "Tank");
  advance_mock_millis(10000);
  meisha_loop();
  assert(std::fabs(meisha_get_pid_output() - (-0.0010f)) < 1e-4f);

  // 4. Clamping anti-windup: force a large error and large time step
  set_mock_topic_value(20, "Room");
  set_mock_topic_value(6, "10.0"); // pid_error = -16.2
  auto* rtc = const_cast<MeishaRtcState*>(meisha_get_rtc_state());
  rtc->pid_output = -0.999f;
  advance_mock_millis(10000);
  meisha_loop();
  assert(meisha_get_pid_output() >= -1.0f);
  assert(std::fabs(meisha_get_pid_output() - (-1.0f)) < 1e-4f);

  rtc->pid_output = 0.999f;
  set_mock_topic_value(6, "40.0"); // pid_error = +13.8
  advance_mock_millis(10000);
  meisha_loop();
  assert(meisha_get_pid_output() <= 1.0f);
  assert(std::fabs(meisha_get_pid_output() - 1.0f) < 1e-4f);

  // 5. Power target minimum floor = 27 °C
  // curve_target ~ 26.2, pid_output = 0 -> raw = max(27, 26.2) = 27 -> power_target = 27
  rtc->pid_output = 0.0f;
  rtc->power_target = 0; // force recompute
  meisha_loop();
  assert(meisha_get_power_target() == 27);

  // 6. Power target ±0.1 K hysteresis around boundary (e.g. 28.5)
  // Let curve_ambient = 28.5, pid_output = 0.0
  rtc->ambient_24h_mean = 7.125f; // produces curve_ambient = 23 + (14 - 7.125)*0.8 = 28.5
  rtc->room_temp_snapshot = 23.0f;
  rtc->power_target = 28; // set previous to 28

  // raw = 28.45 -> round(28.45 - 0.1) = 28, round(28.45 + 0.1) = 29 -> clamp(28, 28, 29) = 28
  rtc->ambient_24h_mean = 7.1875f; // curve_target = 28.45
  meisha_loop();
  assert(meisha_get_power_target() == 28);

  // raw = 28.55 -> round(28.55 - 0.1) = 28, round(28.55 + 0.1) = 29 -> clamp(28, 28, 29) = 28 (holds!)
  rtc->ambient_24h_mean = 7.0625f; // curve_target = 28.55
  meisha_loop();
  assert(meisha_get_power_target() == 28);

  // raw = 28.65 -> round(28.65 - 0.1) = 29, round(28.65 + 0.1) = 29 -> steps to 29!
  rtc->ambient_24h_mean = 6.9375f; // curve_target = 28.65
  meisha_loop();
  assert(meisha_get_power_target() == 29);

  // raw = 28.55 with prev=29 -> round(28.45)=28, round(28.65)=29 -> clamp(29, 28, 29) = 29 (holds!)
  rtc->ambient_24h_mean = 7.0625f; // curve_target = 28.55
  meisha_loop();
  assert(meisha_get_power_target() == 29);

  // raw = 28.35 -> round(28.25)=28, round(28.45)=28 -> steps back to 28!
  rtc->ambient_24h_mean = 7.3125f; // curve_target = 28.35
  meisha_loop();
  assert(meisha_get_power_target() == 28);

  std::cout << "[PASS] test_pid_integration_clamping_and_power_target_hysteresis" << std::endl;
}

void test_compressor_running_hysteresis() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  set_mock_reset_reason(ESP_RST_POWERON);
  meisha_setup();

  actData[0] = 0x71;
  set_mock_topic_value(14, "10.0");
  set_mock_topic_value(33, "23.0");

  // Initial: 0 Hz -> off
  set_mock_topic_value(8, "0.0");
  meisha_loop();
  assert(!meisha_get_compressor_running());

  // Below 12.0 Hz -> stays off
  set_mock_topic_value(8, "8.0");
  meisha_loop();
  assert(!meisha_get_compressor_running());

  set_mock_topic_value(8, "11.9");
  meisha_loop();
  assert(!meisha_get_compressor_running());

  // Above 12.0 Hz -> turns on
  set_mock_topic_value(8, "12.1");
  meisha_loop();
  assert(meisha_get_compressor_running());

  // Between 8.0 and 12.0 Hz -> stays on
  set_mock_topic_value(8, "10.0");
  meisha_loop();
  assert(meisha_get_compressor_running());

  set_mock_topic_value(8, "8.0");
  meisha_loop();
  assert(meisha_get_compressor_running());

  // Below 8.0 Hz -> turns off
  set_mock_topic_value(8, "7.9");
  meisha_loop();
  assert(!meisha_get_compressor_running());

  std::cout << "[PASS] test_compressor_running_hysteresis" << std::endl;
}

void test_degree_hours_dhw_time_and_daily_reset() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(1761984000); // 08:00 UTC (1 hour before 09:00 start hour)
  meisha_setup();

  actData[0] = 0x71;
  set_mock_topic_value(14, "10.0");
  set_mock_topic_value(33, "23.0");
  set_mock_topic_value(1, "10.0");  // Flow > 0
  set_mock_topic_value(20, "Room"); // Valve = Room
  set_mock_topic_value(6, "27.0");  // Outlet = 27.0 -> room_delta = 27.0 - 23.0 = 4.0 K
  meisha_loop();

  assert(std::fabs(meisha_get_room_delta() - 4.0f) < 1e-3f);
  assert(std::fabs(meisha_get_degree_hours_day()) < 1e-4f);
  assert(std::fabs(meisha_get_dhw_time()) < 1e-4f);

  // Run for 30 minutes (1800 s) in Room mode
  advance_mock_millis(1800000);
  advance_mock_time(1800);
  meisha_loop();

  // 4.0 K * 0.5 h = 2.0 Kh
  assert(std::fabs(meisha_get_degree_hours_day() - 2.0f) < 0.05f);
  assert(std::fabs(meisha_get_dhw_time()) < 1e-4f);

  // Switch to Tank (DHW) mode for 30 minutes (1800 s)
  set_mock_topic_value(20, "Tank");
  advance_mock_millis(1800000);
  advance_mock_time(1800); // Now 09:00:00 UTC! (Daily reset time)
  meisha_loop();

  // At 09:00:00, daily reset triggers: resets degree_hours_day and dhw_time to 0.0,
  // updates room_temp_snapshot, and writes NVS checkpoint
  assert(std::fabs(meisha_get_degree_hours_day()) < 1e-4f);
  assert(std::fabs(meisha_get_dhw_time()) < 1e-4f);
  assert(Preferences::write_counts["meisha_cfg/chkpt"] >= 1);

  // Now run 1 hour in Tank mode past 09:00 (until 10:00)
  advance_mock_millis(3600000);
  advance_mock_time(3600);
  meisha_loop();

  // room_delta is 0 when valve is Tank, degree_hours_day = 0
  assert(std::fabs(meisha_get_room_delta()) < 1e-4f);
  assert(std::fabs(meisha_get_degree_hours_day()) < 1e-4f);
  // dhw_time accumulated 1.0 hour
  assert(std::fabs(meisha_get_dhw_time() - 1.0f) < 0.05f);

  // Verify hours_target and hours_time_left
  // room_target=23, curve_target ~ 26.2 -> hours_target = (26.2 - 23)/(30 - 23) * 24 = 3.2 / 7 * 24 = ~10.97 h
  float ht = meisha_get_hours_target();
  assert(ht >= 10.0f && ht <= 12.0f);

  // At 10:00, 1 hour has elapsed since 09:00 start hour.
  // hours_time_left = max(0, min(24, hours_target + dhw_time) - 1.0) = ~10.97 + 1.0 - 1.0 = ~10.97 h
  float htl = meisha_get_hours_time_left();
  assert(std::fabs(htl - ht) < 0.1f);

  std::cout << "[PASS] test_degree_hours_dhw_time_and_daily_reset" << std::endl;
}

void test_mqtt_discovery_and_derived_states() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  set_mock_reset_reason(ESP_RST_POWERON);
  meisha_setup();

  actData[0] = 0x71;
  set_mock_topic_value(14, "10.0");
  set_mock_topic_value(33, "23.0");
  meisha_loop();

  meisha_mqtt_connected();

  // Verify HA discovery published for all 11 Ticket 03 entities
  const char* required_discoveries[] = {
    "homeassistant/sensor/meisha_esp_curve_target/config",
    "homeassistant/sensor/meisha_esp_pid_output/config",
    "homeassistant/sensor/meisha_esp_power_target/config",
    "homeassistant/sensor/meisha_esp_room_delta/config",
    "homeassistant/sensor/meisha_esp_degree_hours_day/config",
    "homeassistant/sensor/meisha_esp_degree_hours_target/config",
    "homeassistant/sensor/meisha_esp_dhw_time/config",
    "homeassistant/sensor/meisha_esp_hours_target/config",
    "homeassistant/sensor/meisha_esp_hours_time_left/config",
    "homeassistant/sensor/meisha_esp_room_temp_snapshot/config",
    "homeassistant/binary_sensor/meisha_esp_compressor_running/config"
  };

  for (const char* disc : required_discoveries) {
    bool found = false;
    for (const auto& msg : mqtt_client.published) {
      if (msg.topic == disc) {
        found = true;
        break;
      }
    }
    if (!found) {
      std::cerr << "Missing discovery topic: " << disc << std::endl;
      assert(false);
    }
  }

  // Verify state topics published
  const char* required_states[] = {
    "panasonic_heat_pump/meisha/state/curve_target",
    "panasonic_heat_pump/meisha/state/pid_output",
    "panasonic_heat_pump/meisha/state/power_target",
    "panasonic_heat_pump/meisha/state/room_delta",
    "panasonic_heat_pump/meisha/state/degree_hours_day",
    "panasonic_heat_pump/meisha/state/degree_hours_target",
    "panasonic_heat_pump/meisha/state/dhw_time",
    "panasonic_heat_pump/meisha/state/hours_target",
    "panasonic_heat_pump/meisha/state/hours_time_left",
    "panasonic_heat_pump/meisha/state/compressor_running"
  };

  for (const char* st : required_states) {
    bool found = false;
    for (const auto& msg : mqtt_client.published) {
      if (msg.topic == st) {
        found = true;
        break;
      }
    }
    if (!found) {
      std::cerr << "Missing state topic: " << st << std::endl;
      assert(false);
    }
  }

  std::cout << "[PASS] test_mqtt_discovery_and_derived_states" << std::endl;
}

int main() {
  test_curve_target_and_offset_clamping();
  test_pid_integration_clamping_and_power_target_hysteresis();
  test_compressor_running_hysteresis();
  test_degree_hours_dhw_time_and_daily_reset();
  test_mqtt_discovery_and_derived_states();

  std::cout << "\nAll Ticket 03 tests passed successfully!" << std::endl;
  return 0;
}
