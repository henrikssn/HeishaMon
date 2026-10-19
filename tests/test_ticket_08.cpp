#include "../HeishaMon/meisha.h"
#include "mock_heishamon.h"
#include <cassert>
#include <cmath>
#include <iostream>

void test_pipe_ambient_delta_and_discovery() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(1762000000);
  meisha_setup();

  actData[0] = 0x71;
  set_mock_topic_value(14, "5.0");  // Outside_Temp = 5.0 °C
  set_mock_topic_value(21, "1.5");  // Outside_Pipe_Temp = 1.5 °C
  set_mock_topic_value(33, "23.0"); // Room_Temp = 23.0 °C
  set_mock_topic_value(18, "0");    // Quiet_Mode_Level = 0
  set_mock_topic_value(26, "0");    // Defrosting_State = 0

  meisha_loop();

  // pipe_ambient_delta = outside_temp - outside_pipe_temp = 5.0 - 1.5 = 3.5 K
  float delta = meisha_get_pipe_ambient_delta();
  assert(std::fabs(delta - 3.5f) < 0.05f);

  meisha_mqtt_connected();

  // Verify HA MQTT discovery published for pipe_ambient_delta
  bool found_disc = false;
  for (const auto &msg : mqtt_client.published) {
    if (msg.topic ==
        "homeassistant/sensor/meisha_esp_pipe_ambient_delta/config") {
      found_disc = true;
      assert(msg.payload.find("\"name\":\"Meisha Pipe Ambient Delta\"") !=
             std::string::npos);
      assert(msg.payload.find("\"unit_of_measurement\":\"K\"") !=
             std::string::npos);
      break;
    }
  }
  assert(found_disc);

  // Verify MQTT state published for pipe_ambient_delta
  bool found_state = false;
  for (const auto &msg : mqtt_client.published) {
    if (msg.topic == "panasonic_heat_pump/meisha/state/pipe_ambient_delta") {
      found_state = true;
      assert(msg.payload == "3.5");
      break;
    }
  }
  assert(found_state);

  std::cout << "[PASS] test_pipe_ambient_delta_and_discovery" << std::endl;
}

void test_quiet_mode_fm_off_triggers_and_guards() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(1762000000);
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);

  actData[0] = 0x71;
  set_mock_topic_value(0, "1");
  set_mock_topic_value(18, "1");   // Quiet Mode level = 1 (!= 0)
  set_mock_topic_value(14, "5.0"); // Outside_Temp = 5.0 °C
  set_mock_topic_value(
      21, "0.5"); // Outside_Pipe_Temp = 0.5 °C (delta = 4.5 K > 4.0 K)
  set_mock_topic_value(8, "26.0"); // Compressor_Freq = 26.0 Hz (> 25.0 Hz)
  set_mock_topic_value(33, "23.0");
  set_mock_topic_value(26, "0");

  meisha_loop();
  g_mock_commands.clear();

  // Test Trigger 1: compressor_freq > 25.0 Hz held for 1 min (60,000 ms)
  // At 30s: still pending
  advance_mock_time(30);
  advance_mock_millis(30000UL);
  meisha_loop();
  assert(g_mock_commands.empty());

  // At 60s (1 min): triggers FM Off!
  advance_mock_time(30);
  advance_mock_millis(30000UL);
  meisha_loop();

  bool found_quiet_off = false;
  for (const auto &cmd : g_mock_commands) {
    if (cmd.cmd == "SetQuietMode" && cmd.value == "0") {
      found_quiet_off = true;
    }
  }
  assert(found_quiet_off);
  assert(std::string(meisha_get_last_action_reason()) == "fm_off");

  // Deduplication check: on the next loop before TOP18 updates to 0, does not
  // re-emit
  g_mock_commands.clear();
  advance_mock_time(1);
  advance_mock_millis(1000UL);
  meisha_loop();
  assert(g_mock_commands.empty());

  // Now simulate heat pump confirming Quiet Mode = 0
  set_mock_topic_value(18, "0");
  meisha_loop();
  assert(g_mock_commands.empty());

  // Test Trigger 2: pipe_ambient_delta > 4.0 K held for 5 min (300,000 ms) with
  // compressor <= 25 Hz
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();
  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(1762000000 + 1000);
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);

  actData[0] = 0x71;
  set_mock_topic_value(18, "1");   // Quiet mode 1
  set_mock_topic_value(14, "5.0"); // Outside_Temp = 5.0 °C
  set_mock_topic_value(21, "0.5"); // Pipe temp = 0.5 °C (delta = 4.5 K > 4.0 K)
  set_mock_topic_value(
      8, "20.0"); // Compressor freq = 20.0 Hz (<= 25.0 Hz, Trigger 1 disabled)
  set_mock_topic_value(33, "23.0");
  set_mock_topic_value(26, "0");

  meisha_loop();
  g_mock_commands.clear();

  // Advance 4 minutes (240s): still pending
  advance_mock_time(240);
  advance_mock_millis(240000UL);
  meisha_loop();
  assert(g_mock_commands.empty());

  // Advance 1 more minute (total 5 min): Trigger 2 fires!
  advance_mock_time(60);
  advance_mock_millis(60000UL);
  meisha_loop();

  found_quiet_off = false;
  for (const auto &cmd : g_mock_commands) {
    if (cmd.cmd == "SetQuietMode" && cmd.value == "0") {
      found_quiet_off = true;
    }
  }
  assert(found_quiet_off);
  assert(std::string(meisha_get_last_action_reason()) == "fm_off");

  // Test Trigger 3 & 4: (pipe_ambient_delta > 3.0 K and outside_temp < 1.1 °C)
  // for 5 min
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();
  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(1762000000 + 2000);
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);

  actData[0] = 0x71;
  set_mock_topic_value(18, "1");   // Quiet mode 1
  set_mock_topic_value(14, "0.5"); // Outside_Temp = 0.5 °C (< 1.1 °C)
  set_mock_topic_value(
      21, "-3.0"); // Pipe temp = -3.0 °C (delta = 3.5 K: > 3.0 K but <= 4.0 K)
  set_mock_topic_value(8, "20.0"); // Compressor freq = 20.0 Hz (<= 25.0 Hz)
  set_mock_topic_value(33, "23.0");
  set_mock_topic_value(26, "0");

  meisha_loop();
  g_mock_commands.clear();

  // Advance 4 minutes: pending
  advance_mock_time(240);
  advance_mock_millis(240000UL);
  meisha_loop();
  assert(g_mock_commands.empty());

  // Advance 1 more minute (total 5 min): Trigger 3 & 4 fires!
  advance_mock_time(60);
  advance_mock_millis(60000UL);
  meisha_loop();

  found_quiet_off = false;
  for (const auto &cmd : g_mock_commands) {
    if (cmd.cmd == "SetQuietMode" && cmd.value == "0") {
      found_quiet_off = true;
    }
  }
  assert(found_quiet_off);
  assert(std::string(meisha_get_last_action_reason()) == "fm_off");

  // Guard A: quiet_mode_level == 0 (already Off) -> inhibited
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();
  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(1762000000 + 3000);
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);

  actData[0] = 0x71;
  set_mock_topic_value(18, "0"); // Quiet mode ALREADY 0
  set_mock_topic_value(14, "5.0");
  set_mock_topic_value(21, "0.5"); // delta = 4.5 K > 4.0 K
  set_mock_topic_value(8, "28.0"); // comp > 25 Hz
  set_mock_topic_value(33, "23.0");

  meisha_loop();
  g_mock_commands.clear();

  advance_mock_time(120);
  advance_mock_millis(120000UL);
  meisha_loop();
  assert(g_mock_commands.empty());

  // Guard B: pipe_ambient_delta > 3.0 K but outside_temp >= 1.1 °C and delta
  // <= 4.0 K -> inhibited
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();
  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(1762000000 + 4000);
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);
  meisha_set_cooling_mode(true);
  set_mock_topic_value(4, "1"); // Cool only

  actData[0] = 0x71;
  set_mock_topic_value(18, "1");
  set_mock_topic_value(14, "2.0"); // Outside_Temp = 2.0 °C (>= 1.1 °C)
  set_mock_topic_value(
      21, "-1.5"); // Pipe temp = -1.5 °C (delta = 3.5 K: > 3.0 K but <= 4.0 K)
  set_mock_topic_value(8, "20.0"); // comp <= 25 Hz
  set_mock_topic_value(33, "23.0");

  meisha_loop();
  g_mock_commands.clear();

  advance_mock_time(600);
  advance_mock_millis(600000UL);
  meisha_loop();
  assert(g_mock_commands.empty());

  // Guard C: pipe_ambient_delta <= 3.0 K even when outside_temp < 1.1 °C ->
  // inhibited
  set_mock_topic_value(14, "0.5");  // < 1.1 °C
  set_mock_topic_value(21, "-1.5"); // delta = 2.0 K <= 3.0 K
  g_mock_commands.clear();

  advance_mock_time(600);
  advance_mock_millis(600000UL);
  meisha_loop();
  assert(g_mock_commands.empty());

  std::cout << "[PASS] test_quiet_mode_fm_off_triggers_and_guards" << std::endl;
}

void test_quiet_mode_fm_on_triggers_and_2h_defrost_guard() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(1762000000);
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);
  meisha_set_cooling_mode(true);
  set_mock_topic_value(4, "1"); // Cool only

  actData[0] = 0x71;
  set_mock_topic_value(18, "0");   // Quiet mode = 0 (Off)
  set_mock_topic_value(14, "5.0"); // Outside_Temp = 5.0 °C (> 1.0 °C)
  set_mock_topic_value(
      21, "3.5"); // Pipe temp = 3.5 °C (delta = 1.5 K < 2.1 K and < 3.1 K)
  set_mock_topic_value(
      8, "15.0"); // Compressor freq = 15.0 Hz (1.0 < 15.0 < 23.5 Hz)
  set_mock_topic_value(33, "23.0");
  set_mock_topic_value(26, "0"); // Defrosting = 0

  meisha_loop();
  g_mock_commands.clear();

  // 1. Advance 1 minute: compressor_freq has held for 1 min, BUT
  // defrosting_state == 0 has only held for 1 minute (< 2 hours)! Must NOT fire
  // yet.
  advance_mock_time(60);
  advance_mock_millis(60000UL);
  meisha_loop();
  assert(g_mock_commands.empty());

  // 2. Advance to 1 hour 59 min (7140s total): still < 2 hours! Must NOT fire.
  advance_mock_time(7080);
  advance_mock_millis(7080000UL);
  meisha_loop();
  assert(g_mock_commands.empty());

  // 3. Advance to 2 hours 1 min (total 7260s):
  // 2 hours defrost guard has elapsed, 1 min compressor light load held ->
  // fires FM On!
  advance_mock_time(120);
  advance_mock_millis(120000UL);
  meisha_loop();

  bool found_quiet_on = false;
  for (const auto &cmd : g_mock_commands) {
    if (cmd.cmd == "SetQuietMode" && cmd.value == "1") {
      found_quiet_on = true;
    }
  }
  assert(found_quiet_on);
  assert(std::string(meisha_get_last_action_reason()) == "fm_on");

  // Deduplication check: on next loop before TOP18 updates to 1, does not
  // re-emit
  g_mock_commands.clear();
  advance_mock_time(1);
  advance_mock_millis(1000UL);
  meisha_loop();
  assert(g_mock_commands.empty());

  // 4. Test Defrost Interruption Resets 2-Hour Guard:
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();
  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(1762000000 + 10000);
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);
  meisha_set_cooling_mode(true);
  set_mock_topic_value(4, "1"); // Cool only

  actData[0] = 0x71;
  set_mock_topic_value(18, "0"); // Quiet mode 0
  set_mock_topic_value(14, "5.0");
  set_mock_topic_value(21, "3.5"); // delta = 1.5 K
  set_mock_topic_value(8, "15.0"); // 1.0 < 15.0 < 23.5
  set_mock_topic_value(33, "23.0");
  set_mock_topic_value(26, "0"); // Defrost off

  meisha_loop();

  // Run 1 hour with no defrost:
  advance_mock_time(3600);
  advance_mock_millis(3600000UL);
  meisha_loop();

  // Now defrost occurs (defrosting_state = 1) for 10 minutes:
  set_mock_topic_value(26, "1");
  advance_mock_time(600);
  advance_mock_millis(600000UL);
  meisha_loop();

  // Defrost ends (defrosting_state = 0): 2-hour timer starts from ZERO!
  set_mock_topic_value(26, "0");
  meisha_loop();
  g_mock_commands.clear();

  // Advance 1 hour 59 min (7140s) after defrost ended: must NOT fire!
  advance_mock_time(7140);
  advance_mock_millis(7140000UL);
  meisha_loop();
  assert(g_mock_commands.empty());

  // Advance 2 more minutes (7260s after defrost ended): fires!
  advance_mock_time(120);
  advance_mock_millis(120000UL);
  meisha_loop();

  found_quiet_on = false;
  for (const auto &cmd : g_mock_commands) {
    if (cmd.cmd == "SetQuietMode" && cmd.value == "1") {
      found_quiet_on = true;
    }
  }
  assert(found_quiet_on);
  assert(std::string(meisha_get_last_action_reason()) == "fm_on");

  // 5. Test Trigger B: pipe_ambient_delta drops below 4.0 K transition
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();
  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(1762000000 + 20000);
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);
  meisha_set_cooling_mode(true);
  set_mock_topic_value(4, "1"); // Cool only

  actData[0] = 0x71;
  set_mock_topic_value(18, "0");   // Quiet mode 0
  set_mock_topic_value(14, "5.0"); // Outside_Temp = 5.0 °C (> 1.0 °C)
  set_mock_topic_value(21,
                       "0.5"); // Pipe temp = 0.5 °C (delta = 4.5 K >= 4.0 K)
  set_mock_topic_value(8, "15.0"); // 1.0 < 15.0 < 23.5 Hz
  set_mock_topic_value(33, "23.0");
  set_mock_topic_value(26, "0");

  meisha_loop();

  // Advance 2 hours 5 minutes so 2h defrost guard is satisfied while delta
  // is 4.5 K (>= 4.0 K)
  advance_mock_time(7500);
  advance_mock_millis(7500000UL);
  meisha_loop();
  g_mock_commands.clear();

  // delta is 4.5 K, so condition delta < 3.1 is false -> no command
  assert(g_mock_commands.empty());

  // Now pipe temp rises to 2.5 °C (delta drops from 4.5 K to 2.5 K < 4.0 K and
  // < 3.1 K) Transition below 4.0 K triggers FM On immediately!
  set_mock_topic_value(21, "2.5");
  advance_mock_time(1);
  advance_mock_millis(1000UL);
  meisha_loop();

  found_quiet_on = false;
  for (const auto &cmd : g_mock_commands) {
    if (cmd.cmd == "SetQuietMode" && cmd.value == "1") {
      found_quiet_on = true;
    }
  }
  assert(found_quiet_on);
  assert(std::string(meisha_get_last_action_reason()) == "fm_on");

  // 6. Test Guards on FM On:
  // Guard A: compressor_freq <= 1.0 Hz (e.g. 0.0 Hz) or >= 23.5 Hz (e.g. 24.0
  // Hz)
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();
  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(1762000000 + 30000);
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_ACTIVE);
  meisha_set_cooling_mode(true);
  set_mock_topic_value(4, "1"); // Cool only

  actData[0] = 0x71;
  set_mock_topic_value(18, "0");
  set_mock_topic_value(14, "5.0");
  set_mock_topic_value(21, "3.5"); // delta = 1.5 K
  set_mock_topic_value(8, "24.0"); // Comp >= 23.5 Hz
  set_mock_topic_value(26, "0");

  meisha_loop();
  advance_mock_time(7500);
  advance_mock_millis(7500000UL);
  g_mock_commands.clear();
  meisha_loop();
  assert(g_mock_commands.empty()); // Inhibited by heavy load

  // Guard B: pipe_ambient_delta >= 3.1 K
  set_mock_topic_value(8, "15.0"); // light load ok
  set_mock_topic_value(21, "1.5"); // delta = 3.5 K >= 3.1 K
  g_mock_commands.clear();
  advance_mock_time(120);
  advance_mock_millis(120000UL);
  meisha_loop();
  assert(g_mock_commands.empty()); // Inhibited by delta >= 3.1 K

  // Guard C: delta >= 2.1 K and outside_temp <= 1.0 °C
  set_mock_topic_value(14, "0.5");  // outside_temp <= 1.0 °C
  set_mock_topic_value(21, "-2.0"); // delta = 2.5 K (between 2.1 K and 3.1 K)
  g_mock_commands.clear();
  advance_mock_time(120);
  advance_mock_millis(120000UL);
  meisha_loop();
  assert(g_mock_commands.empty()); // Inhibited because delta >= 2.1 K and
                                   // outside_temp <= 1.0 °C

  // Recover outside_temp to 2.0 °C (> 1.0 °C) with delta = 2.5 K (< 3.1 K):
  set_mock_topic_value(14, "2.0");
  set_mock_topic_value(21, "-0.5"); // delta = 2.5 K
  g_mock_commands.clear();
  advance_mock_time(120);
  advance_mock_millis(120000UL);
  meisha_loop();

  found_quiet_on = false;
  for (const auto &cmd : g_mock_commands) {
    if (cmd.cmd == "SetQuietMode" && cmd.value == "1")
      found_quiet_on = true;
  }
  assert(found_quiet_on);

  std::cout << "[PASS] test_quiet_mode_fm_on_triggers_and_2h_defrost_guard"
            << std::endl;
}

void test_quiet_mode_shadow_mode() {
  reset_mock_environment();
  Preferences::reset();
  mock_corrupt_rtc_state();

  set_mock_reset_reason(ESP_RST_POWERON);
  set_mock_time(1762000000);
  meisha_setup();
  meisha_set_control_mode(MEISHA_MODE_SHADOW); // SHADOW MODE

  actData[0] = 0x71;
  set_mock_topic_value(18, "1"); // Quiet mode 1
  set_mock_topic_value(14, "5.0");
  set_mock_topic_value(21, "0.5"); // delta = 4.5 K > 4.0 K
  set_mock_topic_value(8, "26.0"); // comp > 25 Hz
  set_mock_topic_value(33, "23.0");
  set_mock_topic_value(26, "0");

  meisha_loop();
  g_mock_commands.clear();

  // Advance 60s (1 min) so FM Off triggers:
  advance_mock_time(60);
  advance_mock_millis(60000UL);
  meisha_loop();

  // In Shadow Mode, calculations run and last_action_reason updates to
  // "fm_off", but g_mock_commands MUST BE EMPTY!
  assert(g_mock_commands.empty());
  assert(std::string(meisha_get_last_action_reason()) == "fm_off");

  // Test FM On in Shadow Mode:
  set_mock_topic_value(18, "0");   // Quiet mode 0
  set_mock_topic_value(21, "3.5"); // delta = 1.5 K < 2.1 K
  set_mock_topic_value(8, "15.0"); // 1.0 < 15.0 < 23.5 Hz
  meisha_loop();
  g_mock_commands.clear();

  // Advance 2 hours 1 minute:
  advance_mock_time(7260);
  advance_mock_millis(7260000UL);
  meisha_loop();

  assert(g_mock_commands.empty());
  assert(std::string(meisha_get_last_action_reason()) == "fm_on");

  std::cout << "[PASS] test_quiet_mode_shadow_mode" << std::endl;
}

int main() {
  test_pipe_ambient_delta_and_discovery();
  test_quiet_mode_fm_off_triggers_and_guards();
  test_quiet_mode_fm_on_triggers_and_2h_defrost_guard();
  test_quiet_mode_shadow_mode();

  std::cout << "\nAll Ticket 08 tests passed successfully!" << std::endl;
  return 0;
}
