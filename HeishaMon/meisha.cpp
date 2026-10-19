#include "meisha.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#ifdef ARDUINO
#include <Arduino.h>
#if defined(ESP32)
#include <Preferences.h>
#elif defined(ESP8266)
// Lightweight dummy Preferences stub for ESP8266 compile target in CI
class Preferences {
public:
  bool begin(const char*, bool = false) { return true; }
  void end() {}
  uint8_t getUChar(const char*, uint8_t def = 0) { return def; }
  size_t putUChar(const char*, uint8_t) { return 1; }
  float getFloat(const char*, float def = 0.0f) { return def; }
  size_t putFloat(const char*, float) { return sizeof(float); }
  bool getBool(const char*, bool def = false) { return def; }
  size_t putBool(const char*, bool) { return 1; }
  size_t getBytes(const char*, void*, size_t) { return 0; }
  size_t putBytes(const char*, const void*, size_t len) { return len; }
};
#endif
#include <PubSubClient.h>
#include "decode.h"
#include "commands.h"
#include "webfunctions.h"

extern char actData[DATASIZE];
extern settingsStruct heishamonSettings;
extern PubSubClient mqtt_client;
extern void send_command(byte* command, int length);
extern void log_message(char* string);
#else
#include "../tests/mock_heishamon.h"
#define time mock_time
#define localtime mock_localtime
#endif

#ifdef ESP32
RTC_NOINIT_ATTR static MeishaRtcState rtc_state;
#else
static MeishaRtcState rtc_state;
#endif

static MeishaControlMode s_control_mode = MEISHA_MODE_SHADOW;
static MeishaTunables s_tunables = {
  23.0f, // room_target (18..25)
  14.0f, // heating_curve_start (12..20)
  0.8f,  // heating_curve_angle (0.2..1.5)
  9,     // heating_start_hour (0..23)
  false  // cooling_mode
};

static Preferences s_prefs;
static bool s_prefs_opened = false;
static uint32_t s_last_sample_ms = 0;
static uint32_t s_last_checkpoint_ms = 0;

static uint32_t calculate_crc32(const uint8_t* data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xEDB88320;
      } else {
        crc >>= 1;
      }
    }
  }
  return ~crc;
}

static uint32_t rtc_crc(const MeishaRtcState* state) {
  return calculate_crc32((const uint8_t*)state, offsetof(MeishaRtcState, crc32));
}

static const char* mode_to_string(MeishaControlMode mode) {
  switch (mode) {
    case MEISHA_MODE_OFF: return "Off";
    case MEISHA_MODE_SHADOW: return "Shadow";
    case MEISHA_MODE_ACTIVE: return "Active";
    default: return "Shadow";
  }
}

static bool parse_mode_string(const char* str, MeishaControlMode* out_mode) {
  if (!str) return false;
  if (strcasecmp(str, "off") == 0) {
    *out_mode = MEISHA_MODE_OFF;
    return true;
  }
  if (strcasecmp(str, "shadow") == 0) {
    *out_mode = MEISHA_MODE_SHADOW;
    return true;
  }
  if (strcasecmp(str, "active") == 0) {
    *out_mode = MEISHA_MODE_ACTIVE;
    return true;
  }
  return false;
}

static void ensure_prefs_opened(void) {
  if (!s_prefs_opened) {
    s_prefs.begin("meisha_cfg", false);
    s_prefs_opened = true;
  }
}

static int32_t get_heating_day_id(time_t epoch, uint8_t start_hour) {
  // Shifting backward by start_hour puts the entire heating day into the same calendar day
  time_t shifted = epoch - (time_t)start_hour * 3600;
  struct tm* tm_val = localtime(&shifted);
  if (!tm_val) return 0;
  return (tm_val->tm_year + 1900) * 10000 + (tm_val->tm_mon + 1) * 100 + tm_val->tm_mday;
}

static void publish_mode_state(void) {
  char topic[256];
  snprintf(topic, sizeof(topic), "%s/meisha/state/control_mode", heishamonSettings.mqtt_topic_base);
  mqtt_client.publish(topic, mode_to_string(s_control_mode), true);
}

static void publish_tunable_state(const char* name, const char* val) {
  char topic[256];
  snprintf(topic, sizeof(topic), "%s/meisha/state/%s", heishamonSettings.mqtt_topic_base, name);
  mqtt_client.publish(topic, val, true);
}

static void publish_sensor_float(const char* name, float val, int decimals) {
  char topic[256];
  snprintf(topic, sizeof(topic), "%s/meisha/state/%s", heishamonSettings.mqtt_topic_base, name);
  char buf[32];
  snprintf(buf, sizeof(buf), "%.*f", decimals, val);
  mqtt_client.publish(topic, buf, true);
}

static void publish_all_tunables_state(void) {
  char buf[32];

  snprintf(buf, sizeof(buf), "%.1f", s_tunables.room_target);
  publish_tunable_state("room_target", buf);

  snprintf(buf, sizeof(buf), "%.1f", s_tunables.heating_curve_start);
  publish_tunable_state("heating_curve_start", buf);

  snprintf(buf, sizeof(buf), "%.1f", s_tunables.heating_curve_angle);
  publish_tunable_state("heating_curve_angle", buf);

  snprintf(buf, sizeof(buf), "%u", s_tunables.heating_start_hour);
  publish_tunable_state("heating_start_hour", buf);

  publish_tunable_state("cooling_mode", s_tunables.cooling_mode ? "ON" : "OFF");
}

static void publish_rolling_average_states(void) {
  publish_sensor_float("ambient_1h", rtc_state.ambient_1h_mean, 1);
  publish_sensor_float("ambient_24h", rtc_state.ambient_24h_mean, 1);
  publish_sensor_float("room_temp_6h", rtc_state.room_temp_6h_mean, 1);
  publish_sensor_float("room_temp_snapshot", rtc_state.room_temp_snapshot, 1);
}

static void publish_ha_discovery(void) {
  char disc_topic[160];
  char payload[1024];

  // 1. Mode select
  snprintf(disc_topic, sizeof(disc_topic), "homeassistant/select/%scontrol_mode/config", MEISHA_ENTITY_PREFIX);
  snprintf(payload, sizeof(payload),
    "{"
      "\"name\":\"Meisha ESP Control Mode\","
      "\"unique_id\":\"%scontrol_mode\","
      "\"object_id\":\"%scontrol_mode\","
      "\"command_topic\":\"%s/meisha/set/control_mode\","
      "\"state_topic\":\"%s/meisha/state/control_mode\","
      "\"availability_topic\":\"%s/%s\","
      "\"payload_available\":\"Online\","
      "\"payload_not_available\":\"Offline\","
      "\"options\":[\"Off\",\"Shadow\",\"Active\"],"
      "\"device\":{\"identifiers\":[\"meisha_esp_controller\"],\"name\":\"Meisha Heat Pump Controller\",\"model\":\"HeishaMon Large ESP32-S3\",\"manufacturer\":\"Custom\"}"
    "}",
    MEISHA_ENTITY_PREFIX,
    MEISHA_ENTITY_PREFIX,
    heishamonSettings.mqtt_topic_base,
    heishamonSettings.mqtt_topic_base,
    heishamonSettings.mqtt_topic_base,
    mqtt_willtopic
  );
  mqtt_client.publish(disc_topic, payload, true);

  // 2. room_target (18..25, step 0.5)
  snprintf(disc_topic, sizeof(disc_topic), "homeassistant/number/%sroom_target/config", MEISHA_ENTITY_PREFIX);
  snprintf(payload, sizeof(payload),
    "{"
      "\"name\":\"Meisha Room Target\","
      "\"unique_id\":\"%sroom_target\","
      "\"object_id\":\"%sroom_target\","
      "\"command_topic\":\"%s/meisha/set/room_target\","
      "\"state_topic\":\"%s/meisha/state/room_target\","
      "\"min\":18.0,\"max\":25.0,\"step\":0.5,\"unit_of_measurement\":\"°C\","
      "\"device\":{\"identifiers\":[\"meisha_esp_controller\"]}"
    "}",
    MEISHA_ENTITY_PREFIX, MEISHA_ENTITY_PREFIX,
    heishamonSettings.mqtt_topic_base, heishamonSettings.mqtt_topic_base
  );
  mqtt_client.publish(disc_topic, payload, true);

  // 3. heating_curve_start (12..20, step 1.0)
  snprintf(disc_topic, sizeof(disc_topic), "homeassistant/number/%sheating_curve_start/config", MEISHA_ENTITY_PREFIX);
  snprintf(payload, sizeof(payload),
    "{"
      "\"name\":\"Meisha Heating Curve Start\","
      "\"unique_id\":\"%sheating_curve_start\","
      "\"object_id\":\"%sheating_curve_start\","
      "\"command_topic\":\"%s/meisha/set/heating_curve_start\","
      "\"state_topic\":\"%s/meisha/state/heating_curve_start\","
      "\"min\":12.0,\"max\":20.0,\"step\":1.0,\"unit_of_measurement\":\"°C\","
      "\"device\":{\"identifiers\":[\"meisha_esp_controller\"]}"
    "}",
    MEISHA_ENTITY_PREFIX, MEISHA_ENTITY_PREFIX,
    heishamonSettings.mqtt_topic_base, heishamonSettings.mqtt_topic_base
  );
  mqtt_client.publish(disc_topic, payload, true);

  // 4. heating_curve_angle (0.2..1.5, step 0.1)
  snprintf(disc_topic, sizeof(disc_topic), "homeassistant/number/%sheating_curve_angle/config", MEISHA_ENTITY_PREFIX);
  snprintf(payload, sizeof(payload),
    "{"
      "\"name\":\"Meisha Heating Curve Angle\","
      "\"unique_id\":\"%sheating_curve_angle\","
      "\"object_id\":\"%sheating_curve_angle\","
      "\"command_topic\":\"%s/meisha/set/heating_curve_angle\","
      "\"state_topic\":\"%s/meisha/state/heating_curve_angle\","
      "\"min\":0.2,\"max\":1.5,\"step\":0.1,"
      "\"device\":{\"identifiers\":[\"meisha_esp_controller\"]}"
    "}",
    MEISHA_ENTITY_PREFIX, MEISHA_ENTITY_PREFIX,
    heishamonSettings.mqtt_topic_base, heishamonSettings.mqtt_topic_base
  );
  mqtt_client.publish(disc_topic, payload, true);

  // 5. heating_start_hour (0..23, step 1)
  snprintf(disc_topic, sizeof(disc_topic), "homeassistant/number/%sheating_start_hour/config", MEISHA_ENTITY_PREFIX);
  snprintf(payload, sizeof(payload),
    "{"
      "\"name\":\"Meisha Heating Start Hour\","
      "\"unique_id\":\"%sheating_start_hour\","
      "\"object_id\":\"%sheating_start_hour\","
      "\"command_topic\":\"%s/meisha/set/heating_start_hour\","
      "\"state_topic\":\"%s/meisha/state/heating_start_hour\","
      "\"min\":0,\"max\":23,\"step\":1,\"unit_of_measurement\":\"h\","
      "\"device\":{\"identifiers\":[\"meisha_esp_controller\"]}"
    "}",
    MEISHA_ENTITY_PREFIX, MEISHA_ENTITY_PREFIX,
    heishamonSettings.mqtt_topic_base, heishamonSettings.mqtt_topic_base
  );
  mqtt_client.publish(disc_topic, payload, true);

  // 6. cooling_mode switch
  snprintf(disc_topic, sizeof(disc_topic), "homeassistant/switch/%scooling_mode/config", MEISHA_ENTITY_PREFIX);
  snprintf(payload, sizeof(payload),
    "{"
      "\"name\":\"Meisha Cooling Mode\","
      "\"unique_id\":\"%scooling_mode\","
      "\"object_id\":\"%scooling_mode\","
      "\"command_topic\":\"%s/meisha/set/cooling_mode\","
      "\"state_topic\":\"%s/meisha/state/cooling_mode\","
      "\"payload_on\":\"ON\",\"payload_off\":\"OFF\","
      "\"device\":{\"identifiers\":[\"meisha_esp_controller\"]}"
    "}",
    MEISHA_ENTITY_PREFIX, MEISHA_ENTITY_PREFIX,
    heishamonSettings.mqtt_topic_base, heishamonSettings.mqtt_topic_base
  );
  mqtt_client.publish(disc_topic, payload, true);

  // 7. ambient_1h sensor
  snprintf(disc_topic, sizeof(disc_topic), "homeassistant/sensor/%sambient_1h/config", MEISHA_ENTITY_PREFIX);
  snprintf(payload, sizeof(payload),
    "{"
      "\"name\":\"Meisha Ambient 1h\","
      "\"unique_id\":\"%sambient_1h\","
      "\"object_id\":\"%sambient_1h\","
      "\"state_topic\":\"%s/meisha/state/ambient_1h\","
      "\"unit_of_measurement\":\"°C\",\"device_class\":\"temperature\","
      "\"device\":{\"identifiers\":[\"meisha_esp_controller\"]}"
    "}",
    MEISHA_ENTITY_PREFIX, MEISHA_ENTITY_PREFIX,
    heishamonSettings.mqtt_topic_base
  );
  mqtt_client.publish(disc_topic, payload, true);

  // 8. ambient_24h sensor
  snprintf(disc_topic, sizeof(disc_topic), "homeassistant/sensor/%sambient_24h/config", MEISHA_ENTITY_PREFIX);
  snprintf(payload, sizeof(payload),
    "{"
      "\"name\":\"Meisha Ambient 24h\","
      "\"unique_id\":\"%sambient_24h\","
      "\"object_id\":\"%sambient_24h\","
      "\"state_topic\":\"%s/meisha/state/ambient_24h\","
      "\"unit_of_measurement\":\"°C\",\"device_class\":\"temperature\","
      "\"device\":{\"identifiers\":[\"meisha_esp_controller\"]}"
    "}",
    MEISHA_ENTITY_PREFIX, MEISHA_ENTITY_PREFIX,
    heishamonSettings.mqtt_topic_base
  );
  mqtt_client.publish(disc_topic, payload, true);

  // 9. room_temp_6h sensor
  snprintf(disc_topic, sizeof(disc_topic), "homeassistant/sensor/%sroom_temp_6h/config", MEISHA_ENTITY_PREFIX);
  snprintf(payload, sizeof(payload),
    "{"
      "\"name\":\"Meisha Room Temp 6h\","
      "\"unique_id\":\"%sroom_temp_6h\","
      "\"object_id\":\"%sroom_temp_6h\","
      "\"state_topic\":\"%s/meisha/state/room_temp_6h\","
      "\"unit_of_measurement\":\"°C\",\"device_class\":\"temperature\","
      "\"device\":{\"identifiers\":[\"meisha_esp_controller\"]}"
    "}",
    MEISHA_ENTITY_PREFIX, MEISHA_ENTITY_PREFIX,
    heishamonSettings.mqtt_topic_base
  );
  mqtt_client.publish(disc_topic, payload, true);

  // 10. room_temp_snapshot sensor
  snprintf(disc_topic, sizeof(disc_topic), "homeassistant/sensor/%sroom_temp_snapshot/config", MEISHA_ENTITY_PREFIX);
  snprintf(payload, sizeof(payload),
    "{"
      "\"name\":\"Meisha Room Temp Snapshot\","
      "\"unique_id\":\"%sroom_temp_snapshot\","
      "\"object_id\":\"%sroom_temp_snapshot\","
      "\"state_topic\":\"%s/meisha/state/room_temp_snapshot\","
      "\"unit_of_measurement\":\"°C\",\"device_class\":\"temperature\","
      "\"device\":{\"identifiers\":[\"meisha_esp_controller\"]}"
    "}",
    MEISHA_ENTITY_PREFIX, MEISHA_ENTITY_PREFIX,
    heishamonSettings.mqtt_topic_base
  );
  mqtt_client.publish(disc_topic, payload, true);
}

static void fill_ring_buffer(int16_t* buf, uint16_t capacity, uint16_t* head, uint16_t* count, float val) {
  int16_t deci = (int16_t)roundf(val * 10.0f);
  for (uint16_t i = 0; i < capacity; i++) {
    buf[i] = deci;
  }
  *head = 0;
  *count = capacity;
}

static void push_ring_sample(int16_t* buf, uint16_t capacity, uint16_t* head, uint16_t* count, float val) {
  int16_t deci = (int16_t)roundf(val * 10.0f);
  buf[*head] = deci;
  *head = (*head + 1) % capacity;
  if (*count < capacity) {
    (*count)++;
  }
}

static float calc_ring_mean(const int16_t* buf, uint16_t count) {
  if (count == 0) return 0.0f;
  int32_t sum = 0;
  for (uint16_t i = 0; i < count; i++) {
    sum += buf[i];
  }
  return ((float)sum / (float)count) / 10.0f;
}

static void write_nvs_checkpoint(void) {
  ensure_prefs_opened();
  MeishaNvsCheckpoint chkpt;
  chkpt.ambient_24h_mean = rtc_state.ambient_24h_mean;
  chkpt.ambient_1h_mean = rtc_state.ambient_1h_mean;
  chkpt.room_temp_6h_mean = rtc_state.room_temp_6h_mean;
  chkpt.room_temp_snapshot = rtc_state.room_temp_snapshot;
  chkpt.degree_hours_day = rtc_state.degree_hours_day;
  chkpt.dhw_time = rtc_state.dhw_time;
  chkpt.pid_output = rtc_state.pid_output;
  chkpt.saved_at_epoch = (uint32_t)time(NULL);

  s_prefs.putBytes("chkpt", &chkpt, sizeof(chkpt));
}

MeishaControlMode meisha_get_control_mode(void) {
  return s_control_mode;
}

const MeishaRtcState* meisha_get_rtc_state(void) {
  return &rtc_state;
}

MeishaTunables meisha_get_tunables(void) {
  return s_tunables;
}

float meisha_get_ambient_1h(void) {
  return rtc_state.ambient_1h_mean;
}

float meisha_get_ambient_24h(void) {
  return rtc_state.ambient_24h_mean;
}

float meisha_get_room_temp_6h(void) {
  return rtc_state.room_temp_6h_mean;
}

float meisha_get_room_temp_snapshot(void) {
  return rtc_state.room_temp_snapshot;
}

bool meisha_set_room_target(float val) {
  if (val < 18.0f || val > 25.0f) return false;
  if (fabsf(s_tunables.room_target - val) > 1e-4f) {
    s_tunables.room_target = val;
    ensure_prefs_opened();
    s_prefs.putFloat("room_tgt", val);
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", val);
    publish_tunable_state("room_target", buf);
  }
  return true;
}

bool meisha_set_heating_curve_start(float val) {
  if (val < 12.0f || val > 20.0f) return false;
  if (fabsf(s_tunables.heating_curve_start - val) > 1e-4f) {
    s_tunables.heating_curve_start = val;
    ensure_prefs_opened();
    s_prefs.putFloat("curve_start", val);
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", val);
    publish_tunable_state("heating_curve_start", buf);
  }
  return true;
}

bool meisha_set_heating_curve_angle(float val) {
  if (val < 0.2f || val > 1.5f) return false;
  if (fabsf(s_tunables.heating_curve_angle - val) > 1e-4f) {
    s_tunables.heating_curve_angle = val;
    ensure_prefs_opened();
    s_prefs.putFloat("curve_angle", val);
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", val);
    publish_tunable_state("heating_curve_angle", buf);
  }
  return true;
}

bool meisha_set_heating_start_hour(uint8_t val) {
  if (val > 23) return false;
  if (s_tunables.heating_start_hour != val) {
    s_tunables.heating_start_hour = val;
    ensure_prefs_opened();
    s_prefs.putUChar("start_hr", val);
    char buf[16];
    snprintf(buf, sizeof(buf), "%u", val);
    publish_tunable_state("heating_start_hour", buf);
  }
  return true;
}

bool meisha_set_cooling_mode(bool val) {
  if (s_tunables.cooling_mode != val) {
    s_tunables.cooling_mode = val;
    ensure_prefs_opened();
    s_prefs.putBool("cool_mode", val);
    publish_tunable_state("cooling_mode", val ? "ON" : "OFF");
  }
  return true;
}

void meisha_set_control_mode(MeishaControlMode mode) {
  if (s_control_mode != mode) {
    s_control_mode = mode;
    ensure_prefs_opened();
    s_prefs.putUChar("ctrl_mode", (uint8_t)mode);
    publish_mode_state();
  }
}

bool meisha_send_command(const char* cmd, const char* value) {
  if (s_control_mode != MEISHA_MODE_ACTIVE) {
    return false;
  }
#if defined(ARDUINO) || defined(MOCK_HEISHAMON)
  send_heatpump_command((char*)cmd, (char*)value, send_command, log_message, heishamonSettings.optionalPCB);
  return true;
#else
  return false;
#endif
}

void meisha_setup(void) {
  ensure_prefs_opened();

  // Load tunables
  s_tunables.room_target = s_prefs.getFloat("room_tgt", 23.0f);
  s_tunables.heating_curve_start = s_prefs.getFloat("curve_start", 14.0f);
  s_tunables.heating_curve_angle = s_prefs.getFloat("curve_angle", 0.8f);
  s_tunables.heating_start_hour = s_prefs.getUChar("start_hr", 9);
  s_tunables.cooling_mode = s_prefs.getBool("cool_mode", false);

  bool rtc_valid = (rtc_state.magic == MEISHA_RTC_MAGIC) && (rtc_crc(&rtc_state) == rtc_state.crc32);

  if (!rtc_valid) {
    // SRAM lost across cold power loss or first boot
    memset(&rtc_state, 0, sizeof(rtc_state));
    rtc_state.magic = MEISHA_RTC_MAGIC;
    rtc_state.version = 2;

    MeishaNvsCheckpoint chkpt;
    bool has_chkpt = (s_prefs.getBytes("chkpt", &chkpt, sizeof(chkpt)) == sizeof(chkpt));
    time_t now = time(NULL);

    if (has_chkpt && now > 1000000 && chkpt.saved_at_epoch > 0 &&
        now >= chkpt.saved_at_epoch && (now - chkpt.saved_at_epoch) < 6 * 3600) {
      // Warm recovery (< 6h old checkpoint)
      fill_ring_buffer(rtc_state.ambient_1h_buf, MEISHA_AMBIENT_1H_SAMPLES,
                       &rtc_state.ambient_1h_head, &rtc_state.ambient_1h_count, chkpt.ambient_1h_mean);
      fill_ring_buffer(rtc_state.ambient_24h_buf, MEISHA_AMBIENT_24H_SAMPLES,
                       &rtc_state.ambient_24h_head, &rtc_state.ambient_24h_count, chkpt.ambient_24h_mean);
      fill_ring_buffer(rtc_state.room_6h_buf, MEISHA_ROOM_6H_SAMPLES,
                       &rtc_state.room_6h_head, &rtc_state.room_6h_count, chkpt.room_temp_6h_mean);

      rtc_state.ambient_1h_mean = chkpt.ambient_1h_mean;
      rtc_state.ambient_24h_mean = chkpt.ambient_24h_mean;
      rtc_state.room_temp_6h_mean = chkpt.room_temp_6h_mean;
      rtc_state.pid_output = chkpt.pid_output;

      int32_t saved_day = get_heating_day_id((time_t)chkpt.saved_at_epoch, s_tunables.heating_start_hour);
      int32_t now_day = get_heating_day_id(now, s_tunables.heating_start_hour);
      if (saved_day == now_day) {
        rtc_state.degree_hours_day = chkpt.degree_hours_day;
        rtc_state.dhw_time = chkpt.dhw_time;
        rtc_state.room_temp_snapshot = chkpt.room_temp_snapshot;
        rtc_state.last_reset_heating_day_id = saved_day;
        rtc_state.seeded = 1;
      } else {
        rtc_state.degree_hours_day = 0.0f;
        rtc_state.dhw_time = 0.0f;
        rtc_state.room_temp_snapshot = 0.0f;
        rtc_state.last_reset_heating_day_id = 0;
        rtc_state.seeded = 0;
      }
    } else {
      // Cold recovery (missing or >= 6h old)
      if (has_chkpt) {
        rtc_state.pid_output = chkpt.pid_output;
      }
      rtc_state.seeded = 0;
    }
    rtc_state.crc32 = rtc_crc(&rtc_state);
  }

#if defined(ESP32) || defined(MOCK_HEISHAMON)
  esp_reset_reason_t reason = esp_reset_reason();
  bool is_crash = (reason == ESP_RST_PANIC ||
                   reason == ESP_RST_INT_WDT ||
                   reason == ESP_RST_TASK_WDT ||
                   reason == ESP_RST_WDT);
#else
  bool is_crash = false;
#endif

  if (is_crash) {
    rtc_state.crash_count++;
    rtc_state.crc32 = rtc_crc(&rtc_state);
    if (rtc_state.crash_count > 3) {
      rtc_state.crash_tripped = 1;
      rtc_state.crc32 = rtc_crc(&rtc_state);
      s_control_mode = MEISHA_MODE_OFF;
      s_prefs.putUChar("ctrl_mode", (uint8_t)MEISHA_MODE_OFF);
      return;
    }
  }

  uint8_t stored_mode = s_prefs.getUChar("ctrl_mode", (uint8_t)MEISHA_MODE_SHADOW);
  if (stored_mode <= (uint8_t)MEISHA_MODE_ACTIVE) {
    s_control_mode = (MeishaControlMode)stored_mode;
  } else {
    s_control_mode = MEISHA_MODE_SHADOW;
  }
}

void meisha_loop(void) {
  // Clear crash counter after 5 minutes of continuous stable uptime (300,000 ms)
  if (millis() >= 300000UL && rtc_state.crash_count > 0) {
    rtc_state.crash_count = 0;
    rtc_state.crc32 = rtc_crc(&rtc_state);
  }

  // If Off, skip loop execution
  if (s_control_mode == MEISHA_MODE_OFF) {
    return;
  }

  // Gate controller execution until first valid 0x10 heat pump frame is decoded
  if ((uint8_t)actData[0] != 0x71) {
    return;
  }

  float outside_temp = getDataValue(actData, 14).toFloat();
  String room_str = getDataValue(actData, 33);
  float room_temp = room_str.length() > 0 ? room_str.toFloat() : s_tunables.room_target;
  if (room_temp <= -40.0f || room_temp == 0.0f) {
    room_temp = s_tunables.room_target;
  }

  // Live seeding on cold start
  if (rtc_state.seeded == 0) {
    fill_ring_buffer(rtc_state.ambient_1h_buf, MEISHA_AMBIENT_1H_SAMPLES,
                     &rtc_state.ambient_1h_head, &rtc_state.ambient_1h_count, outside_temp);
    fill_ring_buffer(rtc_state.ambient_24h_buf, MEISHA_AMBIENT_24H_SAMPLES,
                     &rtc_state.ambient_24h_head, &rtc_state.ambient_24h_count, outside_temp);
    fill_ring_buffer(rtc_state.room_6h_buf, MEISHA_ROOM_6H_SAMPLES,
                     &rtc_state.room_6h_head, &rtc_state.room_6h_count, room_temp);

    rtc_state.ambient_1h_mean = outside_temp;
    rtc_state.ambient_24h_mean = outside_temp;
    rtc_state.room_temp_6h_mean = room_temp;
    rtc_state.room_temp_snapshot = roundf(room_temp * 10.0f) / 10.0f;
    rtc_state.ambient_24h_sub_ticks = 0;
    rtc_state.seeded = 1;
    rtc_state.crc32 = rtc_crc(&rtc_state);

    publish_rolling_average_states();
  }

  // Periodic sampling: once every 60 seconds (60,000 ms)
  if (s_last_sample_ms == 0 || (millis() - s_last_sample_ms) >= 60000UL) {
    s_last_sample_ms = millis();

    push_ring_sample(rtc_state.ambient_1h_buf, MEISHA_AMBIENT_1H_SAMPLES,
                     &rtc_state.ambient_1h_head, &rtc_state.ambient_1h_count, outside_temp);
    push_ring_sample(rtc_state.room_6h_buf, MEISHA_ROOM_6H_SAMPLES,
                     &rtc_state.room_6h_head, &rtc_state.room_6h_count, room_temp);

    rtc_state.ambient_24h_sub_ticks++;
    if (rtc_state.ambient_24h_sub_ticks >= 5) {
      push_ring_sample(rtc_state.ambient_24h_buf, MEISHA_AMBIENT_24H_SAMPLES,
                       &rtc_state.ambient_24h_head, &rtc_state.ambient_24h_count, outside_temp);
      rtc_state.ambient_24h_sub_ticks = 0;
    }

    rtc_state.ambient_1h_mean = calc_ring_mean(rtc_state.ambient_1h_buf, rtc_state.ambient_1h_count);
    rtc_state.ambient_24h_mean = calc_ring_mean(rtc_state.ambient_24h_buf, rtc_state.ambient_24h_count);
    rtc_state.room_temp_6h_mean = calc_ring_mean(rtc_state.room_6h_buf, rtc_state.room_6h_count);
    rtc_state.crc32 = rtc_crc(&rtc_state);

    publish_rolling_average_states();
  }

  // Daily Heating Day reset and NVS checkpoint
  time_t now = time(NULL);
  struct tm* local_tm = localtime(&now);
  if (local_tm) {
    int32_t current_day_id = get_heating_day_id(now, s_tunables.heating_start_hour);
    if (local_tm->tm_hour == s_tunables.heating_start_hour && local_tm->tm_min == 0) {
      if (rtc_state.last_reset_heating_day_id != current_day_id) {
        rtc_state.last_reset_heating_day_id = current_day_id;
        rtc_state.room_temp_snapshot = roundf(rtc_state.room_temp_6h_mean * 10.0f) / 10.0f;
        rtc_state.degree_hours_day = 0.0f;
        rtc_state.dhw_time = 0.0f;
        rtc_state.crc32 = rtc_crc(&rtc_state);

        write_nvs_checkpoint();
        publish_sensor_float("room_temp_snapshot", rtc_state.room_temp_snapshot, 1);
      }
    }
  }

  // 2-hour periodic NVS checkpoint while heat pump is running (TOP0 == 1)
  int hp_state = getDataValue(actData, 0).toInt();
  if (hp_state == 1) {
    if (s_last_checkpoint_ms == 0) {
      s_last_checkpoint_ms = millis();
    } else if ((millis() - s_last_checkpoint_ms) >= 7200000UL) {
      s_last_checkpoint_ms = millis();
      write_nvs_checkpoint();
    }
  } else {
    s_last_checkpoint_ms = millis();
  }
}

void meisha_mqtt_connected(void) {
  char sub_topic[160];
  snprintf(sub_topic, sizeof(sub_topic), "%s/meisha/set/#", heishamonSettings.mqtt_topic_base);
  mqtt_client.subscribe(sub_topic);

  publish_ha_discovery();
  publish_mode_state();
  publish_all_tunables_state();
  publish_rolling_average_states();
}

bool meisha_mqtt_callback(const char* topic, const char* topic_command, const char* msg, unsigned int length) {
  (void)topic;
  (void)length;
  if (!topic_command || !msg) return false;

  const char* prefix = "meisha/set/";
  size_t prefix_len = strlen(prefix);
  if (strncmp(topic_command, prefix, prefix_len) != 0) {
    return false;
  }

  const char* subtopic = topic_command + prefix_len;
  if (strcmp(subtopic, "control_mode") == 0) {
    MeishaControlMode new_mode;
    if (parse_mode_string(msg, &new_mode)) {
      if (new_mode != MEISHA_MODE_OFF && (rtc_state.crash_tripped || rtc_state.crash_count > 0)) {
        rtc_state.crash_tripped = 0;
        rtc_state.crash_count = 0;
        rtc_state.crc32 = rtc_crc(&rtc_state);
      }
      meisha_set_control_mode(new_mode);
      return true;
    }
  } else if (strcmp(subtopic, "room_target") == 0) {
    char* endptr = nullptr;
    float val = strtof(msg, &endptr);
    if (endptr != msg) {
      return meisha_set_room_target(val);
    }
  } else if (strcmp(subtopic, "heating_curve_start") == 0) {
    char* endptr = nullptr;
    float val = strtof(msg, &endptr);
    if (endptr != msg) {
      return meisha_set_heating_curve_start(val);
    }
  } else if (strcmp(subtopic, "heating_curve_angle") == 0) {
    char* endptr = nullptr;
    float val = strtof(msg, &endptr);
    if (endptr != msg) {
      return meisha_set_heating_curve_angle(val);
    }
  } else if (strcmp(subtopic, "heating_start_hour") == 0) {
    char* endptr = nullptr;
    long val = strtol(msg, &endptr, 10);
    if (endptr != msg && val >= 0 && val <= 23) {
      return meisha_set_heating_start_hour((uint8_t)val);
    }
  } else if (strcmp(subtopic, "cooling_mode") == 0) {
    if (strcasecmp(msg, "on") == 0 || strcmp(msg, "1") == 0 || strcasecmp(msg, "true") == 0) {
      return meisha_set_cooling_mode(true);
    } else if (strcasecmp(msg, "off") == 0 || strcmp(msg, "0") == 0 || strcasecmp(msg, "false") == 0) {
      return meisha_set_cooling_mode(false);
    }
  }

  return false;
}
