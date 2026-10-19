#include "meisha.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>

#ifdef ARDUINO
#include <Arduino.h>
#include <Preferences.h>
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
#endif

#ifdef ESP32
RTC_NOINIT_ATTR static MeishaRtcState rtc_state;
#else
static MeishaRtcState rtc_state;
#endif

static MeishaControlMode s_control_mode = MEISHA_MODE_SHADOW;
static Preferences s_prefs;
static bool s_prefs_opened = false;

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

static void publish_mode_state(void) {
  char topic[160];
  snprintf(topic, sizeof(topic), "%s/meisha/state/control_mode", heishamonSettings.mqtt_topic_base);
  mqtt_client.publish(topic, mode_to_string(s_control_mode), true);
}

static void publish_ha_discovery(void) {
  char disc_topic[160];
  snprintf(disc_topic, sizeof(disc_topic), "homeassistant/select/%scontrol_mode/config", MEISHA_ENTITY_PREFIX);

  char payload[1024];
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
}

MeishaControlMode meisha_get_control_mode(void) {
  return s_control_mode;
}

const MeishaRtcState* meisha_get_rtc_state(void) {
  return &rtc_state;
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

  bool rtc_valid = (rtc_state.magic == MEISHA_RTC_MAGIC) && (rtc_crc(&rtc_state) == rtc_state.crc32);
  if (!rtc_valid) {
    memset(&rtc_state, 0, sizeof(rtc_state));
    rtc_state.magic = MEISHA_RTC_MAGIC;
    rtc_state.version = 1;
    rtc_state.crash_count = 0;
    rtc_state.crash_tripped = 0;
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

  // Future ticket control logic runs here
}

void meisha_mqtt_connected(void) {
  char sub_topic[160];
  snprintf(sub_topic, sizeof(sub_topic), "%s/meisha/set/#", heishamonSettings.mqtt_topic_base);
  mqtt_client.subscribe(sub_topic);

  publish_ha_discovery();
  publish_mode_state();
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
  }

  return false;
}
