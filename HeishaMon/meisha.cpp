#include "meisha.h"
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef ARDUINO
#include <Arduino.h>
#if defined(ESP32)
#include <Preferences.h>
#elif defined(ESP8266)
// Lightweight dummy Preferences stub for ESP8266 compile target in CI
class Preferences {
public:
  bool begin(const char *, bool = false) { return true; }
  void end() {}
  uint8_t getUChar(const char *, uint8_t def = 0) { return def; }
  size_t putUChar(const char *, uint8_t) { return 1; }
  float getFloat(const char *, float def = 0.0f) { return def; }
  size_t putFloat(const char *, float) { return sizeof(float); }
  bool getBool(const char *, bool def = false) { return def; }
  size_t putBool(const char *, bool) { return 1; }
  size_t getBytes(const char *, void *, size_t) { return 0; }
  size_t putBytes(const char *, const void *, size_t len) { return len; }
};
#endif
#include "commands.h"
#include "decode.h"
#include "webfunctions.h"
#include <PubSubClient.h>

extern char actData[DATASIZE];
extern settingsStruct heishamonSettings;
extern PubSubClient mqtt_client;
extern bool send_command(byte *command, int length);
extern void log_message(char *string);
#else
#include "../tests/mock_heishamon.h"
#define time mock_time
#define localtime mock_localtime
#endif
#include "meisha_discovery.h"

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
static uint32_t s_last_integral_ms = 0;
static uint32_t s_last_pid_ms = 0;
static bool s_sampled_once = false;
static bool s_checkpoint_started = false;
static bool s_integral_started = false;
static bool s_pid_started = false;

static inline float round1(float val) { return roundf(val * 10.0f) / 10.0f; }

static inline float round2(float val) { return roundf(val * 100.0f) / 100.0f; }

static inline float clampf(float val, float min_val, float max_val) {
  return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
}

enum MeishaValveState : uint8_t {
  MEISHA_VALVE_ROOM = 0,
  MEISHA_VALVE_TANK = 1
};

static MeishaValveState parse_valve_state(const String &val_str) {
  const char *s = val_str.c_str();
  if (!s || s[0] == '\0')
    return MEISHA_VALVE_ROOM;
  if (strcasecmp(s, "1") == 0 || strcasecmp(s, "tank") == 0 ||
      strcasecmp(s, "dhw") == 0) {
    return MEISHA_VALVE_TANK;
  }
  return MEISHA_VALVE_ROOM;
}

enum MeishaOpMode {
  MEISHA_OPMODE_HEAT_ONLY = 0,
  MEISHA_OPMODE_COOL_ONLY = 1,
  MEISHA_OPMODE_AUTO_HEAT = 2,
  MEISHA_OPMODE_DHW_ONLY = 3,
  MEISHA_OPMODE_HEAT_DHW = 4,
  MEISHA_OPMODE_COOL_DHW = 5,
  MEISHA_OPMODE_AUTO_DHW = 6,
  MEISHA_OPMODE_UNKNOWN = -1
};

static MeishaOpMode parse_op_mode(const String &val) {
  const char *s = val.c_str();
  if (!s || s[0] == '\0')
    return MEISHA_OPMODE_UNKNOWN;
  if (strcmp(s, "0") == 0 || strcasecmp(s, "Heat") == 0 ||
      strcasecmp(s, "Heat only") == 0) {
    return MEISHA_OPMODE_HEAT_ONLY;
  }
  if (strcmp(s, "1") == 0 || strcasecmp(s, "Cool") == 0 ||
      strcasecmp(s, "Cool only") == 0) {
    return MEISHA_OPMODE_COOL_ONLY;
  }
  if (strcmp(s, "3") == 0 || strcasecmp(s, "DHW") == 0 ||
      strcasecmp(s, "DHW only") == 0) {
    return MEISHA_OPMODE_DHW_ONLY;
  }
  if (strcmp(s, "2") == 0)
    return MEISHA_OPMODE_AUTO_HEAT;
  if (strcmp(s, "4") == 0)
    return MEISHA_OPMODE_HEAT_DHW;
  if (strcmp(s, "5") == 0)
    return MEISHA_OPMODE_COOL_DHW;
  if (strcmp(s, "6") == 0)
    return MEISHA_OPMODE_AUTO_DHW;
  return MEISHA_OPMODE_UNKNOWN;
}

static bool parse_hp_on(const String &val) {
  const char *s = val.c_str();
  if (!s || s[0] == '\0')
    return false;
  return (strcmp(s, "1") == 0 || strcasecmp(s, "on") == 0 ||
          strcasecmp(s, "true") == 0);
}

static int parse_quiet_mode_level(const String &val) {
  const char *s = val.c_str();
  if (!s || s[0] == '\0')
    return 0;
  if (strcasecmp(s, "off") == 0)
    return 0;
  return val.toInt();
}

static int parse_defrosting_state(const String &val) {
  const char *s = val.c_str();
  if (!s || s[0] == '\0')
    return 0;
  if (strcmp(s, "1") == 0 || strcasecmp(s, "on") == 0 ||
      strcasecmp(s, "true") == 0) {
    return 1;
  }
  return 0;
}

static uint32_t calculate_crc32(const uint8_t *data, size_t length) {
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

static uint32_t rtc_crc(const MeishaRtcState *state) {
  return calculate_crc32((const uint8_t *)state,
                         offsetof(MeishaRtcState, crc32));
}

static const char *mode_to_string(MeishaControlMode mode) {
  switch (mode) {
  case MEISHA_MODE_OFF:
    return "Off";
  case MEISHA_MODE_SHADOW:
    return "Shadow";
  case MEISHA_MODE_ACTIVE:
    return "Active";
  default:
    return "Shadow";
  }
}

static bool parse_mode_string(const char *str, MeishaControlMode *out_mode) {
  if (!str)
    return false;
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
  // Shifting backward by start_hour puts the entire heating day into the same
  // calendar day
  time_t shifted = epoch - (time_t)start_hour * 3600;
  struct tm *tm_val = localtime(&shifted);
  if (!tm_val)
    return 0;
  return (tm_val->tm_year + 1900) * 10000 + (tm_val->tm_mon + 1) * 100 +
         tm_val->tm_mday;
}

static void publish_mode_state(void) {
  char topic[256];
  snprintf(topic, sizeof(topic), "%s/meisha/state/control_mode",
           heishamonSettings.mqtt_topic_base);
  mqtt_client.publish(topic, mode_to_string(s_control_mode), true);
}

static void publish_tunable_state(const char *name, const char *val) {
  char topic[256];
  snprintf(topic, sizeof(topic), "%s/meisha/state/%s",
           heishamonSettings.mqtt_topic_base, name);
  mqtt_client.publish(topic, val, true);
}

static void publish_sensor_float(const char *name, float val, int decimals) {
  char topic[256];
  snprintf(topic, sizeof(topic), "%s/meisha/state/%s",
           heishamonSettings.mqtt_topic_base, name);
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

static void publish_sensor_int(const char *name, int val) {
  char topic[256];
  snprintf(topic, sizeof(topic), "%s/meisha/state/%s",
           heishamonSettings.mqtt_topic_base, name);
  char buf[32];
  snprintf(buf, sizeof(buf), "%d", val);
  mqtt_client.publish(topic, buf, true);
}

static bool is_change_lock_active(uint32_t now_ms) {
  if (!rtc_state.change_lock_active)
    return false;
  if ((now_ms - rtc_state.change_lock_set_ms) <
      rtc_state.change_lock_duration_ms) {
    return true;
  }
  rtc_state.change_lock_active = 0;
  return false;
}

static void set_change_lock(uint32_t now_ms, uint32_t duration_ms) {
  rtc_state.change_lock_active = 1;
  rtc_state.change_lock_set_ms = now_ms;
  rtc_state.change_lock_duration_ms = duration_ms;
  rtc_state.change_lock_remaining_min = round1((float)duration_ms / 60000.0f);
}

static void update_change_lock_remaining(uint32_t now_ms) {
  if (is_change_lock_active(now_ms)) {
    uint32_t elapsed = now_ms - rtc_state.change_lock_set_ms;
    uint32_t remaining = (elapsed < rtc_state.change_lock_duration_ms)
                             ? (rtc_state.change_lock_duration_ms - elapsed)
                             : 0;
    rtc_state.change_lock_remaining_min = round1((float)remaining / 60000.0f);
  } else {
    rtc_state.change_lock_remaining_min = 0.0f;
  }
}

static void set_last_action_reason(const char *reason) {
  if (reason) {
    strncpy(rtc_state.last_action_reason, reason,
            sizeof(rtc_state.last_action_reason) - 1);
    rtc_state.last_action_reason[sizeof(rtc_state.last_action_reason) - 1] =
        '\0';
    publish_tunable_state("last_action_reason", rtc_state.last_action_reason);
  }
}

static void publish_derived_states(void) {
  publish_sensor_float("curve_target", rtc_state.curve_target, 2);
  publish_sensor_float("pid_output", rtc_state.pid_output, 4);
  publish_sensor_int("power_target", rtc_state.power_target);
  publish_sensor_float("room_delta", rtc_state.room_delta, 2);
  publish_sensor_float("degree_hours_day", rtc_state.degree_hours_day, 2);
  publish_sensor_float("degree_hours_target", rtc_state.degree_hours_target, 2);
  publish_sensor_float("dhw_time", rtc_state.dhw_time, 2);
  publish_sensor_float("hours_target", rtc_state.hours_target, 2);
  publish_sensor_float("hours_time_left", rtc_state.hours_time_left, 2);
  publish_sensor_float("pipe_ambient_delta", rtc_state.pipe_ambient_delta, 1);
  publish_tunable_state("compressor_running",
                        rtc_state.compressor_running ? "ON" : "OFF");
  publish_sensor_float("change_lock_remaining_min",
                       rtc_state.change_lock_remaining_min, 1);
  publish_tunable_state("last_action_reason", rtc_state.last_action_reason[0]
                                                  ? rtc_state.last_action_reason
                                                  : "idle");
}

static void publish_ha_discovery(void) {
  meisha_publish_discovery_table(mqtt_client, heishamonSettings.mqtt_topic_base,
                                 mqtt_willtopic);
}

static void fill_ring_buffer(int16_t *buf, uint16_t capacity, uint16_t *head,
                             uint16_t *count, float val) {
  int16_t deci = (int16_t)roundf(val * 10.0f);
  for (uint16_t i = 0; i < capacity; i++) {
    buf[i] = deci;
  }
  *head = 0;
  *count = capacity;
}

static void push_ring_sample(int16_t *buf, uint16_t capacity, uint16_t *head,
                             uint16_t *count, float val) {
  int16_t deci = (int16_t)roundf(val * 10.0f);
  buf[*head] = deci;
  *head = (*head + 1) % capacity;
  if (*count < capacity) {
    (*count)++;
  }
}

static float calc_ring_mean(const int16_t *buf, uint16_t count) {
  if (count == 0)
    return 0.0f;
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
  time_t now = time(NULL);
  chkpt.saved_at_epoch = (now > 1000000) ? (uint32_t)now : 0;

  s_prefs.putBytes("chkpt", &chkpt, sizeof(chkpt));
}

MeishaControlMode meisha_get_control_mode(void) { return s_control_mode; }

const MeishaRtcState *meisha_get_rtc_state(void) { return &rtc_state; }

MeishaTunables meisha_get_tunables(void) { return s_tunables; }

float meisha_get_ambient_1h(void) { return rtc_state.ambient_1h_mean; }

float meisha_get_ambient_24h(void) { return rtc_state.ambient_24h_mean; }

float meisha_get_room_temp_6h(void) { return rtc_state.room_temp_6h_mean; }

float meisha_get_room_temp_snapshot(void) {
  return rtc_state.room_temp_snapshot;
}

bool meisha_set_room_target(float val) {
  if (val < 18.0f || val > 25.0f)
    return false;
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
  if (val < 12.0f || val > 20.0f)
    return false;
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
  if (val < 0.2f || val > 1.5f)
    return false;
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
  if (val > 23)
    return false;
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

    if (val) {
      // Switched to cooling mode: clear in-flight winter sequences and timers
      rtc_state.day_mode_pending = 0;
      rtc_state.turn_off_pending = 0;
      rtc_state.low_setpoint_timer_active = 0;
      rtc_state.idle_shutoff_fired = 0;
      rtc_state.night_mode_sent = 0;
      rtc_state.overshoot_timer_active = 0;
      rtc_state.overshoot_steps_fired = 0;
    } else {
      // Switched to heating mode: clear in-flight cooling sequences and timers
      rtc_state.cool_start_pending = 0;
      rtc_state.cool_low_ambient_timer_active = 0;
      rtc_state.cool_low_ambient_fired = 0;
    }
    rtc_state.crc32 = rtc_crc(&rtc_state);
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

bool meisha_send_command(const char *cmd, const char *value) {
  if (s_control_mode != MEISHA_MODE_ACTIVE) {
    return false;
  }
#if defined(ARDUINO) || defined(MOCK_HEISHAMON)
  send_heatpump_command((char *)cmd, (char *)value, send_command, log_message,
                        heishamonSettings.optionalPCB);
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

  bool rtc_valid = (rtc_state.magic == MEISHA_RTC_MAGIC) &&
                   (rtc_crc(&rtc_state) == rtc_state.crc32);

  if (!rtc_valid) {
    // SRAM lost across cold power loss or first boot
    memset(&rtc_state, 0, sizeof(rtc_state));
    rtc_state.magic = MEISHA_RTC_MAGIC;
    rtc_state.version = 2;

    MeishaNvsCheckpoint chkpt;
    bool has_chkpt =
        (s_prefs.getBytes("chkpt", &chkpt, sizeof(chkpt)) == sizeof(chkpt));
    time_t now = time(NULL);

    if (has_chkpt && chkpt.saved_at_epoch > 0 && now <= 1000000) {
      // Real-hardware cold boot before SNTP syncs: pre-seed ring buffers from checkpoint
      // and defer the < 6h decision to meisha_loop() once SNTP syncs.
      fill_ring_buffer(rtc_state.ambient_1h_buf, MEISHA_AMBIENT_1H_SAMPLES,
                       &rtc_state.ambient_1h_head, &rtc_state.ambient_1h_count,
                       chkpt.ambient_1h_mean);
      fill_ring_buffer(rtc_state.ambient_24h_buf, MEISHA_AMBIENT_24H_SAMPLES,
                       &rtc_state.ambient_24h_head,
                       &rtc_state.ambient_24h_count, chkpt.ambient_24h_mean);
      fill_ring_buffer(rtc_state.room_6h_buf, MEISHA_ROOM_6H_SAMPLES,
                       &rtc_state.room_6h_head, &rtc_state.room_6h_count,
                       chkpt.room_temp_6h_mean);

      rtc_state.ambient_1h_mean = chkpt.ambient_1h_mean;
      rtc_state.ambient_24h_mean = chkpt.ambient_24h_mean;
      rtc_state.room_temp_6h_mean = chkpt.room_temp_6h_mean;
      rtc_state.room_temp_snapshot = chkpt.room_temp_snapshot;
      rtc_state.pid_output = chkpt.pid_output;
      rtc_state.nvs_decision_pending = 1;
      rtc_state.seeded = 0;
    } else if (has_chkpt && now > 1000000 && chkpt.saved_at_epoch > 0 &&
               now >= chkpt.saved_at_epoch &&
               (now - chkpt.saved_at_epoch) < 6 * 3600) {
      // Warm recovery (< 6h old checkpoint)
      fill_ring_buffer(rtc_state.ambient_1h_buf, MEISHA_AMBIENT_1H_SAMPLES,
                       &rtc_state.ambient_1h_head, &rtc_state.ambient_1h_count,
                       chkpt.ambient_1h_mean);
      fill_ring_buffer(rtc_state.ambient_24h_buf, MEISHA_AMBIENT_24H_SAMPLES,
                       &rtc_state.ambient_24h_head,
                       &rtc_state.ambient_24h_count, chkpt.ambient_24h_mean);
      fill_ring_buffer(rtc_state.room_6h_buf, MEISHA_ROOM_6H_SAMPLES,
                       &rtc_state.room_6h_head, &rtc_state.room_6h_count,
                       chkpt.room_temp_6h_mean);

      rtc_state.ambient_1h_mean = chkpt.ambient_1h_mean;
      rtc_state.ambient_24h_mean = chkpt.ambient_24h_mean;
      rtc_state.room_temp_6h_mean = chkpt.room_temp_6h_mean;
      rtc_state.pid_output = chkpt.pid_output;

      int32_t saved_day = get_heating_day_id((time_t)chkpt.saved_at_epoch,
                                             s_tunables.heating_start_hour);
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
    strcpy(rtc_state.last_action_reason, "idle");
    rtc_state.change_lock_remaining_min = 0.0f;
    rtc_state.crc32 = rtc_crc(&rtc_state);
  } else {
    if (rtc_state.last_action_reason[0] == '\0') {
      strcpy(rtc_state.last_action_reason, "idle");
    }
    if (rtc_state.dhw_start_pending && millis() < rtc_state.dhw_start_ms) {
      rtc_state.dhw_start_pending = 0;
    }
    if (rtc_state.dhw_end_cool_pending &&
        millis() < rtc_state.dhw_end_cool_start_ms) {
      rtc_state.dhw_end_cool_pending = 0;
    }
    if (rtc_state.cool_start_pending && millis() < rtc_state.cool_start_ms) {
      rtc_state.cool_start_pending = 0;
    }
  }

#if defined(ESP32) || defined(MOCK_HEISHAMON)
  esp_reset_reason_t reason = esp_reset_reason();
  bool is_crash = (reason == ESP_RST_PANIC || reason == ESP_RST_INT_WDT ||
                   reason == ESP_RST_TASK_WDT || reason == ESP_RST_WDT);
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

  s_sampled_once = false;
  s_checkpoint_started = false;
  s_integral_started = false;
  s_pid_started = false;
  s_last_sample_ms = 0;
  s_last_checkpoint_ms = 0;
  s_last_integral_ms = 0;
  s_last_pid_ms = 0;

  uint8_t stored_mode =
      s_prefs.getUChar("ctrl_mode", (uint8_t)MEISHA_MODE_SHADOW);
  if (stored_mode <= (uint8_t)MEISHA_MODE_ACTIVE) {
    s_control_mode = (MeishaControlMode)stored_mode;
  } else {
    s_control_mode = MEISHA_MODE_SHADOW;
  }
}

void meisha_loop(void) {
  // Clear crash counter after 5 minutes of continuous stable uptime (300,000
  // ms)
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
  float outside_pipe_temp = getDataValue(actData, 21).toFloat();
  rtc_state.pipe_ambient_delta = round1(outside_temp - outside_pipe_temp);

  String room_str = getDataValue(actData, 33);
  float room_temp =
      room_str.length() > 0 ? room_str.toFloat() : s_tunables.room_target;
  if (room_temp <= -40.0f || room_temp == 0.0f) {
    room_temp = s_tunables.room_target;
  }

  bool hp_on = parse_hp_on(getDataValue(actData, 0));
  float pump_flow = getDataValue(actData, 1).toFloat();
  float main_outlet_temp = getDataValue(actData, 6).toFloat();
  float main_target_temp = getDataValue(actData, 7).toFloat();
  float compressor_freq = getDataValue(actData, 8).toFloat();
  String hp_setpoint_str = getDataValue(actData, 27);
  float hp_setpoint = hp_setpoint_str.toFloat();

  String z1_str = getDataValue(actData, 56);
  if (z1_str.length() == 0) {
    z1_str = getDataValue(actData, 34);
  }
  float z1_temp = z1_str.toFloat();

  MeishaValveState valve_state = parse_valve_state(getDataValue(actData, 20));
  MeishaOpMode op_mode = parse_op_mode(getDataValue(actData, 4));
  String dhw_str = getDataValue(actData, 10);
  float dhw_temp = dhw_str.length() > 0 ? dhw_str.toFloat() : 50.0f;
  int quiet_mode_level = parse_quiet_mode_level(getDataValue(actData, 18));
  int defrosting_state = parse_defrosting_state(getDataValue(actData, 26));

  int main_outlet_temp_int = (int)main_outlet_temp;
  int main_target_temp_int = (int)main_target_temp;
  int hp_setpoint_int = (int)hp_setpoint;
  int z1_temp_int = (int)z1_temp;

  // Compressor running hysteresis (10 Hz upper threshold, 2 Hz hysteresis: >12
  // Hz on, <8 Hz off)
  if (compressor_freq > 12.0f) {
    rtc_state.compressor_running = 1;
  } else if (compressor_freq < 8.0f) {
    rtc_state.compressor_running = 0;
  }

  uint32_t now_ms = millis();

  // If waiting for SNTP to decide on NVS checkpoint (< 6h vs >= 6h)
  if (rtc_state.nvs_decision_pending) {
    time_t now_epoch = time(NULL);
    if (now_epoch > 1000000) {
      MeishaNvsCheckpoint chkpt;
      if (s_prefs.getBytes("chkpt", &chkpt, sizeof(chkpt)) == sizeof(chkpt)) {
        if (chkpt.saved_at_epoch > 0 && now_epoch >= chkpt.saved_at_epoch &&
            (now_epoch - chkpt.saved_at_epoch) < 6 * 3600) {
          int32_t saved_day = get_heating_day_id((time_t)chkpt.saved_at_epoch,
                                                 s_tunables.heating_start_hour);
          int32_t now_day =
              get_heating_day_id(now_epoch, s_tunables.heating_start_hour);
          if (saved_day == now_day) {
            rtc_state.degree_hours_day = chkpt.degree_hours_day;
            rtc_state.dhw_time = chkpt.dhw_time;
            rtc_state.room_temp_snapshot = chkpt.room_temp_snapshot;
            rtc_state.last_reset_heating_day_id = saved_day;
          }
          rtc_state.seeded = 1;
        }
      }
      rtc_state.nvs_decision_pending = 0;
      rtc_state.crc32 = rtc_crc(&rtc_state);
    } else if (now_ms >= 60000UL) {
      // 60s without NTP sync: proceed with cold start live seeding
      rtc_state.nvs_decision_pending = 0;
      rtc_state.crc32 = rtc_crc(&rtc_state);
    }
  }

  // Live seeding on cold start (only if not waiting for SNTP NVS decision)
  if (rtc_state.seeded == 0 && !rtc_state.nvs_decision_pending) {
    fill_ring_buffer(rtc_state.ambient_1h_buf, MEISHA_AMBIENT_1H_SAMPLES,
                     &rtc_state.ambient_1h_head, &rtc_state.ambient_1h_count,
                     outside_temp);
    fill_ring_buffer(rtc_state.ambient_24h_buf, MEISHA_AMBIENT_24H_SAMPLES,
                     &rtc_state.ambient_24h_head, &rtc_state.ambient_24h_count,
                     outside_temp);
    fill_ring_buffer(rtc_state.room_6h_buf, MEISHA_ROOM_6H_SAMPLES,
                     &rtc_state.room_6h_head, &rtc_state.room_6h_count,
                     room_temp);

    rtc_state.ambient_1h_mean = outside_temp;
    rtc_state.ambient_24h_mean = outside_temp;
    rtc_state.room_temp_6h_mean = room_temp;
    rtc_state.room_temp_snapshot = round1(room_temp);
    rtc_state.ambient_24h_sub_ticks = 0;
    rtc_state.seeded = 1;

    // Initial curve target & power target
    float curve_ambient =
        round2(s_tunables.room_target +
               (s_tunables.heating_curve_start - rtc_state.ambient_24h_mean) *
                   s_tunables.heating_curve_angle);
    float room_offset = clampf(
        s_tunables.room_target - rtc_state.room_temp_snapshot, -2.0f, 2.0f);
    rtc_state.curve_target =
        round2(curve_ambient + room_offset * s_tunables.heating_curve_angle);
    rtc_state.degree_hours_target =
        24.0f * (rtc_state.curve_target - s_tunables.room_target);

    float raw_power =
        fmaxf(27.0f, rtc_state.curve_target - rtc_state.pid_output);
    rtc_state.power_target = (int16_t)roundf(raw_power);

    s_sampled_once = true;
    s_last_sample_ms = now_ms;
    s_integral_started = true;
    s_last_integral_ms = now_ms;
    s_pid_started = true;
    s_last_pid_ms = now_ms;

    rtc_state.crc32 = rtc_crc(&rtc_state);

    publish_rolling_average_states();
    publish_derived_states();
  }

  // Periodic sampling: once every 60 seconds (60,000 ms)
  if (!s_sampled_once || (now_ms - s_last_sample_ms) >= 60000UL) {
    s_sampled_once = true;
    s_last_sample_ms = now_ms;

    push_ring_sample(rtc_state.ambient_1h_buf, MEISHA_AMBIENT_1H_SAMPLES,
                     &rtc_state.ambient_1h_head, &rtc_state.ambient_1h_count,
                     outside_temp);
    push_ring_sample(rtc_state.room_6h_buf, MEISHA_ROOM_6H_SAMPLES,
                     &rtc_state.room_6h_head, &rtc_state.room_6h_count,
                     room_temp);

    rtc_state.ambient_24h_sub_ticks++;
    if (rtc_state.ambient_24h_sub_ticks >= 5) {
      push_ring_sample(rtc_state.ambient_24h_buf, MEISHA_AMBIENT_24H_SAMPLES,
                       &rtc_state.ambient_24h_head,
                       &rtc_state.ambient_24h_count, outside_temp);
      rtc_state.ambient_24h_sub_ticks = 0;
    }

    rtc_state.ambient_1h_mean =
        calc_ring_mean(rtc_state.ambient_1h_buf, rtc_state.ambient_1h_count);
    rtc_state.ambient_24h_mean =
        calc_ring_mean(rtc_state.ambient_24h_buf, rtc_state.ambient_24h_count);
    rtc_state.room_temp_6h_mean =
        calc_ring_mean(rtc_state.room_6h_buf, rtc_state.room_6h_count);
    rtc_state.crc32 = rtc_crc(&rtc_state);

    publish_rolling_average_states();
    publish_derived_states();
  }

  // Continuous curve target & degree-hour budget calculation
  float curve_ambient =
      round2(s_tunables.room_target +
             (s_tunables.heating_curve_start - rtc_state.ambient_24h_mean) *
                 s_tunables.heating_curve_angle);
  float room_offset = clampf(
      s_tunables.room_target - rtc_state.room_temp_snapshot, -2.0f, 2.0f);
  rtc_state.curve_target =
      round2(curve_ambient + room_offset * s_tunables.heating_curve_angle);
  rtc_state.degree_hours_target =
      24.0f * (rtc_state.curve_target - s_tunables.room_target);

  // Continuous integration of room_delta into degree_hours_day and dhw_time
  rtc_state.room_delta = (pump_flow > 0.0f && valve_state == MEISHA_VALVE_ROOM)
                             ? (main_outlet_temp - s_tunables.room_target)
                             : 0.0f;
  if (!s_integral_started) {
    s_integral_started = true;
    s_last_integral_ms = now_ms;
  } else {
    uint32_t dt_int_ms = now_ms - s_last_integral_ms;
    if (dt_int_ms > 0) {
      s_last_integral_ms = now_ms;
      float dt_hours = (float)dt_int_ms / 3600000.0f;
      rtc_state.degree_hours_day += rtc_state.room_delta * dt_hours;
      if (valve_state == MEISHA_VALVE_TANK) {
        rtc_state.dhw_time += dt_hours;
      }
    }
  }

  // 10-second I-only PID controller with elapsed time
  if (!s_pid_started) {
    s_pid_started = true;
    s_last_pid_ms = now_ms;
  } else {
    uint32_t dt_pid_ms = now_ms - s_last_pid_ms;
    if (dt_pid_ms >= 10000UL) {
      s_last_pid_ms = now_ms;
      float dt_pid_sec = (float)dt_pid_ms / 1000.0f;
      float curve_error = (pump_flow > 0.0f && valve_state == MEISHA_VALVE_ROOM)
                              ? (rtc_state.curve_target - main_outlet_temp)
                              : 0.0f;
      float pid_error = -curve_error;
      rtc_state.pid_output = clampf(
          rtc_state.pid_output + 5e-5f * dt_pid_sec * pid_error, -1.0f, 1.0f);
    }
  }

  // Power Target with ±0.1 K integer hysteresis
  float raw_power = fmaxf(27.0f, rtc_state.curve_target - rtc_state.pid_output);
  int16_t lo = (int16_t)roundf(raw_power - 0.1f);
  int16_t hi = (int16_t)roundf(raw_power + 0.1f);
  if (rtc_state.power_target < 27) {
    rtc_state.power_target = (int16_t)roundf(raw_power);
  } else {
    if (rtc_state.power_target < lo)
      rtc_state.power_target = lo;
    else if (rtc_state.power_target > hi)
      rtc_state.power_target = hi;
  }

  // Remaining schedule hours (hours_target & hours_time_left)
  float denom = 30.0f - s_tunables.room_target;
  rtc_state.hours_target =
      (fabsf(denom) > 1e-4f)
          ? clampf((rtc_state.curve_target - s_tunables.room_target) / denom *
                       24.0f,
                   0.0f, 24.0f)
          : 24.0f;

  bool in_pre_start_blackout = false;
  bool dhw_start_triggered = false;
  time_t now = time(NULL);
  struct tm *tm_ptr = localtime(&now);
  if (now > 1000000 && tm_ptr) {
    struct tm local_tm = *tm_ptr;
    int hours_diff =
        (local_tm.tm_hour - (int)s_tunables.heating_start_hour + 24) % 24;
    float hours_elapsed = (float)hours_diff + (float)local_tm.tm_min / 60.0f +
                          (float)local_tm.tm_sec / 3600.0f;
    rtc_state.hours_time_left =
        fmaxf(0.0f, fminf(24.0f, rtc_state.hours_target + rtc_state.dhw_time) -
                        hours_elapsed);

    int hours_until_start =
        ((int)s_tunables.heating_start_hour - local_tm.tm_hour + 24) % 24;
    in_pre_start_blackout = (hours_until_start == 1 || hours_until_start == 2);

    // Daily 14:00 DHW Schedule Trigger
    int32_t current_cal_day_id = (local_tm.tm_year + 1900) * 10000 +
                                 (local_tm.tm_mon + 1) * 100 + local_tm.tm_mday;
    if (local_tm.tm_hour == 14 && local_tm.tm_min == 0) {
      if (rtc_state.last_dhw_schedule_day_id != current_cal_day_id) {
        rtc_state.last_dhw_schedule_day_id = current_cal_day_id;
        if (valve_state != MEISHA_VALVE_TANK && dhw_temp < 45.0f &&
            !rtc_state.dhw_start_pending && op_mode != MEISHA_OPMODE_DHW_ONLY) {
          dhw_start_triggered = true;
        }
      }
    }

    // Daily 08:00 Cool Turn On Trigger
    if (local_tm.tm_hour == 8 && local_tm.tm_min == 0) {
      if (rtc_state.last_cool_start_day_id != current_cal_day_id) {
        rtc_state.last_cool_start_day_id = current_cal_day_id;
        if (s_tunables.cooling_mode && rtc_state.ambient_24h_mean > 24.0f &&
            op_mode != MEISHA_OPMODE_DHW_ONLY) {
          meisha_send_command("SetOperationMode", "1");
          rtc_state.cool_start_pending = 1;
          rtc_state.cool_start_ms = now_ms;
          set_last_action_reason("cool_turn_on");
          publish_derived_states();
        }
      }
    }

    // Daily 20:00 Cool Turn Off Trigger
    if (local_tm.tm_hour == 20 && local_tm.tm_min == 0) {
      if (rtc_state.last_cool_stop_day_id != current_cal_day_id) {
        rtc_state.last_cool_stop_day_id = current_cal_day_id;
        if (s_tunables.cooling_mode) {
          rtc_state.cool_start_pending = 0;
          if (hp_on && op_mode != MEISHA_OPMODE_DHW_ONLY) {
            meisha_send_command("SetHeatpump", "0");
            set_last_action_reason("cool_turn_off");
            publish_derived_states();
          }
        }
      }
    }

    // Daily Heating Day reset and NVS checkpoint at heating_start_hour:00
    int32_t current_day_id =
        get_heating_day_id(now, s_tunables.heating_start_hour);
    if (local_tm.tm_hour == s_tunables.heating_start_hour &&
        local_tm.tm_min == 0) {
      if (rtc_state.last_reset_heating_day_id != current_day_id) {
        rtc_state.last_reset_heating_day_id = current_day_id;
        rtc_state.room_temp_snapshot = round1(rtc_state.room_temp_6h_mean);
        rtc_state.degree_hours_day = 0.0f;
        rtc_state.dhw_time = 0.0f;

        float new_ambient =
            round2(s_tunables.room_target + (s_tunables.heating_curve_start -
                                             rtc_state.ambient_24h_mean) *
                                                s_tunables.heating_curve_angle);
        float new_offset = clampf(
            s_tunables.room_target - rtc_state.room_temp_snapshot, -2.0f, 2.0f);
        rtc_state.curve_target =
            round2(new_ambient + new_offset * s_tunables.heating_curve_angle);
        rtc_state.degree_hours_target =
            24.0f * (rtc_state.curve_target - s_tunables.room_target);

        // Recompute power target
        float raw_p =
            fmaxf(27.0f, rtc_state.curve_target - rtc_state.pid_output);
        int16_t lo_p = (int16_t)roundf(raw_p - 0.1f);
        int16_t hi_p = (int16_t)roundf(raw_p + 0.1f);
        if (rtc_state.power_target < 27) {
          rtc_state.power_target = (int16_t)roundf(raw_p);
        } else {
          if (rtc_state.power_target < lo_p)
            rtc_state.power_target = lo_p;
          else if (rtc_state.power_target > hi_p)
            rtc_state.power_target = hi_p;
        }

        // Recompute schedule hours
        float d_denom = 30.0f - s_tunables.room_target;
        rtc_state.hours_target =
            (fabsf(d_denom) > 1e-4f)
                ? clampf((rtc_state.curve_target - s_tunables.room_target) /
                             d_denom * 24.0f,
                         0.0f, 24.0f)
                : 24.0f;
        rtc_state.hours_time_left = fmaxf(
            0.0f, fminf(24.0f, rtc_state.hours_target + rtc_state.dhw_time) -
                      hours_elapsed);

        // Schedule non-blocking Day Mode #1 startup after 10 seconds
        if (!s_tunables.cooling_mode) {
          rtc_state.day_mode_pending = 1;
          rtc_state.day_mode_start_ms = now_ms;
        }

        rtc_state.crc32 = rtc_crc(&rtc_state);

        write_nvs_checkpoint();
        publish_sensor_float("room_temp_snapshot", rtc_state.room_temp_snapshot,
                             1);
        publish_derived_states();
      }
    }
  }

  // Low-temperature DHW trigger (dhw_temp < 38.0 C for 1 minute = 60,000 ms)
  if (dhw_temp < 38.0f && valve_state != MEISHA_VALVE_TANK &&
      op_mode != MEISHA_OPMODE_DHW_ONLY) {
    if (!rtc_state.dhw_low_temp_timer_active) {
      rtc_state.dhw_low_temp_timer_active = 1;
      rtc_state.dhw_low_temp_since_ms = now_ms;
      rtc_state.dhw_low_temp_fired = 0;
    } else if (!rtc_state.dhw_low_temp_fired &&
               (now_ms - rtc_state.dhw_low_temp_since_ms) >= 60000UL) {
      rtc_state.dhw_low_temp_fired = 1;
      if (dhw_temp < 45.0f && !rtc_state.dhw_start_pending) {
        dhw_start_triggered = true;
      }
    }
  } else {
    rtc_state.dhw_low_temp_timer_active = 0;
    rtc_state.dhw_low_temp_fired = 0;
  }

  // Update change lock remaining time
  update_change_lock_remaining(now_ms);

  // Day Mode #1 non-blocking execution (10 seconds after daily reset)
  if (rtc_state.day_mode_pending &&
      (now_ms - rtc_state.day_mode_start_ms) >= 10000UL) {
    rtc_state.day_mode_pending = 0;
    if (!s_tunables.cooling_mode && rtc_state.hours_time_left > 1.0f) {
      if (valve_state != MEISHA_VALVE_TANK &&
          op_mode != MEISHA_OPMODE_DHW_ONLY) {
        meisha_send_command("SetOperationMode", "0");
        meisha_send_command("SetHeatpump", "1");
        int target_sp =
            (rtc_state.power_target > 27) ? (int)rtc_state.power_target : 27;
        if (hp_setpoint_int < target_sp) {
          char sp_buf[16];
          snprintf(sp_buf, sizeof(sp_buf), "%d", target_sp);
          meisha_send_command("SetZ1HeatRequestTemperature", sp_buf);
        }
        set_last_action_reason("day_mode_1");
        publish_derived_states();
      }
    }
  }

  // Turn Off non-blocking Step 2 execution (10 seconds after Step 1)
  if (rtc_state.turn_off_pending &&
      (now_ms - rtc_state.turn_off_start_ms) >= 10000UL) {
    rtc_state.turn_off_pending = 0;
    if (!s_tunables.cooling_mode && hp_on && valve_state == MEISHA_VALVE_ROOM &&
        op_mode != MEISHA_OPMODE_DHW_ONLY) {
      meisha_send_command("SetHeatpump", "0");
      set_last_action_reason("idle_shutoff");
      publish_derived_states();
    }
  }

  // DHW Start non-blocking Step 2 execution (10 seconds after turning HP on)
  if (rtc_state.dhw_start_pending &&
      (now_ms - rtc_state.dhw_start_ms) >= 10000UL) {
    rtc_state.dhw_start_pending = 0;
    meisha_send_command("SetOperationMode", "3");
    set_last_action_reason("dhw_start");
    publish_derived_states();
  }

  // DHW End summer non-blocking Step 2 execution (10 seconds after setting Cool
  // only)
  if (rtc_state.dhw_end_cool_pending &&
      (now_ms - rtc_state.dhw_end_cool_start_ms) >= 10000UL) {
    rtc_state.dhw_end_cool_pending = 0;
    if (s_tunables.cooling_mode && rtc_state.ambient_24h_mean < 22.0f) {
      meisha_send_command("SetHeatpump", "0");
      set_last_action_reason("dhw_end");
      publish_derived_states();
    }
  }

  // Cool Turn On non-blocking Step 2 execution (10 seconds after setting Cool
  // only)
  if (rtc_state.cool_start_pending &&
      (now_ms - rtc_state.cool_start_ms) >= 10000UL) {
    rtc_state.cool_start_pending = 0;
    if (s_tunables.cooling_mode) {
      meisha_send_command("SetHeatpump", "1");
      set_last_action_reason("cool_turn_on");
      publish_derived_states();
    }
  }

  // Continuous duration timers for valve == Room and compressor_running == on
  if (valve_state == MEISHA_VALVE_ROOM) {
    if (!rtc_state.valve_room_timer_active) {
      rtc_state.valve_room_timer_active = 1;
      rtc_state.valve_room_since_ms = now_ms;
    }
  } else {
    rtc_state.valve_room_timer_active = 0;
  }

  if (rtc_state.compressor_running != 0) {
    if (!rtc_state.comp_on_timer_active) {
      rtc_state.comp_on_timer_active = 1;
      rtc_state.comp_on_since_ms = now_ms;
    }
  } else {
    rtc_state.comp_on_timer_active = 0;
  }

  bool valve_room_10m = rtc_state.valve_room_timer_active &&
                        ((now_ms - rtc_state.valve_room_since_ms) >= 600000UL);
  bool comp_on_5m = rtc_state.comp_on_timer_active &&
                    ((now_ms - rtc_state.comp_on_since_ms) >= 300000UL);

  // Execute DHW Start
  if (dhw_start_triggered) {
    if (!hp_on) {
      meisha_send_command("SetHeatpump", "1");
    }
    rtc_state.dhw_start_pending = 1;
    rtc_state.dhw_start_ms = now_ms;
    rtc_state.dhw_end_fired = 0;
    rtc_state.dhw_seen_valve_tank = 0;
    rtc_state.dhw_valve_room_timer_active = 0;
    rtc_state.dhw_comp_off_timer_active = 0;
    set_last_action_reason("dhw_start");
    publish_derived_states();
  }

  // DHW End detection (while op_mode == DHW only)
  if (op_mode == MEISHA_OPMODE_DHW_ONLY) {
    if (valve_state == MEISHA_VALVE_TANK) {
      rtc_state.dhw_seen_valve_tank = 1;
      rtc_state.dhw_valve_room_timer_active = 0;
      rtc_state.dhw_end_fired = 0;
    } else if (valve_state == MEISHA_VALVE_ROOM &&
               rtc_state.dhw_seen_valve_tank != 0) {
      if (!rtc_state.dhw_valve_room_timer_active) {
        rtc_state.dhw_valve_room_timer_active = 1;
        rtc_state.dhw_valve_room_since_ms = now_ms;
      }
    } else {
      rtc_state.dhw_valve_room_timer_active = 0;
    }

    if (rtc_state.compressor_running == 0) {
      if (!rtc_state.dhw_comp_off_timer_active) {
        rtc_state.dhw_comp_off_timer_active = 1;
        rtc_state.dhw_comp_off_since_ms = now_ms;
      }
    } else {
      rtc_state.dhw_comp_off_timer_active = 0;
      rtc_state.dhw_end_fired = 0;
    }

    bool valve_room_15m =
        rtc_state.dhw_valve_room_timer_active &&
        ((now_ms - rtc_state.dhw_valve_room_since_ms) >= 900000UL);
    bool comp_off_15m =
        rtc_state.dhw_comp_off_timer_active &&
        ((now_ms - rtc_state.dhw_comp_off_since_ms) >= 900000UL);

    if (!rtc_state.dhw_end_fired && (valve_room_15m || comp_off_15m)) {
      rtc_state.dhw_end_fired = 1;
      rtc_state.dhw_seen_valve_tank = 0;
      rtc_state.dhw_valve_room_timer_active = 0;
      rtc_state.dhw_comp_off_timer_active = 0;

      if (s_tunables.cooling_mode) {
        meisha_send_command("SetOperationMode", "1");
        rtc_state.dhw_end_cool_pending = 1;
        rtc_state.dhw_end_cool_start_ms = now_ms;
        set_last_action_reason("dhw_end");
        publish_derived_states();
      } else {
        meisha_send_command("SetOperationMode", "0");
        if (rtc_state.hours_time_left < 0.1f) {
          meisha_send_command("SetHeatpump", "0");
        }
        set_last_action_reason("dhw_end");
        publish_derived_states();
      }
    }
  } else if (op_mode != MEISHA_OPMODE_DHW_ONLY) {
    rtc_state.dhw_end_fired = 0;
    rtc_state.dhw_seen_valve_tank = (valve_state == MEISHA_VALVE_TANK) ? 1 : 0;
    rtc_state.dhw_valve_room_timer_active = 0;
    rtc_state.dhw_comp_off_timer_active = 0;
  }

  // Space heating control (when cooling_mode == off and not in DHW only)
  if (!s_tunables.cooling_mode && op_mode != MEISHA_OPMODE_DHW_ONLY) {
    // 4-Hour Idle Shutoff (Turn Off) detection
    // Triggers when cooling_mode == off, hp_on, valve == Room, op_mode != DHW
    // only, and heat_pump_setpoint < 25.1 C held continuously for 4 hours
    // (14,400,000 ms).
    bool low_sp_cond = hp_on && (valve_state == MEISHA_VALVE_ROOM) &&
                       (op_mode != MEISHA_OPMODE_DHW_ONLY) &&
                       (hp_setpoint < 25.1f);
    if (low_sp_cond) {
      if (!rtc_state.low_setpoint_timer_active) {
        rtc_state.low_setpoint_timer_active = 1;
        rtc_state.low_setpoint_since_ms = now_ms;
      } else if (!rtc_state.idle_shutoff_fired &&
                 (now_ms - rtc_state.low_setpoint_since_ms) >= 14400000UL) {
        rtc_state.idle_shutoff_fired = 1;
        meisha_send_command("SetOperationMode", "0");
        rtc_state.turn_off_pending = 1;
        rtc_state.turn_off_start_ms = now_ms;
        set_last_action_reason("idle_shutoff");
        publish_derived_states();
      }
    } else {
      rtc_state.low_setpoint_timer_active = 0;
      rtc_state.idle_shutoff_fired = 0;
    }

    // Night Mode budget cutoff
    // Triggers whenever cooling_mode == off, hp_on, degree_hours_day >
    // degree_hours_target, and current hour is not within the 2-hour pre-start
    // blackout window.
    bool night_mode_active =
        hp_on && (rtc_state.degree_hours_day > rtc_state.degree_hours_target) &&
        !in_pre_start_blackout;

    if (night_mode_active) {
      if (hp_setpoint_int != 25 && !rtc_state.night_mode_sent) {
        meisha_send_command("SetZ1HeatRequestTemperature", "25");
        rtc_state.night_mode_sent = 1;
        set_last_action_reason("night_mode");
        publish_derived_states();
      } else if (hp_setpoint_int == 25) {
        rtc_state.night_mode_sent = 0;
      }
    } else {
      rtc_state.night_mode_sent = 0;
    }

    // Flow temperature modulation (Bonusgrad High-Temp, Bonusgrad Curve,
    // Malusgrad) Inhibit modulation when Night Mode is active.
    if (!night_mode_active) {
      // 1. Bonusgrad High-Temp (Anti-Overshoot)
      // Requires valve == Room, compressor_running == on, degree_hours_day <
      // degree_hours_target, and z1_temp >= int(hp_setpoint) + 3 held for 1m
      // (and a second time if still true after 1m, 2m total). Bypasses
      // change_lock!
      bool overshoot_cond =
          (valve_state == MEISHA_VALVE_ROOM) &&
          (rtc_state.compressor_running != 0) &&
          (rtc_state.degree_hours_day < rtc_state.degree_hours_target) &&
          (z1_temp >= (float)(hp_setpoint_int + 3));

      if (overshoot_cond) {
        if (!rtc_state.overshoot_timer_active) {
          rtc_state.overshoot_timer_active = 1;
          rtc_state.overshoot_start_ms = now_ms;
          rtc_state.overshoot_steps_fired = 0;
        } else {
          uint32_t dur = now_ms - rtc_state.overshoot_start_ms;
          if (rtc_state.overshoot_steps_fired == 0 && dur >= 60000UL) {
            int new_sp = hp_setpoint_int + 1;
            char sp_buf[16];
            snprintf(sp_buf, sizeof(sp_buf), "%d", new_sp);
            meisha_send_command("SetZ1HeatRequestTemperature", sp_buf);
            set_change_lock(now_ms, 15 * 60 * 1000UL);
            rtc_state.overshoot_steps_fired = 1;
            set_last_action_reason("bonusgrad_high_temp");
            publish_derived_states();
          } else if (rtc_state.overshoot_steps_fired == 1 && dur >= 120000UL) {
            int new_sp = hp_setpoint_int + 1;
            char sp_buf[16];
            snprintf(sp_buf, sizeof(sp_buf), "%d", new_sp);
            meisha_send_command("SetZ1HeatRequestTemperature", sp_buf);
            set_change_lock(now_ms, 15 * 60 * 1000UL);
            rtc_state.overshoot_steps_fired = 2;
            set_last_action_reason("bonusgrad_high_temp");
            publish_derived_states();
          }
        }
      } else {
        rtc_state.overshoot_timer_active = 0;
        rtc_state.overshoot_steps_fired = 0;
      }

      // 2. Bonusgrad Curve Ramping & 3. Malusgrad Ramping (both require
      // change_lock expired)
      if (!is_change_lock_active(now_ms)) {
        // Bonusgrad Curve
        if (valve_room_10m && comp_on_5m &&
            (main_outlet_temp_int + 1 > main_target_temp_int) &&
            (rtc_state.degree_hours_day * 1.05f <
             rtc_state.degree_hours_target) &&
            (hp_setpoint_int < (int)rtc_state.power_target)) {
          int new_sp = (hp_setpoint_int + 1 > z1_temp_int + 1)
                           ? (hp_setpoint_int + 1)
                           : (z1_temp_int + 1);
          char sp_buf[16];
          snprintf(sp_buf, sizeof(sp_buf), "%d", new_sp);
          meisha_send_command("SetZ1HeatRequestTemperature", sp_buf);
          set_change_lock(now_ms, 30 * 60 * 1000UL);
          set_last_action_reason("bonusgrad_curve");
          publish_derived_states();
        }
        // Malusgrad
        else if (valve_room_10m && comp_on_5m &&
                 (main_outlet_temp_int - 1 < main_target_temp_int) &&
                 (main_target_temp_int > (int)rtc_state.power_target) &&
                 (hp_setpoint_int > (int)rtc_state.power_target)) {
          int stepped_down = (hp_setpoint_int - 1 < z1_temp_int - 1)
                                 ? (hp_setpoint_int - 1)
                                 : (z1_temp_int - 1);
          int new_sp = (stepped_down > (int)rtc_state.power_target)
                           ? stepped_down
                           : (int)rtc_state.power_target;
          char sp_buf[16];
          snprintf(sp_buf, sizeof(sp_buf), "%d", new_sp);
          meisha_send_command("SetZ1HeatRequestTemperature", sp_buf);
          set_change_lock(now_ms, 20 * 60 * 1000UL);
          set_last_action_reason("malusgrad");
          publish_derived_states();
        }
      }
    }
  } else if (s_tunables.cooling_mode) {
    // When cooling_mode == on, ensure winter heating timers and states remain
    // cleared
    rtc_state.low_setpoint_timer_active = 0;
    rtc_state.idle_shutoff_fired = 0;
    rtc_state.night_mode_sent = 0;
    rtc_state.overshoot_timer_active = 0;
    rtc_state.overshoot_steps_fired = 0;

    // Cool Turn Off: ambient_24h < 22.0 C held continuously for 1 hour
    // (3,600,000 ms)
    if (rtc_state.ambient_24h_mean < 22.0f) {
      if (!rtc_state.cool_low_ambient_timer_active) {
        rtc_state.cool_low_ambient_timer_active = 1;
        rtc_state.cool_low_ambient_since_ms = now_ms;
        rtc_state.cool_low_ambient_fired = 0;
      } else if (!rtc_state.cool_low_ambient_fired &&
                 (now_ms - rtc_state.cool_low_ambient_since_ms) >= 3600000UL) {
        rtc_state.cool_low_ambient_fired = 1;
        rtc_state.cool_start_pending = 0;
        if (hp_on && op_mode != MEISHA_OPMODE_DHW_ONLY) {
          meisha_send_command("SetHeatpump", "0");
          set_last_action_reason("cool_turn_off");
          publish_derived_states();
        }
      }
    } else {
      rtc_state.cool_low_ambient_timer_active = 0;
      rtc_state.cool_low_ambient_fired = 0;
    }
  }

  // Evaporator-icing protection (Quiet Mode FM Off)
  // Sets Quiet Mode to 0 (SetQuietMode = 0) whenever quiet_mode_level != 0,
  // (pipe_ambient_delta > 4.0 or (pipe_ambient_delta > 3.0 and outside_temp
  // < 1.1)), and any trigger duration holds (compressor_freq > 25.0 Hz for 1
  // min, pipe_ambient_delta > 4.0 K for 5 min, pipe_ambient_delta > 3.0 K for 5
  // min, or outside_temp < 1.1 C for 5 min).
  bool fm_off_state_cond =
      (quiet_mode_level != 0) &&
      ((rtc_state.pipe_ambient_delta > 4.0f) ||
       (rtc_state.pipe_ambient_delta > 3.0f && outside_temp < 1.1f));

  if (quiet_mode_level != 0 && compressor_freq > 25.0f) {
    if (!rtc_state.fm_off_comp_timer_active) {
      rtc_state.fm_off_comp_timer_active = 1;
      rtc_state.fm_off_comp_since_ms = now_ms;
    }
  } else {
    rtc_state.fm_off_comp_timer_active = 0;
  }
  bool fm_comp_1m = rtc_state.fm_off_comp_timer_active &&
                    ((now_ms - rtc_state.fm_off_comp_since_ms) >= 60000UL);

  if (quiet_mode_level != 0 && rtc_state.pipe_ambient_delta > 4.0f) {
    if (!rtc_state.fm_off_delta4_timer_active) {
      rtc_state.fm_off_delta4_timer_active = 1;
      rtc_state.fm_off_delta4_since_ms = now_ms;
    }
  } else {
    rtc_state.fm_off_delta4_timer_active = 0;
  }
  bool fm_delta4_5m = rtc_state.fm_off_delta4_timer_active &&
                      ((now_ms - rtc_state.fm_off_delta4_since_ms) >= 300000UL);

  if (quiet_mode_level != 0 && rtc_state.pipe_ambient_delta > 3.0f &&
      outside_temp < 1.1f) {
    if (!rtc_state.fm_off_delta3_timer_active) {
      rtc_state.fm_off_delta3_timer_active = 1;
      rtc_state.fm_off_delta3_since_ms = now_ms;
    }
  } else {
    rtc_state.fm_off_delta3_timer_active = 0;
  }
  bool fm_delta3_5m = rtc_state.fm_off_delta3_timer_active &&
                      ((now_ms - rtc_state.fm_off_delta3_since_ms) >= 300000UL);

  if (quiet_mode_level != 0 && outside_temp < 1.1f &&
      rtc_state.pipe_ambient_delta > 3.0f) {
    if (!rtc_state.fm_off_outside_timer_active) {
      rtc_state.fm_off_outside_timer_active = 1;
      rtc_state.fm_off_outside_since_ms = now_ms;
    }
  } else {
    rtc_state.fm_off_outside_timer_active = 0;
  }
  bool fm_outside_5m =
      rtc_state.fm_off_outside_timer_active &&
      ((now_ms - rtc_state.fm_off_outside_since_ms) >= 300000UL);

  bool fm_off_trigger =
      fm_comp_1m || fm_delta4_5m || fm_delta3_5m || fm_outside_5m;

  if (fm_off_state_cond && fm_off_trigger) {
    if (!rtc_state.fm_off_sent) {
      meisha_send_command("SetQuietMode", "0");
      rtc_state.fm_off_sent = 1;
      set_last_action_reason("fm_off");
      publish_derived_states();
    }
  } else if (!fm_off_state_cond) {
    rtc_state.fm_off_sent = 0;
  }

  // Quiet Mode Recovery (FM On)
  // Sets Quiet Mode to 1 (SetQuietMode = 1) whenever quiet_mode_level == 0,
  // 1.0 < compressor_freq < 23.5 Hz, defrosting_state == 0 continuously for 2
  // hours, pipe_ambient_delta < 3.1 K, (pipe_ambient_delta < 2.1 K or
  // outside_temp > 1.0 C), and either (1.0 < compressor_freq < 23.5 Hz for 1
  // min) or (pipe_ambient_delta drops below 4.0 K).
  if (defrosting_state == 0) {
    if (!rtc_state.defrost_off_timer_active) {
      rtc_state.defrost_off_timer_active = 1;
      rtc_state.defrost_off_since_ms = now_ms;
    }
  } else {
    rtc_state.defrost_off_timer_active = 0;
  }
  bool defrosted_2h = rtc_state.defrost_off_timer_active &&
                      ((now_ms - rtc_state.defrost_off_since_ms) >= 7200000UL);

  bool comp_light_load = (compressor_freq > 1.0f && compressor_freq < 23.5f);
  if (comp_light_load) {
    if (!rtc_state.fm_on_comp_timer_active) {
      rtc_state.fm_on_comp_timer_active = 1;
      rtc_state.fm_on_comp_since_ms = now_ms;
    }
  } else {
    rtc_state.fm_on_comp_timer_active = 0;
  }
  bool comp_light_1m = rtc_state.fm_on_comp_timer_active &&
                       ((now_ms - rtc_state.fm_on_comp_since_ms) >= 60000UL);

  bool delta_dropped_below_4 = rtc_state.prev_delta_valid &&
                               (rtc_state.prev_pipe_ambient_delta >= 4.0f) &&
                               (rtc_state.pipe_ambient_delta < 4.0f);

  bool fm_on_state_cond =
      (quiet_mode_level == 0) && comp_light_load && defrosted_2h &&
      (rtc_state.pipe_ambient_delta < 3.1f) &&
      ((rtc_state.pipe_ambient_delta < 2.1f) || (outside_temp > 1.0f));

  if (fm_on_state_cond && (comp_light_1m || delta_dropped_below_4)) {
    if (!rtc_state.fm_on_sent) {
      meisha_send_command("SetQuietMode", "1");
      rtc_state.fm_on_sent = 1;
      set_last_action_reason("fm_on");
      publish_derived_states();
    }
  } else if (!fm_on_state_cond) {
    rtc_state.fm_on_sent = 0;
  }

  rtc_state.prev_pipe_ambient_delta = rtc_state.pipe_ambient_delta;
  rtc_state.prev_delta_valid = 1;

  rtc_state.crc32 = rtc_crc(&rtc_state);

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
  snprintf(sub_topic, sizeof(sub_topic), "%s/meisha/set/#",
           heishamonSettings.mqtt_topic_base);
  mqtt_client.subscribe(sub_topic);

  publish_ha_discovery();
  publish_mode_state();
  publish_all_tunables_state();
  publish_rolling_average_states();
  publish_derived_states();
}

bool meisha_mqtt_callback(const char *topic, const char *topic_command,
                          const char *msg, unsigned int length) {
  (void)topic;
  (void)length;
  if (!topic_command || !msg)
    return false;

  const char *prefix = "meisha/set/";
  size_t prefix_len = strlen(prefix);
  if (strncmp(topic_command, prefix, prefix_len) != 0) {
    return false;
  }

  const char *subtopic = topic_command + prefix_len;
  if (strcmp(subtopic, "control_mode") == 0) {
    MeishaControlMode new_mode;
    if (parse_mode_string(msg, &new_mode)) {
      if (new_mode != MEISHA_MODE_OFF &&
          (rtc_state.crash_tripped || rtc_state.crash_count > 0)) {
        rtc_state.crash_tripped = 0;
        rtc_state.crash_count = 0;
        rtc_state.crc32 = rtc_crc(&rtc_state);
      }
      meisha_set_control_mode(new_mode);
      return true;
    }
  } else if (strcmp(subtopic, "room_target") == 0) {
    char *endptr = nullptr;
    float val = strtof(msg, &endptr);
    if (endptr != msg) {
      return meisha_set_room_target(val);
    }
  } else if (strcmp(subtopic, "heating_curve_start") == 0) {
    char *endptr = nullptr;
    float val = strtof(msg, &endptr);
    if (endptr != msg) {
      return meisha_set_heating_curve_start(val);
    }
  } else if (strcmp(subtopic, "heating_curve_angle") == 0) {
    char *endptr = nullptr;
    float val = strtof(msg, &endptr);
    if (endptr != msg) {
      return meisha_set_heating_curve_angle(val);
    }
  } else if (strcmp(subtopic, "heating_start_hour") == 0) {
    char *endptr = nullptr;
    long val = strtol(msg, &endptr, 10);
    if (endptr != msg && val >= 0 && val <= 23) {
      return meisha_set_heating_start_hour((uint8_t)val);
    }
  } else if (strcmp(subtopic, "cooling_mode") == 0) {
    if (strcasecmp(msg, "on") == 0 || strcmp(msg, "1") == 0 ||
        strcasecmp(msg, "true") == 0) {
      return meisha_set_cooling_mode(true);
    } else if (strcasecmp(msg, "off") == 0 || strcmp(msg, "0") == 0 ||
               strcasecmp(msg, "false") == 0) {
      return meisha_set_cooling_mode(false);
    }
  }

  return false;
}

float meisha_get_curve_target(void) { return rtc_state.curve_target; }

float meisha_get_pid_output(void) { return rtc_state.pid_output; }

int meisha_get_power_target(void) { return (int)rtc_state.power_target; }

float meisha_get_room_delta(void) { return rtc_state.room_delta; }

float meisha_get_degree_hours_day(void) { return rtc_state.degree_hours_day; }

float meisha_get_degree_hours_target(void) {
  return rtc_state.degree_hours_target;
}

float meisha_get_dhw_time(void) { return rtc_state.dhw_time; }

float meisha_get_hours_target(void) { return rtc_state.hours_target; }

float meisha_get_hours_time_left(void) { return rtc_state.hours_time_left; }

float meisha_get_pipe_ambient_delta(void) {
  return rtc_state.pipe_ambient_delta;
}

bool meisha_get_compressor_running(void) {
  return rtc_state.compressor_running != 0;
}

float meisha_get_change_lock_remaining_min(void) {
  return rtc_state.change_lock_remaining_min;
}

const char *meisha_get_last_action_reason(void) {
  return rtc_state.last_action_reason;
}
