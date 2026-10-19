#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef MEISHA_ENTITY_PREFIX
#define MEISHA_ENTITY_PREFIX "meisha_esp_"
#endif

#define MEISHA_RTC_MAGIC 0x4D454953 // "MEIS"

#define MEISHA_AMBIENT_1H_SAMPLES 60
#define MEISHA_AMBIENT_24H_SAMPLES 288
#define MEISHA_ROOM_6H_SAMPLES 360

enum MeishaControlMode : uint8_t {
  MEISHA_MODE_OFF = 0,
  MEISHA_MODE_SHADOW = 1,
  MEISHA_MODE_ACTIVE = 2
};

struct MeishaTunables {
  float room_target;          // 18.0..25.0, default 23.0
  float heating_curve_start;  // 12.0..20.0, default 14.0
  float heating_curve_angle;  // 0.2..1.5, default 0.8
  uint8_t heating_start_hour; // 0..23, default 9
  bool cooling_mode;          // false=off, true=on
};

// Exact 32-byte scalar checkpoint struct written to NVS
struct MeishaNvsCheckpoint {
  float ambient_24h_mean;
  float ambient_1h_mean;
  float room_temp_6h_mean;
  float room_temp_snapshot;
  float degree_hours_day;
  float dhw_time;
  float pid_output;
  uint32_t saved_at_epoch;
};

#if defined(__cplusplus)
static_assert(sizeof(MeishaNvsCheckpoint) == 32,
              "MeishaNvsCheckpoint must be exactly 32 bytes");
#endif

struct MeishaRtcState {
  uint32_t magic;
  uint16_t version;
  uint16_t crash_count;
  uint8_t crash_tripped;
  uint8_t seeded;
  uint8_t ambient_24h_sub_ticks;
  uint8_t compressor_running;
  int32_t last_reset_heating_day_id;

  uint16_t ambient_1h_head;
  uint16_t ambient_1h_count;
  int16_t ambient_1h_buf[MEISHA_AMBIENT_1H_SAMPLES];

  uint16_t ambient_24h_head;
  uint16_t ambient_24h_count;
  int16_t ambient_24h_buf[MEISHA_AMBIENT_24H_SAMPLES];

  uint16_t room_6h_head;
  uint16_t room_6h_count;
  int16_t room_6h_buf[MEISHA_ROOM_6H_SAMPLES];

  float ambient_1h_mean;
  float ambient_24h_mean;
  float room_temp_6h_mean;
  float room_temp_snapshot;
  float degree_hours_day;
  float dhw_time;
  float pid_output;

  float curve_target;
  float room_delta;
  float degree_hours_target;
  float hours_target;
  float hours_time_left;
  float pipe_ambient_delta;
  int16_t power_target;
  int16_t reserved_pad;

  // Duration & sequence timers (ms)
  uint8_t valve_room_timer_active;
  uint8_t comp_on_timer_active;
  uint8_t overshoot_timer_active;
  uint8_t overshoot_steps_fired;
  uint8_t change_lock_active;
  uint8_t day_mode_pending;
  uint8_t low_setpoint_timer_active;
  uint8_t idle_shutoff_fired;
  uint8_t turn_off_pending;
  uint8_t night_mode_sent;
  uint8_t dhw_low_temp_timer_active;
  uint8_t dhw_low_temp_fired;
  uint8_t dhw_start_pending;
  uint8_t dhw_seen_valve_tank;
  uint8_t dhw_valve_room_timer_active;
  uint8_t dhw_comp_off_timer_active;
  uint8_t dhw_end_fired;
  uint8_t dhw_end_cool_pending;
  uint8_t cool_start_pending;
  uint8_t cool_low_ambient_timer_active;
  uint8_t cool_low_ambient_fired;
  uint8_t fm_off_comp_timer_active;
  uint8_t fm_off_comp_fired;
  uint8_t fm_off_delta4_timer_active;
  uint8_t fm_off_delta4_fired;
  uint8_t fm_off_delta3_timer_active;
  uint8_t fm_off_delta3_fired;
  uint8_t fm_off_outside_timer_active;
  uint8_t fm_off_outside_fired;
  uint8_t fm_off_sent;
  uint8_t defrost_off_timer_active;
  uint8_t fm_on_comp_timer_active;
  uint8_t prev_delta_valid;
  uint8_t fm_on_sent;
  uint8_t timer_reserved_pad[1];

  float prev_pipe_ambient_delta;

  int32_t last_dhw_schedule_day_id;
  int32_t last_cool_start_day_id;
  int32_t last_cool_stop_day_id;

  uint32_t valve_room_since_ms;
  uint32_t comp_on_since_ms;
  uint32_t overshoot_start_ms;
  uint32_t change_lock_set_ms;
  uint32_t change_lock_duration_ms;
  uint32_t day_mode_start_ms;
  uint32_t low_setpoint_since_ms;
  uint32_t turn_off_start_ms;
  uint32_t dhw_low_temp_since_ms;
  uint32_t dhw_start_ms;
  uint32_t dhw_valve_room_since_ms;
  uint32_t dhw_comp_off_since_ms;
  uint32_t dhw_end_cool_start_ms;
  uint32_t cool_start_ms;
  uint32_t cool_low_ambient_since_ms;
  uint32_t fm_off_comp_since_ms;
  uint32_t fm_off_delta4_since_ms;
  uint32_t fm_off_delta3_since_ms;
  uint32_t fm_off_outside_since_ms;
  uint32_t defrost_off_since_ms;
  uint32_t fm_on_comp_since_ms;

  float change_lock_remaining_min;
  char last_action_reason[32];

  uint32_t crc32;
};

#ifdef __cplusplus
extern "C" {
#endif

void meisha_setup(void);
void meisha_loop(void);
void meisha_mqtt_connected(void);
bool meisha_mqtt_callback(const char *topic, const char *topic_command,
                          const char *msg, unsigned int length);

MeishaControlMode meisha_get_control_mode(void);
void meisha_set_control_mode(MeishaControlMode mode);
bool meisha_send_command(const char *cmd, const char *value);
const MeishaRtcState *meisha_get_rtc_state(void);

MeishaTunables meisha_get_tunables(void);
bool meisha_set_room_target(float val);
bool meisha_set_heating_curve_start(float val);
bool meisha_set_heating_curve_angle(float val);
bool meisha_set_heating_start_hour(uint8_t val);
bool meisha_set_cooling_mode(bool val);

float meisha_get_ambient_1h(void);
float meisha_get_ambient_24h(void);
float meisha_get_room_temp_6h(void);
float meisha_get_room_temp_snapshot(void);

float meisha_get_curve_target(void);
float meisha_get_pid_output(void);
int meisha_get_power_target(void);
float meisha_get_room_delta(void);
float meisha_get_degree_hours_day(void);
float meisha_get_degree_hours_target(void);
float meisha_get_dhw_time(void);
float meisha_get_hours_target(void);
float meisha_get_hours_time_left(void);
float meisha_get_pipe_ambient_delta(void);
bool meisha_get_compressor_running(void);
float meisha_get_change_lock_remaining_min(void);
const char *meisha_get_last_action_reason(void);

#ifdef __cplusplus
}
#endif
