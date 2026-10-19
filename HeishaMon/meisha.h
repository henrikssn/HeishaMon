#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifndef MEISHA_ENTITY_PREFIX
#define MEISHA_ENTITY_PREFIX "meisha_esp_"
#endif

#define MEISHA_RTC_MAGIC 0x4D454953 // "MEIS"

enum MeishaControlMode : uint8_t {
  MEISHA_MODE_OFF = 0,
  MEISHA_MODE_SHADOW = 1,
  MEISHA_MODE_ACTIVE = 2
};

struct MeishaRtcState {
  uint32_t magic;
  uint16_t version;
  uint16_t crash_count;
  uint8_t crash_tripped;
  uint8_t reserved[23];
  uint32_t crc32;
};

#ifdef __cplusplus
extern "C" {
#endif

void meisha_setup(void);
void meisha_loop(void);
void meisha_mqtt_connected(void);
bool meisha_mqtt_callback(const char* topic, const char* topic_command, const char* msg, unsigned int length);

MeishaControlMode meisha_get_control_mode(void);
void meisha_set_control_mode(MeishaControlMode mode);
bool meisha_send_command(const char* cmd, const char* value);
const MeishaRtcState* meisha_get_rtc_state(void);

#ifdef __cplusplus
}
#endif
