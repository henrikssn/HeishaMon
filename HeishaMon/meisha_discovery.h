#pragma once

#include <stdio.h>
#include <string.h>

#ifndef MEISHA_ENTITY_PREFIX
#define MEISHA_ENTITY_PREFIX "meisha_esp_"
#endif

struct MeishaNumberDiscovery {
  const char *id;
  const char *name;
  const char *min_val;
  const char *max_val;
  const char *step_val;
  const char *unit; // nullable
};

struct MeishaSensorDiscovery {
  const char *component; // "sensor" or "binary_sensor"
  const char *id;
  const char *name;
  const char *unit;      // nullable
  const char *dev_class; // nullable
};

static const MeishaNumberDiscovery kMeishaNumbers[] = {
    {"room_target", "Meisha Room Target", "18.0", "25.0", "0.5", "°C"},
    {"heating_curve_start", "Meisha Heating Curve Start", "12.0", "20.0", "1.0",
     "°C"},
    {"heating_curve_angle", "Meisha Heating Curve Angle", "0.2", "1.5", "0.1",
     NULL},
    {"heating_start_hour", "Meisha Heating Start Hour", "0", "23", "1", "h"},
};

static const MeishaSensorDiscovery kMeishaSensors[] = {
    {"sensor", "ambient_1h", "Meisha Ambient 1h", "°C", "temperature"},
    {"sensor", "ambient_24h", "Meisha Ambient 24h", "°C", "temperature"},
    {"sensor", "room_temp_6h", "Meisha Room Temp 6h", "°C", "temperature"},
    {"sensor", "room_temp_snapshot", "Meisha Room Temp Snapshot", "°C",
     "temperature"},
    {"sensor", "curve_target", "Meisha Curve Target", "°C", "temperature"},
    {"sensor", "pid_output", "Meisha PID Output", "K", NULL},
    {"sensor", "power_target", "Meisha Power Target", "°C", "temperature"},
    {"sensor", "room_delta", "Meisha Room Delta", "K", NULL},
    {"sensor", "degree_hours_day", "Meisha Degree Hours Day", "Kh", NULL},
    {"sensor", "degree_hours_target", "Meisha Degree Hours Target", "Kh", NULL},
    {"sensor", "dhw_time", "Meisha DHW Time", "h", NULL},
    {"sensor", "hours_target", "Meisha Hours Target", "h", NULL},
    {"sensor", "hours_time_left", "Meisha Hours Time Left", "h", NULL},
    {"sensor", "pipe_ambient_delta", "Meisha Pipe Ambient Delta", "K", NULL},
    {"binary_sensor", "compressor_running", "Meisha Compressor Running", NULL,
     "running"},
    {"sensor", "change_lock_remaining_min", "Meisha Change Lock Remaining Min",
     "min", "duration"},
    {"sensor", "last_action_reason", "Meisha Last Action Reason", NULL, NULL},
};

template <typename MqttClientType>
void meisha_publish_discovery_table(MqttClientType &mqtt,
                                    const char *topic_base,
                                    const char *will_topic) {
  char disc_topic[160];
  char payload[512];

  // 1. Control mode select
  snprintf(disc_topic, sizeof(disc_topic),
           "homeassistant/select/%scontrol_mode/config", MEISHA_ENTITY_PREFIX);
  snprintf(payload, sizeof(payload),
           "{"
           "\"name\":\"Meisha ESP Control Mode\","
           "\"unique_id\":\"%scontrol_mode\","
           "\"object_id\":\"%scontrol_mode\","
           "\"command_topic\":\"%s/meisha/set/control_mode\","
           "\"state_topic\":\"%s/meisha/state/control_mode\","
           "\"options\":[\"Off\",\"Shadow\",\"Active\"],"
           "\"device\":{\"identifiers\":[\"meisha_esp_controller\"],\"name\":"
           "\"Meisha Heat Pump Controller\"}"
           "}",
           MEISHA_ENTITY_PREFIX, MEISHA_ENTITY_PREFIX, topic_base, topic_base);
  mqtt.publish(disc_topic, payload, true);

  // 2. Cooling mode switch
  snprintf(disc_topic, sizeof(disc_topic),
           "homeassistant/switch/%scooling_mode/config", MEISHA_ENTITY_PREFIX);
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
           MEISHA_ENTITY_PREFIX, MEISHA_ENTITY_PREFIX, topic_base, topic_base);
  mqtt.publish(disc_topic, payload, true);

  // 3. Numbers table
  for (size_t i = 0; i < sizeof(kMeishaNumbers) / sizeof(kMeishaNumbers[0]);
       i++) {
    const auto &item = kMeishaNumbers[i];
    snprintf(disc_topic, sizeof(disc_topic), "homeassistant/number/%s%s/config",
             MEISHA_ENTITY_PREFIX, item.id);
    if (item.unit) {
      snprintf(
          payload, sizeof(payload),
          "{"
          "\"name\":\"%s\","
          "\"unique_id\":\"%s%s\","
          "\"object_id\":\"%s%s\","
          "\"command_topic\":\"%s/meisha/set/%s\","
          "\"state_topic\":\"%s/meisha/state/%s\","
          "\"min\":%s,\"max\":%s,\"step\":%s,\"unit_of_measurement\":\"%s\","
          "\"device\":{\"identifiers\":[\"meisha_esp_controller\"]}"
          "}",
          item.name, MEISHA_ENTITY_PREFIX, item.id, MEISHA_ENTITY_PREFIX,
          item.id, topic_base, item.id, topic_base, item.id, item.min_val,
          item.max_val, item.step_val, item.unit);
    } else {
      snprintf(payload, sizeof(payload),
               "{"
               "\"name\":\"%s\","
               "\"unique_id\":\"%s%s\","
               "\"object_id\":\"%s%s\","
               "\"command_topic\":\"%s/meisha/set/%s\","
               "\"state_topic\":\"%s/meisha/state/%s\","
               "\"min\":%s,\"max\":%s,\"step\":%s,"
               "\"device\":{\"identifiers\":[\"meisha_esp_controller\"]}"
               "}",
               item.name, MEISHA_ENTITY_PREFIX, item.id, MEISHA_ENTITY_PREFIX,
               item.id, topic_base, item.id, topic_base, item.id, item.min_val,
               item.max_val, item.step_val);
    }
    mqtt.publish(disc_topic, payload, true);
  }

  // 4. Sensors and binary sensors table
  for (size_t i = 0; i < sizeof(kMeishaSensors) / sizeof(kMeishaSensors[0]);
       i++) {
    const auto &item = kMeishaSensors[i];
    snprintf(disc_topic, sizeof(disc_topic), "homeassistant/%s/%s%s/config",
             item.component, MEISHA_ENTITY_PREFIX, item.id);
    char unit_chunk[48] = "";
    char dev_class_chunk[48] = "";
    char binary_chunk[48] = "";

    if (item.unit) {
      snprintf(unit_chunk, sizeof(unit_chunk),
               ",\"unit_of_measurement\":\"%s\"", item.unit);
    }
    if (item.dev_class) {
      snprintf(dev_class_chunk, sizeof(dev_class_chunk),
               ",\"device_class\":\"%s\"", item.dev_class);
    }
    if (strcmp(item.component, "binary_sensor") == 0) {
      snprintf(binary_chunk, sizeof(binary_chunk),
               ",\"payload_on\":\"ON\",\"payload_off\":\"OFF\"");
    }

    snprintf(payload, sizeof(payload),
             "{"
             "\"name\":\"%s\","
             "\"unique_id\":\"%s%s\","
             "\"object_id\":\"%s%s\","
             "\"state_topic\":\"%s/meisha/state/%s\"%s%s%s,"
             "\"device\":{\"identifiers\":[\"meisha_esp_controller\"]}"
             "}",
             item.name, MEISHA_ENTITY_PREFIX, item.id, MEISHA_ENTITY_PREFIX,
             item.id, topic_base, item.id, unit_chunk, dev_class_chunk,
             binary_chunk);
    mqtt.publish(disc_topic, payload, true);
  }
}
