#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string>
#include <vector>
#include <map>
#include <cstring>
#include <cstdio>

#define MOCK_HEISHAMON 1
#define DATASIZE 203

typedef enum {
  ESP_RST_UNKNOWN = 0,
  ESP_RST_POWERON = 1,
  ESP_RST_EXT = 2,
  ESP_RST_SW = 3,
  ESP_RST_PANIC = 4,
  ESP_RST_INT_WDT = 5,
  ESP_RST_TASK_WDT = 6,
  ESP_RST_WDT = 7,
  ESP_RST_DEEPSLEEP = 8,
  ESP_RST_BROWNOUT = 9,
  ESP_RST_SDIO = 10
} esp_reset_reason_t;

struct settingsStruct {
  char mqtt_topic_base[128];
  bool optionalPCB;
};

struct MockMqttMessage {
  std::string topic;
  std::string payload;
  bool retained;
};

struct MockCommandCall {
  std::string cmd;
  std::string value;
};

class PubSubClient {
public:
  std::vector<std::string> subscriptions;
  std::vector<MockMqttMessage> published;

  bool subscribe(const char* topic) {
    subscriptions.push_back(topic ? topic : "");
    return true;
  }

  bool publish(const char* topic, const char* payload, bool retained = false) {
    published.push_back({topic ? topic : "", payload ? payload : "", retained});
    return true;
  }

  void clear() {
    subscriptions.clear();
    published.clear();
  }
};

class Preferences {
public:
  static std::map<std::string, std::map<std::string, uint8_t>> storage;
  static std::map<std::string, int> write_counts;

  std::string ns;

  bool begin(const char* name, bool readOnly = false) {
    (void)readOnly;
    ns = name ? name : "";
    return true;
  }

  void end() {}

  uint8_t getUChar(const char* key, uint8_t defaultValue = 0) {
    if (storage.find(ns) != storage.end() && storage[ns].find(key) != storage[ns].end()) {
      return storage[ns][key];
    }
    return defaultValue;
  }

  size_t putUChar(const char* key, uint8_t value) {
    write_counts[ns + "/" + key]++;
    storage[ns][key] = value;
    return 1;
  }

  static void reset() {
    storage.clear();
    write_counts.clear();
  }
};

extern char actData[DATASIZE];
extern settingsStruct heishamonSettings;
extern PubSubClient mqtt_client;
extern const char* mqtt_willtopic;
extern std::vector<MockCommandCall> g_mock_commands;

void set_mock_reset_reason(esp_reset_reason_t reason);
esp_reset_reason_t esp_reset_reason(void);

void set_mock_millis(uint32_t ms);
void advance_mock_millis(uint32_t ms);
uint32_t millis(void);

void send_heatpump_command(char* cmd, char* value, void (*send_cmd)(uint8_t*, int), void (*log_fn)(char*), bool optionalPCB);
void send_command(uint8_t* command, int length);
void log_message(char* string);
void reset_mock_environment(void);
