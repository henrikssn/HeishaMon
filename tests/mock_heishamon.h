#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string>
#include <vector>
#include <map>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <ctime>

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

class String {
public:
  std::string str;

  String() : str("") {}
  String(const char* s) : str(s ? s : "") {}
  String(const std::string& s) : str(s) {}

  float toFloat() const {
    if (str.empty()) return 0.0f;
    return std::strtof(str.c_str(), nullptr);
  }

  int toInt() const {
    if (str.empty()) return 0;
    return std::atoi(str.c_str());
  }

  const char* c_str() const {
    return str.c_str();
  }

  size_t length() const {
    return str.length();
  }
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
  static std::map<std::string, std::map<std::string, std::vector<uint8_t>>> storage;
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
      if (storage[ns][key].size() >= sizeof(uint8_t)) {
        return storage[ns][key][0];
      }
    }
    return defaultValue;
  }

  size_t putUChar(const char* key, uint8_t value) {
    write_counts[ns + "/" + key]++;
    storage[ns][key] = {value};
    return 1;
  }

  float getFloat(const char* key, float defaultValue = 0.0f) {
    if (storage.find(ns) != storage.end() && storage[ns].find(key) != storage[ns].end()) {
      if (storage[ns][key].size() >= sizeof(float)) {
        float val;
        memcpy(&val, storage[ns][key].data(), sizeof(float));
        return val;
      }
    }
    return defaultValue;
  }

  size_t putFloat(const char* key, float value) {
    write_counts[ns + "/" + key]++;
    std::vector<uint8_t> buf(sizeof(float));
    memcpy(buf.data(), &value, sizeof(float));
    storage[ns][key] = buf;
    return sizeof(float);
  }

  bool getBool(const char* key, bool defaultValue = false) {
    return getUChar(key, defaultValue ? 1 : 0) != 0;
  }

  size_t putBool(const char* key, bool value) {
    return putUChar(key, value ? 1 : 0);
  }

  size_t getBytes(const char* key, void* buf, size_t maxLen) {
    if (storage.find(ns) != storage.end() && storage[ns].find(key) != storage[ns].end()) {
      size_t len = storage[ns][key].size();
      if (len > maxLen) len = maxLen;
      if (len > 0) {
        memcpy(buf, storage[ns][key].data(), len);
        return len;
      }
    }
    return 0;
  }

  size_t putBytes(const char* key, const void* value, size_t len) {
    write_counts[ns + "/" + key]++;
    std::vector<uint8_t> buf(len);
    if (len > 0) {
      memcpy(buf.data(), value, len);
    }
    storage[ns][key] = buf;
    return len;
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

void set_mock_time(time_t t);
void advance_mock_time(time_t t);
time_t mock_time(time_t* tloc);
struct tm* mock_localtime(const time_t* timep);

void set_mock_topic_value(unsigned int topic, const char* val);
String getDataValue(char* data, unsigned int Topic_Number);
void send_heatpump_command(char* cmd, char* value, bool (*send_cmd)(uint8_t*, int), void (*log_fn)(char*), bool optionalPCB);
bool send_command(uint8_t* command, int length);
void log_message(char* string);
void mock_corrupt_rtc_state(void);
void reset_mock_environment(void);
