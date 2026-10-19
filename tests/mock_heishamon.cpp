#include "mock_heishamon.h"
#include "../HeishaMon/meisha.h"

char actData[DATASIZE];
settingsStruct heishamonSettings;
PubSubClient mqtt_client;
const char* mqtt_willtopic = "LWT";
std::vector<MockCommandCall> g_mock_commands;

std::map<std::string, std::map<std::string, std::vector<uint8_t>>> Preferences::storage;
std::map<std::string, int> Preferences::write_counts;

static esp_reset_reason_t s_mock_reason = ESP_RST_POWERON;
static uint32_t s_mock_millis = 0;
static time_t s_mock_epoch = 1761984000; // Monday 2025-11-03 08:00:00 UTC
static std::map<unsigned int, std::string> s_mock_topics;

void set_mock_reset_reason(esp_reset_reason_t reason) {
  s_mock_reason = reason;
}

esp_reset_reason_t esp_reset_reason(void) {
  return s_mock_reason;
}

void set_mock_millis(uint32_t ms) {
  s_mock_millis = ms;
}

void advance_mock_millis(uint32_t ms) {
  s_mock_millis += ms;
}

uint32_t millis(void) {
  return s_mock_millis;
}

void set_mock_time(time_t t) {
  s_mock_epoch = t;
}

void advance_mock_time(time_t t) {
  s_mock_epoch += t;
}

time_t mock_time(time_t* tloc) {
  if (tloc) *tloc = s_mock_epoch;
  return s_mock_epoch;
}

struct tm* mock_localtime(const time_t* timep) {
  static struct tm tm_buf;
  time_t ep = timep ? *timep : s_mock_epoch;
  gmtime_r(&ep, &tm_buf); // Use UTC as predictable local time in tests
  return &tm_buf;
}

void set_mock_topic_value(unsigned int topic, const char* val) {
  s_mock_topics[topic] = val ? val : "";
}

String getDataValue(char* data, unsigned int Topic_Number) {
  (void)data;
  if (s_mock_topics.find(Topic_Number) != s_mock_topics.end()) {
    return String(s_mock_topics[Topic_Number].c_str());
  }
  return String("");
}

void send_heatpump_command(char* cmd, char* value, bool (*send_cmd)(uint8_t*, int), void (*log_fn)(char*), bool optionalPCB) {
  (void)send_cmd;
  (void)log_fn;
  (void)optionalPCB;
  g_mock_commands.push_back({cmd ? cmd : "", value ? value : ""});
}

bool send_command(uint8_t* command, int length) {
  (void)command;
  (void)length;
  return true;
}

void log_message(char* string) {
  (void)string;
}

void mock_corrupt_rtc_state(void) {
  auto* rtc = const_cast<MeishaRtcState*>(meisha_get_rtc_state());
  if (rtc) {
    memset(rtc, 0, sizeof(MeishaRtcState));
  }
}

void reset_mock_environment(void) {
  memset(actData, 0, sizeof(actData));
  strncpy(heishamonSettings.mqtt_topic_base, "panasonic_heat_pump", sizeof(heishamonSettings.mqtt_topic_base));
  heishamonSettings.optionalPCB = false;
  mqtt_client.clear();
  g_mock_commands.clear();
  s_mock_topics.clear();
  s_mock_reason = ESP_RST_POWERON;
  s_mock_millis = 0;
  s_mock_epoch = 1761984000;
}
