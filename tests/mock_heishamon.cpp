#include "mock_heishamon.h"

char actData[DATASIZE];
settingsStruct heishamonSettings;
PubSubClient mqtt_client;
const char* mqtt_willtopic = "LWT";
std::vector<MockCommandCall> g_mock_commands;

std::map<std::string, std::map<std::string, uint8_t>> Preferences::storage;
std::map<std::string, int> Preferences::write_counts;

static esp_reset_reason_t s_mock_reason = ESP_RST_POWERON;
static uint32_t s_mock_millis = 0;

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

void send_heatpump_command(char* cmd, char* value, void (*send_cmd)(uint8_t*, int), void (*log_fn)(char*), bool optionalPCB) {
  (void)send_cmd;
  (void)log_fn;
  (void)optionalPCB;
  g_mock_commands.push_back({cmd ? cmd : "", value ? value : ""});
}

void send_command(uint8_t* command, int length) {
  (void)command;
  (void)length;
}

void log_message(char* string) {
  (void)string;
}

void reset_mock_environment(void) {
  memset(actData, 0, sizeof(actData));
  strncpy(heishamonSettings.mqtt_topic_base, "panasonic_heat_pump", sizeof(heishamonSettings.mqtt_topic_base));
  heishamonSettings.optionalPCB = false;
  mqtt_client.clear();
  g_mock_commands.clear();
  s_mock_reason = ESP_RST_POWERON;
  s_mock_millis = 0;
}
