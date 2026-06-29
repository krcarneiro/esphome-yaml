#include "espresense_bridge.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include "esphome/components/json/json_util.h"
#include "esphome/components/network/ip_address.h"
#include "esphome/components/wifi/wifi_component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/version.h"

#ifdef USE_ESP32
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

namespace esphome {
namespace espresense_bridge {

static const char *const TAG = "espresense_bridge";

void ESPresenseBridge::setup() {
  this->configs_.reserve(this->max_configs_);
  this->devices_.reserve(this->max_devices_);

  // Device aliases/configuration retained by ESPresense Companion.
  this->subscribe(
      this->settings_base_() + "+/config",
      &ESPresenseBridge::on_device_setting_, 1);

  // Compatibility with older/direct settings topics observed in existing brokers.
  this->subscribe(
      this->settings_base_() + "+",
      &ESPresenseBridge::on_device_setting_, 1);

  // The real ESPresense nodes listen to global room settings first...
  this->subscribe(
      this->mqtt_prefix_ + "/rooms/*/+/set",
      &ESPresenseBridge::on_room_setting_, 1);

  // ...and then room-specific settings, which therefore take precedence.
  this->subscribe(
      this->room_base_() + "+/set",
      &ESPresenseBridge::on_room_setting_, 1);
}

void ESPresenseBridge::dump_config() {
  ESP_LOGCONFIG(TAG, "ESPresense Bridge:");
  ESP_LOGCONFIG(TAG, "  MQTT prefix: %s", this->mqtt_prefix_.c_str());
  ESP_LOGCONFIG(TAG, "  Room ID: %s", this->room_id_.c_str());
  ESP_LOGCONFIG(TAG, "  Room name: %s", this->room_name_.c_str());
  ESP_LOGCONFIG(TAG, "  Max distance: %.2f m", this->max_distance_);
  ESP_LOGCONFIG(TAG, "  Absorption: %.2f", this->absorption_);
  ESP_LOGCONFIG(TAG, "  Rx Adj RSSI: %d", this->rx_adj_rssi_);
  ESP_LOGCONFIG(TAG, "  Tx Ref RSSI: %d", this->tx_ref_rssi_);
  ESP_LOGCONFIG(TAG, "  Max devices/configs: %u/%u",
                this->max_devices_, this->max_configs_);
}

bool ESPresenseBridge::parse_device(
    const esp32_ble_tracker::ESPBTDevice &device) {
  this->adverts_++;

  const int rssi = device.get_rssi();
  if (rssi >= 0 || rssi < -127) {
    return false;
  }

  const FingerprintResult fp = this->fingerprint_(device);
  if (fp.fingerprint.empty()) {
    return false;
  }

  DeviceState *state = this->find_state_(fp.fingerprint);
  if (state == nullptr) {
    state = this->create_or_recycle_state_(fp.fingerprint);
  }
  if (state == nullptr) {
    return false;
  }

  const uint32_t now = millis();
  if (state->last_advertisement != 0) {
    state->advertisement_interval = now - state->last_advertisement;
  }
  state->last_advertisement = now;
  state->last_seen = now;
  state->mac = this->compact_mac_(device);
  state->advertised_name = device.get_name();
  state->rssi_1m = fp.rssi_1m;

  const float sample = static_cast<float>(rssi);
  if (state->samples == 0) {
    state->filtered_rssi = sample;
    state->rssi_variance = 0.0f;
  } else {
    const float delta = sample - state->filtered_rssi;
    state->filtered_rssi += this->rssi_alpha_ * delta;
    state->rssi_variance =
        (1.0f - this->rssi_alpha_) *
        (state->rssi_variance + this->rssi_alpha_ * delta * delta);
  }

  const float adjusted_rssi =
      state->filtered_rssi + static_cast<float>(this->rx_adj_rssi_);
  const float denominator = 10.0f * this->absorption_;
  float distance = 0.0f;
  if (denominator > 0.0f) {
    distance = powf(
        10.0f,
        (static_cast<float>(state->rssi_1m) - adjusted_rssi) / denominator);
  }

  if (std::isfinite(distance)) {
    if (state->samples == 0) {
      state->distance = distance;
      state->distance_variance = 0.0f;
    } else {
      const float distance_delta = distance - state->distance;
      state->distance += this->rssi_alpha_ * distance_delta;
      state->distance_variance =
          (1.0f - this->rssi_alpha_) *
          (state->distance_variance +
           this->rssi_alpha_ * distance_delta * distance_delta);
    }
  }

  state->samples++;
  state->dirty = true;
  this->seen_++;
  return true;
}

void ESPresenseBridge::loop() {
  const uint32_t now = millis();
  const bool connected = this->is_connected();

  if (!connected) {
    this->mqtt_was_connected_ = false;
    this->announced_after_connect_ = false;
    return;
  }

  if (!this->mqtt_was_connected_) {
    this->mqtt_was_connected_ = true;
    this->connected_at_ = now;
    this->announced_after_connect_ = false;
    this->last_telemetry_ = 0;
  }

  // Give retained /set and /settings messages time to arrive before publishing
  // our current state back to retained room topics.
  if (!this->announced_after_connect_ &&
      now - this->connected_at_ >= this->connect_grace_ms_) {
    this->publish_room_settings_();
    this->publish_telemetry_();
    this->announced_after_connect_ = true;
    this->room_settings_dirty_ = false;
    this->last_telemetry_ = now;
  }

  if (this->announced_after_connect_ && this->room_settings_dirty_) {
    this->publish_room_settings_();
    this->room_settings_dirty_ = false;
  }

  if (this->announced_after_connect_ &&
      (this->last_telemetry_ == 0 ||
       now - this->last_telemetry_ >= this->telemetry_interval_ms_)) {
    this->publish_telemetry_();
    this->last_telemetry_ = now;
  }

  uint8_t published_this_loop = 0;
  for (auto &state : this->devices_) {
    if (published_this_loop >= this->max_publishes_per_loop_) {
      break;
    }
    if (!state.dirty) {
      continue;
    }
    if (state.last_publish != 0 &&
        now - state.last_publish < this->publish_interval_ms_) {
      continue;
    }

    state.dirty = false;
    state.last_publish = now;
    if (this->publish_device_(state, now)) {
      published_this_loop++;
    }
  }

  this->cleanup_devices_(now);
}

void ESPresenseBridge::on_device_setting_(
    const std::string &topic, const std::string &payload) {
  const std::string base = this->settings_base_();
  if (!this->starts_with_(topic, base)) {
    return;
  }

  std::string fingerprint = topic.substr(base.size());
  if (this->ends_with_(fingerprint, "/config")) {
    fingerprint.resize(fingerprint.size() - 7);
  }
  if (fingerprint.empty() || fingerprint.find('/') != std::string::npos) {
    return;
  }

  if (payload.empty()) {
    this->remove_device_config_(fingerprint);
  } else {
    this->upsert_device_config_(fingerprint, payload);
  }
  this->mark_devices_dirty_();
}

void ESPresenseBridge::on_room_setting_(
    const std::string &topic, const std::string &payload) {
  const std::string global_base = this->mqtt_prefix_ + "/rooms/*/";
  const std::string local_base = this->room_base_();

  std::string suffix;
  if (this->starts_with_(topic, local_base)) {
    suffix = topic.substr(local_base.size());
  } else if (this->starts_with_(topic, global_base)) {
    suffix = topic.substr(global_base.size());
  } else {
    return;
  }

  if (!this->ends_with_(suffix, "/set")) {
    return;
  }
  const std::string key = suffix.substr(0, suffix.size() - 4);

  if (key == "name") {
    if (!payload.empty() && payload.size() <= 96) {
      this->room_name_ = payload;
      this->room_settings_dirty_ = true;
    }
    return;
  }

  float float_value = 0.0f;
  int int_value = 0;

  if ((key == "max_distance" || key == "max_dist") &&
      this->parse_float_(payload, float_value) &&
      float_value >= 0.0f && float_value <= 1000.0f) {
    this->max_distance_ = float_value;
    this->room_settings_dirty_ = true;
    this->mark_devices_dirty_();
  } else if (key == "absorption" &&
             this->parse_float_(payload, float_value) &&
             float_value >= 0.1f && float_value <= 10.0f) {
    this->absorption_ = float_value;
    this->room_settings_dirty_ = true;
    this->mark_devices_dirty_();
  } else if (key == "tx_ref_rssi" &&
             this->parse_int_(payload, int_value) &&
             int_value >= -127 && int_value <= 0) {
    this->tx_ref_rssi_ = int_value;
    this->room_settings_dirty_ = true;
  } else if (key == "rx_adj_rssi" &&
             this->parse_int_(payload, int_value) &&
             int_value >= -100 && int_value <= 100) {
    this->rx_adj_rssi_ = int_value;
    this->room_settings_dirty_ = true;
    this->mark_devices_dirty_();
  }
}

ESPresenseBridge::FingerprintResult ESPresenseBridge::fingerprint_(
    const esp32_ble_tracker::ESPBTDevice &device) {
  // First resolve enrolled rotating addresses using IRKs received from Companion.
  for (const auto &config : this->configs_) {
    if (config.has_irk && device.resolve_irk(config.irk.data())) {
      int rssi_1m = this->default_rssi_1m_;
      if (std::isfinite(config.rssi_1m)) {
        rssi_1m = static_cast<int>(lroundf(config.rssi_1m));
      }
      return {config.fingerprint, rssi_1m};
    }
  }

  const auto maybe_ibeacon = device.get_ibeacon();
  if (maybe_ibeacon.has_value()) {
    auto ibeacon = *maybe_ibeacon;
    std::string uuid = this->lower_(ibeacon.get_uuid().to_string());
    std::string id =
        "iBeacon:" + uuid + "-" +
        std::to_string(ibeacon.get_major()) + "-" +
        std::to_string(ibeacon.get_minor());

    int rssi_1m = ibeacon.get_signal_power();
    if (rssi_1m >= 0 || rssi_1m < -127) {
      rssi_1m = this->default_rssi_1m_;
    }

    const DeviceConfig *config = this->find_config_(id);
    if (config != nullptr && std::isfinite(config->rssi_1m)) {
      rssi_1m = static_cast<int>(lroundf(config->rssi_1m));
    }
    return {id, rssi_1m};
  }

  const std::string mac = this->compact_mac_(device);
  const std::string name = this->lower_(device.get_name());

  // Common inexpensive iTag/iTrack tags normally advertise a recognizable name.
  if (name.find("itag") != std::string::npos ||
      name.find("i-tag") != std::string::npos ||
      name.find("itrack") != std::string::npos) {
    const std::string id = "itag:" + mac;
    int rssi_1m = this->itag_rssi_1m_;
    const DeviceConfig *config = this->find_config_(id);
    if (config != nullptr && std::isfinite(config->rssi_1m)) {
      rssi_1m = static_cast<int>(lroundf(config->rssi_1m));
    }
    return {id, rssi_1m};
  }

  int rssi_1m = this->default_rssi_1m_;
  const DeviceConfig *config = this->find_config_(mac);
  if (config != nullptr && std::isfinite(config->rssi_1m)) {
    rssi_1m = static_cast<int>(lroundf(config->rssi_1m));
  }
  return {mac, rssi_1m};
}

ESPresenseBridge::DeviceConfig *ESPresenseBridge::find_config_(
    const std::string &fingerprint) {
  for (auto &config : this->configs_) {
    if (config.fingerprint == fingerprint) {
      return &config;
    }
  }
  return nullptr;
}

const ESPresenseBridge::DeviceConfig *ESPresenseBridge::find_config_(
    const std::string &fingerprint) const {
  for (const auto &config : this->configs_) {
    if (config.fingerprint == fingerprint) {
      return &config;
    }
  }
  return nullptr;
}

ESPresenseBridge::DeviceState *ESPresenseBridge::find_state_(
    const std::string &fingerprint) {
  for (auto &state : this->devices_) {
    if (state.fingerprint == fingerprint) {
      return &state;
    }
  }
  return nullptr;
}

ESPresenseBridge::DeviceState *ESPresenseBridge::create_or_recycle_state_(
    const std::string &fingerprint) {
  if (this->devices_.size() < this->max_devices_) {
    this->devices_.emplace_back();
    this->devices_.back().fingerprint = fingerprint;
    return &this->devices_.back();
  }

  if (this->devices_.empty()) {
    return nullptr;
  }

  auto oldest = std::min_element(
      this->devices_.begin(), this->devices_.end(),
      [](const DeviceState &a, const DeviceState &b) {
        return a.last_seen < b.last_seen;
      });
  *oldest = DeviceState{};
  oldest->fingerprint = fingerprint;
  return &(*oldest);
}

bool ESPresenseBridge::publish_device_(DeviceState &state, uint32_t now) {
  if (state.last_seen == 0 || now - state.last_seen > this->forget_interval_ms_) {
    return false;
  }

  const DeviceConfig *config = this->find_config_(state.fingerprint);
  std::string publish_id = state.fingerprint;
  std::string publish_name = state.advertised_name;
  int rssi_1m = state.rssi_1m;

  if (config != nullptr) {
    if (!config->id.empty()) {
      publish_id = config->id;
    }
    if (!config->name.empty()) {
      publish_name = config->name;
    }
    if (std::isfinite(config->rssi_1m)) {
      rssi_1m = static_cast<int>(lroundf(config->rssi_1m));
    }
  }

  const float adjusted_rssi =
      state.filtered_rssi + static_cast<float>(this->rx_adj_rssi_);
  float distance = powf(
      10.0f,
      (static_cast<float>(rssi_1m) - adjusted_rssi) /
          (10.0f * this->absorption_));

  if (!std::isfinite(distance)) {
    return false;
  }

  // max_distance == 0 means "no distance limit".
  if (this->max_distance_ > 0.0f && distance > this->max_distance_) {
    return false;
  }

  const std::string topic =
      this->mqtt_prefix_ + "/devices/" +
      this->safe_topic_level_(publish_id) + "/" + this->room_id_;

  const std::string mac = state.mac;
  const float filtered_rssi = this->round_2_(state.filtered_rssi);
  distance = this->round_2_(distance);
  const float rssi_variance = this->round_2_(state.rssi_variance);
  const float distance_variance = this->round_2_(state.distance_variance);
  const uint32_t interval = state.advertisement_interval;
  const int rx_adj = this->rx_adj_rssi_;

  const bool ok = this->publish_json(
      topic,
      [mac, publish_id, publish_name, rssi_1m, filtered_rssi, rx_adj,
       rssi_variance, distance, distance_variance, interval](JsonObject root) {
        root["mac"] = mac;
        root["id"] = publish_id;
        if (!publish_name.empty()) {
          root["name"] = publish_name;
        }
        root["rssi@1m"] = rssi_1m;
        root["rssi"] = filtered_rssi;
        root["rxAdj"] = rx_adj;
        root["rssiVar"] = rssi_variance;
        root["distance"] = distance;
        root["var"] = distance_variance;
        root["int"] = interval;
      },
      0, false);

  if (ok) {
    this->reported_++;
  }
  return ok;
}

void ESPresenseBridge::publish_room_settings_() {
  const std::string base = this->room_base_();
  this->publish(base + "name", this->room_name_, 1, true);
  this->publish(
      base + "max_distance",
      this->format_float_(this->max_distance_, 2), 1, true);
  this->publish(
      base + "absorption",
      this->format_float_(this->absorption_, 2), 1, true);
  this->publish(
      base + "tx_ref_rssi",
      std::to_string(this->tx_ref_rssi_), 1, true);
  this->publish(
      base + "rx_adj_rssi",
      std::to_string(this->rx_adj_rssi_), 1, true);
}

void ESPresenseBridge::publish_telemetry_() {
  std::string ip;
  int wifi_rssi = -127;

#ifdef USE_WIFI
  if (wifi::global_wifi_component != nullptr) {
    wifi_rssi = wifi::global_wifi_component->wifi_rssi();
    const auto addresses = wifi::global_wifi_component->get_ip_addresses();
    for (const auto &address : addresses) {
      if (address.is_set()) {
        char buffer[network::IP_ADDRESS_BUFFER_SIZE];
        ip = address.str_to(buffer);
        break;
      }
    }
  }
#endif

  uint32_t free_heap = 0;
  uint32_t max_heap = 0;
  uint32_t loop_stack = 0;

#ifdef USE_ESP32
  free_heap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  max_heap = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  loop_stack =
      uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t);
#endif

  uint32_t active_fingerprints = 0;
  const uint32_t now = millis();
  for (const auto &state : this->devices_) {
    if (state.last_seen != 0 &&
        now - state.last_seen <= this->forget_interval_ms_) {
      active_fingerprints++;
    }
  }

  const std::string version =
      this->firmware_version_.empty()
          ? std::string(ESPHOME_VERSION)
          : this->firmware_version_;

  const std::string topic = this->room_base_() + "telemetry";
  const std::string firmware = this->firmware_name_;
  const uint32_t uptime = millis() / 1000U;
  const uint32_t adverts = this->adverts_;
  const uint32_t seen = this->seen_;
  const uint32_t reported = this->reported_;

  this->publish_json(
      topic,
      [ip, uptime, firmware, wifi_rssi, version, adverts, seen, reported,
       free_heap, max_heap, active_fingerprints, loop_stack](JsonObject root) {
        root["ip"] = ip;
        root["uptime"] = uptime;
        root["firm"] = firmware;
        root["rssi"] = wifi_rssi;
        root["ver"] = version;
        root["adverts"] = adverts;
        root["seen"] = seen;
        root["reported"] = reported;
        root["freeHeap"] = free_heap;
        root["maxHeap"] = max_heap;
        root["fingerprints"] = active_fingerprints;

        // ESPHome does not expose stable handles for the scanner and BLE host tasks.
        root["scanStack"] = 0;
        root["loopStack"] = loop_stack;
        root["bleStack"] = 0;
      },
      0, false);
}

void ESPresenseBridge::cleanup_devices_(uint32_t now) {
  auto it = this->devices_.begin();
  while (it != this->devices_.end()) {
    if (it->last_seen != 0 &&
        now - it->last_seen > this->forget_interval_ms_) {
      it = this->devices_.erase(it);
    } else {
      ++it;
    }
  }
}

void ESPresenseBridge::mark_devices_dirty_() {
  for (auto &state : this->devices_) {
    state.dirty = true;
  }
}

void ESPresenseBridge::upsert_device_config_(
    const std::string &fingerprint, const std::string &payload) {
  DeviceConfig parsed;
  parsed.fingerprint = fingerprint;

  const bool valid = json::parse_json(
      payload,
      [&parsed](JsonObject root) -> bool {
        if (!root["id"].isNull()) {
          parsed.id = root["id"].as<std::string>();
        }
        if (!root["name"].isNull()) {
          parsed.name = root["name"].as<std::string>();
        }

        const char *rssi_keys[] = {
            "rssi@1m", "rssi1m", "refRssi", "ref_rssi"};
        for (const char *key : rssi_keys) {
          if (!root[key].isNull()) {
            parsed.rssi_1m = root[key].as<float>();
            break;
          }
        }
        return true;
      });

  if (!valid) {
    ESP_LOGW(TAG, "Invalid JSON for fingerprint %s", fingerprint.c_str());
    return;
  }

  parsed.has_irk = this->parse_irk_(fingerprint, parsed.irk);

  DeviceConfig *existing = this->find_config_(fingerprint);
  if (existing != nullptr) {
    *existing = parsed;
    ESP_LOGD(TAG, "Updated device config: %s -> %s",
             fingerprint.c_str(), parsed.id.c_str());
    return;
  }

  if (this->configs_.size() >= this->max_configs_) {
    ESP_LOGW(TAG, "Device config limit reached (%u)", this->max_configs_);
    return;
  }

  this->configs_.push_back(parsed);
  ESP_LOGD(TAG, "Added device config: %s -> %s",
           fingerprint.c_str(), parsed.id.c_str());
}

void ESPresenseBridge::remove_device_config_(
    const std::string &fingerprint) {
  auto it = std::remove_if(
      this->configs_.begin(), this->configs_.end(),
      [&fingerprint](const DeviceConfig &config) {
        return config.fingerprint == fingerprint;
      });
  this->configs_.erase(it, this->configs_.end());
}

std::string ESPresenseBridge::settings_base_() const {
  return this->mqtt_prefix_ + "/settings/";
}

std::string ESPresenseBridge::room_base_() const {
  return this->mqtt_prefix_ + "/rooms/" + this->room_id_ + "/";
}

std::string ESPresenseBridge::compact_mac_(
    const esp32_ble_tracker::ESPBTDevice &device) {
  std::string mac = device.address_str();
  mac.erase(
      std::remove(mac.begin(), mac.end(), ':'),
      mac.end());
  return lower_(mac);
}

std::string ESPresenseBridge::lower_(std::string value) {
  std::transform(
      value.begin(), value.end(), value.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

std::string ESPresenseBridge::safe_topic_level_(std::string value) {
  for (char &c : value) {
    if (c == '/' || c == '+' || c == '#') {
      c = '_';
    }
  }
  return value;
}

std::string ESPresenseBridge::format_float_(
    float value, uint8_t decimals) {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%.*f", decimals, value);
  return std::string(buffer);
}

bool ESPresenseBridge::parse_float_(
    std::string payload, float &value) {
  std::replace(payload.begin(), payload.end(), ',', '.');
  char *end = nullptr;
  value = strtof(payload.c_str(), &end);
  return end != payload.c_str() && *end == '\0' && std::isfinite(value);
}

bool ESPresenseBridge::parse_int_(
    std::string payload, int &value) {
  char *end = nullptr;
  const long parsed = strtol(payload.c_str(), &end, 10);
  if (end == payload.c_str() || *end != '\0' ||
      parsed < std::numeric_limits<int>::min() ||
      parsed > std::numeric_limits<int>::max()) {
    return false;
  }
  value = static_cast<int>(parsed);
  return true;
}

bool ESPresenseBridge::starts_with_(
    const std::string &value, const std::string &prefix) {
  return value.size() >= prefix.size() &&
         value.compare(0, prefix.size(), prefix) == 0;
}

bool ESPresenseBridge::ends_with_(
    const std::string &value, const std::string &suffix) {
  return value.size() >= suffix.size() &&
         value.compare(
             value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool ESPresenseBridge::parse_irk_(
    const std::string &fingerprint,
    std::array<uint8_t, 16> &irk) {
  if (!starts_with_(fingerprint, "irk:")) {
    return false;
  }

  std::string hex = fingerprint.substr(4);
  hex.erase(
      std::remove_if(
          hex.begin(), hex.end(),
          [](char c) { return c == ':' || c == '-' || c == ' '; }),
      hex.end());

  if (hex.size() != 32) {
    return false;
  }

  for (size_t i = 0; i < 16; i++) {
    const int high = hex_value_(hex[i * 2]);
    const int low = hex_value_(hex[i * 2 + 1]);
    if (high < 0 || low < 0) {
      return false;
    }
    irk[i] = static_cast<uint8_t>((high << 4) | low);
  }
  return true;
}

int ESPresenseBridge::hex_value_(char c) {
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  if (c >= 'a' && c <= 'f') {
    return c - 'a' + 10;
  }
  return -1;
}

float ESPresenseBridge::round_2_(float value) {
  return roundf(value * 100.0f) / 100.0f;
}

}  // namespace espresense_bridge
}  // namespace esphome
