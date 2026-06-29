#pragma once

#include <array>
#include <cstdint>
#include <cmath>
#include <string>
#include <vector>

#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include "esphome/components/mqtt/custom_mqtt_device.h"
#include "esphome/core/component.h"

namespace esphome {
namespace espresense_bridge {

class ESPresenseBridge : public Component,
                         public esp32_ble_tracker::ESPBTDeviceListener,
                         public mqtt::CustomMQTTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  bool parse_device(const esp32_ble_tracker::ESPBTDevice &device) override;

  void set_mqtt_prefix(const std::string &value) { this->mqtt_prefix_ = value; }
  void set_room_id(const std::string &value) { this->room_id_ = value; }
  void set_room_name(const std::string &value) { this->room_name_ = value; }
  void set_firmware_name(const std::string &value) { this->firmware_name_ = value; }
  void set_firmware_version(const std::string &value) { this->firmware_version_ = value; }

  void set_max_devices(uint16_t value) { this->max_devices_ = value; }
  void set_max_configs(uint16_t value) { this->max_configs_ = value; }
  void set_max_distance(float value) { this->max_distance_ = value; }
  void set_absorption(float value) { this->absorption_ = value; }
  void set_tx_ref_rssi(int value) { this->tx_ref_rssi_ = value; }
  void set_rx_adj_rssi(int value) { this->rx_adj_rssi_ = value; }
  void set_default_rssi_1m(int value) { this->default_rssi_1m_ = value; }
  void set_itag_rssi_1m(int value) { this->itag_rssi_1m_ = value; }
  void set_rssi_alpha(float value) { this->rssi_alpha_ = value; }

  void set_publish_interval(uint32_t value) { this->publish_interval_ms_ = value; }
  void set_telemetry_interval(uint32_t value) { this->telemetry_interval_ms_ = value; }
  void set_forget_interval(uint32_t value) { this->forget_interval_ms_ = value; }
  void set_connect_grace(uint32_t value) { this->connect_grace_ms_ = value; }
  void set_max_publishes_per_loop(uint8_t value) {
    this->max_publishes_per_loop_ = value;
  }

 protected:
  struct DeviceConfig {
    std::string fingerprint;
    std::string id;
    std::string name;
    float rssi_1m{NAN};
    bool has_irk{false};
    std::array<uint8_t, 16> irk{};
  };

  struct DeviceState {
    std::string fingerprint;
    std::string mac;
    std::string advertised_name;

    float filtered_rssi{-100.0f};
    float rssi_variance{0.0f};
    float distance{0.0f};
    float distance_variance{0.0f};
    int rssi_1m{-65};

    uint32_t samples{0};
    uint32_t last_seen{0};
    uint32_t last_advertisement{0};
    uint32_t advertisement_interval{0};
    uint32_t last_publish{0};
    bool dirty{false};
  };

  struct FingerprintResult {
    std::string fingerprint;
    int rssi_1m{-65};
  };

  void on_device_setting_(const std::string &topic, const std::string &payload);
  void on_room_setting_(const std::string &topic, const std::string &payload);

  FingerprintResult fingerprint_(const esp32_ble_tracker::ESPBTDevice &device);
  DeviceConfig *find_config_(const std::string &fingerprint);
  const DeviceConfig *find_config_(const std::string &fingerprint) const;
  DeviceState *find_state_(const std::string &fingerprint);
  DeviceState *create_or_recycle_state_(const std::string &fingerprint);

  bool publish_device_(DeviceState &state, uint32_t now);
  void publish_room_settings_();
  void publish_telemetry_();
  void cleanup_devices_(uint32_t now);

  void mark_devices_dirty_();
  void upsert_device_config_(const std::string &fingerprint, const std::string &payload);
  void remove_device_config_(const std::string &fingerprint);

  std::string settings_base_() const;
  std::string room_base_() const;

  static std::string compact_mac_(const esp32_ble_tracker::ESPBTDevice &device);
  static std::string lower_(std::string value);
  static std::string safe_topic_level_(std::string value);
  static std::string format_float_(float value, uint8_t decimals);
  static bool parse_float_(std::string payload, float &value);
  static bool parse_int_(std::string payload, int &value);
  static bool starts_with_(const std::string &value, const std::string &prefix);
  static bool ends_with_(const std::string &value, const std::string &suffix);
  static bool parse_irk_(const std::string &fingerprint, std::array<uint8_t, 16> &irk);
  static int hex_value_(char c);
  static float round_2_(float value);

  std::string mqtt_prefix_{"espresense"};
  std::string room_id_;
  std::string room_name_;
  std::string firmware_name_{"esphome-espresense-bridge"};
  std::string firmware_version_;

  uint16_t max_devices_{64};
  uint16_t max_configs_{128};
  float max_distance_{16.0f};
  float absorption_{2.7f};
  int tx_ref_rssi_{-59};
  int rx_adj_rssi_{0};
  int default_rssi_1m_{-65};
  int itag_rssi_1m_{-75};
  float rssi_alpha_{0.25f};

  uint32_t publish_interval_ms_{1000};
  uint32_t telemetry_interval_ms_{30000};
  uint32_t forget_interval_ms_{150000};
  uint32_t connect_grace_ms_{2000};
  uint8_t max_publishes_per_loop_{2};

  std::vector<DeviceConfig> configs_;
  std::vector<DeviceState> devices_;

  uint32_t adverts_{0};
  uint32_t seen_{0};
  uint32_t reported_{0};

  bool mqtt_was_connected_{false};
  bool announced_after_connect_{false};
  bool room_settings_dirty_{false};
  uint32_t connected_at_{0};
  uint32_t last_telemetry_{0};
};

}  // namespace espresense_bridge
}  // namespace esphome
