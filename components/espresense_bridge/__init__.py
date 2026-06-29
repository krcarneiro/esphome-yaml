import esphome.codegen as cg
from esphome.components import esp32_ble_tracker
import esphome.config_validation as cv
from esphome.const import CONF_ID

DEPENDENCIES = ["esp32_ble_tracker", "mqtt", "wifi"]
AUTO_LOAD = ["json"]

CONF_MQTT_PREFIX = "mqtt_prefix"
CONF_ROOM_ID = "room_id"
CONF_ROOM_NAME = "room_name"
CONF_FIRMWARE_NAME = "firmware_name"
CONF_FIRMWARE_VERSION = "firmware_version"
CONF_MAX_DEVICES = "max_devices"
CONF_MAX_CONFIGS = "max_configs"
CONF_MAX_DISTANCE = "max_distance"
CONF_ABSORPTION = "absorption"
CONF_TX_REF_RSSI = "tx_ref_rssi"
CONF_RX_ADJ_RSSI = "rx_adj_rssi"
CONF_DEFAULT_RSSI_1M = "default_rssi_1m"
CONF_ITAG_RSSI_1M = "itag_rssi_1m"
CONF_RSSI_ALPHA = "rssi_alpha"
CONF_PUBLISH_INTERVAL = "publish_interval"
CONF_TELEMETRY_INTERVAL = "telemetry_interval"
CONF_FORGET_INTERVAL = "forget_interval"
CONF_CONNECT_GRACE = "connect_grace"
CONF_MAX_PUBLISHES_PER_LOOP = "max_publishes_per_loop"

espresense_bridge_ns = cg.esphome_ns.namespace("espresense_bridge")
ESPresenseBridge = espresense_bridge_ns.class_(
    "ESPresenseBridge",
    cg.Component,
    esp32_ble_tracker.ESPBTDeviceListener,
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ESPresenseBridge),
            cv.Optional(CONF_MQTT_PREFIX, default="espresense"): cv.string_strict,
            cv.Required(CONF_ROOM_ID): cv.string_strict,
            cv.Optional(CONF_ROOM_NAME): cv.string_strict,
            cv.Optional(CONF_FIRMWARE_NAME, default="esphome-espresense-bridge"): cv.string_strict,
            cv.Optional(CONF_FIRMWARE_VERSION, default=""): cv.string_strict,
            cv.Optional(CONF_MAX_DEVICES, default=64): cv.int_range(min=1, max=256),
            cv.Optional(CONF_MAX_CONFIGS, default=128): cv.int_range(min=1, max=512),
            cv.Optional(CONF_MAX_DISTANCE, default=16.0): cv.float_range(min=0.0, max=1000.0),
            cv.Optional(CONF_ABSORPTION, default=2.7): cv.float_range(min=0.1, max=10.0),
            cv.Optional(CONF_TX_REF_RSSI, default=-59): cv.int_range(min=-127, max=0),
            cv.Optional(CONF_RX_ADJ_RSSI, default=0): cv.int_range(min=-100, max=100),
            cv.Optional(CONF_DEFAULT_RSSI_1M, default=-65): cv.int_range(min=-127, max=0),
            cv.Optional(CONF_ITAG_RSSI_1M, default=-75): cv.int_range(min=-127, max=0),
            cv.Optional(CONF_RSSI_ALPHA, default=0.25): cv.float_range(
                min=0.01, max=1.0
            ),
            cv.Optional(CONF_PUBLISH_INTERVAL, default="1s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_TELEMETRY_INTERVAL, default="30s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_FORGET_INTERVAL, default="150s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_CONNECT_GRACE, default="2s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_MAX_PUBLISHES_PER_LOOP, default=2): cv.int_range(
                min=1, max=20
            ),
        }
    )
    .extend(esp32_ble_tracker.ESP_BLE_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await esp32_ble_tracker.register_ble_device(var, config)

    room_name = config.get(CONF_ROOM_NAME, config[CONF_ROOM_ID])

    cg.add(var.set_mqtt_prefix(config[CONF_MQTT_PREFIX]))
    cg.add(var.set_room_id(config[CONF_ROOM_ID]))
    cg.add(var.set_room_name(room_name))
    cg.add(var.set_firmware_name(config[CONF_FIRMWARE_NAME]))
    cg.add(var.set_firmware_version(config[CONF_FIRMWARE_VERSION]))
    cg.add(var.set_max_devices(config[CONF_MAX_DEVICES]))
    cg.add(var.set_max_configs(config[CONF_MAX_CONFIGS]))
    cg.add(var.set_max_distance(config[CONF_MAX_DISTANCE]))
    cg.add(var.set_absorption(config[CONF_ABSORPTION]))
    cg.add(var.set_tx_ref_rssi(config[CONF_TX_REF_RSSI]))
    cg.add(var.set_rx_adj_rssi(config[CONF_RX_ADJ_RSSI]))
    cg.add(var.set_default_rssi_1m(config[CONF_DEFAULT_RSSI_1M]))
    cg.add(var.set_itag_rssi_1m(config[CONF_ITAG_RSSI_1M]))
    cg.add(var.set_rssi_alpha(config[CONF_RSSI_ALPHA]))
    cg.add(
        var.set_publish_interval(config[CONF_PUBLISH_INTERVAL].total_milliseconds)
    )
    cg.add(
        var.set_telemetry_interval(
            config[CONF_TELEMETRY_INTERVAL].total_milliseconds
        )
    )
    cg.add(var.set_forget_interval(config[CONF_FORGET_INTERVAL].total_milliseconds))
    cg.add(var.set_connect_grace(config[CONF_CONNECT_GRACE].total_milliseconds))
    cg.add(var.set_max_publishes_per_loop(config[CONF_MAX_PUBLISHES_PER_LOOP]))
