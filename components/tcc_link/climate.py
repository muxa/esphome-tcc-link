import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import climate, uart, binary_sensor, sensor, switch, text_sensor, template
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_HARDWARE_UART,
    CONF_BAUD_RATE,
    CONF_UPDATE_INTERVAL,
    CONF_MODE,
    CONF_FAN_MODE,
    CONF_SWING_MODE,
    CONF_TRIGGER_ID,
    DEVICE_CLASS_CONNECTIVITY,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
)

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["climate", "binary_sensor", "sensor", "switch"]
CODEOWNERS = ["@muxa"]

tcc_link_ns = cg.esphome_ns.namespace("tcc_link")

CONF_CONNECTED = "connected"
CONF_VENT = "vent"
CONF_FAILED_CRCS = "failed_crcs"

CONF_ON_DATA_RECEIVED = "on_data_received"

TccLinkClimate =  tcc_link_ns.class_(
    "TccLinkClimate", climate.Climate, uart.UARTDevice, cg.Component
)

TccLinkVentSwitch =  tcc_link_ns.class_(
    "TccLinkVentSwitch", switch.Switch, cg.Component
)

TccLinkOnDataReceivedTrigger = tcc_link_ns.class_(
    "TccLinkOnDataReceivedTrigger", automation.Trigger.template()
)

CONFIG_SCHEMA = climate.CLIMATE_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(TccLinkClimate),
        cv.Optional(CONF_CONNECTED): binary_sensor.binary_sensor_schema(
            device_class = DEVICE_CLASS_CONNECTIVITY,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_VENT): cv.maybe_simple_value(
            switch.SWITCH_SCHEMA.extend(
                cv.Schema(
                    {
                        cv.GenerateID(): cv.declare_id(TccLinkVentSwitch),
                    }
                )
            ),
            key=CONF_NAME,
        ),
        cv.Optional(CONF_FAILED_CRCS): sensor.sensor_schema(
            accuracy_decimals=0,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ON_DATA_RECEIVED): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    TccLinkOnDataReceivedTrigger
                ),
            }
        ),
    }
).extend(uart.UART_DEVICE_SCHEMA).extend(cv.COMPONENT_SCHEMA)

def validate_uart(config):
    uart.final_validate_device_schema(
        "tcc_link", baud_rate=2400, require_rx=True, require_tx=False
    )(config)


FINAL_VALIDATE_SCHEMA = validate_uart

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    await cg.register_component(var, config)
    await climate.register_climate(var, config)
    await uart.register_uart_device(var, config)

    if CONF_CONNECTED in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_CONNECTED])
        cg.add(var.set_connected_binary_sensor(sens))

    if CONF_FAILED_CRCS in config:
        sens = await sensor.new_sensor(config[CONF_FAILED_CRCS])
        cg.add(var.set_failed_crcs_sensor(sens))

    if CONF_VENT in config:
        sw = await switch.new_switch(config[CONF_VENT], var)
        cg.add(var.set_vent_switch(sw))

    if CONF_ON_DATA_RECEIVED in config:
        for on_data_received in config.get(CONF_ON_DATA_RECEIVED, []):
            data_trigger = cg.new_Pvariable(on_data_received[CONF_TRIGGER_ID], var)
            await automation.build_automation(
                data_trigger, [(cg.std_vector.template(cg.uint8), "x")], on_data_received
            )
