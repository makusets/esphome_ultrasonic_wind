import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, spi, gpio
from esphome.const import (
    CONF_ID,
    CONF_UPDATE_INTERVAL,
    CONF_WIND_SPEED,
    CONF_WIND_DIRECTION,
    UNIT_KILOMETER_PER_HOUR,
    UNIT_DEGREES,
    ICON_WEATHER_WINDY,
    ICON_COMPASS_OUTLINE,
    DEVICE_CLASS_EMPTY,
)

CONF_BURST_PIN = "burst_pin"

ultrasonic_wind_ns = cg.esphome_ns.namespace("ultrasonic_wind")
UltrasonicWindSensor = ultrasonic_wind_ns.class_("UltrasonicWindSensor", cg.PollingComponent, spi.SPIDevice)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(UltrasonicWindSensor),
            cv.Required(CONF_WIND_SPEED): sensor.sensor_schema(
                unit_of_measurement=UNIT_KILOMETER_PER_HOUR,
                icon=ICON_WEATHER_WINDY,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_EMPTY,
            ),
            cv.Required(CONF_WIND_DIRECTION): sensor.sensor_schema(
                unit_of_measurement=UNIT_DEGREES,
                icon=ICON_COMPASS_OUTLINE,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_EMPTY,
            ),
            cv.Optional(CONF_BURST_PIN, default=33): cv.gpio_output_pin_schema,
        }
    )
    .extend(cv.polling_component_schema("5s"))
    .extend(spi.spi_device_schema())
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)

    wind_speed = await sensor.new_sensor(config[CONF_WIND_SPEED])
    cg.add(var.set_wind_speed_sensor(wind_speed))

    wind_direction = await sensor.new_sensor(config[CONF_WIND_DIRECTION])
    cg.add(var.set_wind_direction_sensor(wind_direction))

    burst_pin = await cg.gpio_pin_expression(config[CONF_BURST_PIN])
    cg.add(var.set_burst_pin(burst_pin))
