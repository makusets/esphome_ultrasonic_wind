print("Registering ultrasonic_wind platform...")
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, spi, gpio
from esphome.const import (
    CONF_ID, CONF_UPDATE_INTERVAL,
    UNIT_KILOMETER_PER_HOUR, UNIT_DEGREES,
    ICON_WEATHER_WINDY, DEVICE_CLASS_EMPTY
)
#from esphome.core.gpio import output_pin_schema, input_pin_schema

CONF_WIND_SPEED = "wind_speed"
CONF_WIND_DIRECTION = "wind_direction"
CONF_BURST_PIN = "burst_pin"
CONF_TOF_INTERRUPT_PIN = "tof_interrupt_pin"
CONF_BME280_ID = "bme280_id"
CONF_SENSOR_DISTANCE = "sensor_distance"

ultrasonic_wind_ns = cg.esphome_ns.namespace("ultrasonic_wind")
UltrasonicWindSensor = ultrasonic_wind_ns.class_(
    "UltrasonicWindSensor", cg.PollingComponent, spi.SPIDevice
)
print("Registering ultrasonic_wind platform...")
CONFIG_SCHEMA = (
    sensor.sensor_platform_schema(UltrasonicWindSensor)
    .extend({
        cv.Required(CONF_WIND_SPEED): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOMETER_PER_HOUR,
            icon="mdi:weather-windy",
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_EMPTY,
        ),
        cv.Required(CONF_WIND_DIRECTION): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES,
            icon="mdi:compass",
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_EMPTY,
        ),
#        cv.Optional(CONF_BURST_PIN, default=33): output_pin_schema,
#        cv.Optional(CONF_TOF_INTERRUPT_PIN, default=14): input_pin_schema,
        cv.Optional(CONF_BME280_ID): cv.use_id(cg.Component),
        cv.Optional(CONF_SENSOR_DISTANCE, default=200.0): cv.float_range(min=10.0, max=1000.0),
    })
    .extend(cv.polling_component_schema("5s"))
    .extend(spi.spi_device_schema())
)
print("Registering ultrasonic_wind platform...")
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)
    await sensor.register_sensor(var, config)

    wind_speed = await sensor.new_sensor(config[CONF_WIND_SPEED])
    cg.add(var.set_wind_speed_sensor(wind_speed))

    wind_direction = await sensor.new_sensor(config[CONF_WIND_DIRECTION])
    cg.add(var.set_wind_direction_sensor(wind_direction))

    burst_pin = await cg.gpio_pin_expression(config[CONF_BURST_PIN])
    cg.add(var.set_burst_pin(burst_pin))

    interrupt_pin = await cg.gpio_pin_expression(config[CONF_TOF_INTERRUPT_PIN])
    cg.add(var.set_interrupt_pin(interrupt_pin))

    cg.add(var.set_sensor_distance_mm(config[CONF_SENSOR_DISTANCE]))

    if CONF_BME280_ID in config:
        bme = await cg.get_variable(config[CONF_BME280_ID])
        cg.add(var.set_bme280_sensor(bme))

