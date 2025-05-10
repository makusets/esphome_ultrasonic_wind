import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import sensor, spi
from esphome.const import (
    CONF_ID,
    UNIT_KILOMETER_PER_HOUR,
    UNIT_DEGREES,
    DEVICE_CLASS_EMPTY,
)

CONF_BURST_PIN = "burst_pin"
CONF_TOF_INTERRUPT_PIN = "tof_interrupt_pin"
CONF_SENSOR_DISTANCE = "sensor_distance"
CONF_WIND_SPEED = "wind_speed"
CONF_WIND_DIRECTION = "wind_direction"
CONF_CS_PIN = "cs_pin"
CONF_TEMPERATURE_SENSOR = "temperature_sensor"
CONF_HUMIDITY_SENSOR = "humidity_sensor"

ultrasonic_wind_ns = cg.esphome_ns.namespace("ultrasonic_wind")
UltrasonicWindSensor = ultrasonic_wind_ns.class_(
    "UltrasonicWindSensor", cg.PollingComponent, spi.SPIDevice
)

CONFIG_SCHEMA = (
    cv.Schema({
        cv.GenerateID(): cv.declare_id(UltrasonicWindSensor),
        cv.Required(CONF_WIND_SPEED): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOMETER_PER_HOUR,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_EMPTY,
            icon="mdi:weather-windy",
        ),
        cv.Required(CONF_WIND_DIRECTION): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_EMPTY,
            icon="mdi:compass",
        ),
        cv.Optional(CONF_BURST_PIN, default="GPIO33"): pins.gpio_output_pin_schema,
        cv.Optional(CONF_TOF_INTERRUPT_PIN, default="GPIO14"): pins.gpio_input_pin_schema,
        cv.Optional(CONF_SENSOR_DISTANCE, default=200.0): cv.float_range(min=10.0, max=1000.0),
        cv.Optional(CONF_CS_PIN, default="GPIO5"): pins.gpio_output_pin_schema,
        cv.Optional(CONF_TEMPERATURE_SENSOR): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_HUMIDITY_SENSOR): cv.use_id(sensor.Sensor),
    })
    .extend(cv.polling_component_schema("5s"))
    .extend(spi.spi_device_schema())
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)

    burst_pin = await cg.gpio_pin_expression(config[CONF_BURST_PIN])
    cg.add(var.set_burst_pin(burst_pin))

    interrupt_pin = await cg.gpio_pin_expression(config[CONF_TOF_INTERRUPT_PIN])
    cg.add(var.set_interrupt_pin(interrupt_pin))

    if CONF_TEMPERATURE_SENSOR in config:
        temp = await cg.get_variable(config[CONF_TEMPERATURE_SENSOR])
        cg.add(var.set_temp_sensor(temp))

    if CONF_HUMIDITY_SENSOR in config:
        hum = await cg.get_variable(config[CONF_HUMIDITY_SENSOR])
        cg.add(var.set_hum_sensor(hum))

    cg.add(var.set_sensor_distance_mm(config[CONF_SENSOR_DISTANCE]))

    wind_speed = await sensor.new_sensor(config[CONF_WIND_SPEED])
    cg.add(var.set_wind_speed_sensor(wind_speed))

    wind_direction = await sensor.new_sensor(config[CONF_WIND_DIRECTION])
    cg.add(var.set_wind_direction_sensor(wind_direction))
