#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/gpio/gpio.h"

namespace esphome {
namespace ultrasonic_wind {

class UltrasonicWindSensor : public PollingComponent,
                              public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST,
                                                    spi::CLOCK_POLARITY_LOW,
                                                    spi::CLOCK_PHASE_LEADING,
                                                    spi::DATA_RATE_1MHZ> {
 public:
  void setup() override;
  void update() override;

  void set_wind_speed_sensor(sensor::Sensor *sensor) { wind_speed_sensor_ = sensor; }
  void set_wind_direction_sensor(sensor::Sensor *sensor) { wind_direction_sensor_ = sensor; }
  void set_burst_pin(GPIOPin *pin) { burst_pin_ = pin; }

 protected:
  sensor::Sensor *wind_speed_sensor_{nullptr};
  sensor::Sensor *wind_direction_sensor_{nullptr};
  GPIOPin *burst_pin_{nullptr};  // GPIO pin for IO2 clock pulses

  float calculate_wind_speed_from_distance(float distance_cm);
  void write_register(uint8_t reg, uint8_t value);
  uint8_t read_register(uint8_t reg);
};

}  // namespace ultrasonic_wind
}  // namespace esphome
