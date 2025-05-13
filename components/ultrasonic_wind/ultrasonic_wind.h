#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/spi/spi.h"

namespace esphome {
namespace ultrasonic_wind {

class UltrasonicWindSensor : public PollingComponent,
                              public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST,
                                                    spi::CLOCK_POLARITY_LOW,
                                                    spi::CLOCK_PHASE_LEADING,
                                                    spi::DATA_RATE_200KHZ> {
 public:
  void setup() override;
  void update() override;

  void set_wind_speed_sensor(sensor::Sensor *sensor) { wind_speed_sensor_ = sensor; }
  void set_wind_direction_sensor(sensor::Sensor *sensor) { wind_direction_sensor_ = sensor; }
  void set_burst_pin(InternalGPIOPin *pin) { burst_pin_ = pin; }
  void set_interrupt_pin(InternalGPIOPin *pin) { interrupt_pin_ = pin; }
  void set_temp_sensor(sensor::Sensor *s) { temp_sensor_ = s; }
  void set_hum_sensor(sensor::Sensor *s) { hum_sensor_ = s; }
  void set_sensor_distance_mm(float distance) { sensor_distance_mm_ = distance; }

 protected:
  sensor::Sensor *wind_speed_sensor_{nullptr};
  sensor::Sensor *wind_direction_sensor_{nullptr};
  InternalGPIOPin *burst_pin_{nullptr};
  InternalGPIOPin *interrupt_pin_{nullptr};
  sensor::Sensor *temp_sensor_{nullptr};
  sensor::Sensor *hum_sensor_{nullptr};

  float sensor_distance_mm_{200.0f};

  uint32_t burst_start_time_us_{0};
  volatile bool tof_available_{false};
  volatile uint32_t tof_capture_time_us_{0};

  static void IRAM_ATTR gpio_interrupt_handler(UltrasonicWindSensor *arg);
//
  float calculate_wind_speed_from_tof();
  float calculate_speed_of_sound();
  void write_register(uint8_t reg, uint8_t value);
  uint8_t read_register(uint8_t reg);
  void log_register(uint8_t addr);
};

}  // namespace ultrasonic_wind
}  // namespace esphome
