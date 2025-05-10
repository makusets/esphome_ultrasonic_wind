#include "ultrasonic_wind.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ultrasonic_wind {

static const char *const TAG = "ultrasonic_wind";

#define REG_TOF_CONFIG 0x22
#define CMD_TRIGGER 0x01

void UltrasonicWindSensor::setup() {
  ESP_LOGI(TAG, "Setting up ultrasonic wind sensor...");
  this->spi_setup();
  this->burst_pin_->setup();
  this->burst_pin_->digital_write(true);
  this->interrupt_pin_->setup();
  this->interrupt_pin_->attach_interrupt(UltrasonicWindSensor::gpio_interrupt_handler, this, gpio::INTERRUPT_RISING_EDGE);
}

void IRAM_ATTR UltrasonicWindSensor::gpio_interrupt_handler(UltrasonicWindSensor *arg) {
  arg->tof_capture_time_us_ = micros();
  arg->tof_available_ = true;
}

void UltrasonicWindSensor::update() {
  this->tof_available_ = false;
  this->burst_start_time_us_ = micros();

  write_register(REG_TOF_CONFIG, CMD_TRIGGER);

  for (int i = 0; i < 8; i++) {
    this->burst_pin_->digital_write(false);
    delayMicroseconds(13);
    this->burst_pin_->digital_write(true);
    delayMicroseconds(12);
  }

  for (int i = 0; i < 50; i++) {
    if (this->tof_available_) break;
    delayMicroseconds(100);
  }

  if (!this->tof_available_) {
    ESP_LOGW(TAG, "No echo received");
    return;
  }

  uint32_t tof_us = this->tof_capture_time_us_ - this->burst_start_time_us_;

  float wind_speed = calculate_wind_speed_from_tof();
  float wind_direction = 90.0f;

  if (this->wind_speed_sensor_ != nullptr)
    this->wind_speed_sensor_->publish_state(wind_speed);

  if (this->wind_direction_sensor_ != nullptr)
    this->wind_direction_sensor_->publish_state(wind_direction);
}

float UltrasonicWindSensor::calculate_speed_of_sound() {
  float T = (temp_sensor_ && temp_sensor_->has_state()) ? temp_sensor_->state : 20.0f;
  float RH = (hum_sensor_ && hum_sensor_->has_state()) ? hum_sensor_->state : 50.0f;
  return 331.3f + 0.606f * T + 0.0124f * RH;
}

float UltrasonicWindSensor::calculate_wind_speed_from_tof() {
  float t = (tof_capture_time_us_ - burst_start_time_us_) / 1e6f;
  float d = sensor_distance_mm_ / 1000.0f;
  float sos = calculate_speed_of_sound();
  return clamp((d / t) - sos, -20.0f, 20.0f) * 3.6f;
}

void UltrasonicWindSensor::write_register(uint8_t reg, uint8_t value) {
  this->enable();
  this->transfer_byte(reg & 0x7F);
  this->transfer_byte(value);
  this->disable();
}

uint8_t UltrasonicWindSensor::read_register(uint8_t reg) {
  this->enable();
  this->transfer_byte(0x80 | reg);
  uint8_t value = this->transfer_byte(0x00);
  this->disable();
  return value;
}

}  // namespace ultrasonic_wind
}  // namespace esphome
