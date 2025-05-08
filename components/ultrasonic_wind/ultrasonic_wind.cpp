
#include "ultrasonic_wind.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ultrasonic_wind {

static const char *const TAG = "ultrasonic_wind";

// Register addresses (TUSS4470)
#define REG_TOF_CONFIG 0x22
#define CMD_TRIGGER 0x01

void UltrasonicWindSensor::setup() {
  ESP_LOGI(TAG, "Setting up TUSS4470 ultrasonic wind sensor...");

  this->spi_dev_->setup();
  delay(10);

  // Set up burst pin
  pinMode(this->burst_pin_->get_pin(), OUTPUT);
  digitalWrite(this->burst_pin_->get_pin(), HIGH);

  // Set up interrupt pin
  pinMode(this->interrupt_pin_->get_pin(), INPUT);
  attachInterruptArg(this->interrupt_pin_->get_pin(),
                     reinterpret_cast<interrupt_callback_t>(&UltrasonicWindSensor::gpio_interrupt_handler),
                     this, RISING);
}

void IRAM_ATTR UltrasonicWindSensor::gpio_interrupt_handler(UltrasonicWindSensor *arg) {
  arg->tof_capture_time_us_ = micros();
  arg->tof_available_ = true;
}

void UltrasonicWindSensor::update() {
  this->tof_available_ = false;
  this->burst_start_time_us_ = micros();

  // Trigger burst via SPI
  write_register(REG_TOF_CONFIG, CMD_TRIGGER);

  // Send 8-cycle 40 kHz burst on burst pin
  for (int i = 0; i < 8; i++) {
    digitalWrite(this->burst_pin_->get_pin(), LOW);
    delayMicroseconds(13);
    digitalWrite(this->burst_pin_->get_pin(), HIGH);
    delayMicroseconds(12);
  }

  // Wait for OUT4 interrupt
  for (int i = 0; i < 50; i++) {
    if (this->tof_available_) break;
    delayMicroseconds(100);
  }

  if (!this->tof_available_) {
    ESP_LOGW(TAG, "No echo received");
    return;
  }

  uint32_t tof_us = this->tof_capture_time_us_ - this->burst_start_time_us_;

  // Log BME280 values
  if (bme280_ != nullptr) {
    ESP_LOGD(TAG, "BME280: T=%.2f°C H=%.2f%% P=%.2f hPa",
             bme280_->get_temperature(), bme280_->get_humidity(), bme280_->get_pressure());
  }

  float wind_speed = calculate_wind_speed_from_distance(0);  // 0 unused
  float wind_direction = 90.0f;

  if (this->wind_speed_sensor_ != nullptr)
    this->wind_speed_sensor_->publish_state(wind_speed);

  if (this->wind_direction_sensor_ != nullptr)
    this->wind_direction_sensor_->publish_state(wind_direction);
}

float UltrasonicWindSensor::calculate_speed_of_sound() {
  float T = (bme280_ != nullptr) ? bme280_->get_temperature() : 20.0f;
  float RH = (bme280_ != nullptr) ? bme280_->get_humidity() : 50.0f;
  return 331.3f + 0.606f * T + 0.0124f * RH;
}

float UltrasonicWindSensor::calculate_wind_speed_from_distance(float) {
  float t = (this->tof_capture_time_us_ - this->burst_start_time_us_) / 1e6f;
  float d = this->sensor_distance_mm_ / 1000.0f;
  float sos = calculate_speed_of_sound();

  float v = (d / t) - sos;

  if (v < -20.0f) v = -20.0f;
  if (v > 20.0f) v = 20.0f;

  return v * 3.6f;  // Convert to km/h
}

void UltrasonicWindSensor::write_register(uint8_t reg, uint8_t value) {
  this->spi_dev_->enable();
  this->spi_dev_->transfer_byte(reg & 0x7F);
  this->spi_dev_->transfer_byte(value);
  this->spi_dev_->disable();
}

uint8_t UltrasonicWindSensor::read_register(uint8_t reg) {
  this->spi_dev_->enable();
  this->spi_dev_->transfer_byte(0x80 | reg);
  uint8_t value = this->spi_dev_->transfer_byte(0x00);
  this->spi_dev_->disable();
  return value;
}

}  // namespace ultrasonic_wind
}  // namespace esphome
