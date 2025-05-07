// SPDX-License-Identifier: MIT
#include "ultrasonic_wind.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ultrasonic_wind {

static const char *const TAG = "ultrasonic_wind";

// Register definitions (verify actual values from the full TUSS4470 register map)
#define REG_DEV_CTRL_3      0x15  // IO_MODE control
#define REG_TOF_CONFIG      0x22  // CMD_TRIGGER
#define REG_RESULT_TOF_LSB  0x2E  // ToF LSB result
#define REG_RESULT_TOF_MSB  0x2F  // ToF MSB result
#define CMD_TRIGGER         0x01  // Trigger bit

void UltrasonicWindSensor::setup() {
  ESP_LOGI(TAG, "Setting up TUSS4470 ultrasonic wind sensor...");

  // Setup SPI
  this->spi_dev_->setup();
  delay(10);

  // Configure TUSS4470 to IO_MODE = 0 (controlled by SPI + clock via IO2)
  write_register(REG_DEV_CTRL_3, 0x00);

  // Configure ESP32 pin (connected to IO2) to output 40kHz clock bursts
  pinMode(this->burst_pin_, OUTPUT);
  digitalWrite(this->burst_pin_, HIGH);  // Datasheet: IO2 high before burst
}

void UltrasonicWindSensor::update() {
  ESP_LOGD(TAG, "Triggering burst and reading ToF...");

  // Ensure IO2 is high before triggering burst
  digitalWrite(this->burst_pin_, HIGH);
  delayMicroseconds(10);

  // Trigger the burst via SPI (CMD_TRIGGER = 1)
  write_register(REG_TOF_CONFIG, CMD_TRIGGER);

  // Generate 8-cycle 40kHz burst on IO2 pin
  const uint8_t pulse_count = 8;
  const int pulse_period_us = 25;  // 40kHz = 25us period
  for (uint8_t i = 0; i < pulse_count; i++) {
    digitalWrite(this->burst_pin_, LOW);
    delayMicroseconds(pulse_period_us / 2);
    digitalWrite(this->burst_pin_, HIGH);
    delayMicroseconds(pulse_period_us / 2);
  }

  // Wait for ToF result to settle
  delay(2);

  // Read ToF result (assuming 2-byte register)
  uint8_t lsb = read_register(REG_RESULT_TOF_LSB);
  uint8_t msb = read_register(REG_RESULT_TOF_MSB);
  uint16_t tof_raw = (msb << 8) | lsb;

  // Convert ToF to microseconds (62.5ns tick size from datasheet)
  float tof_us = tof_raw * 0.0625f;
  float distance_cm = (tof_us / 1e6f) * 34300.0f / 2.0f;  // One-way

  ESP_LOGD(TAG, "ToF raw: %u, µs: %.2f, cm: %.2f", tof_raw, tof_us, distance_cm);

  float wind_speed = calculate_wind_speed_from_distance(distance_cm);
  float wind_direction = 90.0f;  // Placeholder

  if (this->wind_speed_sensor_ != nullptr)
    this->wind_speed_sensor_->publish_state(wind_speed);

  if (this->wind_direction_sensor_ != nullptr)
    this->wind_direction_sensor_->publish_state(wind_direction);
}

float UltrasonicWindSensor::calculate_wind_speed_from_distance(float distance_cm) {
  // TODO: Implement actual ToF-differential wind speed algorithm
  return distance_cm * 0.1f;  // Placeholder scaling
}

void UltrasonicWindSensor::write_register(uint8_t reg, uint8_t value) {
  this->spi_dev_->enable();
  this->spi_dev_->transfer_byte(reg & 0x7F);  // MSB = 0 for write
  this->spi_dev_->transfer_byte(value);
  this->spi_dev_->disable();
}

uint8_t UltrasonicWindSensor::read_register(uint8_t reg) {
  this->spi_dev_->enable();
  this->spi_dev_->transfer_byte(0x80 | reg);  // MSB = 1 for read
  uint8_t result = this->spi_dev_->transfer_byte(0x00);
  this->spi_dev_->disable();
  return result;
}

}  // namespace ultrasonic_wind
}  // namespace esphome