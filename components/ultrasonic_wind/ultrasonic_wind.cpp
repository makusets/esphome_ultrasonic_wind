#include "ultrasonic_wind.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ultrasonic_wind {

static const char *const TAG = "ultrasonic_wind";

#define REG_TOF_CONFIG 0x1B
#define CMD_TRIGGER 0x03

void UltrasonicWindSensor::setup() {
  ESP_LOGI(TAG, "Setting up ultrasonic wind sensor...");
  this->spi_setup();
  delay(10);
  // Configure burst pin for IO2 output (used to generate 40kHz falling edges)
  this->burst_pin_->setup();
  this->burst_pin_->digital_write(true);  // Set HIGH before CMD_TRIGGER (per datasheet)
  // Set up GPIO interrupt for OUT4 (used to capture echo time)
  this->interrupt_pin_->setup();
  this->interrupt_pin_->attach_interrupt(UltrasonicWindSensor::gpio_interrupt_handler, this, gpio::INTERRUPT_RISING_EDGE);

  // Set IO_MODE = 0 on TUSS4470: external burst control via clock on IO2 (burst_pin), done using SPI comms
  write_register(0x15, 0x00);  // REG_DEV_CTRL_3

  // Enable echo interrupt (OUT4) output on TUSS4470 and set threshold level, done using SPI comms

  // 1. Enable ECHO_INT_EN (bit 4 of REG_DEV_CTRL_2, assumed address 0x14)
  uint8_t ctrl2 = read_register(0x14);
  ctrl2 |= (1 << 4);  // Set ECHO_INT_EN
  write_register(0x14, ctrl2);

  // 2. Set threshold to mid-high value (~0.8 V if VDD = 3.3V)
  write_register(0x17, 0x0A);  // ECHO_INT_THR_SEL = 0x0A
}

void IRAM_ATTR UltrasonicWindSensor::gpio_interrupt_handler(UltrasonicWindSensor *arg) {
  arg->tof_capture_time_us_ = micros();
  arg->tof_available_ = true;
}

/***************************************************************
 Main loop function to trigger the ultrasonic burst and capture the echo time
 This function is called periodically by the ESPHome framework
 The component is a sensor, so it gets called depending on update_interval set in the YAML config
****************************************************************/
void UltrasonicWindSensor::update() {
  this->tof_available_ = false;


  // Trigger the ultrasonic burst by writing to the TOF_CONFIG register, uses SPI comms
  write_register(REG_TOF_CONFIG, CMD_TRIGGER);
  //Start the timer
  this->burst_start_time_us_ = micros();
  // Start the burst by toggling the burst pin (IO2) 8 times, each time with a 13us low and 12us high pulse
  // This generates a 40kHz signal on the burst pin (IO2) for the TUSS4470

  ESP_LOGI(TAG, "Starting clock");
  for (int i = 0; i < 8; i++) {
    this->burst_pin_->digital_write(false);
    delayMicroseconds(13);
    this->burst_pin_->digital_write(true);
    delayMicroseconds(12);
  }
  // Wait for the echo to be captured
  // The interrupt handler will set tof_available_ to true when the echo is received
  for (int i = 0; i < 50; i++) {
    if (this->tof_available_) break;
    delayMicroseconds(100);
  }
  // If the echo is not received within 5ms, assume no echo was received and quit the update function
  if (!this->tof_available_) {
    ESP_LOGW(TAG, "No echo received");
    return;
  }
  // If the echo is received, start the calculations

  // Calculate the time of flight (TOF) in microseconds
  uint32_t tof_us = this->tof_capture_time_us_ - this->burst_start_time_us_;

  // Calculate the wind speed and direction using the TOF and the distance to the sensor
  float wind_speed = calculate_wind_speed_from_tof();
  float wind_direction = 90.0f;  // Placeholder for wind direction calculation, currently set to 90 degrees (east)

  ESP_LOGI(TAG, "Wind speed: %.2f km/h, Wind direction: %.2f degrees", wind_speed, wind_direction);
  ESP_LOGI(TAG, "Temperature: %.2f °C, Humidity: %.2f %%", temp_sensor_ ? temp_sensor_->state : 0.0f, hum_sensor_ ? hum_sensor_->state : 0.0f);
 


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
