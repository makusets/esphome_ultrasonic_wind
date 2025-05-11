#include "ultrasonic_wind.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ultrasonic_wind {

static const char *const TAG = "ultrasonic_wind";



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
  write_register(0x14, 0x00);  // REG_DEV_CTRL_3

  // Enable echo interrupt (OUT4) output on TUSS4470 and set threshold level, done using SPI comms
  write_register(0x17, 0x11);  // 
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


  // Get TUSS4470 ready for the ultrasonic burst by writing to the TOF_CONFIG register, uses SPI comms
  write_register(0x1B, 0x01);  // REG_TOF_CONFIG, CMD_TRIGGER_ON
  delayMicroseconds(10);  // allow start
  //Start the timer
  this->burst_start_time_us_ = micros();
  // Trigger the burst by toggling the burst pin (IO2) 8 times, each time with a 13us low and 12us high pulse
  // This generates a 40kHz signal on the burst pin (IO2) for the TUSS4470 and will start the ultrasonic burst in OUTA and OUTB

  ESP_LOGI(TAG, "Starting clock");
  for (int i = 0; i < 8; i++) {
    this->burst_pin_->digital_write(false);
    delayMicroseconds(13);
    this->burst_pin_->digital_write(true);
    delayMicroseconds(12);
  }
  //Reset the TUSS4470 as per datasheet
  write_register(0x1B, 0x00);  // REG_TOF_CONFIG, CMD_TRIGGER_OFF

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

//calculate the odd parity bit for a 16-bit word before sending it to the TUSS4470 through SPI
// The odd parity bit is set to 1 if the number of 1 bits in the word is even, and 0 if it is odd
uint8_t calculate_odd_parity(uint16_t word) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < 16; i++) {
    count += (word >> i) & 0x1;
  }
  return count % 2 == 0 ? 1 : 0;  // Return 1 if even (to make it odd)
}

// write register to TUSS4470 using SPI
void UltrasonicWindSensor::write_register(uint8_t reg, uint8_t value) {
  // Build command byte: RW(1 bit) + Addr(6 bits) + Parity(1 bit)
  uint8_t command = (0 << 7) | ((reg & 0x3F) << 1);  // Write = 0
  uint16_t frame = (command << 8) | value;

  // Set parity bit (bit 8)
  if (calculate_odd_parity(frame)) {
    frame |= (1 << 8);
  }

  this->spi_dev_->enable();
  this->spi_dev_->transfer16(frame);
  this->spi_dev_->disable();
}

// read register from TUSS4470 using SPI
uint8_t UltrasonicWindSensor::read_register(uint8_t reg) {
  // Build command byte: RW = 1
  uint8_t command = (1 << 7) | ((reg & 0x3F) << 1);
  uint16_t frame = (command << 8);

  // Set parity bit
  if (calculate_odd_parity(frame)) {
    frame |= (1 << 8);
  }

  this->spi_dev_->enable();
  uint16_t result = this->spi_dev_->transfer16(frame);
  this->spi_dev_->disable();

  return result & 0xFF;  // Only data byte matters
}


}  // namespace ultrasonic_wind
}  // namespace esphome
