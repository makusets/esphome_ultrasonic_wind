#include "ultrasonic_wind.h"
#include "esphome/core/log.h"
#include <bitset>

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

  // Enable regulator
  write_register(0x16, 0x00);  // VDRV_HI_Z = 0, level = 0x00 → 5V
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

  read_register(0x14);
  read_register(0x17);
  read_register(0x16);
  read_register(0x18);

    // Set IO_MODE = 0 on TUSS4470: external burst control via clock on IO2 (burst_pin), done using SPI comms
  write_register(0x14, 0x00);  // REG_DEV_CTRL_3

  // Enable echo interrupt (OUT4) output on TUSS4470 and set threshold level, done using SPI comms
  write_register(0x17, 0x11);  // 

  // Enable regulator
  write_register(0x16, 0x00);  // VDRV_HI_Z = 0, level = 0x00 → 5V

  // Ensure the TUSS4470 driver voltage (VDRV) is charged and ready
  write_register(0x1B, 0x02);  // REG_TOF_CONFIG, VDRV_TRIGGER = 1
  delay(1);  // Small delay to allow VDRV regulator to begin charging
  // Check if VDRV is ready by reading DEV_STAT (0x1C), bit 3 = VDRV_READY
  if (!this->vdrv_ready_) {
    ESP_LOGW(TAG, "VDRV not ready");
    //return;  // Skip triggering if not ready
  }
    
  
  this->tof_available_ = false;


  // Get TUSS4470 ready for the ultrasonic burst by writing to the TOF_CONFIG register, uses SPI comms
  write_register(0x1B, 0x01);  // REG_TOF_CONFIG, CMD_TRIGGER_ON
  delayMicroseconds(10);  // allow start
  log_register(0x1B);
  //
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
static uint8_t calculate_odd_parity(uint16_t frame_without_parity) {
  uint16_t masked = frame_without_parity & ~(1 << 8);  // Mask out bit 8
  uint8_t count = 0;
  for (uint8_t i = 0; i < 16; i++) {
    count += (masked >> i) & 0x1;
  }
  return (count % 2 == 0) ? 1 : 0;  // Set parity to 1 if even (to force odd)
}

// write register to TUSS4470 using SPI
void UltrasonicWindSensor::write_register(uint8_t reg, uint8_t value) {
  // Bit 15: R/W = 0 (write), Bits 14–9: address, Bit 8: parity, Bits 7–0: data
  uint16_t frame = ((uint16_t)(reg & 0x3F) << 9) | value;

  if (calculate_odd_parity(frame)) {
    frame |= (1 << 8);  // Set parity bit
  }

  uint8_t high = frame >> 8;
  uint8_t low = frame & 0xFF;

  this->enable();
  uint8_t resp_hi = this->transfer_byte(high);
  uint8_t resp_lo = this->transfer_byte(low);
  this->disable();

  uint16_t response = (resp_hi << 8) | resp_lo;

  if (response != 0x0000) {
    ESP_LOGW(TAG, "Unexpected response 0x%04X during write to reg 0x%02X", response, reg);
  }
  if (response & (1 << 15)) {
    ESP_LOGW(TAG, "SPI error during write (bit 15 set)");
  }
  if (response & (1 << 14)) {
    ESP_LOGW(TAG, "Invalid register address (bit 14 set)");
  }

  // ESP_LOGD(TAG, "Wrote 0x%02X to reg 0x%02X (response = 0x%04X)", value, reg, response);
}
uint8_t UltrasonicWindSensor::read_register(uint8_t reg) {
  uint8_t cmd[2];
  cmd[0] = ((reg & 0x3F) << 2) | 0x01;  // R/W = 1
  cmd[1] = 0x00;

  if (calculate_odd_parity(cmd[0], cmd[1])) {
    cmd[0] |= (1 << 1);  // set parity bit
  }

  this->enable();
  uint8_t resp_hi = this->transfer_byte(cmd[0]);
  uint8_t resp_lo = this->transfer_byte(cmd[1]);
  this->disable();

  uint16_t response = (resp_hi << 8) | resp_lo;
  uint8_t value = response & 0xFF;

  // Decode individual status flags
  this->tuss4470_spi_error_ = (response >> 15) & 0x01;
  this->vdrv_ready_ =        (response >> 14) & 0x01;
  this->pulse_num_fault_ =   (response >> 13) & 0x01;
  this->driver_fault_ =      (response >> 12) & 0x01;
  this->eeprom_crc_fault_ =  (response >> 11) & 0x01;
  this->dev_state_ =         (response >> 9)  & 0x03;  // bits 10–9

  // Optional logging
  if (this->tuss4470_spi_error_)      ESP_LOGW(TAG, "Previous SPI parity error detected");
  if (this->pulse_num_fault_)         ESP_LOGW(TAG, "Pulse number fault (burst ended early)");
  if (this->driver_fault_)            ESP_LOGW(TAG, "Driver fault detected");
  if (this->eeprom_crc_fault_)        ESP_LOGW(TAG, "EEPROM CRC fault");
  if (this->vdrv_ready_)              ESP_LOGD(TAG, "VDRV regulator is ready");

  switch (this->dev_state_) {
    case 0x00: ESP_LOGD(TAG, "DEV_STATE: LISTEN"); break;
    case 0x01: ESP_LOGD(TAG, "DEV_STATE: BURST"); break;
    case 0x02: ESP_LOGD(TAG, "DEV_STATE: STANDBY"); break;
    case 0x03: ESP_LOGD(TAG, "DEV_STATE: SLEEP"); break;
  }

  ESP_LOGD(TAG, "Read reg 0x%02X = 0x%02X (raw response = 0x%04X)", reg, value, response);
  return value;
}
//log register read
void UltrasonicWindSensor::log_register(uint8_t reg) {
  uint8_t val = this->read_register(reg);
  ESP_LOGD(TAG, "TUSS4470 Reg[0x%02X] = 0x%02X (0b%s)",
           reg, val, std::bitset<8>(val).to_string().c_str());
}




}  // namespace ultrasonic_wind
}  // namespace esphome
