#pragma once
#include <Arduino.h>
#include <Wire.h>
 
// ─────────────────────────────────────────────────────────────────────
//  CustomIMU – minimal BMI270 driver for Arduino Nano 33 BLE Sense Rev2
//
//  Drop-in replacement for Arduino_BMI270_BMM150.h.
//  Only implements the five functions your code actually uses.
//
//  One required step BEFORE THIS WILL COMPILE:
//    Copy  bmi270_config_file.h  into the same folder as this file.
//    Source: Arduino_BMI270_BMM150/src/BMI270-Sensor-API/bmi270_config_file.h
//    (or grab it from github.com/boschsensortec/BMI270_SensorAPI)
//
//  To switch your main .ino:
//    Replace:  #include "Arduino_BMI270_BMM150.h"
//    With:     #include "CustomIMU.h"
//    (everything else stays the same — IMU.begin(), IMU.readGyroscope(), etc.)
// ─────────────────────────────────────────────────────────────────────
 
class CustomIMU_Class {
public:
  // Returns true if the sensor was found, reset, configured, and passed
  // its internal self-test. Call once in setup().
  bool begin();
 
  // True when a fresh accelerometer sample is ready to be read.
  // Polls the STATUS register (bit 7).
  bool accelerationAvailable();
 
  // True when a fresh gyroscope sample is ready to be read.
  // Polls the STATUS register (bit 6).
  bool gyroscopeAvailable();
 
  // Reads acceleration in g  (±2 g range, 16-bit resolution).
  // x / y / z match the same axes as the original library.
  bool readAcceleration(float &x, float &y, float &z);
 
  // Reads angular rate in °/s  (±2000 °/s range, 16-bit resolution).
  bool readGyroscope(float &x, float &y, float &z);
 
  // ── ODR tweaking helpers (call before begin()) ─────────────────────
  // Default: 200 Hz accelerometer / 200 Hz gyroscope.
  // Valid odr values (Hz): 25 / 50 / 100 / 200 / 400
  void setAccelODR(uint16_t hz);   // change accelerometer output rate
  void setGyroODR(uint16_t hz);    // change gyroscope output rate
 
private:
  // ── BMI270 I2C address ─────────────────────────────────────────────
  // SDO pin LOW (default on Nano 33 BLE Sense Rev2) → 0x68
  // SDO pin HIGH                                    → 0x69
  static constexpr uint8_t ADDR = 0x68;
 
  // ── Register addresses (BMI270 datasheet, section 5.1) ────────────
  static constexpr uint8_t REG_CHIP_ID         = 0x00; // read → 0x24
  static constexpr uint8_t REG_STATUS          = 0x03; // bit7=drdy_acc, bit6=drdy_gyr
  static constexpr uint8_t REG_ACC_X_LSB       = 0x0C; // 6 bytes acc XYZ (little-endian)
  static constexpr uint8_t REG_GYR_X_LSB       = 0x12; // 6 bytes gyr XYZ (little-endian)
  static constexpr uint8_t REG_INTERNAL_STATUS = 0x21; // bits[3:0]==0x01 → init OK
  static constexpr uint8_t REG_ACC_CONF        = 0x40; // ODR + BWP + perf mode
  static constexpr uint8_t REG_ACC_RANGE       = 0x41; // 0x00=±2g … 0x03=±16g
  static constexpr uint8_t REG_GYR_CONF        = 0x42; // ODR + BWP + perf mode
  static constexpr uint8_t REG_GYR_RANGE       = 0x43; // 0x00=±2000°/s … 0x04=±125°/s
  static constexpr uint8_t REG_INIT_CTRL       = 0x59; // 0=halt init, 1=start init
  static constexpr uint8_t REG_INIT_ADDR_0     = 0x5B; // word-address bits [3:0]
  static constexpr uint8_t REG_INIT_ADDR_1     = 0x5C; // word-address bits [11:4]
  static constexpr uint8_t REG_INIT_DATA       = 0x5E; // burst-write config blob here
  static constexpr uint8_t REG_PWR_CONF        = 0x7C; // 0=active, 2=FIFO self-wake
  static constexpr uint8_t REG_PWR_CTRL        = 0x7D; // bit3=acc, bit2=gyr
  static constexpr uint8_t REG_CMD             = 0x7E; // 0xB6=soft reset
 
  // ── Scale factors (set in begin() from configured range) ──────────
  float _accelScale = 2.0f   / 32768.0f; // g per raw LSB
  float _gyroScale  = 2000.0f / 32768.0f; // °/s per raw LSB
 
  // ── Desired ODR byte values for ACC_CONF / GYR_CONF ───────────────
  uint8_t _accOdrByte = 0x09; // 200 Hz
  uint8_t _gyrOdrByte = 0x09; // 200 Hz
 
  // ── I2C primitives ────────────────────────────────────────────────
  void    writeReg (uint8_t reg, uint8_t val);
  uint8_t readReg  (uint8_t reg);
  void    readRegs (uint8_t reg, uint8_t *buf, uint8_t len);
 
  // ── BMI270 requires an 8 KB config blob to be uploaded at boot ────
  bool uploadConfigFile();
};
 
// Global singleton — keeps the same "IMU." name your code already uses.
extern CustomIMU_Class IMU;
 