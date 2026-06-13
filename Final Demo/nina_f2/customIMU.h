/*
  customIMU.h
  ---------------------------------------------------------------------------
  On-board BMI270 + BMM150 IMU for the Arduino Nano 33 BLE Sense Rev2,
  re-configured to run the accelerometer and gyroscope FASTER than the
  Arduino_BMI270_BMM150 library's hard-coded default of ~100 Hz.

  WHY THIS IS NEEDED
  ---------------------------------------------------------------------------
  Inside the official Arduino_BMI270_BMM150 library, BoschSensorClass::begin()
  calls a protected virtual function, configure_sensor(), which hard-codes:
      sens_cfg[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
      sens_cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
  That's the "fixed at 99.84 Hz" rate you've been stuck at.

  HOW THIS FIXES IT
  ---------------------------------------------------------------------------
  configure_sensor() is declared 'virtual' specifically so it can be
  overridden. This file defines CustomIMUClass, a subclass of
  BoschSensorClass that does the exact same setup but with a higher ODR,
  then creates a global object and points the name "IMU" at it via a
  macro - so every existing IMU.begin() / accelerationAvailable() /
  readAcceleration() / gyroscopeAvailable() / readGyroscope() call in your
  .ino keeps working completely unchanged.

  REQUIREMENTS
  ---------------------------------------------------------------------------
  - Install the official "Arduino_BMI270_BMM150" library (Library Manager).
  - Save this file as customIMU.h next to your .ino.
  - Your existing #include "customIMU.h" + IMU.* calls need no changes.
  ---------------------------------------------------------------------------
*/

#ifndef CUSTOM_IMU_H
#define CUSTOM_IMU_H

#include <Arduino_BMI270_BMM150.h>

// ============================================================================
// 1) PICK YOUR OUTPUT DATA RATE (ODR) HERE
// ----------------------------------------------------------------------------
// With filter_perf = high-performance mode (used below, same as the stock
// library), valid options are:
//
//   Accelerometer            Gyroscope
//   BMI2_ACC_ODR_100HZ        BMI2_GYR_ODR_100HZ   <- library default
//   BMI2_ACC_ODR_200HZ        BMI2_GYR_ODR_200HZ
//   BMI2_ACC_ODR_400HZ        BMI2_GYR_ODR_400HZ   <- good match for a
//   BMI2_ACC_ODR_800HZ        BMI2_GYR_ODR_800HZ      ~5 ms (200 Hz) loop
//   BMI2_ACC_ODR_1600HZ       BMI2_GYR_ODR_1600HZ  <- accel max
//                             BMI2_GYR_ODR_3200HZ  <- gyro max (no accel match)
//
// Keeping accel and gyro at the same rate means a fresh accel sample and a
// fresh gyro sample become available together.
// ============================================================================
#define CUSTOM_IMU_ACC_ODR   BMI2_ACC_ODR_400HZ
#define CUSTOM_IMU_GYR_ODR   BMI2_GYR_ODR_400HZ

// ============================================================================
// 2) I2C SPEED FOR THE INTERNAL IMU BUS (Wire1)
// ----------------------------------------------------------------------------
// Wire1 (the internal bus the BMI270 sits on) defaults to 100 kHz. Reading
// the status register plus 12 bytes of accel+gyro data takes roughly
// ~1-1.5 ms at 100 kHz. That's fine at 100/200 Hz, but starts to eat into
// your loop budget once new samples arrive every 1.25-2.5 ms (800/400 Hz).
// Bumping to 400 kHz "fast mode" gives comfortable headroom up to 800 Hz
// and is usually fine at 1600 Hz too. Set to false to leave it at 100 kHz.
// ============================================================================
#define CUSTOM_IMU_FAST_I2C  true

// ----------------------------------------------------------------------------
// The stock header (BoschSensorClass.h) ends with:
//     extern BoschSensorClass IMU_BMI270_BMM150;
//     #define IMU IMU_BMI270_BMM150
// We undo that #define so we can repoint "IMU" at our own object below,
// without colliding with the library's own IMU_BMI270_BMM150 instance.
// ----------------------------------------------------------------------------
#undef IMU

class CustomIMUClass : public BoschSensorClass {
  public:
    CustomIMUClass(TwoWire &wire = Wire1) : BoschSensorClass(wire) {}

    // Same signature as the base class begin(), but bumps the I2C clock
    // (if enabled) once the sensor + bus have been initialized.
    int begin(CfgBoshSensor_t cfg = BOSCH_ACCEL_AND_MAGN) {
      int ok = BoschSensorClass::begin(cfg);
#if CUSTOM_IMU_FAST_I2C
      Wire1.setClock(400000);
#endif
      return ok;
    }

  protected:
    // Identical to the stock BoschSensorClass::configure_sensor(), except
    // acc.odr / gyr.odr now come from the macros above instead of being
    // hard-coded to 100 Hz.
    int8_t configure_sensor(struct bmi2_dev *dev) override {

      int8_t rslt;
      uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };

      struct bmi2_int_pin_config int_pin_cfg;
      int_pin_cfg.pin_type = BMI2_INT1;
      int_pin_cfg.int_latch = BMI2_INT_NON_LATCH;
      int_pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
      int_pin_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
      int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
      int_pin_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;

      struct bmi2_sens_config sens_cfg[2];

      sens_cfg[0].type = BMI2_ACCEL;
      sens_cfg[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
      sens_cfg[0].cfg.acc.odr = CUSTOM_IMU_ACC_ODR;
      sens_cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
      sens_cfg[0].cfg.acc.range = BMI2_ACC_RANGE_4G;

      sens_cfg[1].type = BMI2_GYRO;
      sens_cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
      sens_cfg[1].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
      sens_cfg[1].cfg.gyr.odr = CUSTOM_IMU_GYR_ODR;
      sens_cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
      sens_cfg[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

      rslt = bmi2_set_int_pin_config(&int_pin_cfg, dev);
      if (rslt != BMI2_OK) return rslt;

      rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, dev);
      if (rslt != BMI2_OK) return rslt;

      rslt = bmi2_set_sensor_config(sens_cfg, 2, dev);
      if (rslt != BMI2_OK) return rslt;

      rslt = bmi2_sensor_enable(sens_list, 2, dev);
      return rslt;
    }
};

// ----------------------------------------------------------------------------
// Global instance + alias, so "IMU.begin()", "IMU.accelerationAvailable()",
// "IMU.readAcceleration()", "IMU.gyroscopeAvailable()",
// "IMU.readGyroscope()", etc. all keep working exactly as before.
// ----------------------------------------------------------------------------
CustomIMUClass CustomIMU(Wire1);
#define IMU CustomIMU

#endif // CUSTOM_IMU_H