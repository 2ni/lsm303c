#ifndef __LSM303C__
#define __LSM303C__

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// I2C addresses
#define LSM303C_ADDR_ACC 0x1D
#define LSM303C_ADDR_MAG 0x1E

// Chip ID's
#define LSM303C_ID_ACC 0X41
#define LSM303C_ID_MAG 0X3D

// Various
#define GRAVITY (9.80665F)
#define GAUSS2MICROTESLA (100)

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} LSM303C_raw;

typedef struct {
  LSM303C_raw raw;
  float x;
  float y;
  float z;
} LSM303C_sensor;

class LSM303C {
  private:
    byte read8(byte addr, byte reg);
    void write8(byte addr, byte reg, byte value);
    LSM303C_sensor read(byte addr);

    float my_acc_mg_lsb;
    float my_mag_gauss_lsb_xy;
    float my_mag_gauss_lsb_z;
    int my_sda;
    int my_scl;
    LSM303C_raw my_mag_min;
    LSM303C_raw my_mag_max;

  public:
    LSM303C(int sda, int scl);
    bool begin(void);
    LSM303C_sensor read_acc(void);
    LSM303C_sensor read_mag(void);
    double get_heading();
    void set_mag_min_max(LSM303C_raw mag_min, LSM303C_raw mag_max);
    double calibrate_acc(void);
    void calibrate_mag(void);
};

#endif
