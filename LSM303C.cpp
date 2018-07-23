#include "LSM303C.h"

LSM303C::LSM303C(int sda, int scl) {
  my_sda = sda;
  my_scl = scl;

  my_acc_mg_lsb = 0.000244F; //0.244 mg per lsb @ 8g
  my_mag_gauss_lsb_xy = 1724.0F;     // Varies with gain
  my_mag_gauss_lsb_z  = 1724.0F;     // Varies with gain

  // preset min, max values for magnetometer
  // must be calibrated for each sensor!
  my_mag_min = {-924, -891, -1264};
  my_mag_max = {578, 675, 385};
}

void LSM303C::set_mag_min_max(LSM303C_raw mag_min, LSM303C_raw mag_max) {
  my_mag_min = mag_min;
  my_mag_max = mag_max;
}

byte LSM303C::read8(byte addr, byte reg) {
  byte value;

  Wire.beginTransmission(addr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom(addr, (byte)1);

  value = Wire.read();
  Wire.endTransmission();

  return value;
}

void LSM303C::write8(byte addr, byte reg, byte value) {
  Wire.beginTransmission(addr);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)value);
  Wire.endTransmission();
}

LSM303C_sensor LSM303C::read(byte addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x28);
  Wire.endTransmission();
  Wire.requestFrom(addr, (byte)6);
  while (Wire.available() < 6); // wait for data

  uint8_t xlo = Wire.read();
  uint8_t xhi = Wire.read();
  uint8_t ylo = Wire.read();
  uint8_t yhi = Wire.read();
  uint8_t zlo = Wire.read();
  uint8_t zhi = Wire.read();

  LSM303C_sensor value;
  value.raw.x = (int16_t)(xlo | (xhi << 8));
  value.raw.y = (int16_t)(ylo | (yhi << 8));
  value.raw.z = (int16_t)(zlo | (zhi << 8));

  return value;
}

bool LSM303C::begin() {
  Wire.begin(my_sda, my_scl);
  Wire.setClock(400000);

  uint8_t id_acc, id_mag;
  byte ok = 1;

  id_acc = read8(LSM303C_ADDR_ACC, 0x0F);
  if (id_acc != LSM303C_ID_ACC) {
    Serial.println("error initializing accelerometer!");
    ok = false;
  }

  id_mag = read8(LSM303C_ADDR_MAG, 0x0F);
  if (id_mag != LSM303C_ID_MAG) {
    Serial.println("error initializing magnetometer!");
    ok = false;
  }

  if (ok) {
    // enable accelerometer
    write8(LSM303C_ADDR_ACC, 0x20, 0xBF); //enable acc 100Hz
    write8(LSM303C_ADDR_ACC, 0x23, 0x34); //+-8g full scale

    // enable magnetometer
    write8(LSM303C_ADDR_MAG, 0x20, 0x5c);
    write8(LSM303C_ADDR_MAG, 0x21, 0x60);
    write8(LSM303C_ADDR_MAG, 0x22, 0x00);
    write8(LSM303C_ADDR_MAG, 0x23, 0x08);
    write8(LSM303C_ADDR_MAG, 0x24, 0x40);
  }

  return ok;
}

LSM303C_sensor LSM303C::read_acc() {
  LSM303C_sensor v = read(LSM303C_ADDR_ACC);

  v.x = v.raw.x * my_acc_mg_lsb * GRAVITY;
  v.y = v.raw.y * my_acc_mg_lsb * GRAVITY;
  v.z = v.raw.z * my_acc_mg_lsb * GRAVITY;

  return v;
}

LSM303C_sensor LSM303C::read_mag() {
  LSM303C_sensor v = read(LSM303C_ADDR_MAG);

  v.x = v.raw.x / my_mag_gauss_lsb_xy * GAUSS2MICROTESLA;
  v.y = v.raw.y / my_mag_gauss_lsb_xy * GAUSS2MICROTESLA;
  v.z = v.raw.z / my_mag_gauss_lsb_z * GAUSS2MICROTESLA;

  return v;
}

double LSM303C::get_heading() {
    LSM303C_sensor m = read_mag();
    LSM303C_sensor a = read_acc();

    //Serial.printf("m(x, y, z): (%i, %i, %i)\n", m.raw.x, m.raw.y, m.raw.z);
    //Serial.printf("a(x, y, z): (%i, %i, %i)\n", a.raw.x, a.raw.y, a.raw.z);

    // use calibration values to shift and scale magnetometer measurements
    double x_mag = (0.0+m.raw.x-my_mag_min.x)/(my_mag_max.x-my_mag_min.x)*2-1;
    double y_mag = (0.0+m.raw.y-my_mag_min.y)/(my_mag_max.y-my_mag_min.y)*2-1;
    double z_mag = (0.0+m.raw.z-my_mag_min.z)/(my_mag_max.z-my_mag_min.z)*2-1;
    //Serial.printf("Mag norm (x, y, z): (%f, %f, %f)\n", x_mag, y_mag, z_mag);

    // Normalize acceleration measurements so they range from 0 to 1
    double s = sqrt(pow(a.raw.x,2) + pow(a.raw.y,2) + pow(a.raw.z,2));
    double xAccelNorm = a.raw.x/s;
    double yAccelNorm = a.raw.y/s;
    //DF("Acc norm (x, y): (%f, %f)\n", xAccelNorm, yAccelNorm);

    double pitch = asin(-xAccelNorm);
    double roll = asin(yAccelNorm/cos(pitch));

    // tilt compensated magnetic sensor measurements
    double x_mag_comp = x_mag*cos(pitch)+z_mag*sin(pitch);
    double y_mag_comp = x_mag*sin(roll)*sin(pitch)+y_mag*cos(roll)-z_mag*sin(roll)*cos(pitch);

    // arctangent of y/x converted to degrees
    double heading = 180*atan2(y_mag_comp, x_mag_comp)/PI;

    if (heading <= 0) {
      heading = -heading;
    } else {
      heading = 360 -heading;
    }

    return heading;
}

/*
 * get factor to correct accelerometer
 * use afterwards: acc.x * factor
 *
 */
double LSM303C::calibrate_acc() {
  Serial.println("Do not move sensor while calibration in progress!");
  LSM303C_sensor a = read_acc();
  double g_not_calibrated = sqrt(pow(a.x,2)+pow(a.y,2)+pow(a.z,2));
  double factor = GRAVITY/g_not_calibrated;

  Serial.println("Calibration done.");
  return factor;
}

void LSM303C::calibrate_mag() {
  Serial.println("Move sensor in all directions until max/min value do not change anymore");

  LSM303C_raw mag_min = {9999, 9999, 9999};
  LSM303C_raw mag_max = {-9999, -9999, -9999};
  LSM303C_sensor m, m_last;
  long now = millis();

  while(1) {
    m = read_mag();
    if (m.raw.x < mag_min.x) mag_min.x = m.raw.x;
    if (m.raw.x > mag_max.x) mag_max.x = m.raw.x;

    if (m.raw.y < mag_min.y) mag_min.y = m.raw.y;
    if (m.raw.y > mag_max.y) mag_max.y = m.raw.y;

    if (m.raw.z < mag_min.z) mag_min.z = m.raw.z;
    if (m.raw.z > mag_max.z) mag_max.z = m.raw.z;

    if (millis() - now > 1000) {
      Serial.printf("Mag mins (X, Y, Z): (%i, %i, %i)\n", mag_min.x, mag_min.y, mag_min.z);
      Serial.printf("Mag maxs (X, Y, Z): (%i, %i, %i)\n", mag_max.x, mag_max.y, mag_max.z);
      now = millis();
    }
  }
}
