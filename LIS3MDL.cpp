#include <LIS3MDL.h>
#include <Wire.h>
#include <math.h>

// Defines ////////////////////////////////////////////////////////////////

#define LIS3MDL_SA1_HIGH_ADDRESS  0b0011110
#define LIS3MDL_SA1_LOW_ADDRESS   0b0011100

#define TEST_REG_ERROR -1
#define LIS3MDL_WHO_ID  0x3D

// Constructors ////////////////////////////////////////////////////////////////

LIS3MDL::LIS3MDL(void)
  : bus(&Wire), _device(device_auto)
{
}

// Public Methods //////////////////////////////////////////////////////////////

bool LIS3MDL::init(deviceType device, sa1State sa1)
{
  if (device == device_auto || sa1 == sa1_auto)
  {
    if (device == device_auto || device == device_LIS3MDL)
    {
      if (sa1 != sa1_low && testReg(LIS3MDL_SA1_HIGH_ADDRESS, WHO_AM_I) == LIS3MDL_WHO_ID)
      {
        sa1 = sa1_high;
        if (device == device_auto) device = device_LIS3MDL;
      }
      else if (sa1 != sa1_high && testReg(LIS3MDL_SA1_LOW_ADDRESS, WHO_AM_I) == LIS3MDL_WHO_ID)
      {
        sa1 = sa1_low;
        if (device == device_auto) device = device_LIS3MDL;
      }
    }

    if (device == device_auto || sa1 == sa1_auto)
    {
      return false;
    }
  }

  _device = device;

  switch (device)
  {
    case device_LIS3MDL:
      address = (sa1 == sa1_high) ? LIS3MDL_SA1_HIGH_ADDRESS : LIS3MDL_SA1_LOW_ADDRESS;
      break;
  }

  return true;
}

void LIS3MDL::enableDefault(void)
{
  if (_device == device_LIS3MDL)
  {
    writeReg(CTRL_REG1, 0x70); // ultra-high-perf X/Y, 10Hz
    writeReg(CTRL_REG2, 0x00); // ±4 gauss
    writeReg(CTRL_REG3, 0x00); // continuous mode
    writeReg(CTRL_REG4, 0x0C); // ultra-high-perf Z
    writeReg(CTRL_REG5, 0x40); // block data update
  }
}

void LIS3MDL::writeReg(uint8_t reg, uint8_t value)
{
  bus->beginTransmission(address);
  bus->write(reg);
  bus->write(value);
  last_status = bus->endTransmission();
}

uint8_t LIS3MDL::readReg(uint8_t reg)
{
  uint8_t value;

  bus->beginTransmission(address);
  bus->write(reg);
  last_status = bus->endTransmission();

  bus->requestFrom(address, (uint8_t)1);
  value = bus->read();

  return value;
}

void LIS3MDL::read()
{
  bus->beginTransmission(address);
  bus->write(OUT_X_L | 0x80); // auto-increment
  bus->endTransmission();

  bus->requestFrom(address, (uint8_t)6);
  uint8_t xlm = bus->read();
  uint8_t xhm = bus->read();
  uint8_t ylm = bus->read();
  uint8_t yhm = bus->read();
  uint8_t zlm = bus->read();
  uint8_t zhm = bus->read();

  m.x = (int16_t)(xhm << 8 | xlm);
  m.y = (int16_t)(yhm << 8 | ylm);
  m.z = (int16_t)(zhm << 8 | zlm);
}

void LIS3MDL::vector_normalize(vector<float> *a)
{
  float mag = sqrt(vector_dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

int16_t LIS3MDL::testReg(uint8_t addr, regAddr reg)
{
  bus->beginTransmission(addr);
  bus->write((uint8_t)reg);
  if (bus->endTransmission() != 0)
    return TEST_REG_ERROR;

  bus->requestFrom(addr, (uint8_t)1);
  if (bus->available())
    return bus->read();
  else
    return TEST_REG_ERROR;
}


