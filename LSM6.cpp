#include "LSM6.h"
#include <Wire.h>
#include <math.h>

#define DS33_SA0_HIGH_ADDRESS 0b1101011
#define DS33_SA0_LOW_ADDRESS  0b1101010
#define DSO_SA0_HIGH_ADDRESS  0b1101011
#define DSO_SA0_LOW_ADDRESS   0b1101010

#define TEST_REG_ERROR -1
#define DS33_WHO_ID    0x69
#define DSO_WHO_ID     0x6C

LSM6::LSM6()
  : bus(&Wire), _device(device_auto)
{
}

bool LSM6::init(deviceType device, sa0State sa0)
{
  if (device == device_auto || sa0 == sa0_auto)
  {
    if (device == device_auto || device == device_DS33)
    {
      if (sa0 != sa0_low && testReg(DS33_SA0_HIGH_ADDRESS, LSM6::WHO_AM_I) == DS33_WHO_ID)
      {
        sa0 = sa0_high;
        device = device_DS33;
      }
      else if (sa0 != sa0_high && testReg(DS33_SA0_LOW_ADDRESS, LSM6::WHO_AM_I) == DS33_WHO_ID)
      {
        sa0 = sa0_low;
        device = device_DS33;
      }
    }

    if (device == device_auto || device == device_DSO)
    {
      if (sa0 != sa0_low && testReg(DSO_SA0_HIGH_ADDRESS, LSM6::WHO_AM_I) == DSO_WHO_ID)
      {
        sa0 = sa0_high;
        device = device_DSO;
      }
      else if (sa0 != sa0_high && testReg(DSO_SA0_LOW_ADDRESS, LSM6::WHO_AM_I) == DSO_WHO_ID)
      {
        sa0 = sa0_low;
        device = device_DSO;
      }
    }

    if (device == device_auto || sa0 == sa0_auto)
    {
      return false;
    }
  }

  _device = device;

  switch (device)
  {
    case device_DS33:
      address = (sa0 == sa0_high) ? DS33_SA0_HIGH_ADDRESS : DS33_SA0_LOW_ADDRESS;
      break;
    case device_DSO:
      address = (sa0 == sa0_high) ? DSO_SA0_HIGH_ADDRESS : DSO_SA0_LOW_ADDRESS;
      break;
    default:
      return false;
  }

  return true;
}

void LSM6::enableDefault()
{
  if (_device == device_DS33 || _device == device_DSO)
  {
    writeReg(LSM6::CTRL1_XL, 0x80);
    writeReg(LSM6::CTRL2_G, 0x80);
    writeReg(LSM6::CTRL3_C, 0x04);
  }
}

void LSM6::writeReg(uint8_t reg, uint8_t value)
{
  bus->beginTransmission(address);
  bus->write(reg);
  bus->write(value);
  last_status = bus->endTransmission();
}

uint8_t LSM6::readReg(uint8_t reg)
{
  bus->beginTransmission(address);
  bus->write(reg);
  last_status = bus->endTransmission();

  bus->requestFrom(address, (uint8_t)1);
  return bus->read();
}

void LSM6::readAcc()
{
  bus->beginTransmission(address);
  bus->write(LSM6::OUTX_L_XL);
  bus->endTransmission();

  bus->requestFrom(address, (uint8_t)6);
  uint8_t xla = bus->read();
  uint8_t xha = bus->read();
  uint8_t yla = bus->read();
  uint8_t yha = bus->read();
  uint8_t zla = bus->read();
  uint8_t zha = bus->read();

  a.x = (int16_t)(xha << 8 | xla);
  a.y = (int16_t)(yha << 8 | yla);
  a.z = (int16_t)(zha << 8 | zla);
}

void LSM6::readGyro()
{
  bus->beginTransmission(address);
  bus->write(LSM6::OUTX_L_G);
  bus->endTransmission();

  bus->requestFrom(address, (uint8_t)6);
  uint8_t xlg = bus->read();
  uint8_t xhg = bus->read();
  uint8_t ylg = bus->read();
  uint8_t yhg = bus->read();
  uint8_t zlg = bus->read();
  uint8_t zhg = bus->read();

  g.x = (int16_t)(xhg << 8 | xlg);
  g.y = (int16_t)(yhg << 8 | ylg);
  g.z = (int16_t)(zhg << 8 | zlg);
}

void LSM6::read()
{
  readAcc();
  readGyro();
}

void LSM6::vector_normalize(vector<float> *a)
{
  float mag = sqrt(vector_dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

int16_t LSM6::testReg(uint8_t addr, regAddr reg)
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

