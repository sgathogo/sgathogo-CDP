#ifndef LSM6_h
#define LSM6_h

#include <Arduino.h>
#include <Wire.h>

class LSM6
{
  public:
    template <typename T> struct vector
    {
      T x, y, z;
    };

    enum deviceType { device_DS33, device_DSO, device_auto };
    enum sa0State { sa0_low, sa0_high, sa0_auto };

    enum regAddr
    {
      FUNC_CFG_ACCESS = 0x01,
      WHO_AM_I = 0x0F,
      CTRL1_XL = 0x10,
      CTRL2_G = 0x11,
      CTRL3_C = 0x12,
      OUTX_L_G = 0x22,
      OUTX_L_XL = 0x28,
    };

    vector<int16_t> a; // accelerometer readings
    vector<int16_t> g; // gyro readings

    uint8_t last_status;

    LSM6();

    void setBus(TwoWire * bus) { this->bus = bus; }
    TwoWire * getBus() { return bus; }

    bool init(deviceType device = device_auto, sa0State sa0 = sa0_auto);
    deviceType getDeviceType() { return _device; }

    void enableDefault();

    void writeReg(uint8_t reg, uint8_t value);
    uint8_t readReg(uint8_t reg);

    void readAcc();
    void readGyro();
    void read();

    template <typename Ta, typename Tb, typename To> static void vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out);
    template <typename Ta, typename Tb> static float vector_dot(const vector<Ta> *a, const vector<Tb> *b);
    static void vector_normalize(vector<float> *a);

  private:
    TwoWire * bus;
    deviceType _device;
    uint8_t address;

    int16_t testReg(uint8_t address, regAddr reg);
};

template <typename Ta, typename Tb, typename To>
void LSM6::vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out)
{
  out->x = (a->y * b->z) - (a->z * b->y);
  out->y = (a->z * b->x) - (a->x * b->z);
  out->z = (a->x * b->y) - (a->y * b->x);
}

template <typename Ta, typename Tb>
float LSM6::vector_dot(const vector<Ta> *a, const vector<Tb> *b)
{
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

#endif
