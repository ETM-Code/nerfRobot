#ifndef __sparkfun_lis331_h__
#define __sparkfun_lis331_h__

#include <stdint.h>
#include <Arduino.h>

#define CTRL_REG1        0x20
#define CTRL_REG2        0x21
#define CTRL_REG3        0x22
#define CTRL_REG4        0x23
#define CTRL_REG5        0x24
#define HP_FILTER_RESET  0x25
#define REFERENCE        0x26
#define STATUS_REG       0x27
#define OUT_X_L          0x28
#define OUT_X_H          0x29
#define OUT_Y_L          0x2A
#define OUT_Y_H          0x2B
#define OUT_Z_L          0x2C
#define OUT_Z_H          0x2D
#define INT1_CFG         0x30
#define INT1_SOURCE      0x31
#define INT1_THS         0x32
#define INT1_DURATION    0x33
#define INT2_CFG         0x34
#define INT2_SOURCE      0x35
#define INT2_THS         0x36
#define INT2_DURATION    0x37

class LIS331
{
  public:
    // typedefs for this class
    enum comm_mode {USE_I2C, USE_SPI};
    enum power_mode {
      POWER_DOWN, 
      NORMAL, 
      LOW_POWER_0_5HZ, 
      LOW_POWER_1HZ,
      LOW_POWER_2HZ, 
      LOW_POWER_5HZ, 
      LOW_POWER_10HZ
    };
    enum data_rate {DR_50HZ, DR_100HZ, DR_400HZ, DR_1000HZ};
    enum high_pass_cutoff_freq_cfg {HPC_8, HPC_16, HPC_32, HPC_64};
    enum pp_od {PUSH_PULL, OPEN_DRAIN_MODE};
    enum int_sig_src {INT_SRC, INT1_2_SRC, DRDY, BOOT};
    enum fs_range {LOW_RANGE, MED_RANGE, NO_RANGE, HIGH_RANGE};
    enum int_axis {X_AXIS, Y_AXIS, Z_AXIS};
    enum trig_on_level {TRIG_ON_HIGH, TRIG_ON_LOW};

    // public functions
    LIS331();   // Constructor. Defers all functionality to .begin()
    void begin(comm_mode mode);
    void setI2CAddr(uint8_t address);
    void setSPICSPin(uint8_t pin);
    void axesEnable(bool enable);
    void setPowerMode(power_mode pmode);
    void setODR(data_rate drate);
    void readAxes(int16_t &x, int16_t &y, int16_t &z);
    uint8_t readReg(uint8_t reg_address);
    float convertToG(int maxScale, int reading);
    void setHighPassCoeff(high_pass_cutoff_freq_cfg hpcoeff);
    void enableHPF(bool enable);
    void HPFOnIntPin(bool enable, uint8_t pin);
    void intActiveHigh(bool enable);
    void intPinMode(pp_od pinMode);
    void latchInterrupt(bool enable, uint8_t intSource);
    void intSrcConfig(int_sig_src src, uint8_t pin);
    void setFullScale(fs_range range);
    bool newXData();
    bool newYData();
    bool newZData();
    void enableInterrupt(int_axis axis, trig_on_level trigLevel,
                        uint8_t interrupt, bool enable);
    void setIntDuration(uint8_t duration, uint8_t intSource);
    void setIntThreshold(uint8_t threshold, uint8_t intSource);

  private:
    comm_mode mode;    // comms mode, I2C or SPI
    uint8_t address;   // I2C address
    uint8_t CSPin;
    void LIS331_write(uint8_t address, uint8_t *data, uint8_t len);
    void LIS331_read(uint8_t address, uint8_t *data, uint8_t len);
};

#endif
