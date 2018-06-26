
#ifndef __L3GD20_H__
#define __L3GD20_H__

#include "Arduino.h"
#include "Wire.h"

#define L3GD20_ADDRESS                (0x6B)        
#define L3GD20_POLL_TIMEOUT           (100)   
#define L3GD20_ID                     0xD4
#define L3GD20H_ID                    0xD7

#define L3GD20_SENSITIVITY_250DPS  (0.00875F)   
#define L3GD20_SENSITIVITY_500DPS  (0.0175F)   
#define L3GD20_SENSITIVITY_2000DPS (0.070F)  
#define L3GD20_DPS_TO_RADS         (0.017453293F) 

class Adafruit_L3GD20
{
  public:
    typedef enum
    {                                      
      L3GD20_REGISTER_WHO_AM_I            = 0x0F,  
      L3GD20_REGISTER_CTRL_REG1           = 0x20, 
      L3GD20_REGISTER_CTRL_REG2           = 0x21,  
      L3GD20_REGISTER_CTRL_REG3           = 0x22,  
      L3GD20_REGISTER_CTRL_REG4           = 0x23, 
      L3GD20_REGISTER_CTRL_REG5           = 0x24,   
      L3GD20_REGISTER_REFERENCE           = 0x25,  
      L3GD20_REGISTER_OUT_TEMP            = 0x26, 
      L3GD20_REGISTER_STATUS_REG          = 0x27, 
      L3GD20_REGISTER_OUT_X_L             = 0x28, 
      L3GD20_REGISTER_OUT_X_H             = 0x29, 
      L3GD20_REGISTER_OUT_Y_L             = 0x2A,  
      L3GD20_REGISTER_OUT_Y_H             = 0x2B,  
      L3GD20_REGISTER_OUT_Z_L             = 0x2C, 
      L3GD20_REGISTER_OUT_Z_H             = 0x2D, 
      L3GD20_REGISTER_FIFO_CTRL_REG       = 0x2E, 
      L3GD20_REGISTER_FIFO_SRC_REG        = 0x2F,
      L3GD20_REGISTER_INT1_CFG            = 0x30,  
      L3GD20_REGISTER_INT1_SRC            = 0x31,  
      L3GD20_REGISTER_TSH_XH              = 0x32, 
      L3GD20_REGISTER_TSH_XL              = 0x33,  
      L3GD20_REGISTER_TSH_YH              = 0x34,  
      L3GD20_REGISTER_TSH_YL              = 0x35,  
      L3GD20_REGISTER_TSH_ZH              = 0x36,  
      L3GD20_REGISTER_TSH_ZL              = 0x37, 
      L3GD20_REGISTER_INT1_DURATION       = 0x38  
    } l3gd20Registers_t;

    typedef enum
    {
      L3DS20_RANGE_250DPS,
      L3DS20_RANGE_500DPS,
      L3DS20_RANGE_2000DPS
    } l3gd20Range_t;

    typedef struct l3gd20Data_s
    {
      float x;
      float y;
      float z;
    } l3gd20Data;

    Adafruit_L3GD20(int8_t cs, int8_t mosi, int8_t miso, int8_t clk);

    bool begin(l3gd20Range_t rng=L3DS20_RANGE_250DPS, byte addr=L3GD20_ADDRESS);
    void read(void);

    l3gd20Data data; 

  private:
    void write8(l3gd20Registers_t reg, byte value);
    byte read8(l3gd20Registers_t reg);
    uint8_t SPIxfer(uint8_t x);
    byte address;
    l3gd20Range_t range;
    int8_t _miso, _mosi, _clk, _cs;
};

#endif
