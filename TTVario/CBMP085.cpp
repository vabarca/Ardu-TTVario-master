/****************************************************************************
* CBMP085.cpp - CBMP085/I2C (Digital Pressure Sensor) library for Arduino     *
* Copyright 2010-2012 Filipe Vieira & various contributors                  *
*                                                                           *
* This file is part of CBMP085 Arduino library.                              *
*                                                                           *
* This library is free software: you can redistribute it and/or modify      *
* it under the terms of the GNU Lesser General Public License as published  *
* by the Free Software Foundation, either version 3 of the License, or      *
* (at your option) any later version.                                       *
*                                                                           *
* This program is distributed in the hope that it will be useful,           *
* but WITHOUT ANY WARRANTY; without even the implied warranty of            *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU Lesser General Public License for more details.                       *
*                                                                           *
* You should have received a copy of the GNU Lesser General Public License  *
* along with this program.  If not, see <http://www.gnu.org/licenses/>.     *
****************************************************************************/
/****************************************************************************
* Tested on Arduino Mega with CBMP085 Breakout                               *
* SDA   -> pin 20   (no pull up resistors)                                  *
* SCL   -> pin 21   (no pull up resistors)                                  *
* XCLR  -> not connected                                                    *
* EOC   -> not connected                                                    *
* GND   -> pin GND                                                          *
* VCC   -> pin 3.3V                                                         *
* NOTE: SCL and SDA needs pull-up resistors for each I2C bus.               *
*  2.2kOhm..10kOhm, typ. 4.7kOhm                                            *
*****************************************************************************/

#include "CBMP085.h"
#include "i2c.h"

CBMP085::CBMP085() 
{
  _dev_address = BMP085_ADDR;
  _pressure_waittime[0] = 5; // These are maximum convertion times.
  _pressure_waittime[1] = 8; // It is possible to use pin EOC (End Of Conversion)
  _pressure_waittime[2] = 14;// to check if conversion is finished (logic 1) 
  _pressure_waittime[3] = 26;// or running (logic 0) insted of waiting for convertion times.
  _cm_Offset = 0;
  _Pa_Offset = 0;               // 1hPa = 100Pa = 1mbar
  
  oldEMA = 0;
}

void CBMP085::init() 
{  
  init(MODE_STANDARD, 0, true);
}

void CBMP085::init(byte _BMPMode, int32_t _initVal, bool _Unitmeters)
{     
  getCalData();               // initialize cal data
  calcTrueTemperature();      // initialize b5
  setMode(_BMPMode);
  _Unitmeters ? setLocalAbsAlt(_initVal) : setLocalPressure(_initVal); 
}

byte CBMP085::getDevAddr()
{   
  return _dev_address;
}

byte CBMP085::getMode()
{
  return _oss;
}

void CBMP085::setMode(byte _BMPMode)
{
  _oss = _BMPMode;
}

void CBMP085::setLocalPressure(int32_t _Pa)
{   
  int32_t tmp_alt;
 
  _param_datum = _Pa;   
  getAltitude(&tmp_alt);    // calc altitude based on current pressure   
  _param_centimeters = tmp_alt;
}

void CBMP085::setLocalAbsAlt(int32_t _centimeters)
{  
  int32_t tmp_Pa;
 
  _param_centimeters = _centimeters;   
  getPressure(&tmp_Pa);    // calc pressure based on current altitude
  _param_datum = tmp_Pa;
}

void CBMP085::setAltOffset(int32_t _centimeters)
{
  _cm_Offset = _centimeters;
}

void CBMP085::sethPaOffset(int32_t _Pa)
{
  _Pa_Offset = _Pa;
}

void CBMP085::zeroCal(int32_t _Pa, int32_t _centimeters)
{
  setAltOffset(_centimeters - _param_centimeters);    
  sethPaOffset(_Pa - _param_datum);    
}

void CBMP085::getPressure(int32_t *_Pa)
{   
  long TruePressure;

  calcTruePressure(&TruePressure); 
  *_Pa = TruePressure / pow((1 - (float)_param_centimeters / 4433000), 5.255) + _Pa_Offset;
  // converting from float to int32_t truncates toward zero, 1010.999985 becomes 1010 resulting in 1 Pa error (max).  
  // Note that CBMP085 abs accuracy from 700...1100hPa and 0..+65�C is +-100Pa (typ.)
}

void CBMP085::getAltitude(int32_t *_centimeters)
{
  long TruePressure;

  calcTruePressure(&TruePressure); 
  *_centimeters =  4433000 * (1 - pow((TruePressure / (float)_param_datum), 0.1903)) + _cm_Offset;  
  // converting from float to int32_t truncates toward zero, 100.999985 becomes 100 resulting in 1 cm error (max).
}

void CBMP085::getTemperature(int32_t *_Temperature) 
{
  calcTrueTemperature();                            // force b5 update
  *_Temperature = ((b5 + 8) >> 4);
}

void CBMP085::calcTrueTemperature()
{
  long ut,x1,x2;

  //read Raw Temperature
  i2c_writeReg(_dev_address,CONTROL, READ_TEMPERATURE);
  delay(5);                                         // min. 4.5ms read Temp delay
  i2c_read_reg_to_buf(_dev_address,CONTROL_OUTPUT, _buff, 2); 
  ut = ((long)_buff[0] << 8 | ((long)_buff[1]));    // uncompensated temperature value
  
  // calculate temperature
  x1 = ((long)ut - ac6) * ac5 >> 15;
  x2 = ((long)mc << 11) / (x1 + md);
  b5 = x1 + x2;
}

void CBMP085::calcTruePressure(long *_TruePressure) 
{
  long up,x1,x2,x3,b3,b6,p;
  unsigned long b4,b7;
  int32_t tmp; 

  #if AUTO_UPDATE_TEMPERATURE
  calcTrueTemperature();        // b5 update 
  #endif 
 
 //read Raw Pressure
  i2c_writeReg(_dev_address,CONTROL, READ_PRESSURE+(_oss << 6));
  delay(_pressure_waittime[_oss]);    
  i2c_read_reg_to_buf(_dev_address,CONTROL_OUTPUT, _buff, 3);  
  up = ((((long)_buff[0] <<16) | ((long)_buff[1] <<8) | ((long)_buff[2])) >> (8-_oss)); // uncompensated pressure value
  
  // calculate true pressure
  b6 = b5 - 4000;             // b5 is updated by calcTrueTemperature().
  x1 = (b2* (b6 * b6 >> 12)) >> 11;
  x2 = ac2 * b6 >> 11;
  x3 = x1 + x2;
  tmp = ac1;
  tmp = (tmp * 4 + x3) << _oss;
  b3 = (tmp + 2) >> 2;
  x1 = ac3 * b6 >> 13;
  x2 = (b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
  b7 = ((uint32_t)up - b3) * (50000 >> _oss);
  p = b7 < 0x80000000 ? (b7 << 1) / b4 : (b7 / b4) << 1;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  *_TruePressure = p + ((x1 + x2 + 3791) >> 4);
}

//PRIVATE methods

void CBMP085::getCalData()
{
  i2c_read_reg_to_buf(_dev_address,CAL_AC1, _buff, 2);
  ac1 = ((int)_buff[0] <<8 | ((int)_buff[1]));
  i2c_read_reg_to_buf(_dev_address,CAL_AC2, _buff, 2);
  ac2 = ((int)_buff[0] <<8 | ((int)_buff[1]));
  i2c_read_reg_to_buf(_dev_address,CAL_AC3, _buff, 2);
  ac3 = ((int)_buff[0] <<8 | ((int)_buff[1]));
  i2c_read_reg_to_buf(_dev_address,CAL_AC4, _buff, 2);
  ac4 = ((unsigned int)_buff[0] <<8 | ((unsigned int)_buff[1]));
  i2c_read_reg_to_buf(_dev_address,CAL_AC5, _buff, 2);
  ac5 = ((unsigned int)_buff[0] <<8 | ((unsigned int)_buff[1]));
  i2c_read_reg_to_buf(_dev_address,CAL_AC6, _buff, 2);
  ac6 = ((unsigned int)_buff[0] <<8 | ((unsigned int)_buff[1])); 
  i2c_read_reg_to_buf(_dev_address,CAL_B1, _buff, 2);
  b1 = ((int)_buff[0] <<8 | ((int)_buff[1])); 
  i2c_read_reg_to_buf(_dev_address,CAL_B2, _buff, 2);
  b2 = ((int)_buff[0] <<8 | ((int)_buff[1])); 
  i2c_read_reg_to_buf(_dev_address,CAL_MB, _buff, 2);
  mb = ((int)_buff[0] <<8 | ((int)_buff[1]));
  i2c_read_reg_to_buf(_dev_address,CAL_MC, _buff, 2);
  mc = ((int)_buff[0] <<8 | ((int)_buff[1]));
  i2c_read_reg_to_buf(_dev_address,CAL_MD, _buff, 2);
  md = ((int)_buff[0] <<8 | ((int)_buff[1])); 
}

