/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/*
  
  by Vicente Abarca
*/

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

#ifndef DEF_H_
#define DEF_H_

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

#define SOFT_VERSION  100

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
////             Proc specific definitions            
// Proc auto detection
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  #define PROMINI
#endif

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

#if defined(PROMINI)

  #define LEDPIN                    13
  #define TX_CTRLPIN                5
  #define BUZZER_PINPOS             9
  #define BUZZER_PINNEG             8


  #define LEDPIN_PINMODE            pinMode (13, OUTPUT);
  #define LEDPIN_TOGGLE             PINB |= 1<<5;     //switch LEDPIN state (digital PIN 13)
  #define LEDPIN_OFF                PORTB &= ~(1<<5);
  #define LEDPIN_ON                 PORTB |= (1<<5);
  
  #define I2C_PULLUPS_ENABLE        PORTC |= 1<<4; PORTC |= 1<<5;   // PIN A4&A5 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE       PORTC &= ~(1<<4); PORTC &= ~(1<<5);

#endif

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

//// I2C

//#define INTERNAL_I2C_PULLUPS
#define I2C_SPEED 100000L     //100kHz normal mode, this value must be used for a genuine WMP
//#define I2C_SPEED 400000L   //400kHz fast mode, it works only with some WMP clones

//// SERIAL COMM

#define RX_BUFFER_SIZE              255 // 255 (NOT 256!!) RX buffer is needed for GPS communication (64 or 128 was too short)
#define TX_BUFFER_SIZE              128

#define DATA_BAUD                   38400
                                    // avoid using 115200 baud because with 16MHz arduino 
                                    // the 115200 baudrate have more 
                                    // than 2% speed error (57600 have 0.8% error)

//// SENSOR

//#define SENSOR_BMP085
#define SENSOR_CMS5611

//// GPS

#define GPS_NMEA_OUTPUT_RMC_GGA_VTG
//#define GPS_NMEA_OUTPUT_RMC_VTG
//#define GPS_NMEA_OUTPUT_RMC

#define TX_CTRLPIN_MODE             pinMode(TX_CTRLPIN, OUTPUT);
#define TX_2_GPS_ENABLE             digitalWrite(TX_CTRLPIN, LOW);
#define TX_2_GPS_DISABLE            digitalWrite(TX_CTRLPIN, HIGH);

//// BUZZER

#define VOLUME                      2        // volume setting 0-no sound, 1-low sound volume, 2-high sound volume

//// PROTOCOL

#define PROTOCOL                    2        //define NMEA output: 1 = $LK8EX1, 2 = FlymasterF1, 3 = Off-Line Test with Serial Monitor (Arduino) 
#define NMEA_OUT_per_SEC            3        // NMEA output string samples per second (1 to 20)

//// ALGORITHYM

#define SAMPLES_ARR                 25       //define moving average filter array size (2->30), more means vario is less sensitive and slower, NMEA output

#define VARIO_CLIMB_RATE_START      0.5  //minimum climb beeping value(ex. start climbing beeping at 0.5m/s)
#define VARIO_SINK_RATE_START       -1.1 //maximum sink beeping value (ex. start sink beep at -1.1m/s)

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

//// ERROR CHECKING SECTION

#if !defined(DATA_BAUD) 
  #error "DATA_BAUD has to be defined"
#endif

#if !defined(GPS_NMEA_OUTPUT_RMC_GGA_VTG) &&  !defined(GPS_NMEA_OUTPUT_RMC_VTG) && !defined(GPS_NMEA_OUTPUT_RMC)
  #error "GPS_NEMEA_OUTPUT has to be defined"
#endif

#if !defined(PROMINI) 
  #error "328P based board has to be defined"
#endif 

#if !defined(SENSOR_BMP085) &&  !defined(SENSOR_CMS5611) 
  #error "BARO SENSOR has to be defined"
#endif

#if defined(SENSOR_BMP085) &&  defined(SENSOR_CMS5611) 
  #error "Only one BARO SENSOR has to be defined"
#endif

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

#endif /* DEF_H_ */

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
