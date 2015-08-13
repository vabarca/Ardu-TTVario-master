/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/*
  gps
  by Vicente Abarca
*/

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "Serial.h" 
#include "GPS.h" 

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

void iniGPS()
{
  uint32_t init_speed[5] = {9600,19200,38400,57600,115200};
  SerialEnd();

  for (int i = 0; i<5 ;i++)
  {
    SerialOpen(init_speed[i]);
    delay(200);
    SerialPrintPROGMEM(MTK_BAUD);
    SerialPrintPROGMEM(MTK_BAUD); 
    SerialPrintPROGMEM(MTK_BAUD); 
    while(!SerialTXfree());
    SerialEnd();
  }

  // at this point we have GPS working at selected (via #define GPS_BAUD) baudrate
  // So now we have to set the desired mode and update rate (which depends on the NMEA or MTK_BINARYxx settings)
  SerialOpen(DATA_BAUD);
  delay(200);

  SerialPrintPROGMEM(MTK_NAVTHRES_OFF);
  while(!SerialTXfree());
  SerialPrintPROGMEM(MTK_NMEA_OUTPUT);
  while(!SerialTXfree());
  SerialPrintPROGMEM(MTK_SET_DATUM);
  while(!SerialTXfree());
  SerialPrintPROGMEM(MTK_OUTPUT_1HZ);           // 1 Hz update rate
  while(!SerialTXfree());
  SerialPrintPROGMEM(MTK_TEST);                 // 1 Hz update rate
  while(!SerialTXfree());
  
  SerialRxPurge();
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

