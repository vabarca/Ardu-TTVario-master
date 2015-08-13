/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/*
  GPS
  by Vicente Abarca
*/

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

#ifndef CGPS_H_
#define CGPS_H_

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

#include "def.h"

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

#if (DATA_BAUD==9600)
  #define MTK_BAUD              PSTR("$PMTK251,9600*17\r\n")
#endif
#if (DATA_BAUD==19200)
  #define MTK_BAUD              PSTR("$PMTK251,19200*22\r\n")
#endif
#if (DATA_BAUD==38400)
  #define MTK_BAUD              PSTR("$PMTK251,38400*27\r\n")
#endif
#if (DATA_BAUD==57600)
  #define MTK_BAUD              PSTR("$PMTK251,57600*2C\r\n")
#endif
#if (DATA_BAUD==115200)
  #define MTK_BAUD              PSTR("$PMTK251,115200*1F\r\n")
#endif

#ifdef GPS_NMEA_OUTPUT_RMC_GGA_VTG
  #define MTK_NMEA_OUTPUT       PSTR("$PMTK314,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n")
#endif
#ifdef GPS_NMEA_OUTPUT_RMC_VTG
  #define MTK_NMEA_OUTPUT       PSTR("$PMTK314,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n")
#endif
#ifdef GPS_NMEA_OUTPUT_RMC
  #define MTK_NMEA_OUTPUT       PSTR("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n")
#endif

#define MTK_SET_DATUM           PSTR("$PMTK330,0*2E\r\n")
#define MTK_NAVTHRES_OFF        PSTR("$PMTK397,0*23\r\n") // Set Nav Threshold (the minimum speed the GPS must be moving to update the position) to 0 m/s

#define MTK_OUTPUT_1HZ          PSTR("$PMTK220,1000*1F\r\n")
#define MTK_OUTPUT_2HZ          PSTR("$PMTK220,500*2B\r\n")
#define MTK_OUTPUT_4HZ          PSTR("$PMTK220,250*29\r\n")
#define MTK_OUTPUT_5HZ          PSTR("$PMTK220,200*2C\r\n")
#define MTK_OUTPUT_10HZ         PSTR("$PMTK220,100*2F\r\n")

#define MTK_TEST                PSTR("$PMTK000*32\r\n")
#define MTK_ACK                 PSTR("$PMTK001,0,3*30\r\n")
#define MTK_HOT_START           PSTR("$PMTK101*32\r\n")
#define MTK_WARM_START          PSTR("$PMTK102*31\r\n")
#define MTK_COLD_START          PSTR("$PMTK103*30\r\n")
#define MTK_RESET               PSTR("$PMTK104*37\r\n")
  
#define SBAS_ON                 PSTR("$PMTK313,1*2E\r\n")
#define WAAS_ON                 PSTR("$PMTK301,2*2E\r\n")
#define SBAS_TEST_MODE          PSTR("$PMTK319,0*25\r\n")  //Enable test use of sbas satelite in test mode (usually PRN124 is in test mode)

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

void    iniGPS(); 

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

#endif //CGPS_H_

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
