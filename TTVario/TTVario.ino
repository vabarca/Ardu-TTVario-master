/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/*
  
  by Vicente Abarca
*/

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

#include <stdlib.h>                     //we need that to use dtostrf() and convert float to string
#include <Tone.h>                      //tone library, download from url link (3)

//---------------

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "def.h"
#include "Serial.h"
#include "GPS.h"
#include "i2c.h"                      //i2c library

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

#if defined(SENSOR_BMP085)
  #include "CBMP085.h"
  CBMP085       oBaro;
#elif defined(SENSOR_CMS5611)
  #include "CMS561101BA.h"
  CMS561101BA   oBaro;
#endif

Tone          tone_out1;
Tone          tone_out2;
long          k[SAMPLES_ARR];
long          Temperature = 0;
long          Pressure = 101325;
float         Altitude=0;
const float   p0 = 101325;                 //Pressure at sea level (Pa)
unsigned long get_time1 = millis();
unsigned long get_time2 = millis();
unsigned long get_time3 = millis();
int           my_temperature = 1;
char          altitude_arr[6];            //wee need this array to translate float to string 
char          vario_arr[6];               //wee need this array to translate float to string
char          batt_arr[6];
int           samples=40;
int           maxsamples=50;
float         alt[51];
float         tim[51];
float         beep;
float         Beep_period;

float         nmea_vario_cms =0;
float         nmea_time_s=0;
float         nmea_alt_m=0;
float         nmea_old_alt_m=0;
char          variostring[6], altstring[6]; // create string arrays

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

long Averaging_Filter(long input) // moving average filter function
{
  long sum = 0;
  for (int i = 0; i < SAMPLES_ARR; i++) {
    k[i] = k[i+1];
  }
  k[SAMPLES_ARR - 1] = input;
  for (int i = 0; i < SAMPLES_ARR; i++) {
    sum += k[i];
  }
  return ( sum / SAMPLES_ARR ) ;
}

////////////////////////////////
////////////////////////////////

void play_welcome_beep()                 //play only once welcome beep after turning on arduino vario
{
  for (int aa=300;aa<=1500;aa=aa+100)
  {
    tone_out1.play(aa,200);             // play beep on pin (note,duration)
   if (VOLUME==2){ tone_out2.play(aa+5,200);}             // play beep on pin (note,duration)
    delay(100);
    LEDPIN_TOGGLE
  }
  for (int aa=1500;aa>=100;aa=aa-100)
  {
    tone_out1.play(aa,200);             // play beep on pin (note,duration)
   if (VOLUME==2){ tone_out2.play(aa+5,200);}             // play beep on pin (note,duration)
    delay(100);
    LEDPIN_TOGGLE
  }
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

void setup() 
{
  LEDPIN_PINMODE
  LEDPIN_ON

  TX_CTRLPIN_MODE
  TX_2_GPS_DISABLE

  SerialOpen(DATA_BAUD);
  SerialPrintPROGMEM(PSTR("Initializing GPS...\r\n"));

  TX_2_GPS_ENABLE
  iniGPS();
  TX_2_GPS_DISABLE

  SerialPrintPROGMEM(PSTR("Doit!\r\n"));

  i2c_init();                   // lets init i2c protocol

  #if defined(SENSOR_BMP085)
    oBaro.init(MODE_ULTRA_HIGHRES, p0, false); 
                            // BMP085 ultra-high-res mode, 101325Pa = 1013.25hPa, false = using Pa units
                            // this initialization is useful for normalizing pressure to specific datum.
                            // OR setting current local hPa information from a weather station/local airport (QNH).
  #elif defined(SENSOR_CMS5611)
    oBaro.init(MS561101BA_ADDR_CSB_LOW);  // Suppose that the CSB pin is connected to GND.
                                          // You'll have to check this on your breakout schematics
  #endif

  delay(500);

  if (VOLUME==1)
  {
    tone_out1.begin(BUZZER_PINNEG);       // piezo speaker output pin8 -
    pinMode(BUZZER_PINPOS, OUTPUT);      // set pin for output NMEA LED blink;
    digitalWrite(BUZZER_PINPOS, LOW); 
  } 
  else if (VOLUME==2)
  {
    tone_out1.begin(BUZZER_PINNEG);       // piezo speaker output pin8 -
    tone_out2.begin(BUZZER_PINPOS);       // piezo speaker output pin9 +
  }
  
  play_welcome_beep();      //everything is ready, play "welcome" sound
  delay(500);

  LEDPIN_OFF
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

boolean bGPSTx = false;
byte bytGPSEnding = 0;

void loop() 
{
    
  while(SerialAvailable())
  {
    char c = SerialRead();
    if (bGPSTx)
    {
      SerialWrite((uint8_t)c);
      if (c == '*' || bytGPSEnding != 0)
        bytGPSEnding++;

      if (bytGPSEnding == 5)
      {
        bytGPSEnding = 0;
        bGPSTx = false;
        LEDPIN_TOGGLE
        break;
      }
    }
    else
    {
      bGPSTx = (c =='$');
      if (bGPSTx)
        SerialWrite((uint8_t)c);
    }
  }

  float time=millis();              //take time, look into arduino millis() function
  float vario=0;
  float N1=0;
  float N2=0;
  float N3=0;
  float D1=0;
  float D2=0;

  #if defined (SENSOR_BMP085)
    oBaro.calcTruePressure(&Pressure);   //get one sample from BMP085 in every loop
  #elif defined(SENSOR_CMS5611)
    Pressure = oBaro.getPressure(MS561101BA_OSR_4096);
  #endif

  long average_pressure = Averaging_Filter(Pressure);                   //put it in filter and take average, this averaging is for NMEA output
  Altitude = (float)44330 * (1 - pow(((float)Pressure/p0), 0.190295));  //take new altitude in meters from pressure sample, not from average pressure
  nmea_alt_m = (float)44330 * (1 - pow(((float)average_pressure/p0), 0.190295));

  if ((millis() >= (nmea_time_s+1000)))
  {
    nmea_vario_cms = ((nmea_alt_m-nmea_old_alt_m))*100; 
    nmea_old_alt_m = nmea_alt_m;
    nmea_time_s = millis();
  }
 
  for(int cc=1;cc<=maxsamples;cc++)
  {                                 //vario algorithm
    alt[(cc-1)]=alt[cc];            //move "altitude" value in every loop inside table
    tim[(cc-1)]=tim[cc];            //move "time" value in every loop inside table
  }                                 //now we have altitude-time tables
                                    //put current "altitude" value on the end of the table
  alt[maxsamples]=Altitude;         //put current "time" value on the end of the table  
  tim[maxsamples]=time;                                                 
  float stime=tim[maxsamples-samples];

  for(int cc=(maxsamples-samples);cc<maxsamples;cc++)
  {
    N1+=(tim[cc]-stime)*alt[cc];
    N2+=(tim[cc]-stime);
    N3+=(alt[cc]);
    D1+=(tim[cc]-stime)*(tim[cc]-stime);
    D2+=(tim[cc]-stime);
  }

  vario=1000*((samples*N1)-N2*N3)/(samples*D1-D2*D2);
  if ((time-beep)>Beep_period)                          // make some beep
  {
    beep=time;
    if (vario>VARIO_CLIMB_RATE_START && vario<=10 )
    {
      switch (VOLUME) 
      {
        case 0: 
          break;
        case 1:
          Beep_period=550-(vario*(30+vario));
          tone_out1.play((1400+(200*vario)),420-(vario*(20+vario))); //when climbing make faster and shorter beeps
        case 2:
          Beep_period=550-(vario*(30+vario));
          tone_out1.play((1400+(200*vario)),420-(vario*(20+vario))); //when climbing make faster and shorter beeps
          tone_out2.play((1406+(200*vario)),420-(vario*(20+vario)));
      }               
    } 
    else if (vario >10) 
    {
      switch (VOLUME) 
      {
        case 0: 
          break;
        case 1:
          Beep_period=160;
          tone_out1.play(3450,120);                          //spike climb rate
        case 2:
          Beep_period=160;
          tone_out1.play(3450,120);                          //spike climb rate
          tone_out2.play(3456,120);
      }               
    } 
    else if (vario< VARIO_SINK_RATE_START)            //if you have high performace glider you can change sink beep to -0.95m/s ;)
    { 
      switch (VOLUME) 
      {
        case 0: 
          break;
        case 1:
          Beep_period=200;
          tone_out1.play(300,340);
        case 2:
          Beep_period=200;
          tone_out1.play(300,340);
          tone_out2.play(320,340);
      }               
    }
  }

  if (millis() >= (get_time2+1000))      //every second get temperature and battery level
  {

    #if defined(SENSOR_BMP085)
      oBaro.getTemperature(&Temperature); // get temperature in celsius from time to time, we have to divide that by 10 to get XY.Z
    #elif defined(SENSOR_CMS5611)
      Temperature = oBaro.getTemperature(MS561101BA_OSR_4096);
    #endif

    my_temperature = Temperature/10;
    get_time2 = millis();
  }

  if (!bGPSTx && (millis() >= (get_time3+(1000/NMEA_OUT_per_SEC))) && (PROTOCOL == 1))       //every NMEA_OUT/second send NMEA output over serial port (LK8000 protocol)
  {
    int battery_percentage = 0;  
    String str_out =                                                                 //combine all values and create part of NMEA data string output
      String("LK8EX1"+String(",")+String(average_pressure,DEC)+String(",")+String(dtostrf(Altitude,0,0,altitude_arr))+String(",")+
      String(dtostrf(nmea_vario_cms,0,0,vario_arr))+String(",")+String(my_temperature,DEC)+String(",")+String(battery_percentage+1000)+String(","));
      unsigned int checksum_end,ai,bi;     

    //creating now NMEA serial output for LK8000. LK8EX1 protocol format:
    //$LK8EX1,pressure,altitude,vario,temperature,battery,*checksum
    SerialPrintPROGMEM(PSTR("$"));                     //print first sign of NMEA protocol
    // Calculating checksum for data string and send data
    for (checksum_end = 0, ai = 0; ai < str_out.length(); ai++)
    {
      bi = (unsigned char)str_out[ai];
      checksum_end ^= bi;
      SerialWrite((char)bi);
    }

    SerialPrintPROGMEM(PSTR("*"));                     //end of protocol string
    SerialWrite(char2hex((char)checksum_end>>4));
    SerialWrite(char2hex((char)checksum_end));
    SerialWrite('\r');
    SerialWrite('\n');
    get_time3 = millis();

    LEDPIN_TOGGLE
  }
   
  if (!bGPSTx && (millis() >= (get_time3+(1000/NMEA_OUT_per_SEC))) && (PROTOCOL == 2))       //every NMEA_OUT/second send NMEA output over serial port (Flymaster F1 Protocol)
  {
    String str_out =
    String("VARIO,"+String(average_pressure,DEC)+ String(",")+String(dtostrf((nmea_vario_cms/10),0,0,vario_arr))+
    String(",100,0,1")+String(",")+String(my_temperature,DEC)+String(",0"));
    unsigned int checksum_end,ai,bi;                                                 // Calculating checksum for data string

    //creating now NMEA serial output for LK8000. Flymaster F1 protocol format:
    //// $VARIO,fPressure,fVario,Bat1Volts,Bat2Volts,BatBank,TempSensor1,TempSensor2*Checksum
    SerialPrintPROGMEM(PSTR("$"));                     //print first sign of NMEA protocol
    for (checksum_end = 0, ai = 0; ai < str_out.length(); ai++)
    {
      bi = (unsigned char)str_out[ai];
      checksum_end ^= bi;
      SerialWrite((char)bi);
    }

    SerialPrintPROGMEM(PSTR("*"));                     //end of protocol string
    SerialWrite(char2hex((char)checksum_end>>4));
    SerialWrite(char2hex((char)checksum_end));
    SerialWrite('\r');
    SerialWrite('\n');

    get_time3 = millis();

    LEDPIN_TOGGLE
  }
  
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////