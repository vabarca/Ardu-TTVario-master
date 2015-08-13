/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/*
  
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
#include "def.h"

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

static volatile uint8_t serialHeadRX,serialTailRX;
static uint8_t serialBufferRX[RX_BUFFER_SIZE];
static volatile uint8_t serialHeadTX,serialTailTX;
static uint8_t serialBufferTX[TX_BUFFER_SIZE];

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

void SerialSerialize(uint8_t a);

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

ISR(USART_UDRE_vect) 
{  
  uint8_t t = serialTailTX;
  if (serialHeadTX != t) 
  {
    if (++t >= TX_BUFFER_SIZE) 
      t = 0;
    UDR0 = serialBufferTX[t];  // Transmit next byte in the ring
    serialTailTX = t;
  }
  if (t == serialHeadTX) 
    UCSR0B &= ~(1<<UDRIE0); // Check if all data is transmitted . if yes disable transmitter UDRE interrupt
}

////////////////////////////////
////////////////////////////////

ISR(USART_RX_vect)  
{ 
  uint8_t data = UDR0;
  uint8_t h = serialHeadRX;
  if (++h >= RX_BUFFER_SIZE) 
    h = 0;
  if (h == serialTailRX) 
    return; // we did not bite our own tail?
  serialBufferRX[serialHeadRX] = data;  
  serialHeadRX = h;
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

void SerialOpen( uint32_t baud) 
{
  serialTailRX = 0;
  serialHeadRX = 0;
  serialTailTX = 0;
  serialHeadTX = 0;
  uint8_t h = ((F_CPU  / 4 / baud -1) / 2) >> 8;
  uint8_t l = ((F_CPU  / 4 / baud -1) / 2);
  UCSR0A  = (1<<U2X0); 
  UBRR0H = h; 
  UBRR0L = l; 
  UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
}

////////////////////////////////
////////////////////////////////

void SerialEnd() 
{
  UCSR0B &= ~((1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<UDRIE0));
}

////////////////////////////////
////////////////////////////////

uint8_t SerialRead() 
{
  uint8_t t = serialTailRX;
  uint8_t c = serialBufferRX[t];
  if (serialHeadRX != t) 
  {
    if (++t >= RX_BUFFER_SIZE) 
      t = 0;
    serialTailRX = t;
  }
  return c;
}

////////////////////////////////
////////////////////////////////

uint8_t SerialUsedTXBuff() 
{
  return ((uint8_t)(serialHeadTX - serialTailTX))%TX_BUFFER_SIZE;
}

////////////////////////////////
////////////////////////////////

uint8_t SerialAvailable() 
{
  return ((uint8_t)(serialHeadRX - serialTailRX))%RX_BUFFER_SIZE;
}

////////////////////////////////
////////////////////////////////

bool SerialTXfree() 
{
  return (serialHeadTX == serialTailTX);
}

////////////////////////////////
////////////////////////////////

void SerialSerialize(uint8_t a) 
{
  uint8_t t = serialHeadTX;
  if (++t >= TX_BUFFER_SIZE) 
    t = 0;
  serialBufferTX[t] = a;
  serialHeadTX = t;
}

////////////////////////////////
////////////////////////////////

void SerialWrite(uint8_t c)
{
  SerialSerialize(c);
  //enable transmitter UDRE interrupt
  UCSR0B |= (1<<UDRIE0);
}

////////////////////////////////
////////////////////////////////

void SerialRxPurge()
{
  while(SerialAvailable())
    SerialRead();
}

////////////////////////////////
////////////////////////////////

void SerialPrintPROGMEM(const char PROGMEM * str) 
{
  char b;
  while(str && (b = pgm_read_byte(str++))) 
    SerialWrite(b);      
}

////////////////////////////////
////////////////////////////////

// read a Hex value and return the decimal equivalent
uint8_t parseHex(char c) 
{
    if (c < '0')  return 0;
    if (c <= '9') return c - '0';
    if (c < 'A')  return 0;
    if (c <= 'F') return (c - 'A')+10;
}

////////////////////////////////
////////////////////////////////

char char2hex(char c)
{  
  return pgm_read_byte(PSTR("0123456789ABCDEF") + (0x0F & (unsigned char)c));
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
