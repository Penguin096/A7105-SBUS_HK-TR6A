#define SERIAL_BAUD_RATE 115200  //115.200 baud serial port speed
#define SBUS_BAUD_RATE 100000    //100.000 baud serial port speed
//Pins configuration
#define GIO_pin 2    //GIO-D2
#define SDI_pin 12   //SDIO-B4
#define SCLK_pin 13  //SCK-B5
#define CS_pin 10    //CS-B2
//#############################
#define bind 15  //bind plug
//##############################
#define CS_on PORTB |= 0x04    //B2
#define CS_off PORTB &= 0xFB   //B2
#define SCK_on PORTB |= 0x20   //B5
#define SCK_off PORTB &= 0xDF  //B5
#define SDI_on PORTB |= 0x10   //B4
#define SDI_off PORTB &= 0xEF  //B4
#define GIO_on PORTD |= 0x04   //D2
//#####################################
#define GIO_1 (PIND & 0x04) == 0x04  //D2 input
#define GIO_0 (PIND & 0x04) == 0x00  //D2
#define SDI_1 (PINB & 0x10) == 0x10  //B4
#define SDI_0 (PINB & 0x10) == 0x00  //B4
//############################################
#define RED_LED_pin 14
#define Red_LED_ON PORTC &= ~_BV(0);
#define Red_LED_OFF PORTC |= _BV(0);
#define NOP() __asm__ __volatile__("nop")
//#######################################
#define RC_CHANNEL_MIN 1155   // Servo rx minimum position 
#define RC_CHANNEL_MAX 1929  // Servo rx maximum position 
#define SBUS_MIN_VALUE 173   // 
#define SBUS_MAX_VALUE 1811  // 
#define SBUS_UPDATE_RATE 14  //ms NORM SPEED:14   HIGH SPEED:7
//#######################################

#include <EEPROM.h>
#include "A7105.h"
#include <avr/wdt.h>

//#define DEBUG
#define FAILSAFE
//////////////////////CONFIGURATION///////////////////////////////
#define chanel_number 6  //set the number of chanels
//////////////////////////////////////////////////////////////////

static const uint8_t A7105_regs[] = {
  0xff, 0x42, 0x00, 0x14, 0x00, 0xff, 0xff, 0x00,
  0x00, 0x00, 0x00, 0x01, 0x21, 0x05, 0x00, 0x50,
  0x9e, 0x4b, 0x00, 0x02, 0x16, 0x2b, 0x12, 0x00,
  0x62, 0x80, 0x80, 0x00, 0x0a, 0x32, 0xc3, 0x0f,
  0x13, 0xc3, 0x00, 0xff, 0x00, 0x00, 0x3b, 0x00,
  0x17, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00,
  0x01, 0x0f, 0xff
};  //51 reg 0x00...0x32

static const uint8_t tx_channels[16][16] = {
  { 0x0a, 0x5a, 0x14, 0x64, 0x1e, 0x6e, 0x28, 0x78, 0x32, 0x82, 0x3c, 0x8c, 0x46, 0x96, 0x50, 0xa0 },
  { 0xa0, 0x50, 0x96, 0x46, 0x8c, 0x3c, 0x82, 0x32, 0x78, 0x28, 0x6e, 0x1e, 0x64, 0x14, 0x5a, 0x0a },
  { 0x0a, 0x5a, 0x50, 0xa0, 0x14, 0x64, 0x46, 0x96, 0x1e, 0x6e, 0x3c, 0x8c, 0x28, 0x78, 0x32, 0x82 },
  { 0x82, 0x32, 0x78, 0x28, 0x8c, 0x3c, 0x6e, 0x1e, 0x96, 0x46, 0x64, 0x14, 0xa0, 0x50, 0x5a, 0x0a },
  { 0x28, 0x78, 0x0a, 0x5a, 0x50, 0xa0, 0x14, 0x64, 0x1e, 0x6e, 0x3c, 0x8c, 0x32, 0x82, 0x46, 0x96 },
  { 0x96, 0x46, 0x82, 0x32, 0x8c, 0x3c, 0x6e, 0x1e, 0x64, 0x14, 0xa0, 0x50, 0x5a, 0x0a, 0x78, 0x28 },
  { 0x50, 0xa0, 0x28, 0x78, 0x0a, 0x5a, 0x1e, 0x6e, 0x3c, 0x8c, 0x32, 0x82, 0x46, 0x96, 0x14, 0x64 },
  { 0x64, 0x14, 0x96, 0x46, 0x82, 0x32, 0x8c, 0x3c, 0x6e, 0x1e, 0x5a, 0x0a, 0x78, 0x28, 0xa0, 0x50 },
  { 0x50, 0xa0, 0x46, 0x96, 0x3c, 0x8c, 0x28, 0x78, 0x0a, 0x5a, 0x32, 0x82, 0x1e, 0x6e, 0x14, 0x64 },
  { 0x64, 0x14, 0x6e, 0x1e, 0x82, 0x32, 0x5a, 0x0a, 0x78, 0x28, 0x8c, 0x3c, 0x96, 0x46, 0xa0, 0x50 },
  { 0x46, 0x96, 0x3c, 0x8c, 0x50, 0xa0, 0x28, 0x78, 0x0a, 0x5a, 0x1e, 0x6e, 0x32, 0x82, 0x14, 0x64 },
  { 0x64, 0x14, 0x82, 0x32, 0x6e, 0x1e, 0x5a, 0x0a, 0x78, 0x28, 0xa0, 0x50, 0x8c, 0x3c, 0x96, 0x46 },
  { 0x46, 0x96, 0x0a, 0x5a, 0x3c, 0x8c, 0x14, 0x64, 0x50, 0xa0, 0x28, 0x78, 0x1e, 0x6e, 0x32, 0x82 },
  { 0x82, 0x32, 0x6e, 0x1e, 0x78, 0x28, 0xa0, 0x50, 0x64, 0x14, 0x8c, 0x3c, 0x5a, 0x0a, 0x96, 0x46 },
  { 0x46, 0x96, 0x0a, 0x5a, 0x50, 0xa0, 0x3c, 0x8c, 0x28, 0x78, 0x1e, 0x6e, 0x32, 0x82, 0x14, 0x64 },
  { 0x64, 0x14, 0x82, 0x32, 0x6e, 0x1e, 0x78, 0x28, 0x8c, 0x3c, 0xa0, 0x50, 0x5a, 0x0a, 0x96, 0x46 },
};

uint8_t rfChannelMap[16];

//########## Variables #################
static uint16_t word_temp;
static uint8_t aid[4];
static uint8_t packet[21];
static byte jumper = 0;
static uint16_t failsafeCnt = 0;
static uint16_t nopacket = 0;
static uint16_t rssi;

static uint32_t sbusTime = 0;
static uint8_t sbusPacket[25];
bool isFailsafe;

void setup() {

  pinMode(bind, INPUT_PULLUP);  //pull up
  pinMode(RED_LED_pin, OUTPUT);
  //RF module pins
  pinMode(GIO_pin, INPUT);    //GIO 1
  pinMode(SDI_pin, OUTPUT);   //SDI   SDIO
  pinMode(SCLK_pin, OUTPUT);  //SCLK SCK
  pinMode(CS_pin, OUTPUT);    //CS output

  CS_on;    //start CS high
  SDI_on;   //start SDIO high
  SCK_off;  //start sck low

#if defined(DEBUG)
  Serial.begin(SERIAL_BAUD_RATE);  //for debug
#else
  Serial.begin(SBUS_BAUD_RATE, SERIAL_8E2);  //for SBUS
#endif

  //START A7105 init

  uint8_t i;
  delay(10);                               //wait 10ms for A7105 wakeup
  _spi_write_adress(A7105_00_MODE, 0x00);  //reset A7105
  A7105_WriteID(0x5475c52A);               //A7105 id

#if defined(DEBUG)
  A7105_ReadID();
  Serial.print(aid[0], HEX);
  Serial.print(aid[1], HEX);
  Serial.print(aid[2], HEX);
  Serial.println(aid[3], HEX);
#endif

  for (i = 0; i < 0x33; i++) {
    if (A7105_regs[i] != 0xff)
      _spi_write_adress(i, A7105_regs[i]);
  }
  //
  _spi_strobe(A7105_STANDBY);  //stand-by
  _spi_write_adress(A7105_02_CALC, 0x01);
  while (_spi_read_adress(A7105_02_CALC)) {
  }
  _spi_read_adress(A7105_22_IF_CALIB_I);
  _spi_write_adress(A7105_24_VCO_CURCAL, 0x13);
  _spi_write_adress(A7105_25_VCO_SBCAL_I, 0x09);
  _spi_strobe(A7105_STANDBY);  //stand-by

  //END A7105 init

  jumper = bind_jumper();

  while (1) {
    Red_LED_ON;
    delay(500);
    Red_LED_OFF;
    delay(500);
    if (jumper == 0) {
      for (int i = 0; i < 16; i++) {
        rfChannelMap[i] = EEPROM.read(i);
      }
      break;
    } else {
      bind_Flysky();
      Red_LED_ON;
      while (1)
        ;
    }
  }
  wdt_enable(WDTO_250MS);  //Enable watchdog
}

//############ MAIN LOOP ##############
void loop() {

  if ((millis() - sbusTime) > SBUS_UPDATE_RATE) {
    // Serial.write(millis() - sbusTime);  //Отладка
    sbusTime = millis();
    sbusPreparePacket(false, false, false, isFailsafe);

    // Reset Watchdog Timer
    wdt_reset();

#if defined(DEBUG)
#else
    Serial.write(sbusPacket, 25);
#endif
  }

  if (nopacket > 32) {
    if (nopacket >= 61) {
      nopacket = 32;
      next_chan();
    }
  } else {
    next_chan();
  }

  unsigned long pause = micros();
  while (1) {

#if defined(FAILSAFE)
    if (failsafeCnt > 700) {  //fs delay 1575ms
      failsafeCnt = 0;
      isFailsafe = true;

#if defined(DEBUG)
      Serial.println("failsafe!");
#endif
    }
#endif

    if ((micros() - pause) > 1419) {
      Red_LED_OFF;
      failsafeCnt++;
      nopacket++;
      break;
    }
    if (GIO_1) {
      continue;
    }
    if ((_spi_read_adress(A7105_00_MODE) & (A7105_MODE_CRCF | A7105_MODE_TRER)) != 0) {
      continue;
    }
    Read_Packet();
    rssi += map(_spi_read_adress(A7105_1D_RSSI_THOLD), 100, 10, RC_CHANNEL_MIN, RC_CHANNEL_MAX);
    rssi >>= 1;
    Red_LED_ON;
    failsafeCnt = 0;
    nopacket = 0;
    isFailsafe = false;
    break;
  }
}

//BIND_TX
void bind_Flysky() {
  static byte counter1 = 255;
  _spi_strobe(A7105_STANDBY);
  _spi_strobe(A7105_RST_RDPTR);
  _spi_write_adress(A7105_0F_CHANNEL, 0x00);  //binding listen on channel 0
  _spi_strobe(A7105_RX);
  while (counter1) {  //
    delay(10);        //wait 10ms
    if (bitRead(counter1, 3) == 1) {
      Red_LED_ON;
    }
    if (bitRead(counter1, 3) == 0) {
      Red_LED_OFF;
    }
    if (GIO_0) {
      if ((_spi_read_adress(A7105_00_MODE) & (A7105_MODE_CRCF | A7105_MODE_TRER)) == 0) {  //test CRCF&TRER bits
        Read_Packet();
        if (packet[0] == 0xaa) {
          uint32_t channelRow = packet[1] & 0x0F;
          uint32_t channelOffset = ((packet[1] & 0xF0) >> 4) + 1;
          if (channelOffset > 9) {
            channelOffset = 9;
          }
          for (unsigned i = 0; i < 16; i++) {
            rfChannelMap[i] = tx_channels[channelRow][i] - channelOffset;
          }
          for (int i = 0; i < 16; i++) {
            EEPROM.write(i, rfChannelMap[i]);
          }
        }
        break;
      } else {
        _spi_strobe(A7105_STANDBY);
        _spi_strobe(A7105_RST_RDPTR);
        _spi_write_adress(A7105_0F_PLL_I, 0x00);  //binding listen on channel 0
        _spi_strobe(A7105_RX);                    //try again
        continue;
      }
    } else {
      --counter1;
      if (counter1 == 0) {
        counter1 = 255;
      }
    }
  }
}

//bind jumper
unsigned char bind_jumper(void) {
  if (digitalRead(bind) == LOW) {
    //delayMicroseconds(1);
    return 1;
  }
  return 0;
}

void next_chan() {
  static uint8_t channel = 0;
  channel = (channel + 1) & 0x0F;
  _spi_strobe(A7105_STANDBY);
  _spi_strobe(A7105_RST_RDPTR);
  _spi_write_adress(A7105_0F_CHANNEL, rfChannelMap[channel]);
  _spi_strobe(A7105_RX);
}

void sbusPreparePacket(bool digitalCH1, bool digitalCH2, bool isSignalLoss, bool isFailsafe) {

  memset(sbusPacket, 0x00, 25);  //Zero out packet data
  sbusPacket[0] = 0x0F;          //Header
  sbusPacket[24] = 0x00;         //Footer 0x00 for SBUS

  uint8_t SBUS_Current_Packet_Bit = 0;
  uint8_t SBUS_Packet_Position = 1;

  for (uint8_t SBUS_Current_Channel = 0; SBUS_Current_Channel < chanel_number + 1; SBUS_Current_Channel++) {

    uint16_t sbusval;
    if (isFailsafe) {
      if (SBUS_Current_Channel == chanel_number || SBUS_Current_Channel == 2) {
        sbusval = RC_CHANNEL_MIN;
      } else sbusval = 1543;
    } else {
      if (SBUS_Current_Channel != chanel_number) {
        sbusval = packet[6 + (2 * SBUS_Current_Channel)] << 8 | packet[5 + (2 * SBUS_Current_Channel)];
      } else {
        sbusval = rssi;
      }
      sbusval = min(sbusval, RC_CHANNEL_MAX);
      sbusval = max(sbusval, RC_CHANNEL_MIN);
    }

#if defined(DEBUG)
    static uint16_t min_sensVal = 1300;
    static uint16_t max_sensVal = 1700;
    if (SBUS_Current_Channel != chanel_number) {
      if (sbusval < 1300) min_sensVal = min(min_sensVal, sbusval);
      if (sbusval > 1700) max_sensVal = max(max_sensVal, sbusval);
    }
    Serial.print(" min:");
    Serial.print(min_sensVal);
    Serial.print(" max:");
    Serial.println(max_sensVal);
#endif

    sbusval = map(sbusval, RC_CHANNEL_MIN, RC_CHANNEL_MAX, SBUS_MIN_VALUE, SBUS_MAX_VALUE);

    for (uint8_t SBUS_Current_Channel_Bit = 0; SBUS_Current_Channel_Bit < 11; SBUS_Current_Channel_Bit++) {
      if (SBUS_Current_Packet_Bit > 7) {
        SBUS_Current_Packet_Bit = 0;  //If we just set bit 7 in a previous step, reset the packet bit to 0 and
        SBUS_Packet_Position++;       //Move to the next packet uint8_t
      }
      sbusPacket[SBUS_Packet_Position] |= (((sbusval >> SBUS_Current_Channel_Bit) & 0x1) << SBUS_Current_Packet_Bit);  //Downshift the channel data bit, then upshift it to set the packet data uint8_t
      SBUS_Current_Packet_Bit++;
    }
  }

  if (digitalCH1) sbusPacket[23] |= (1 << 0);
  if (digitalCH2) sbusPacket[23] |= (1 << 1);
  if (isSignalLoss) sbusPacket[23] |= (1 << 2);
  if (isFailsafe) sbusPacket[23] |= (1 << 3);
}
