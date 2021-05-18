//-------------------------------
//-------------------------------
//A7105 SPI routines
//-------------------------------
//-------------------------------
void A7105_WriteID(uint32_t ida) {
  CS_off;
  _spi_write(A7105_06_ID_DATA);
  _spi_write((ida >> 24) & 0xff);
  _spi_write((ida >> 16) & 0xff);
  _spi_write((ida >> 8) & 0xff);
  _spi_write((ida >> 0) & 0xff);
  CS_on;
}
void A7105_ReadID() {
  uint8_t i;
  CS_off;
  _spi_write(0x46);
  for (i = 0; i < 4; i++) {
    aid[i] = _spi_read();
  }
  CS_on;
}
//----------------------
void Read_Packet() {
  CS_off;
  _spi_write(0x45);
  for (uint8_t i = 0; i < 21; i++) {
    packet[i] = _spi_read();
  }
  CS_on;
}
//---------------------------------
//--------------------------------------
void _spi_write(uint8_t command) {
  uint8_t n = 8;
  SCK_off;//SCK starts low
  SDI_off;
  while (n--) {
    if (command & 0x80)
      SDI_on;
    else
      SDI_off;
    SCK_on;
    NOP();
    SCK_off;
    command = command << 1;
  }
  SDI_on;
}
void _spi_write_adress(uint8_t address, uint8_t data) {
  CS_off;
  _spi_write(address);
  NOP();
  _spi_write(data);
  CS_on;
}
//-----------------------------------------
uint8_t _spi_read(void) {
  uint8_t result;
  uint8_t i;
  result = 0;
  pinMode(SDI_pin, INPUT); //make SDIO pin input
  //SDI_on;
  for (i = 0; i < 8; i++) {
    if (SDI_1) //if SDIO ==1
      result = (result << 1) | 0x01;
    else
      result = result << 1;
    SCK_on;
    NOP();
    SCK_off;
    NOP();
  }
  pinMode(SDI_pin, OUTPUT); //make SDIO pin output again
  return result;
}
//--------------------------------------------
uint8_t _spi_read_adress(uint8_t address) {
  uint8_t result;
  CS_off;
  address |= 0x40;
  _spi_write(address);
  result = _spi_read();
  CS_on;
  return (result);
}
//------------------------
void _spi_strobe(uint8_t address) {
  CS_off;
  _spi_write(address);
  CS_on;
}
//------------------------
