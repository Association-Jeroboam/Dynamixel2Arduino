#include "port_handler.h"


DXLPortHandler::DXLPortHandler()
 : open_state_(false)
{}

/* DXLPortHandler */
bool DXLPortHandler::getOpenState()
{
  return open_state_;
}

void DXLPortHandler::setOpenState(bool state)
{
  open_state_ = state;
}


using namespace DYNAMIXEL;

/* SerialPortHandler */
SerialPortHandler::SerialPortHandler(SerialDriver* port, const int dir_pin)
 : DXLPortHandler(), port_(port), dir_pin_(dir_pin), baud_(57600)
{}

void SerialPortHandler::begin()
{
  begin(baud_);
}

void SerialPortHandler::begin(unsigned long baud)
{
  config = (SerialConfig){
    .speed = baud,
    .cr1 = 0,
    .cr2 = USART_CR2_STOP1_BITS,
    .cr3 = USART_CR3_HDSEL,
  };

  baud_ = baud;
  sdStart(port_, &config);
  
  if( dir_pin_ != (ioline_t)-1 ) {
    palWriteLine(dir_pin_, PAL_LOW);
    while (palReadLine(dir_pin_) != PAL_LOW);
  }
  setOpenState(true);
}

void SerialPortHandler::end(void)
{
  sdStop(port_);
  setOpenState(false);
}

int SerialPortHandler::read(uint8_t * data, uint32_t timeout_ms)
{
  return sdReadTimeout(port_, data, 1, TIME_MS2I(timeout_ms));
}

size_t SerialPortHandler::write(uint8_t c)
{

  if( dir_pin_ != (ioline_t)-1 ) {
    palWriteLine(dir_pin_, PAL_HIGH);
    while (palReadLine(dir_pin_) != PAL_HIGH);
  }

  sdWrite(port_, &c, 1);
  sdReadTimeout(port_, &c, 1, TIME_MS2I(1));
  if( dir_pin_ != (ioline_t)-1 ) {
    palWriteLine(dir_pin_, PAL_LOW);
    while (palReadLine(dir_pin_) != PAL_LOW);
  }
  return 1;
}


size_t SerialPortHandler::write(uint8_t *buf, size_t len)
{

  if( dir_pin_ != (ioline_t)-1 ) {
    palWriteLine(dir_pin_, PAL_HIGH);
    while (palReadLine(dir_pin_) != PAL_HIGH);
  }
  sdWrite(port_, buf, len);
  sdReadTimeout(port_, buf, len, TIME_MS2I(1));
  if( dir_pin_ != (ioline_t)-1 ) {
    palWriteLine(dir_pin_, PAL_LOW);
    while (palReadLine(dir_pin_) != PAL_LOW);
  }

  return len;
}

unsigned long SerialPortHandler::getBaud() const
{
  return baud_;
}

