#include "port_handler.h"
#include "BuildConf.hpp"

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
{

}

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
  
  palWriteLine(XL320_DIR_PIN, PAL_LOW);
  while(palReadLine(XL320_DIR_PIN) != PAL_LOW);
  setOpenState(true);
}

void SerialPortHandler::end(void)
{

  sdStop(port_);
  setOpenState(false);
}

int SerialPortHandler::read(uint8_t * data)
{
  return sdReadTimeout(port_, data, 1, TIME_IMMEDIATE);
}

size_t SerialPortHandler::write(uint8_t c)
{

    palWriteLine(XL320_DIR_PIN, PAL_HIGH);
    while(palReadLine(XL320_DIR_PIN) != PAL_HIGH);

    sdWrite(port_, &c, 1);
    sdReadTimeout(port_, &c, 1, TIME_INFINITE);
    palWriteLine(XL320_DIR_PIN, PAL_LOW);
    while(palReadLine(XL320_DIR_PIN) != PAL_LOW);
    return 1;
}


size_t SerialPortHandler::write(uint8_t *buf, size_t len)
{

    palWriteLine(XL320_DIR_PIN, PAL_HIGH);
    while(palReadLine(XL320_DIR_PIN) != PAL_HIGH);
    sdWrite(port_, buf, len);

    sdReadTimeout(port_, buf, len, TIME_INFINITE);
    palWriteLine(XL320_DIR_PIN, PAL_LOW);
    while(palReadLine(XL320_DIR_PIN) != PAL_LOW);

  return len;
}

unsigned long SerialPortHandler::getBaud() const
{
  return baud_;
}

