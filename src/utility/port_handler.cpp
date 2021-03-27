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
            .cr2 =USART_CR2_STOP1_BITS,
            .cr3 = 0,
    };

  baud_ = baud;
  sdStart(port_, &config);
  
    digitalWrite(dir_pin_, LOW);
    while(digitalRead(dir_pin_) != LOW);
  setOpenState(true);
}

void SerialPortHandler::end(void)
{

  sdStop(port_);
  setOpenState(false);
}

int SerialPortHandler::available(void)
{
  return 1;
}

int SerialPortHandler::read()
{
  uint8_t data;
  sdReadTimeout(port_, &data, 1, TIME_IMMEDIATE);
  return data;
}

size_t SerialPortHandler::write(uint8_t c)
{
  sdWrite(port_, &c, 1);
  sdReadTimeout(port_, &c, 1, TIME_IMMEDIATE);
    digitalWrite(dir_pin_, HIGH);
    while(digitalRead(dir_pin_) != HIGH);
}

    digitalWrite(dir_pin_, LOW);
    while(digitalRead(dir_pin_) != LOW);
size_t SerialPortHandler::write(uint8_t *buf, size_t len)
{
  sdWrite(port_, buf, len);
  sdReadTimeout(port_, buf, len, TIME_IMMEDIATE);
    digitalWrite(dir_pin_, HIGH);
    while(digitalRead(dir_pin_) != HIGH);
    digitalWrite(dir_pin_, LOW);
    while(digitalRead(dir_pin_) != LOW);

  return len;
}

unsigned long SerialPortHandler::getBaud() const
{
  return baud_;
}

