/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef DYNAMIXEL_PORT_HANDLER_HPP_
#define DYNAMIXEL_PORT_HANDLER_HPP_

#include <Arduino.h>
#include <ch.hpp>
#include "hal.h"

class DXLPortHandler
{
  public:
    DXLPortHandler();
    
    virtual void begin() = 0;
    virtual void end() = 0;
    virtual int read(uint8_t *, uint32_t) = 0;
    virtual size_t write(uint8_t) = 0;
    virtual size_t write(uint8_t *buf, size_t len) = 0;
    bool getOpenState();
    void setOpenState(bool);

  private:
    bool open_state_;
};


namespace DYNAMIXEL{

class SerialPortHandler : public DXLPortHandler
{
  public:
    SerialPortHandler(SerialDriver* port, const int dir_pin);

    virtual void begin() override;
    virtual void end() override;
    virtual int read(uint8_t *, uint32_t timeout_ms) override;
    virtual size_t write(uint8_t) override;
    virtual size_t write(uint8_t *buf, size_t len) override;

    virtual void begin(unsigned long baud);
    virtual unsigned long getBaud() const;

  private:
    SerialDriver* port_;
    SerialConfig  config;
    ioline_t dir_pin_;
    unsigned long baud_;
};


}//namespace DYNAMIXEL

#endif /* DYNAMIXEL_PORT_HANDLER_HPP_ */