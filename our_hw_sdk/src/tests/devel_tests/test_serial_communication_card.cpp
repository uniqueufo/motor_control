/**
* @file        basic_hw_test.cpp
* @author      Nan Lin
* @contact     fhln@mail.ustc.edu.cn
* @date        22-6-10
* @description test serial card connecty

* Copyright (c) 2022 Nan Lin
*/

#include <iostream>
#include <unistd.h>
//#include "../../communication/async_serial.h"
#include "../../hal/serial_device_manager.h"
#include "../../hal/protocol/serial_port_common_def.h"
using namespace hal;

void parse_buffer(uint8_t *buf, const size_t bufsize, size_t bytes_received)
{
  // std::cout << "parser called" << std::endl;

  // for (int i = 0; i < bytes_received; ++i)
  // {
  //     // auto c = *buf++;
  //     std::cout << std::hex << static_cast<int>(buf[i]) << std::dec << " ";
  // }

  if (bytes_received > 2)
  {
    for (int i = 0; i < bytes_received - 1; ++i)
    {
      uint8_t first = buf[i];
      uint8_t second = buf[i + 1];

      if (first == 0xB5 && second == 0x62)
        std::cout << "- start bytes found" << std::endl;
      // std::cout << std::hex << static_cast<int>(buf[i]) << std::dec << " ";
    }
  }
}

class TestSerialDevice: public SerialDeviceManager{
 public:
    TestSerialDevice(const std::string &serial_device_name,
                     const std::string &port_name,
                     BaudRate baud_rate = BR115200): SerialDeviceManager(serial_device_name, port_name, baud_rate){}
   void SendRobotCmd() { }
   void ParseUARTBuffer(uint8_t *buf, const size_t bufsize, size_t bytes_received) {}
};

int main(int argc, char *argv[])
{
  std::string device_id="/dev/ttyUSB0";
  BaudRate baud_rate = BR115200;

  TestSerialDevice test_serial_device("serial_card", device_id, baud_rate);

//  std::shared_ptr<AsyncSerialDevice> serial = std::make_shared<AsyncSerialDevice>();
//
//  serial->set_receive_callback(parse_buffer);
//
//  if (serial->is_open())
//    std::cout << "serial port opened" << std::endl;



  while (true)
  {
    // serial->send_bytes(data, 3);
    sleep(1);
  }
}

