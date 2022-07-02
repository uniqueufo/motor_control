/**
* @file        AsyncSerialDevice.cpp
* @author      Nan Lin
* @contact     fhln@mail.ustc.edu.cn
* @date        22-5-30
* @description 

* Copyright (c) 2022 Nan Lin
*/

#include "communication/async_serial.h"
#if defined(__linux__)
#include <linux/serial.h>
#endif

#include <cstring>
#include <iostream>

namespace communication {
bool AsyncSerialDevice::IsOpened() {
  if(serial_config_.fake_mode) return true;
  is_opened_ = serial_port_.is_open();
  return is_opened_;
}

AsyncSerialDevice::AsyncSerialDevice(const SerialConfig serial_config)
    : AsyncListenerInterface(serial_config.serial_device_name),
      serial_port_(io_context_),
      serial_config_(serial_config) {
  serial_config_.fake_mode = false;
}

void AsyncSerialDevice::SetBaudRate(unsigned int baudrate) {
  serial_port_.set_option(asio::serial_port_base::baud_rate(baudrate));
}

void AsyncSerialDevice::CloseDevice() {
  if(serial_config_.fake_mode) {
    is_opened_ = false;
    return;
  }
  if (!is_opened_) return;
  serial_port_.cancel();
  serial_port_.close();
  StopListening();
  is_opened_ = false;
}

bool AsyncSerialDevice::SendBytes(const uint8_t *bytes, size_t length) {
  if(serial_config_.fake_mode) return true;
  if (!is_opened_) {
    std::cerr << "Failed to send, port closed" << std::endl;
    return false;
  }
  assert(length < rxtx_buffer_size);
  std::lock_guard<std::recursive_mutex> lock(tx_mutex_);
  if (tx_rbuf_.GetFreeSize() < length) {
    throw std::length_error(
        "AsyncSerialDevice::SendBytes: tx buffer overflow, try to slow down sending "
        "data");
  }
  tx_rbuf_.Write(bytes, length);
  io_context_.post(std::bind(&AsyncSerialDevice::WriteToPort, shared_from_this(), true));
  return true;
}

bool AsyncSerialDevice::SetupDevice() {
  if(serial_config_.fake_mode) {
    is_opened_ = true;
    return true;
  }
  using SPB = asio::serial_port_base;

  try {
    serial_port_.open(serial_config_.port_name);

    // Set baud_rate and 8N1 mode
    serial_port_.set_option(SPB::baud_rate(serial_config_.baud_rate));
    serial_port_.set_option(SPB::character_size(serial_config_.character_size));
    serial_port_.set_option(SPB::parity(serial_config_.party));
    serial_port_.set_option(SPB::stop_bits(serial_config_.stop_bits));
    serial_port_.set_option(SPB::flow_control(
        (serial_config_.hardware_flow_control) ? SPB::flow_control::hardware : SPB::flow_control::none));

#if defined(__linux__)
    // Enable low latency mode on Linux
    {
      int                  fd = serial_port_.native_handle();
      struct serial_struct ser_info;
      ioctl(fd, TIOCGSERIAL, &ser_info);
      ser_info.flags |= ASYNC_LOW_LATENCY;
      ioctl(fd, TIOCSSERIAL, &ser_info);
    }
#endif

    is_opened_ = true;
    std::cout << "Start listening to port: " << serial_config_.port_name << "@" << serial_config_.baud_rate
              << std::endl;

  } catch (std::system_error &e) {
    std::cout << e.what();
    return false;
  }

  // give some work to io_service to start async io chain
  asio::post(io_context_, std::bind(&AsyncSerialDevice::ReadFromPort, this));
  return true;
}

void AsyncSerialDevice::DefaultReceiveCallback(uint8_t *data, const size_t bufsize, size_t len) {}

void AsyncSerialDevice::ReadFromPort() {
  if(serial_config_.fake_mode) return;

  auto sthis = shared_from_this();
  serial_port_.async_read_some(
      asio::buffer(rx_buf_),
      [sthis](asio::error_code error, size_t bytes_transferred) {
        if (error) {
          sthis->CloseDevice();
          return;
        }
        // Note: this will setup a callback function for serial data processing
        if (sthis->rcv_cb_ != nullptr) {
          sthis->rcv_cb_(sthis->rx_buf_.data(), sthis->rx_buf_.size(),
                         bytes_transferred);
        } else {
          sthis->DefaultReceiveCallback(
              sthis->rx_buf_.data(), sthis->rx_buf_.size(), bytes_transferred);
        }
        sthis->ReadFromPort();
      });
}

void AsyncSerialDevice::WriteToPort(bool check_if_busy) {
// do nothing if an async tx has already been initiated
  if(serial_config_.fake_mode) return;
  if (check_if_busy && tx_in_progress_) return;

  std::lock_guard<std::recursive_mutex> lock(tx_mutex_);
  if (tx_rbuf_.IsEmpty()) return;

  auto sthis = shared_from_this();
  tx_in_progress_ = true;
  auto len = tx_rbuf_.Read(tx_buf_, tx_rbuf_.GetOccupiedSize());
  serial_port_.async_write_some(
      asio::buffer(tx_buf_, len),
      [sthis](asio::error_code error, size_t bytes_transferred) {
        if (error) {
          sthis->CloseDevice();
          return;
        }
        std::lock_guard<std::recursive_mutex> lock(sthis->tx_mutex_);
        if (sthis->tx_rbuf_.IsEmpty()) {
          sthis->tx_in_progress_ = false;
          return;
        } else {
          sthis->WriteToPort(false);
        }
      });
}

bool AsyncSerialDevice::SendDataImpl(const std::any &send_data) {
  auto sdf = std::any_cast<SerialDataFrame>(send_data);
    std::cout << "function AsyncSerialDevice::SendDataImpl => df.length:"  <<sdf.length << std::endl;
  for (size_t i=0; i<sdf.length; ++i){
//    std::cout <<  sdf.bytes;
    printf("%02x ",sdf.bytes[i]);
  }
  printf("\n");

  return SendBytes(sdf.bytes, sdf.length);
}

std::any AsyncSerialDevice::ReceiveDataImpl() const {
  SerialDataFrame ret_frame; //TODO: how to get the data_len??
  return std::any_cast<SerialDataFrame>(rx_buf_);
}

void AsyncSerialDevice::SetFakeMode(bool fake_mode) {
  // 之前时正常物理通讯模式的话，先关闭物理串口
  if(!serial_config_.fake_mode){
    std::cout << "Try to close the real serial port, and begins fake-mode ......" << std::endl;
    if(is_opened_){
      CloseDevice();
    }
  }
  serial_config_.fake_mode = fake_mode;
}
void AsyncSerialDevice::SetSerialConfig(const SerialConfig& serial_config) {
  serial_config_ = serial_config;
}
}
//bool AsyncSerialDevice::SendData(const std::any &send_data) {
//    std::any_cast<std::string>(send_data);
//}
//
//bool AsyncSerialDevice::ReceiveData(std::any &receive_data) {
//    return CommunicationDevice::ReceiveData(receive_data);
//}
