/**
* @file        AsyncCanDevice.cpp
* @author      Nan Lin
* @contact     fhln@mail.ustc.edu.cn
* @date        22-5-30
* @description 

* Copyright (c) 2022 Nan Lin
*/

#include "communication/async_can.h"

namespace communication {
AsyncCanDevice::AsyncCanDevice(const std::string &can_port, const std::string &device_name)
    : AsyncListenerInterface(device_name), can_port_(can_port), socketcan_stream_(io_context_) {}

void AsyncCanDevice::CloseDevice() {
  const int close_result = ::close(can_fd_);
  can_fd_ = -1;
  // stop io thread
  StopListening();
  is_opened_ = false;
}

bool AsyncCanDevice::SendFrame(const can_frame &frame) {
  socketcan_stream_.async_write_some(
      asio::buffer(&frame, sizeof(frame)),
      [](asio::error_code error, size_t bytes_transferred) {
        if (error) {
          std::cerr << "Failed to send CAN frame" << std::endl;
          return false;
        }
        // std::cout << "frame sent" << std::endl;
        return !error;
      } );
  return true;
}

bool AsyncCanDevice::SetupDevice() {
  try {
    const size_t iface_name_size = strlen(can_port_.c_str()) + 1;
    if (iface_name_size > IFNAMSIZ) return false;

    can_fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (can_fd_ < 0) return false;

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    memcpy(ifr.ifr_name, can_port_.c_str(), iface_name_size);

    const int ioctl_result = ioctl(can_fd_, SIOCGIFINDEX, &ifr);
    if (ioctl_result < 0) CloseDevice();

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    const int bind_result =
                  bind(can_fd_, (struct sockaddr *) &addr, sizeof(addr));
    if (bind_result < 0) CloseDevice();

    is_opened_ = true;
    std::cout << "Start listening to port: " << can_port_ << std::endl;
  } catch (std::system_error &e) {
    std::cout << e.what();
    return false;
  }

  // give some work to io_service to start async io chain
  socketcan_stream_.assign(can_fd_);
  asio::post(io_context_,
             std::bind(&AsyncCanDevice::ReadFromPort, this, std::ref(rcv_frame_),
                       std::ref(socketcan_stream_)));
  return true;
}

void AsyncCanDevice::DefaultReceiveCallback(can_frame *rx_frame) {
  std::cout << std::hex << rx_frame->can_id << "  ";
  for (int i = 0; i < rx_frame->can_dlc; i++)
    std::cout << std::hex << int(rx_frame->data[i]) << " ";
  std::cout << std::dec << std::endl;
}

void AsyncCanDevice::ReadFromPort(can_frame &rec_frame, asio::posix::basic_stream_descriptor<> &stream) {
  auto sthis = shared_from_this();
  stream.async_read_some(
      asio::buffer(&rec_frame, sizeof(rec_frame)),
      [sthis](asio::error_code error, size_t bytes_transferred) {
        if (error) {
          sthis->CloseDevice();
          return;
        }

        if (sthis->rcv_cb_ != nullptr)
          sthis->rcv_cb_(&sthis->rcv_frame_);
        else
          sthis->DefaultReceiveCallback(&sthis->rcv_frame_);

        sthis->ReadFromPort(std::ref(sthis->rcv_frame_),
                            std::ref(sthis->socketcan_stream_));
      });
}

bool AsyncCanDevice::SendDataImpl(const std::any &send_data) {
  return SendFrame(std::any_cast<can_frame>(send_data));
}
std::any AsyncCanDevice::ReceiveDataImpl() const {
  return std::any_cast<can_frame>(rcv_frame_);
}
bool AsyncCanDevice::IsOpened() {
  return CommunicationDevice::IsOpened();
}

}


