/**
* @file        communication_typecasting.hpp
* @author      Nan Lin
* @contact     fhln@mail.ustc.edu.cn
* @date        22-6-9
* @description  typecasting for field-bus data transmission

* Copyright (c) 2022 Nan Lin
*/

#ifndef COMMUNICATION_TYPECASTING_HPP
#define COMMUNICATION_TYPECASTING_HPP
#include <cstdint>

namespace communication {

#define ByteCast(x) ((uint8_t)(x))

uint32_t bytes_to_uint32(const uint8_t *b) {
  uint32_t ret;
  ret = (uint32_t) (ByteCast(b[0]));
  ret |= (uint32_t) (ByteCast(b[1])) << 8;
  ret |= (uint32_t) (ByteCast(b[2])) << 16;
  ret |= (uint32_t) (ByteCast(b[3])) << 24;
  return ret;
}

uint16_t bytes_to_uint16(const uint8_t *b) {
  uint16_t ret;
  ret = (uint16_t)(ByteCast(b[0]));
  ret |= (uint16_t)(ByteCast(b[1])) << 8;
  return ret;
}

int32_t bytes_to_int32(const uint8_t *b) {
  return (int32_t) bytes_to_uint32(b);
}

int16_t bytes_to_int16(const uint8_t *b) {
  return (int16_t) bytes_to_uint16(b);
}

};

#endif /* RING_BUFFER_HPP */
