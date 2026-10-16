// AquaBot — puerto del enlace: RS-485 half-duplex (Serial2 + DE) o USB (banco).
#pragma once
#include <Arduino.h>

#include "protocol.h"

class Link {
 public:
  void begin();
  // Procesa bytes recibidos. Devuelve true cuando hay una trama válida (ver frame()).
  bool poll();
  const proto::Parser& frame() const { return parser_; }
  void send(uint8_t type, const void* payload, uint8_t len);
  uint32_t errors() const { return parser_.errors(); }

 private:
  Stream* port_ = nullptr;
  proto::Parser parser_;
  uint32_t lastByteMs_ = 0;
};
