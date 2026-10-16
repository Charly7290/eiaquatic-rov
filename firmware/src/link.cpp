#include "link.h"

#include "config.h"

void Link::begin() {
#if LINK_USB
  Serial.begin(LINK_BAUD);
  port_ = &Serial;
#else
  pinMode(PIN_RS485_DE, OUTPUT);
  digitalWrite(PIN_RS485_DE, LOW);  // escuchando
  Serial2.setRxBufferSize(512);
  Serial2.begin(LINK_BAUD, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);
  port_ = &Serial2;
#endif
}

bool Link::poll() {
  while (port_->available() > 0) {
    uint32_t now = millis();
    if (parser_.midFrame() && now - lastByteMs_ > FRAME_GAP_MS) parser_.reset();
    lastByteMs_ = now;
    if (parser_.feed((uint8_t)port_->read())) return true;
  }
  return false;
}

void Link::send(uint8_t type, const void* payload, uint8_t len) {
  uint8_t buf[proto::MAX_FRAME];
  size_t n = proto::encode(type, payload, len, buf);
  if (!n) return;
#if LINK_USB
  Serial.write(buf, n);
#else
  digitalWrite(PIN_RS485_DE, HIGH);
  Serial2.write(buf, n);
  Serial2.flush();  // espera a que salga el último bit antes de soltar el bus
  digitalWrite(PIN_RS485_DE, LOW);
#endif
}
