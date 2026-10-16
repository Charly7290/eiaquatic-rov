#include "protocol.h"

#include <string.h>

namespace proto {

uint16_t crc16Byte(uint16_t crc, uint8_t b) {
  crc ^= (uint16_t)b << 8;
  for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  return crc;
}

uint16_t crc16(const uint8_t* data, size_t len, uint16_t crc) {
  for (size_t i = 0; i < len; i++) crc = crc16Byte(crc, data[i]);
  return crc;
}

size_t encode(uint8_t type, const void* payload, uint8_t len, uint8_t* out) {
  if (len > MAX_PAYLOAD) return 0;
  out[0] = SYNC1;
  out[1] = SYNC2;
  out[2] = type;
  out[3] = len;
  if (len) memcpy(out + 4, payload, len);
  uint16_t c = crc16(out + 2, (size_t)len + 2);
  out[4 + len] = (uint8_t)(c & 0xFF);
  out[5 + len] = (uint8_t)(c >> 8);
  return (size_t)len + 6;
}

bool Parser::feed(uint8_t b) {
  switch (state_) {
    case S_SYNC1:
      if (b == SYNC1) state_ = S_SYNC2;
      break;
    case S_SYNC2:
      if (b == SYNC2) state_ = S_TYPE;
      else if (b != SYNC1) state_ = S_SYNC1;
      break;
    case S_TYPE:
      type_ = b;
      crc_ = crc16Byte(0xFFFF, b);
      state_ = S_LEN;
      break;
    case S_LEN:
      if (b > MAX_PAYLOAD) {
        errors_++;
        state_ = S_SYNC1;
        break;
      }
      len_ = b;
      idx_ = 0;
      crc_ = crc16Byte(crc_, b);
      state_ = len_ ? S_PAYLOAD : S_CRC1;
      break;
    case S_PAYLOAD:
      buf_[idx_++] = b;
      crc_ = crc16Byte(crc_, b);
      if (idx_ >= len_) state_ = S_CRC1;
      break;
    case S_CRC1:
      rxCrc_ = b;
      state_ = S_CRC2;
      break;
    case S_CRC2:
      rxCrc_ |= (uint16_t)b << 8;
      state_ = S_SYNC1;
      if (rxCrc_ == crc_) return true;
      errors_++;
      break;
  }
  return false;
}

}  // namespace proto
