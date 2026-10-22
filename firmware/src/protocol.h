// AquaBot — protocolo serial PC <-> ROV. Sin dependencias de Arduino (se prueba en el PC).
// Debe coincidir con pc/aquabot_pc/protocol.py y con docs/diseno_software.md.
//
// Trama: AA 55 | TIPO | LEN | PAYLOAD | CRC16 (LSB, MSB)
// CRC16-CCITT-FALSE (0x1021, init 0xFFFF) sobre TIPO, LEN y PAYLOAD. Little-endian.
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace proto {

constexpr uint8_t SYNC1 = 0xAA;
constexpr uint8_t SYNC2 = 0x55;
constexpr uint8_t MAX_PAYLOAD = 64;
constexpr size_t MAX_FRAME = MAX_PAYLOAD + 6;

enum MsgType : uint8_t {
  CMD = 0x01,
  PARAM_SET = 0x02,
  PARAM_GET = 0x03,
  TELEM = 0x81,
  PARAM_VALUE = 0x82,
};

enum CmdFlag : uint8_t {
  CF_ARM = 1 << 0,
  CF_PITCH_HOLD = 1 << 1,
  CF_DEPTH_HOLD = 1 << 2,
  CF_ZERO_DEPTH = 1 << 3,  // flanco de subida
  CF_LEVEL_IMU = 1 << 4,   // flanco de subida, solo desarmado
  CF_REPLY = 1 << 5,       // 22 oct: responder con telemetría solo si el PC la pide (comandos redundantes)
};

enum TelFlag : uint16_t {
  TF_ARMED = 1 << 0,
  TF_PITCH_HOLD = 1 << 1,
  TF_DEPTH_HOLD = 1 << 2,
  TF_LINK_FAILSAFE = 1 << 3,
  TF_SURFACING = 1 << 4,
  TF_LOWBAT_WARN = 1 << 5,
  TF_LOWBAT_CRIT = 1 << 6,
  TF_LIM_FULL = 1 << 7,
  TF_LIM_EMPTY = 1 << 8,
  TF_DEPTH_OK = 1 << 9,
  TF_IMU_OK = 1 << 10,
  TF_TEMP_OK = 1 << 11,
  TF_BALLAST_FAULT = 1 << 12,
  TF_BALLAST_POS_KNOWN = 1 << 13,
};

#pragma pack(push, 1)
struct Cmd {
  uint8_t seq;
  uint8_t flags;
  int16_t surge;  // -1000..1000, + adelante
  int16_t yaw;    // + derecha
  int16_t pitch;  // con hold: setpoint (x PITCH_SP_MAX); sin hold: empuje de M5
  int16_t vert;   // + subir
  uint8_t power;  // 0..100 %
  uint8_t reserved;
};

struct Telem {
  uint8_t seq;
  uint16_t flags;
  int16_t depth_mm;
  int16_t depth_sp_mm;
  int16_t vz_mms;
  uint16_t pressure_mbar;
  int16_t water_temp_cC;
  int16_t ms_temp_cC;
  int16_t pitch_cdeg;
  int16_t roll_cdeg;
  int16_t pitch_sp_cdeg;
  uint16_t vbat_mV;
  uint8_t ballast_pos;  // 0..100 %, 255 = desconocida
  int8_t ballast_dir;   // -1 vaciando, 0, +1 llenando
  int8_t motor[5];      // -100..100 %
  uint32_t uptime_ms;
  uint16_t rx_err;
};

struct ParamSet {
  uint8_t id;
  uint8_t save;
  float value;
};

struct ParamGet {
  uint8_t id;
};

struct ParamValue {
  uint8_t id;  // 0xFF = inválido
  uint8_t count;
  float value;
};
#pragma pack(pop)

static_assert(sizeof(Cmd) == 12, "Cmd debe medir 12 bytes");
static_assert(sizeof(Telem) == 36, "Telem debe medir 36 bytes");
static_assert(sizeof(ParamSet) == 6, "ParamSet debe medir 6 bytes");
static_assert(sizeof(ParamValue) == 6, "ParamValue debe medir 6 bytes");

uint16_t crc16(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF);
uint16_t crc16Byte(uint16_t crc, uint8_t b);

// Arma una trama completa en out (tamaño mínimo len + 6). Devuelve el número de bytes.
size_t encode(uint8_t type, const void* payload, uint8_t len, uint8_t* out);

class Parser {
 public:
  // Alimenta un byte. Devuelve true cuando hay una trama válida completa.
  bool feed(uint8_t b);
  void reset() { state_ = S_SYNC1; }
  bool midFrame() const { return state_ != S_SYNC1; }
  uint8_t type() const { return type_; }
  uint8_t len() const { return len_; }
  const uint8_t* payload() const { return buf_; }
  uint32_t errors() const { return errors_; }

 private:
  enum State : uint8_t { S_SYNC1, S_SYNC2, S_TYPE, S_LEN, S_PAYLOAD, S_CRC1, S_CRC2 };
  State state_ = S_SYNC1;
  uint8_t type_ = 0, len_ = 0, idx_ = 0;
  uint16_t crc_ = 0, rxCrc_ = 0;
  uint32_t errors_ = 0;
  uint8_t buf_[MAX_PAYLOAD];
};

}  // namespace proto
