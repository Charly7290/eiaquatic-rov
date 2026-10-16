"""Pruebas del protocolo, incluida la compatibilidad con las tramas que genera el firmware (C++).

  cd firmware/test_host && g++ -std=c++17 -I../src test_host.cpp ../src/protocol.cpp ../src/control.cpp -o test_host && ./test_host
  cd pc && python -m pytest tests
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from aquabot_pc import protocol as P  # noqa: E402

VECTORS = Path(__file__).resolve().parents[2] / "firmware" / "test_host" / "vectors.txt"


def test_crc_check_value():
    assert P.crc16(b"123456789") == 0x29B1


def test_roundtrip_command():
    c = P.Command(seq=7, flags=P.CF_ARM | P.CF_DEPTH_HOLD, surge=0.5, yaw=-0.25, pitch=1.0, vert=-1.0, power=80)
    frames = P.Parser().feed(b"\x00\xaa\x13" + P.encode(P.CMD, c.pack()))
    assert len(frames) == 1 and frames[0][0] == P.CMD
    d = P.Command.unpack(frames[0][1])
    assert (d.surge, d.yaw, d.pitch, d.vert, d.power) == (0.5, -0.25, 1.0, -1.0, 80)


def test_corrupt_frame_rejected_and_resync():
    p = P.Parser()
    good = P.encode(P.CMD, P.Command().pack())
    bad = bytearray(good)
    bad[6] ^= 1
    frames = p.feed(bytes(bad) + good)
    assert len(frames) == 1 and p.errors == 1


def test_split_feed():
    p = P.Parser()
    f = P.encode(P.CMD, P.Command(surge=0.3).pack())
    out = []
    for b in f:
        out += p.feed(bytes([b]))
    assert len(out) == 1


def test_params_table_matches_firmware():
    src = (Path(__file__).resolve().parents[2] / "firmware" / "src" / "params.cpp").read_text(encoding="utf-8")
    import re

    names = re.findall(r'\{"([A-Z_]+)",\s*([-0-9.]+)f', src)
    assert [n for n, _ in names] == [p.name for p in P.PARAMS]
    for (n, d), p in zip(names, P.PARAMS):
        assert abs(float(d) - p.default) < 1e-9, n


@pytest.mark.skipif(not VECTORS.exists(), reason="ejecute primero firmware/test_host/test_host")
def test_firmware_vectors():
    vec = dict(line.split() for line in VECTORS.read_text().splitlines() if line.strip())
    # CMD generado en C++
    (t, payload), = P.Parser().feed(bytes.fromhex(vec["CMD"]))
    c = P.Command.unpack(payload)
    assert t == P.CMD and c.seq == 7 and c.surge == 0.5 and c.yaw == -0.25 and c.power == 80
    assert P.encode(P.CMD, P.Command(7, P.CF_ARM | P.CF_DEPTH_HOLD, 0.5, -0.25, 1.0, -1.0, 80).pack()) \
        == bytes.fromhex(vec["CMD"])
    # TELEM generada en C++
    (t, payload), = P.Parser().feed(bytes.fromhex(vec["TELEM"]))
    tel = P.Telemetry.unpack(payload)
    assert t == P.TELEM
    assert tel.depth == 1.234 and tel.depth_sp == 1.0 and tel.vz == -0.025
    assert tel.pressure_mbar == 1135 and tel.vbat == 11.42 and tel.ballast_pos == 0.42 and tel.ballast_dir == -1
    assert tel.pitch == -1.5 and tel.roll == 0.75 and tel.motors == [0.5, 0.5, -0.3, -0.3, 0.12]
    assert tel.uptime == 123.456 and tel.rx_err == 3 and tel.has(P.TF_ARMED) and tel.water_temp is None
    # PARAM_VALUE
    (t, payload), = P.Parser().feed(bytes.fromhex(vec["PARAM_VALUE"]))
    pid, count, value = P.unpack_param_value(payload)
    assert (pid, count) == (5, 23) and abs(value - 0.1) < 1e-6
