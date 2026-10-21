"""La detección debe encontrar el cubo, las monedas y el objeto genérico en la escena sintética
(con la configuración por defecto: genérico + naranja), y no inventar objetos sobre el fondo vacío."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from aquabot_pc.detection import Detector  # noqa: E402
from aquabot_pc.video import SyntheticScene  # noqa: E402


class FakeSim:
    def __init__(self, x, y, heading=0.0, depth=1.0):
        self.p = (x, y, heading, depth, 0.0)

    def pose(self):
        return self.p


def run(x, y, depth, frames=10, cfg=None):
    scene = SyntheticScene(FakeSim(x, y, 0.0, depth))
    det = Detector(cfg)
    tracks = []
    for _ in range(frames):
        tracks = det.process(scene.render())
    return {t.cls for t in tracks if t.stable}


def test_cube_detected():
    assert "cubo" in run(3.0, 0.8, 1.0)


def test_coins_detected_close_to_bottom():
    # por defecto las monedas salen como objeto genérico
    assert run(2.1, -0.45, 1.5) == {"generico"}


def test_coin_detector_optional():
    assert "moneda" in run(2.1, -0.45, 1.5, cfg={"moneda": {"activo": True}})


def test_generic_only_finds_everything():
    only = {"cubo": {"activo": False}}
    assert run(3.0, 0.8, 1.0, cfg=only) == {"generico"}
    assert run(2.1, -0.45, 1.5, cfg=only) == {"generico"}
    assert run(0.0, 2.5, 1.0, cfg=only) == set()


def test_generic_object_detected():
    assert "generico" in run(5.0, -1.0, 1.2)


def test_empty_floor_no_false_positives():
    assert run(0.0, 2.5, 1.0) == set()  # incluye una línea de carril
