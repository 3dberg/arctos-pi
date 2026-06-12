"""Config loading round-trips the homing fields and stays backward-compatible."""
import textwrap
from pathlib import Path

from backend.config import AppConfig

REPO = Path(__file__).resolve().parent.parent


def test_load_without_homing_keys_uses_defaults(tmp_path):
    p = tmp_path / "config.yaml"
    p.write_text(textwrap.dedent("""
        can: { backend: mock }
        axes:
          - { can_id: 1, name: J1, gear_ratio: 13.5, pulses_per_rev: 3200 }
    """))
    cfg = AppConfig.load(p)
    assert cfg.require_home_before_move is True  # default
    ax = cfg.axis_by_id(1)
    assert ax.home_enabled is True
    assert ax.home_dir == 0
    assert ax.home_seek_max_deg == 400.0


def test_load_with_homing_keys_roundtrips(tmp_path):
    p = tmp_path / "config.yaml"
    p.write_text(textwrap.dedent("""
        can: { backend: mock }
        require_home_before_move: false
        axes:
          - { can_id: 2, name: J2, home_enabled: false, home_dir: 1, home_speed: 333,
              home_trig_low: false, home_offset_deg: -12.5, home_seek_max_deg: 123.0,
              home_order: 4, end_limit: false }
    """))
    cfg = AppConfig.load(p)
    assert cfg.require_home_before_move is False
    ax = cfg.axis_by_id(2)
    assert ax.home_enabled is False
    assert ax.home_dir == 1
    assert ax.home_speed == 333
    assert ax.home_trig_low is False
    assert ax.home_offset_deg == -12.5
    assert ax.home_seek_max_deg == 123.0
    assert ax.home_order == 4
    assert ax.end_limit is False


def test_example_config_parses():
    cfg = AppConfig.load(REPO / "config.example.yaml")
    assert len(cfg.axes) == 6
    assert cfg.require_home_before_move is True
    assert all(ax.home_enabled for ax in cfg.axes)
    # home_order is set per axis in the example
    assert [ax.home_order for ax in cfg.axes] == [0, 1, 2, 3, 4, 5]


def test_driver_config_fields_default_none_and_roundtrip(tmp_path):
    p = tmp_path / "config.yaml"
    p.write_text(textwrap.dedent("""
        can: { backend: mock }
        axes:
          - { can_id: 1, name: J1 }
          - { can_id: 3, name: J3, default_work_mode: 5, hold_current_ma: 1200 }
    """))
    cfg = AppConfig.load(p)
    j1 = cfg.axis_by_id(1)
    assert j1.default_work_mode is None   # unset -> driver left as-is
    assert j1.hold_current_ma is None
    j3 = cfg.axis_by_id(3)
    assert j3.default_work_mode == 5      # SR_vFOC
    assert j3.hold_current_ma == 1200
