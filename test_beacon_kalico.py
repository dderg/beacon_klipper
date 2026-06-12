# test_beacon_kalico.py
import sys
import types


def _install_klippy_stubs():
    if "klippy" in sys.modules:
        return
    klippy = types.ModuleType("klippy")
    pins = types.ModuleType("klippy.pins")

    class PinsError(Exception):
        pass

    pins.error = PinsError
    bridge_endstop = types.ModuleType("klippy.bridge_endstop")

    class FakeRemoteBridgeEndstop:
        def __init__(self, printer, mcu, trsync_oid):
            self.printer = printer
            self.mcu = mcu
            self.trsync_oid = trsync_oid
            self.endstop_id = 99

    bridge_endstop.RemoteBridgeEndstop = FakeRemoteBridgeEndstop
    klippy.pins = pins
    sys.modules["klippy"] = klippy
    sys.modules["klippy.pins"] = pins
    sys.modules["klippy.bridge_endstop"] = bridge_endstop


_install_klippy_stubs()

import beacon_kalico  # noqa: E402


def test_classify_future():
    msg = (
        "motion_state_at: query clock 123 is in the future for axis "
        "AxisKey { mcu_id: 1, axis: 2 } (now≈100) — motion history "
        "answers the past only"
    )
    assert beacon_kalico.classify_history_error(msg) == beacon_kalico.ERR_FUTURE


def test_classify_before_window():
    msg = "query clock 5 precedes retained motion history for axis ..."
    assert (
        beacon_kalico.classify_history_error(msg)
        == beacon_kalico.ERR_BEFORE_WINDOW
    )


def test_classify_no_history():
    msg = "no motion history recorded for axis AxisKey { mcu_id: 1, axis: 2 }"
    assert (
        beacon_kalico.classify_history_error(msg)
        == beacon_kalico.ERR_NO_HISTORY
    )


def test_classify_unknown_is_none():
    assert beacon_kalico.classify_history_error("segfault adjacent") is None
