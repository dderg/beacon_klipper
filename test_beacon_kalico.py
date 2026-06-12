# test_beacon_kalico.py
import sys
import types


def _install_klippy_stubs():
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
    klippy.bridge_endstop = bridge_endstop
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


class FakeReactor:
    NEVER = 9e99

    def __init__(self):
        self.now = 100.0
        self.timers = []
        self.paused = []

    def monotonic(self):
        return self.now

    def register_timer(self, cb, when):
        self.timers.append((cb, when))
        return (cb, when)

    def unregister_timer(self, handle):
        self.timers.remove(handle)

    def pause(self, until):
        self.paused.append(until)
        self.now = max(self.now, until)


class FakeCommandError(Exception):
    pass


class FakePrinter:
    def __init__(self, objects=None):
        self.command_error = FakeCommandError
        self.config_error = FakeCommandError
        self.reactor = FakeReactor()
        self.objects = objects or {}

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default="__raise__"):
        if name in self.objects:
            return self.objects[name]
        if default != "__raise__":
            return default
        raise FakeCommandError("missing object %s" % name)


class FakeCommand:
    def __init__(self, log):
        self.log = log

    def send(self, args=()):
        self.log.append(list(args))


class FakeMcu:
    def __init__(self):
        self.oids = 0
        self.config_cmds = []
        self.responses = {}
        self.sent = {}
        self.config_cbs = []

    def create_oid(self):
        self.oids += 1
        return self.oids

    def register_config_callback(self, cb):
        self.config_cbs.append(cb)

    def add_config_cmd(self, cmd):
        self.config_cmds.append(cmd)

    def lookup_command(self, fmt, cq=None):
        name = fmt.split()[0]
        self.sent.setdefault(name, [])
        mcu = self

        class LiveCommand:
            def send(self, args=()):
                mcu.sent[name].append(list(args))

        return LiveCommand()

    def register_response(self, cb, name, oid=None):
        self.responses[(name, oid)] = cb

    def estimated_print_time(self, eventtime):
        return eventtime

    def print_time_to_clock(self, print_time):
        return int(print_time * 1000)

    def clock32_to_clock64(self, clock32):
        return clock32


class FakeBeacon:
    def __init__(self, printer, mcu):
        self.printer = printer
        self._mcu = mcu
        self.model = object()
        self.trigger_distance = 2.0
        self.z_settling_time = 1
        self.applied_thresholds = 0
        self.sampled_async = 0
        self.cmd_log = {}
        for name in (
            "beacon_home_cmd",
            "beacon_stop_home_cmd",
            "beacon_contact_home_cmd",
            "beacon_contact_stop_home_cmd",
        ):
            log = []
            self.cmd_log[name] = log
            setattr(self, name, FakeCommand(log))
        self.beacon_contact_set_latency_min_cmd = None
        self.beacon_contact_set_sensitivity_cmd = None
        self.contact_latency_min = 0
        self.contact_sensitivity = 0
        self.mcu_contact_probe = None

    def _apply_threshold(self):
        self.applied_thresholds += 1

    def _sample_async(self):
        self.sampled_async += 1
        return {"freq": 1.0, "dist": 2.0, "temp": 25.0}


def make_seam():
    printer = FakePrinter()
    mcu = FakeMcu()
    beacon = FakeBeacon(printer, mcu)
    seam = beacon_kalico.KalicoSeam(beacon)
    for cb in mcu.config_cbs:
        cb()
    return seam, beacon, printer, mcu


def test_seam_config_allocates_trsync():
    seam, beacon, printer, mcu = make_seam()
    assert mcu.config_cmds == ["config_trsync oid=%d" % seam.trsync_oid]
    assert ("trsync_state", seam.trsync_oid) in mcu.responses


def test_terminal_reason_recorded():
    seam, beacon, printer, mcu = make_seam()
    handler = mcu.responses[("trsync_state", seam.trsync_oid)]
    handler({"can_trigger": 1, "trigger_reason": 0})
    assert seam.last_reason is None
    handler({"can_trigger": 0, "trigger_reason": 1})
    assert seam.last_reason == beacon_kalico.REASON_ENDSTOP_HIT


def test_proximity_begin_arms_device_and_heartbeat():
    seam, beacon, printer, mcu = make_seam()
    seam.trip_move_begin({"endstop": seam.endstop, "provider": beacon,
                          "trigger_height": 2.0})
    assert beacon.applied_thresholds == 1
    assert beacon.sampled_async == 1
    assert mcu.sent["trsync_start"] == [
        [seam.trsync_oid, 0, 0, beacon_kalico.REASON_COMMS_TIMEOUT]
    ]
    assert beacon.cmd_log["beacon_home_cmd"] == [
        [seam.trsync_oid, beacon_kalico.REASON_ENDSTOP_HIT, 0]
    ]
    assert len(mcu.sent["trsync_set_timeout"]) == 1
    assert len(printer.reactor.timers) == 1


def test_proximity_begin_requires_model():
    seam, beacon, printer, mcu = make_seam()
    beacon.model = None
    try:
        seam.trip_move_begin({"endstop": seam.endstop, "provider": beacon,
                              "trigger_height": 2.0})
        assert False, "expected command_error"
    except FakeCommandError:
        pass


def test_trip_move_end_forces_terminal_and_accepts_host_request():
    seam, beacon, printer, mcu = make_seam()
    seam.trip_move_begin({"endstop": seam.endstop, "provider": beacon,
                          "trigger_height": 2.0})
    handler = mcu.responses[("trsync_state", seam.trsync_oid)]

    real_send = mcu.sent["trsync_trigger"].append

    def trigger_and_report(args):
        real_send(args)
        handler({"can_trigger": 0,
                 "trigger_reason": beacon_kalico.REASON_HOST_REQUEST})

    mcu.sent["trsync_trigger"] = type(
        "L", (list,), {"append": lambda self, a: trigger_and_report(a)}
    )()
    seam.trip_move_end({})
    assert beacon.cmd_log["beacon_stop_home_cmd"] == [[]]
    assert printer.reactor.timers == []
    assert seam.last_reason == beacon_kalico.REASON_HOST_REQUEST


def test_trip_move_end_raises_on_comms_timeout():
    seam, beacon, printer, mcu = make_seam()
    seam.trip_move_begin({"endstop": seam.endstop, "provider": beacon,
                          "trigger_height": 2.0})
    handler = mcu.responses[("trsync_state", seam.trsync_oid)]
    handler({"can_trigger": 0,
             "trigger_reason": beacon_kalico.REASON_COMMS_TIMEOUT})
    try:
        seam.trip_move_end({})
        assert False, "expected command_error"
    except FakeCommandError:
        pass
