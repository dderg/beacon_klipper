# beacon_kalico.py
# Kalico integration seam for the Beacon fork. Everything that touches the
# kalico motion engine lives in this file; beacon.py keeps the device
# protocol and delegates here. Design:
# docs/superpowers/specs/2026-06-12-beacon-fork-seam-design.md (kalico repo).
import logging
import math

from klippy import pins
from klippy.bridge_endstop import RemoteBridgeEndstop

REASON_ENDSTOP_HIT = 1
REASON_HOST_REQUEST = 2
REASON_COMMS_TIMEOUT = 4

MODE_PROXIMITY = "proximity"
MODE_CONTACT = "contact"

Z_AXIS = 2

TRSYNC_WINDOW = 0.200
TRSYNC_HEARTBEAT = 0.050
TERMINAL_REASON_DEADLINE = 2.0
FUTURE_RETRY_PAUSE = 0.050
CRUISE_ACCEL_TOLERANCE = 1.0

ERR_FUTURE = "future"
ERR_BEFORE_WINDOW = "before_window"
ERR_NO_HISTORY = "no_history"


def is_cruise_acceleration(accel):
    return abs(accel) <= CRUISE_ACCEL_TOLERANCE


def classify_history_error(message):
    if "is in the future" in message:
        return ERR_FUTURE
    if "precedes retained motion history" in message:
        return ERR_BEFORE_WINDOW
    if "no motion history recorded" in message:
        return ERR_NO_HISTORY
    return None


class KalicoSeam:
    def __init__(self, beacon):
        self.beacon = beacon
        self.printer = beacon.printer
        self.mcu = beacon._mcu
        self.trsync_oid = self.mcu.create_oid()
        self.endstop = RemoteBridgeEndstop(
            self.printer, self.mcu, trsync_oid=self.trsync_oid
        )
        self.last_reason = None
        self._mode = None
        self._heartbeat_timer = None
        self._trsync_start_cmd = None
        self._trsync_set_timeout_cmd = None
        self._trsync_trigger_cmd = None
        self._dropped_samples = 0
        self.mcu.register_config_callback(self._build_config)
        self.mcu.register_response(
            self._handle_trsync_state, "trsync_state", self.trsync_oid
        )

    def _build_config(self):
        self.mcu.add_config_cmd("config_trsync oid=%d" % (self.trsync_oid,))
        self._trsync_start_cmd = self.mcu.lookup_command(
            "trsync_start oid=%c report_clock=%u report_ticks=%u"
            " expire_reason=%c"
        )
        self._trsync_set_timeout_cmd = self.mcu.lookup_command(
            "trsync_set_timeout oid=%c clock=%u"
        )
        self._trsync_trigger_cmd = self.mcu.lookup_command(
            "trsync_trigger oid=%c reason=%c"
        )

    def _handle_trsync_state(self, params):
        if not params["can_trigger"]:
            self.last_reason = params["trigger_reason"]

    def _arm_trsync(self):
        self.last_reason = None
        self._trsync_start_cmd.send(
            [self.trsync_oid, 0, 0, REASON_COMMS_TIMEOUT]
        )
        reactor = self.printer.get_reactor()
        self._send_heartbeat(reactor.monotonic())
        self._heartbeat_timer = reactor.register_timer(
            self._heartbeat, reactor.monotonic() + TRSYNC_HEARTBEAT
        )

    def _send_heartbeat(self, eventtime):
        expire = self.mcu.estimated_print_time(eventtime) + TRSYNC_WINDOW
        self._trsync_set_timeout_cmd.send(
            [self.trsync_oid, self.mcu.print_time_to_clock(expire)]
        )

    def _heartbeat(self, eventtime):
        self._send_heartbeat(eventtime)
        return eventtime + TRSYNC_HEARTBEAT

    def trip_move_begin(self, entry):
        mode = self._mode if self._mode is not None else MODE_PROXIMITY
        beacon = self.beacon
        if mode == MODE_PROXIMITY:
            if beacon.model is None:
                raise self.printer.command_error("No Beacon model loaded")
            beacon._apply_threshold()
            beacon._sample_async()
            self._arm_trsync()
            beacon.beacon_home_cmd.send(
                [self.trsync_oid, REASON_ENDSTOP_HIT, 0]
            )
        else:
            self._check_hotend_temp()
            beacon._sample_async()
            self._arm_trsync()
            if beacon.beacon_contact_set_latency_min_cmd is not None:
                beacon.beacon_contact_set_latency_min_cmd.send(
                    [beacon.contact_latency_min]
                )
            if beacon.beacon_contact_set_sensitivity_cmd is not None:
                beacon.beacon_contact_set_sensitivity_cmd.send(
                    [beacon.contact_sensitivity]
                )
            beacon.beacon_contact_home_cmd.send(
                [self.trsync_oid, REASON_ENDSTOP_HIT, 0]
            )

    def _check_hotend_temp(self):
        contact_probe = self.beacon.mcu_contact_probe
        toolhead = self.printer.lookup_object("toolhead")
        extruder = toolhead.get_extruder()
        if extruder is None or contact_probe is None:
            return
        curtime = self.printer.get_reactor().monotonic()
        cur_temp = extruder.get_heater().get_status(curtime)["temperature"]
        if cur_temp >= contact_probe.max_hotend_temp:
            raise self.printer.command_error(
                "Current hotend temperature %.1f exceeds maximum allowed"
                " temperature %.1f" % (cur_temp, contact_probe.max_hotend_temp)
            )

    def trip_move_end(self, entry):
        reactor = self.printer.get_reactor()
        if self._heartbeat_timer is not None:
            reactor.unregister_timer(self._heartbeat_timer)
            self._heartbeat_timer = None
        mode = self._mode if self._mode is not None else MODE_PROXIMITY
        beacon = self.beacon
        if mode == MODE_PROXIMITY:
            beacon.beacon_stop_home_cmd.send([])
        else:
            beacon.beacon_contact_stop_home_cmd.send([])
        self._trsync_trigger_cmd.send([self.trsync_oid, REASON_HOST_REQUEST])
        deadline = reactor.monotonic() + TERMINAL_REASON_DEADLINE
        while self.last_reason is None:
            if reactor.monotonic() > deadline:
                raise self.printer.command_error(
                    "beacon: no terminal trsync_state received after homing"
                )
            reactor.pause(reactor.monotonic() + 0.010)
        if self.last_reason not in (REASON_ENDSTOP_HIT, REASON_HOST_REQUEST):
            raise self.printer.command_error(
                "beacon: trsync terminated with reason %d"
                % (self.last_reason,)
            )

    def setup_bridge_endstop(self, pin_params, axis):
        if pin_params["pin"] != "z_virtual_endstop" or axis != Z_AXIS:
            raise pins.error(
                "beacon only provides z_virtual_endstop on the Z axis"
            )
        if pin_params["invert"] or pin_params["pullup"]:
            raise pins.error(
                "Can not pullup/invert beacon virtual endstop"
            )
        return self.endstop

    def measured_trip_position(self, axis, trip_pos, final_pos):
        if self.last_reason != REASON_ENDSTOP_HIT:
            raise self.printer.command_error(
                "beacon: homing completed with trsync reason %s, not"
                " endstop-hit" % (self.last_reason,)
            )
        beacon = self.beacon
        if beacon.model is None:
            return None
        dist, samples = beacon._sample(beacon.z_settling_time, 10)
        if math.isinf(dist):
            logging.error(
                "beacon post-homing adjustment measured samples %s", samples
            )
            raise self.printer.command_error(
                "Toolhead stopped below model range"
            )
        return dist

    def _bridge(self):
        return self.printer.lookup_object("motion_bridge")

    def position_at_clock(self, clock64):
        state = self._motion_state(int(clock64))
        if state is None:
            return None
        try:
            return [state["x"][0], state["y"][0], state["z"][0]]
        except KeyError:
            return None

    def position_at_clock32(self, clock32):
        return self.position_at_clock(self.mcu.clock32_to_clock64(clock32))

    def _motion_state(self, clock64, retried=False):
        try:
            return self._bridge().motion_state_at(self.mcu, clock=clock64)
        except RuntimeError as e:
            kind = classify_history_error(str(e))
            if kind == ERR_NO_HISTORY:
                return None
            if kind == ERR_BEFORE_WINDOW:
                self._dropped_samples += 1
                if self._dropped_samples == 1:
                    logging.warning(
                        "beacon: dropping stream sample older than retained"
                        " motion history: %s", e
                    )
                return None
            if kind == ERR_FUTURE and not retried:
                reactor = self.printer.get_reactor()
                reactor.pause(reactor.monotonic() + FUTURE_RETRY_PAUSE)
                return self._motion_state(clock64, retried=True)
            raise

    def proximity_descend(self, gcmd, bottom_z, speed):
        self._descend(gcmd, MODE_PROXIMITY, bottom_z, speed)

    def contact_descend(self, gcmd, bottom_z, speed):
        trip_pos, final_pos = self._descend(
            gcmd, MODE_CONTACT, bottom_z, speed
        )
        detect_clock = self._query_detect_clock()
        state = self._bridge().motion_state_at(
            self.mcu, clock=self.mcu.clock32_to_clock64(detect_clock)
        )
        z_pos, z_vel, z_accel = state["z"]
        if not is_cruise_acceleration(z_accel):
            raise self.printer.command_error(
                "beacon: contact triggered while %s (z accel %.3f mm/s^2)"
                % ("decelerating" if z_accel * z_vel > 0 else "accelerating",
                   z_accel)
            )
        return [final_pos[0], final_pos[1], z_pos]

    def _descend(self, gcmd, mode, bottom_z, speed):
        printer = self.printer
        toolhead = printer.lookup_object("toolhead")
        homing_obj = printer.lookup_object("homing")
        bridge = printer.lookup_object("motion_bridge")
        if gcmd is None:
            gcode = printer.lookup_object("gcode")
            gcmd = gcode.create_gcode_command(
                "BEACON_DESCEND", "BEACON_DESCEND", {}
            )
        start_z = toolhead.get_position()[Z_AXIS]
        max_travel = start_z - bottom_z
        if max_travel <= 0.0:
            raise printer.command_error(
                "beacon: descend target %.3f is not below current Z %.3f"
                % (bottom_z, start_z)
            )
        self._mode = mode
        try:
            trip_pos, final_pos = homing_obj.trip_move(
                gcmd,
                toolhead,
                bridge,
                Z_AXIS,
                -1.0,
                speed,
                max_travel,
                {
                    "endstop": self.endstop,
                    "provider": self.beacon,
                    "trigger_height": None,
                },
            )
        finally:
            self._mode = None
        if self.last_reason != REASON_ENDSTOP_HIT:
            raise printer.command_error(
                "beacon: descend completed with trsync reason %s, not"
                " endstop-hit" % (self.last_reason,)
            )
        newpos = list(toolhead.get_position())
        newpos[Z_AXIS] = final_pos[Z_AXIS]
        toolhead.set_position(newpos)
        return trip_pos, final_pos

    def _query_detect_clock(self):
        beacon = self.beacon
        reactor = self.printer.get_reactor()
        deadline = reactor.monotonic() + 0.5
        while True:
            ret = beacon.beacon_contact_query_cmd.send([])
            if ret["triggered"]:
                return ret["detect_clock"]
            now = reactor.monotonic()
            if now >= deadline:
                raise self.printer.command_error(
                    "Timeout getting contact time"
                )
            reactor.pause(now + 0.001)
