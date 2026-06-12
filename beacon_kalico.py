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


def classify_history_error(message):
    if "is in the future" in message:
        return ERR_FUTURE
    if "precedes retained motion history" in message:
        return ERR_BEFORE_WINDOW
    if "no motion history recorded" in message:
        return ERR_NO_HISTORY
    return None
