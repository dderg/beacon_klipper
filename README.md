# Beacon Klipper — kalico-seam fork

Fork of [beacon3d/beacon_klipper](https://github.com/beacon3d/beacon_klipper)
ported to the rewritten motion engine on
[dderg/kalico `sota-motion`](https://github.com/dderg/kalico/tree/sota-motion).

Upstream beacon.py reaches into klipper internals the rewrite no longer has
(`HomingMove`, `MCU_trsync`/trdispatch, chelper trapq lookups). This branch
replaces that integration layer: everything touching the kalico motion engine
lives in `beacon_kalico.py` (homing provider contract, a single trsync with a
host-fed deadman, `motion_state_at` for streamed sample positions), while
`beacon.py` keeps the device protocol and delegates through one-line seams.
Decision record and design live in the kalico repo under
`docs/kalico-rewrite/beacon-fork-survey.md` and
`docs/superpowers/specs/2026-06-12-beacon-fork-seam-design.md`.

`install.sh` links both `beacon.py` and `beacon_kalico.py` into
`klippy/extras/`. Upstream documentation below still applies for the device
itself.

## Documentation

[Beacon](https://docs.beacon3d.com)

## Firmware Release Notes

### Beacon 2.1.0 - July 11, 2024
 - Added parameters to adjust contact noise tolerance
 - Adjusted contact latency values to match new parameters
 - Increased robustness of the primary contact trigger

### Beacon 2.0.1 - June 4, 2024
 - Fixed USB enumeration issue affecting fast host controllers

### Beacon 2.0.0 - May 29, 2024
 - Beacon Contact Release
 - Adopted RTIC - The Hardware Accelerated Rust RTOS
 - Added nozzle contact detection processing
 - Improved data transmit and processing efficiency
 - Reports MCU temperature and supply voltage
 - Added watchdog superviser
 - Improved error detection, reporting, and recovery
 - Reduced current consumption 10% overall
 - Reduced current consumption 55% when used above rated temperature

### Beacon 1.1.0 - Dec 27, 2023
 - RevH Enabling Release
 - Added Accel Driver

### Beacon 1.0.0 - Jan 26, 2023
 - Initial Release

