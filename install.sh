#!/bin/bash

KDIR="${HOME}/klipper"
KENV="${HOME}/klippy-env"

BKDIR="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

if [ ! -d "$KDIR" ] || [ ! -d "$KENV" ]; then
    echo "beacon: klipper or klippy env doesn't exist"
    exit 1
fi

# install beacon requirements to env
echo "beacon: installing python requirements to env, this may take 10+ minutes."
"${KENV}/bin/pip" install -r "${BKDIR}/requirements.txt"

# link klippy to the beacon sources. kalico auto-imports every module in
# extras/, so a leftover beacon_kalico.py (renamed to beacon_motion_engine.py)
# would be enumerated and fail to import every boot — remove it.
echo "beacon: linking klippy to beacon sources."
rm -f "${KDIR}/klippy/extras/beacon_kalico.py"
for f in beacon.py beacon_motion_engine.py; do
    dest="${KDIR}/klippy/extras/${f}"
    if [ -e "$dest" ] || [ -L "$dest" ]; then
        rm "$dest"
    fi
    ln -s "${BKDIR}/${f}" "$dest"
    # exclude the linked file from klipper git tracking
    if ! grep -q "klippy/extras/${f}" "${KDIR}/.git/info/exclude"; then
        echo "klippy/extras/${f}" >> "${KDIR}/.git/info/exclude"
    fi
done
echo "beacon: installation successful."

echo "Updating firmware."
"$KENV/bin/python" "$BKDIR/update_firmware.py" update all
