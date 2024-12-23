#!/bin/sh

set -ex


./off.sh || true
make
#./send.py
./send_sens.py
sleep 20
./on.sh
#scp -r build_output/firmware/tasmota-minimal.bin.gz build_output/firmware/tasmota-sensors.bin.gz asm:
#mount /flash
#cp build_output/firmware/tasmota-minimal.bin.gz build_output/firmware/tasmota-sensors.bin.gz  /flash
#umount /flash
