#!/bin/sh

set -ex


make
#./send.py
./send_sens.py
#scp -r build_output/firmware/tasmota-minimal.bin.gz build_output/firmware/tasmota-sensors.bin.gz asm:
#mount /flash
#cp build_output/firmware/tasmota-minimal.bin.gz build_output/firmware/tasmota-sensors.bin.gz  /flash
#umount /flash
