#!/bin/bash

ARDUINO="/usr/lib/arduino-1.8.12"
ROOT="/home/aysaha/Documents/AutoBot/arduino"

# check arguments
if [ "$#" -ne 1 ]; then
    echo "Usage: upload.sh port"
    exit 1
fi

# create directory
mkdir -p $ROOT/build

# build firmware
$ARDUINO/arduino-builder \
-compile \
-warnings=all \
-fqbn=arduino:avr:nano:cpu=atmega328 \
-hardware $ARDUINO/hardware \
-tools $ARDUINO/tools-builder \
-tools $ARDUINO/hardware/tools/avr \
-built-in-libraries $ARDUINO/libraries \
-prefs=build.warn_data_percentage=75 \
-prefs=runtime.tools.avr-gcc.path=$ARDUINO/hardware/tools/avr \
-prefs=runtime.tools.arduinoOTA.path=$ARDUINO/hardware/tools/avr \
-prefs=runtime.tools.avrdude.path=$ARDUINO/hardware/tools/avr \
-build-path $ROOT/build \
$ROOT/src/main.ino

# upload firmware
if [ "$?" -eq 0 ]; then
    $ARDUINO/hardware/tools/avr/bin/avrdude \
    -v \
    -C $ARDUINO/hardware/tools/avr/etc/avrdude.conf \
    -c arduino \
    -p atmega328p \
    -P $1 \
    -b 115200 \
    -D \
    -U flash:w:$ROOT/build/main.ino.hex:i
fi
