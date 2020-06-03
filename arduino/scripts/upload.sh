#!/bin/bash

ARDUINO='C:\Program Files (x86)\Arduino'
ROOT='C:\Users\Ayusman\Documents\AutoBot\arduino'

# check arguments
if [ "$#" -ne 1 ]; then
    echo "Usage: upload.sh port"
    exit 1
fi

# create directory
mkdir -p "$(wslpath "$ROOT\build")"

# build firmware
"$(wslpath "$ARDUINO\arduino-builder.exe")" \
-compile \
-warnings=all \
-fqbn=arduino:avr:nano:cpu=atmega328 \
-hardware 'C:\Program Files (x86)\Arduino\hardware' \
-tools 'C:\Program Files (x86)\Arduino\tools-builder' \
-tools 'C:\Program Files (x86)\Arduino\hardware\tools\avr' \
-built-in-libraries 'C:\Program Files (x86)\Arduino\libraries' \
-prefs=build.warn_data_percentage=75 \
-prefs=runtime.tools.avr-gcc.path='C:\Program Files (x86)\Arduino\hardware\tools\avr' \
-prefs=runtime.tools.arduinoOTA.path='C:\Program Files (x86)\Arduino\hardware\tools\avr' \
-prefs=runtime.tools.avrdude.path='C:\Program Files (x86)\Arduino\hardware\tools\avr' \
-build-path "$ROOT\build" \
"$ROOT\src\main.ino"

# upload firmware
if [ "$?" -eq 0 ]; then
    "$(wslpath "$ARDUINO\hardware\tools\avr\bin\avrdude.exe")" \
    -v \
    -C 'C:\Program Files (x86)\Arduino\hardware\tools\avr\etc\avrdude.conf' \
    -c arduino \
    -p atmega328p \
    -P $1 \
    -b 115200 \
    -D \
    -U flash:w:"$ROOT\build\main.ino.hex":i
fi
