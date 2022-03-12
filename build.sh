#!/bin/bash

platform=$1
fqbn=$2

set -e
sudo apt-get update
curl -fsSL https://downloads.arduino.cc/arduino-1.8.19-linux64.tar.xz | tar Jxf -
mkdir ~/bin
ln -s ~/arduino*/arduino ~/bin/arduino
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/bin sh
mkdir -p ~/.arduino15/packages
PATH=~/bin:$PATH arduino-cli config init
PATH=~/bin:$PATH arduino-cli core update-index
PATH=~/bin:$PATH arduino-cli core install $platform
PATH=~/bin:$PATH arduino-cli lib install "MIDI Library"
echo "void PIOC_Handler (void) __attribute__ ((weak));" >> `find ~/.arduino15/ -name WInterrupts.c`
PATH=~/bin:$PATH arduino-cli compile --fqbn $fqbn vessel.ino
