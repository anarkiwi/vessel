#!/bin/bash

platform=$1
fqbn=$2

set -e
sudo apt-get update && sudo apt-get install wget unzip
wget -O/tmp/ide.zip https://downloads.arduino.cc/arduino-ide/arduino-ide_2.0.2_Linux_64bit.zip
unzip /tmp/ide.zip
find arduino-ide* -name python3 -exec ln -sf $(which python3) {} \;
mkdir ~/bin
ln -s ~/arduino*/arduino-ide ~/bin/arduino
PATH=~/bin:$PATH curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/bin sh
mkdir -p ~/.arduino15/packages
PATH=~/bin:$PATH arduino-cli config init
PATH=~/bin:$PATH arduino-cli core update-index
PATH=~/bin:$PATH arduino-cli core install $platform
PATH=~/bin:$PATH arduino-cli lib install "MIDI Library"

if [[ "$platform" == "arduino:sam" ]] ; then
  WINT=$(find ~/.arduino15/packages/arduino/hardware/sam -name WInterrupts.c)
  if [[ "$WINT" != "" ]] ; then
    echo "void PIOC_Handler (void) __attribute__ ((weak));" >> $WINT
  fi
fi
PATH=~/bin:$PATH arduino-cli compile --fqbn $fqbn vessel.ino
