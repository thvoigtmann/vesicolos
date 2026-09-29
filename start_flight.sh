#!/bin/bash
cd $HOME/Desktop/vesicolos
export RASPI_MODEL=`cat /proc/device-tree/model | sed 's/.*Pi *\([345]\).*/\1/'`
source venv/bin/activate
if type -p screen >/dev/null 2>/dev/null; then
  SCR=screen
else
  SCR=""
fi
$SCR python3 ./vesicolos.py
