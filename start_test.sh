#!/bin/bash
cd $HOME/Desktop/vesicolos
export RASPI_MODEL=`cat /proc/device-tree/model | sed 's/.*Pi *\([345]\).*/\1/'`
export DEBUG=1
source venv/bin/activate
#python3 ./tests/new_driver.py
python3 ./vesicolos.py test_config
