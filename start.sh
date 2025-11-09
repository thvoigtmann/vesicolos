#!/bin/bash
source venv/bin/activate
python3 ./vesicolos.py
# the following to keep the heater off when program exits
python3 ./tests/heater.py
