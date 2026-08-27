#!/bin/bash
export NO_GPIO=1
source mock/venv/bin/activate
python3 ./vesicolos.py test_config
