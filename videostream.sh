#!/bin/bash
ffplay -i udp://192.168.100.42:3333 -fflags nobuffer -flags low_delay -framedrop
