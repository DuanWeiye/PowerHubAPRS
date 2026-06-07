#!/bin/bash
stty -F /dev/ttyACM0 115200 && cat /dev/ttyACM0 | tr -d '\r'
