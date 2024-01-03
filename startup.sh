#!/bin/bash

#Copy file to /etc/enigma2

killall -9 serialinput
/media/hdd/serialinput > /dev/null 2>&1 &