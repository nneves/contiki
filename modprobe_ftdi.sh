#!/bin/bash
sudo modprobe ftdi-sio vendor=0x403 product=0xa6d1
lsmod | grep "ftdi"
ls -la /dev/ttyUSB*