#!/bin/bash
#for Xavior Board
REGPIN=332
REGOFF=0
REGON=1

echo $REGPIN > /sys/class/gpio/export 2>/dev/null
echo out > /sys/class/gpio/gpio"$REGPIN"/direction
echo REG OFF
echo $REGOFF > /sys/class/gpio/gpio"$REGPIN"/value
sleep 0.5
echo $REGON > /sys/class/gpio/gpio"$REGPIN"/value
echo REG ON

echo $REGPIN > /sys/class/gpio/unexport 2>/dev/null

