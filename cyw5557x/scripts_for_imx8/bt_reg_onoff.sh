#!/bin/bash
#for IMX8
REGPIN=gpiochip5
OFFSET=0
REGOFF=0
REGON=1

echo REG OFF
gpioset $REGPIN $OFFSET=$REGOFF
sleep 0.5
gpioset $REGPIN $OFFSET=$REGON
echo REG ON


