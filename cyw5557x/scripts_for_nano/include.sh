#!/bin/bash

##
# Common options
##
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

##
# Set paths
##
BT_TARGET_DIR="./"
RLS_ROOT=$SCRIPT_DIR'/../'
BT_RLS_BIN=${SCRIPT_DIR}
BT_RLS_FW=${RLS_ROOT}${BT_TARGET_DIR}

##
# Setup for BT
##
BT_MBT=${BT_RLS_BIN}/mbt
# FOR NVIDIA NANO
BT_MBT_PORT=/dev/ttyTHS2

BT_MBT_BAUDRATE=921600
#BT_MBT_BAUDRATE=115200

#
# Set FW 
#
BT_FW=FW.hcd

##
# Setup for MBT
##
export MBT_TRANSPORT=${BT_MBT_PORT}
export MBT_BAUD_RATE=${BT_MBT_BAUDRATE}
export TRANSPORT_MODE=0

