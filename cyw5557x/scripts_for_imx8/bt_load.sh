#!/bin/bash
###########################################################################################
#
# Script to download PatchRam BT FW
#
# v2.02
###########################################################################################

#BT_TARGET_DIR="BT/CYW55560A1_001.002.087/"

SCRIPTS_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source ${SCRIPTS_PATH}/include.sh

if [ "$1" == "autobaud" ]; then
    DOWNLOAD_MODE=--autobaud
else 
    if [ "$1" == "minidriver" ]; then
        DOWNLOAD_MODE=--minidriver
    else
        if [ "$1" == "auto3m" ]; then
            DOWNLOAD_MODE=--autobaud3M
        else
            if [ "$1" == "soft3m" ]; then
                DOWNLOAD_MODE=--soft3M
            else
                if [ "$1" == "normal" ]; then
                    DOWNLOAD_MODE=""
                else 
		    echo "Wrong Download Mode: usage: ./bt_load.sh autobaud or ./bt_load.sh minidriver or ./bt_load.sh auto3m or ./bt_load.sh normal ir ./bt_load.sh soft3m"
                    exit 1
		fi
            fi
        fi
    fi
fi

echo "BT_RLS_FW is:"
echo $BT_RLS_FW

echo "BT_TARGET_DIR: "
echo $BT_TARGET_DIR

BT_DM_HCD=$BT_FW
echo "BT TARGET FILE is: "
echo $BT_DM_HCD

# Download the FW
MY_BT_FW=${BT_DM_HCD}
echo -e "\t<>Downloading BT FW:" ${MY_BT_FW}
if [[ "${DOWNLOAD_MODE}" == "--autobaud" || "${DOWNLOAD_MODE}" == "--autobaud3M" ]]; then
    source ${SCRIPTS_PATH}/bt_autobaud.sh
fi
if [ "${DOWNLOAD_MODE}" == "--soft3M" ]; then
    source ${SCRIPTS_PATH}/bt_reg_onoff.sh
    sleep 0.5
fi
${BT_MBT} download ${MY_BT_FW} ${DOWNLOAD_MODE}


