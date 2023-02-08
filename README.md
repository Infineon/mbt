## MBT
   ```
    MBT tool source code and download FW scripts for support platform
   ```

## Version
```
  1.32.0
```

## Requirements

- Programming language: C
- Build code Environment:
   ```
	OS: Ubuntu 20.04
   ```
- Build Tool, Use the following command to install the cross compiler:
   ```
   sudo apt install gcc-aarch64-linux-gnu build-essential
   ```

## How to Build
   ```
    cd mbt/
   ```
### for x86
   ```
    make clean
    make x86
   ```
### for arm64
   ```
    make clean
    make arm64
   ```
##Binary execute file in Release folder and scripts folder

### Folder Structure
    <SCRIPT_FOLDER> for different Bluetooth chip script
    - cyw5557x
    - cyw5551x
    ```
 	-<SCRIPT_FOLDER>/
		- scripts_for_imx8/
		- scripts_for_nano/
		- scripts_for_xaiver/
		- scripts_for_rpicm4/
	-Release/
		- arm64/
		- x86/
```
# HOW TO USE SCRIPT FOR DOWNLOAD FW
```
	after build complete
	copy <SCRIPT_FOLDER> to target board through usb disk or wifi or ethernet
	cd <SCRIPT_FOLDER>
	cd scripts_for_<BOARD>
	<BOARD> is the target board you use. the script support 4 target board:
          RPICM4
          IMX8
          NANO
          XAVIER

	DOWNLOAD WITH PARAMETER:
		sudo ./bt_load.sh autobaud
		sudo ./bt_load.sh auto3m
		sudo ./bt_load.sh softo3m
		sudo ./bt_load.sh minidriver
		sudo ./bt_load.sh normal
		Reset the chip to autobaud state and press ENTER key!
		REG ON
		.........
		.........
		Current state: Completed successfully
```

# TEST IN RPI4CM BOARD
## FOLOW THE BELOW STEP
## MBT COMMAND detail in readme.txt
## 1. scripts for autobaud download
   After build mbt and copy <SCRIPT_FOLDER> to rpicm4 board
   ```
	cd <SCRIPT_FOLDER>
	cd scripts_for_rpicm4
sudo ./bt_load_autobaud.sh fcbga_iPA_dLNA_ANT0
   ```
## 2. USE MBT OTHER COMMAND
   set environment first
   ```
   export MBT_TRANSPORT=/dev/ttyAMA0
   ```

## 3. reset
   ```
./mbt reset
    tx:
     01 03 0c 00
    rx: (7 bytes)
     04 0e 04 01 03 0c 00

   ```
## 4. write_bdaddr
   ```
./mbt write_bdaddr beefbeef0011
tx:
 01 01 fc 06 11 00 ef be ef be
rx: (7 bytes)
 04 0e 04 01 01 fc 00
   ```
## 5. read_bdaddr
   ```
./mbt read_bdaddr
tx:
 01 09 10 00
rx: (13 bytes)
 04 0e 0a 01 09 10 00 11 00 ef be ef be
   ```
## 6. Tx AND Rx Test
   PREPARE 2 RPI4CM board both have same copy of <SCRIPT_FOLDER>.
   one run as TX,  another run as RX
### TX
   ```
./mbt tx_test beefbeef0011 2402 3 1 3 1 1 0
./mbt reset to stop tx
   ```
### RX
   ```
./mbt rx_test beefbeef0011 2402 3 1 3 0
   ```

# HOW TO PUT or REPLACE FW
change your FW file name into FW.hcd
```
	put the hcd file into scripts_for_<board>
```

# FOR REFERENCE
## YOU CAN MODIFY include.sh at scripts_for_xxx
```
BT_FW=FW.hcd
```
to what the name you want and change the FW with your name.

============

(c) (2021-2022), Cypress Semiconductor Corporation (an Infineon company) or an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
This software, including source code, documentation and related materials ("Software") is owned by Cypress Semiconductor Corporation or one of its affiliates ("Cypress") and is protected by and subject to worldwide patent protection (United States and foreign), United States copyright laws and international treaty provisions.  Therefore, you may use this Software only as provided in the license agreement accompanying the software package from which you obtained this Software ("EULA").
If no EULA applies, Cypress hereby grants you a personal, non-exclusive, non-transferable license to copy, modify, and compile the Software source code solely for use in connection with Cypress's integrated circuit products.  Any reproduction, modification, translation, compilation, or representation of this Software except as specified above is prohibited without the express written permission of Cypress.
Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to the Software without notice. Cypress does not assume any liability arising out of the application or use of the Software or any product or circuit described in the Software. Cypress does not authorize its products for use in any products where a malfunction or failure of the Cypress product may reasonably be expected to result in significant property damage, injury or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the manufacturer of such system or application assumes all risk of such use and in doing so agrees to indemnify Cypress against all liability.
