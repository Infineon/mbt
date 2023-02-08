Manufacturing Bluetooth Test Tool for Linux

Overview
--------
The manufacturing Bluetooth test tool (MBT) is used to test and verify the RF
performance of the Infineon Bluetooth Classic and Bluetooth Low Energy (BLE) devices.
Each test sends an HCI command to the device and then waits for an HCI Command
Complete event from the device.
This readme.txt shows example commands and events for each commands.

# HOW TO USE SCRIPT FOR DOWNLOAD FW
```
        copy cyw5557x to target board through usb disk or wifi or ethernet
        cd cyw5557x
        cd scripts_for_nano
                sudo ./bt_load.sh autobaud or
                sudo ./bt_load.sh auto3m or
                Reset the chip to autobaud state and press ENTER key!
                REG ON
                .........
                .........
                .........
                Current state: Completed successfully
```
# HOW TO PUT or REPLACE FW
change your FW file name into FW.hcd
```
        put the hcd file into scripts_for_rpicm4
```

# FOR REFERENCE
## YOU CAN MODIFY BT_FW at include.sh 
```
BT_FW=FW.hcd
```
to what the name you want and change the FW with your name.


Environment Variable
-------------------- 
Need to set the environment variable MBT_TRANSPORT for MBT before testing.
For Nvidia Jeston Naon,
  export MBT_TRANSPORT=/dev/ttyTHS2
The MBT read the MBT_TRANSPORT at run time and use it to communicate with device

Reset Test
----------
This Reset verifies that the device is connected to the host platform

Usage: mbt reset

Command: ./mbt reset
tx:
 01 03 0c 00
rx: (7 bytes)
 04 0e 04 01 03 0c 00

Write BD Address
----------------
This test writes the bd address to the local device.

Usage: mbt write_bdaddr <bdaddr>
- bdaddr = 6 bytes of bd address (no space between bytes)
- this bdaddr is temporary for testing only

Command: ./mbt write_bdaddr beefbeef0011
tx:
 01 01 fc 06 11 00 ef be ef be 
rx: (7 bytes)
 04 0e 04 01 01 fc 00 


Read BD Address
---------------
This test reads the bd address of local device.

Usage: mbt read_bdaddr

Command: ./mbt read_bdaddr
tx:
 01 09 10 00 
rx: (13 bytes)
 04 0e 0a 01 09 10 00 11 00 ef be ef be 

LE Receiver Test
----------------
This test configures the BT to receive reference packets at a fixed interval. 
External test equipment should be used to generate the reference packets.

Usage: mbt le_receiver_test <rx_channel>
    rx_channel = 0 - 39
    (Frequency: 2402 MHz to 2480 MHz, rx_channel: (Frequency - 2402) / 2 )

The example below starts the LE receiver test on Channel 2 (2406 MHz).
Command: ./mbt le_receiver_test 2
tx:
 01 1d 20 01 02
rx: (7 bytes)
 04 0e 04 01 1d 20 00

Note: issue command 'le_test_end' or 'reset' to end this test.

LE Transmitter Test
-------------------
This configures the BT to send test packets at a fixed interval. 
External test equipment may be used to receive and analyze the reference packets.

Usage: mbt le_transmitter_test <tx_channel> <data_length> <packet_payload>
    tx_channel = 0 - 39
    (Frequency: 2402 - 2480 MHz, rx_channel: (Frequency - 2402) / 2 )
    data_length = 0 - 37
    data_pattern = 0 - 7
        0 = Pseudo-random bit sequence 9
        1 = Pattern of alternating bits: 11110000
        2 = Pattern of alternating bits: 10101010
        3 = Pseudo-random bit sequence 15
        4 = Pattern of all 1s
        5 = Pattern of all 0s
        6 = Pattern of alternating bits: 00001111
        7 = Pattern of alternating bits: 0101

The example below starts the test on Channel 2 (2406 MHz) with a 10-byte payload of all ones (1s).
Command: ./mbt le_transmitter_test 2 10 4
tx:
 01 1e 20 03 02 0a 04
rx: (7 bytes)
 04 0e 04 01 1e 20 0c 

Note: issue command 'le_test_end' or 'reset' to end this test.

LE Test End
-----------
This command stops the LE Transmitter or LE Receiver Test that is in progress.
The number of packets received during the test is reported.

Usage: mbt le_test_end

Command: ./mbt le_test_end
tx:
 01 1f 20 00
rx: (9 bytes)
 04 0e 06 01 1f 20 00 00 00
Packets_received 0

Continuous Transmit Test
------------------------
This test configures the BT to turn the carrier ON or OFF. 
When the carrier is ON the device transmits the carrier which specified modulation mode
and type on the specified frequency at the specified power level.

Usage: mbt set_tx_frequency_arm <on/off> <tx_frequency> <tx_mode> <tx_modulation_type> <tx_power_select> <tx_power_dBm_index>
    on/off = 0 -1
        1 = carrier ON
        0 = carrier OFF
    tx_frequency = 2402 - 2480 MHz
    tx_mode = 0 - 5
        0: Unmodulated
        1: PRBS9
        2: PRBS15
        3: All Zeros
        4: All Ones
        5: Incrementing Symbols
    tx_modulation_type = 0 - 3 (ignored when 'Unmodulated' selected).
        0: GFSK
        1: QPSK
        2: 8PSK
        3: LE
    tx_power_select = 0 - 9
        0 - 7: 0 - -28dBm (4dBm steps)
        8: power in dBm
        9: index from Power Table
    tx_power_dBm_index = -128 - 127dBm, 0 - 7 index from Power Table in FW
The example below turns the carrier ON with frequency 2402MHz, mode PRBS9, type 8PSK, and the tx_power.
The tx_power is specified by tx_power_select and tx_power_dBm_index
Command: ./mbt set_tx_frequency_arm 1 2402 1 2 1 0 (1: -4dBm in steps)
Command: ./mbt set_tx_frequency_arm 1 2402 1 2 8 3 (8: dBm, 3: 3dBm)
Command: ./mbt set_tx_frequency_arm 1 2402 1 2 9 0 (9: Power Table, 0: index 0)
tx:
 01 14 fc 07 00 02 01 02 08 03 00 
rx: (7 bytes)
 04 0e 04 01 14 fc 00 

To stop the test, send the command with the carrier off
Command: ./mbt set_tx_frequency_arm 0
tx:
 01 14 fc 07 01 02 00 00 00 00 00
rx: (7 bytes)
 04 0e 04 01 14 fc 00

Tx Test
-------
This test configures the device to transmit the selected data pattern which is on the specified frequency
and specified logical channel at a specified power level.

Usage: mbt tx_test <bdaddr> <frequency> <modulation_type> <logical_channel> <bb_packet_type> <packet_length> <tx_power_select> <tx_power_dBm_index>
    bdaddr: bd address of tx device (6 bytes, no space between bytes)
    frequency: 0 or transmit frequency (2402 - 2480) in MHz
        0: normal Bluetooth hopping sequence (79 channels)
        2402 - 2480: single frequency without hopping
    modulation_type:
        1: 0x00 8-bit Pattern
        2: 0xFF 8-bit Pattern
        3: 0xAA 8-bit Pattern
        9: 0xF0 8-bit Pattern
        4: PRBS9 Pattern
    logical_channel:
        0: EDR
        1: BR
    bb_packet_type:
        3: DM1
        4: DH1 / 2-DH1
        8: 3-DH1
        10: DM3 / 2-DH3
        11: DH3 / 3-DH3
        14: DM5 / 2-DH5
        15: DH5 / 3-DH5
    packet_length: 0 - 65535. Device will limit the length to the max for the baseband packet type.
        when the DM1 packet is selected, the maximum packet size will be 17 bytes.
    tx_power_select = 0 - 9
        0 - 7: 0 - -28dBm (4dBm steps)
        8: power in dBm
        9: index from Power Table
    tx_power_dBm_index = -128 - 127dBm, 0 - 7 index from Power Table in FW

The example below instructs the device to transmit 0xAA 8-bit Pattern on the 2402 MHz and ACL Basic
with DM1 packet (max 17 bytes) type at -3 dBm.

Command: ./mbt tx_test beefbeef0011 2402 3 1 3 0 -3
tx:
 01 51 fc 10 11 00 ef be ef be 01 00 03 01 03 00 
 00 08 fd 00 
rx: (7 bytes)
 04 0e 04 01 51 fc 00

Rx Test
-------
This test issues a command to the device to set radio to camp on a specified frequency.
While test is running the device periodically sends reports about received packets.

Usage: mbt rx_test <bdaddr> <frequency> <modulation_type> <logical_channel> <bb_packet_type> <packet_length>
    bdaddr: bd address of the remote tx device (6 bytes, no space between bytes)
    frequency = receive frequency (2402 - 2480) in MHz
    modulation_type:
        0: 0x00 8-bit Pattern
        1: 0xFF 8-bit Pattern
        2: 0xAA 8-bit Pattern
        3: 0xF0 8-bit Pattern
        4: PRBS9 Pattern
    logical_channel:
        0: EDR
        1: BR
    bb_packet_type:
        3: DM1
        4: DH1 / 2-DH1
        8: 3-DH1
        10: DM3 / 2-DH3
        11: DH3 / 3-DH3
        14: DM5 / 2-DH5
        15: DH5 / 3-DH5
    packet_length: 0 - 65535.
        Device will compare length of the received packets with the specified packet_length.

Device will generate the statistics report of the Rx Test every second.

The example below instructs the device to receive 0xAA 8-bit Pattern on the 2402 MHz and ACL Basic with DM1 packet type.
Command: ./mbt rx_test beefbeef0011 2402 3 1 3 0
(frequency:2402 modulation_type:3 logical_channel:1 packet_type:3 packet_length:0)
tx:
 01 52 fc 0e 11 00 ef be ef be e8 03 00 03 01 03 
 00 00 
rx: (7 bytes)
 04 0e 04 01 52 fc 00 

Mbt reports the Rx Test statistics every second.
 04 FF 21 07 01 00 00 00 00 00 00 00 DE 15 00 00
 DE 15 00 00 00 00 00 00 00 00 00 00 00 00 00 00
 00 00 00 00
[rx test statistics]
    Sync_Timeout_Count:     0x1
    HEC_Error_Count:        0x0
    Total_Received_Packets: 0x15de
    Good_Packets:           0x15de
    CRC_Error_Packets:      0x0
    Total_Received_Bits:    0x0
    Good_Bits:              0x0
    Error_Bits:             0x0

download
--------
Usage: mbt download <firmware_filename>
    firmware_filename: firmware file name including path.

The example below downloads the firmware file
Command: ./mbt download firmware.hcd
Sending HCI Command:
0000 < 52 FC 0E 45 23 01 3A 70 20 E8 03 00 02 01 03 >
.... downloading
00B0 < 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 >
Received HCI Event:
0000 < 0E 04 01 4C FC 00 >
Success
sending record at:0xffffffff
Sending HCI Command:
0000 < 4C FC 04 FF FF FF FF >
Received HCI Event:
0000 < 0E 04 01 4C FC 00 >
Success
Download .hcd file success
Sending HCI Command:
0000 < 4E FC 04 FF FF FF FF >
Received HCI Event:
0000 < 0E 04 01 4E FC 00 >
Success
Launch RAM success
Issue HCI Reset after downloading Application
Sending HCI Command:
0000 < 03 0C 00 >
Received HCI Event:
0000 < 0E 04 01 03 0C 00 >
Success
switch the device baudrate to 115200. test commands use 115200
Sending HCI Command:
0000 < 18 FC 06 00 00 00 C2 01 00 >
Received HCI Event:
0000 < 0E 04 01 18 FC 00 >
Success
