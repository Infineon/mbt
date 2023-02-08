/*
 * mbt_common.c
 */
/*
* Copyright 2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mbt.h"

/******************************************************
 *             Function Definitions
 *                    wlai
 * Note: change modulation_type of tx_test/rx_test
 *
 * 1: 0x00 8-bit Pattern
 * 2: 0xFF 8-bit Pattern
 * 3: 0xAA 8-bit Pattern
 * 9: 0xF0 8-bit Pattern
 * 4: PRBS9 Pattern
 *
 * Note: add new interface /dev/ttyTHS1 for Jetson-TX2
 ******************************************************/
void print_usage_set_environment_variables(BOOL full)
{
    printf ("Command: set the environment variable MBT_TRANSPORT. eg) ttyUSBx, ttymxcx, btusbx\n");
    //printf (" - DOWNLOAD_BAUDRATE for faster downloading when a COM Port selected (default: 115200)\n");
    if (!full) return;
    printf ("ex)\n");
    printf ("$ export MBT_TRANSPORT=/dev/ttyUSB0\n");
    printf ("$ export MBT_TRANSPORT=/dev/ttymxc2\n");
    printf ("$ export MBT_TRANSPORT=/dev/ttySDIO\n");
    printf ("$ export MBT_TRANSPORT=/dev/ttyTHS1\n");
    //printf (" - export DOWNLOAD_BAUDRATE=4000000\n");
    printf("\n");
}

void print_usage_help(BOOL full)
{
    printf("Command: mbt help, mbt help all\n");
    if (!full) return;
    printf ("ex)\n");
    printf("$ ./mbt help\n");
    printf("$ ./mbt help all\n");
    printf("\n");
}

void print_usage_reset(BOOL full)
{
    printf("Command: mbt reset\n");
    if (!full) return;
    printf ("ex)\n");
    printf("$ ./mbt reset\n");
    printf("\n");
}

static void print_usage_input_command(BOOL full)
{
    printf("Command: mbt input_command <hci_commands>\n");
    if (!full)
        return;
    printf("ex)\n");
    printf("to enter HCI command to reset the BT\n");
    printf("$ ./mbt input_command 030C00\n");
    printf("or\n");
    printf("$ ./mbt input_command 030c00\n");
}

void print_usage_download_minidriver(BOOL full)
{
    printf("Command: mbt download_minidriver\n");
    if (!full) return;
    printf ("ex)\n");
    printf("$ ./mbt download_minidriver\n");
    printf("\n");
}

void print_usage_otp_write_word(BOOL full)
{
    printf("Command: mbt otp_write_word <word_offset> <write_data>\n");
    if (!full) return;
    printf ("ex)\n");
    printf("$ ./mbt otp_write_word 84 06104F50\n");
    printf("$ ./mbt otp_write_word 85 33445566\n");
    printf("$ ./mbt otp_write_word 86 01001122\n");
    printf("\n");
}


void print_usage_write_bdaddr(BOOL full)
{
    printf("Command: mbt write_bdaddr <bdaddr>\n");
    if (!full) return;
    printf ("ex)\n");
    printf("$ ./mbt write_bdaddr beefbeef0011\n");
    printf("\n");
}

void print_usage_read_bdaddr(BOOL full)
{
    printf("Command: mbt read_bdaddr\n");
    if (!full) return;
    printf ("ex)\n");
    printf("$ ./mbt read_bdaddr\n");
    printf("\n");
}

 void print_usage_download(BOOL full)
{
    printf("Command: mbt download <fw filename (*.hcd)>\n");
    if (!full) return;
    printf ("ex)\n");
    printf("$ ./mbt download fw.hcd\n");
    printf("$ ./mbt download fw.hcd --noreset\n");
    printf("$ ./mbt download fw.hex [--noreset] [--launch_ram FFFFFFFF]\n");
    printf("$ ./mbt download fw.hcd --autobaud\n");
    printf("$ ./mbt download fw.hcd --minidriver\n");
    printf("$ ./mbt download fw.hcd --autobaud3M\n");
    printf("$ ./mbt download fw.hcd --soft3M\n");
    printf("\n");
}

void print_usage_le_receiver_test(BOOL full)
{
    printf ("Command: mbt le_receiver_test <rx_channel>\n");
    if (!full) return;
    printf ("     rx_channel = 0 - 39\n");
    printf ("         0:2402MHz, 39:2480MHz, (F-2402)/2)\n");
    printf ("ex)\n");
    printf ("$ ./mbt le_receiver_test 2\n");
    printf("\n");
}

void print_usage_le_test_end(BOOL full)
{
    printf ("Command: mbt le_test_end\n");
    if (!full) return;
    printf ("ex)\n");
    printf("$ ./mbt le_test_end\n");
    printf("\n");
}

void print_usage_le_transmitter_test(BOOL full)
{
    printf ("Command: mbt le_transmitter_test <tx_channel> <data_length> <data_pattern>\n");
    if (!full) return;
    printf ("     tx_channel = 0 - 39\n");
    printf ("         0:2402MHz, 39:2480MHz, (F-2402)/2)\n");
    printf ("     data_length = 0 - 37\n");
    printf ("     data_pattern = 0 - 7\n");
    printf ("         0: Pseudo-Random bit sequence 9\n");
    printf ("         1: Pattern of alternating bits '11110000'\n");
    printf ("         2: Pattern of alternating bits '10101010'\n");
    printf ("         3: Pseudo-Random bit sequence 15\n");
    printf ("         4: Pattern of All '1' bits\n");
    printf ("         5: Pattern of All '0' bits\n");
    printf ("         6: Pattern of alternating bits '00001111'\n");
    printf ("         7: Pattern of alternating bits '0101'\n");
    printf ("ex)\n");
    printf ("$ ./mbt mbt le_transmitter_test 2 10 4\n");
    printf("\n");
}

void print_usage_set_tx_frequency_arm(BOOL full)
{
    printf ("Command: mbt set_tx_frequency_arm <carrier on/off> <tx_frequency> <tx_mode> <tx_modulation_type> <tx_power_select> <tx_power_dBm_index>\n");
    if (!full) return;
    printf ("     carrier = 0 - 1\n");
    printf ("         0: off\n");
    printf ("         1: on\n");
    printf ("     tx_frequency = 2402 - 2480 MHz\n");
    printf ("     tx_mode = 0 - 5\n");
    printf ("         0: Unmodulated\n");
    printf ("         1: PRBS9\n");
    printf ("         2: PRBS15\n");
    printf ("         3: All Zeros\n");
    printf ("         4: All Ones\n");
    printf ("         5: Incrementing Symbols\n");
    printf ("     tx_modulation_type = 0 - 3 (ignored when 'Unmodulated' selected)\n");
    printf ("         0: GFSK\n");
    printf ("         1: QPSK\n");
    printf ("         2: 8PSK\n");
    printf ("         3: LE\n");
    printf ("     tx_power_select = 0 - 9\n");
    printf ("         0 - 7: 0 - -28dBm (4dBm steps)\n");
    printf ("         8: power in dBm\n");
    printf ("         9: index from Power Table\n");
    printf ("     tx_power_dBm_index = -128 - 127dBm, 0 - 7 index from Power Table in FW\n");
    printf ("ex)\n");
    printf ("$ ./mbt set_tx_frequency_arm 1 2402 1 2 1 0 (1: -4dBm in steps)\n");
    printf ("$ ./mbt set_tx_frequency_arm 1 2402 1 2 8 3 (8: dBm, 3: 3dBm)\n");
    printf ("$ ./mbt set_tx_frequency_arm 1 2402 1 2 9 0 (9: Power Table, 0: index 0)\n");
    printf("\n");
}

void print_usage_receive_only_test(BOOL full)
{
    printf ("Command: mbt receive_only <rx_frequency>\n");
    if (!full) return;
    printf ("     rx_frequency = 2402 - 2480 MHz\n");
    printf ("ex)\n");
    printf ("$ ./mbt receive_only 2402\n");
    printf("\n");
}

void print_usage_tx_test(BOOL full)
{
    printf ("Command: mbt tx_test <bdaddr> <frequency> <modulation_type> <logical_channel> <packet_type> <packet_length> <tx_power_select> <tx_power_dBm_index>\n");
    if (!full) return;
    printf ("     bdaddr = bd address of tx device\n");
    printf ("     frequency = 0 for hopping or 2402 - 2480 MHz\n");
    printf ("         0: hopping 79 channels\n");
    printf ("         2402 - 2480: single frequency, no hopping\n");
    printf ("     modulation_type = 0 - 4,9 (data pattern)\n");
    printf ("         1: 0x00 8-bit Pattern\n");
    printf ("         2: 0xFF 8-bit Pattern\n");
    printf ("         3: 0xAA 8-bit Pattern\n");
    printf ("         9: 0xF0 8-bit Pattern\n");
    printf ("         4: PRBS9 Pattern\n");
    printf ("     logical_channel = 0 - 1 (Basic Rate or Enhanced Data Rate for ACL packets)\n");
    printf ("         0: EDR\n");
    printf ("         1: BR\n");
    printf ("     packet_type = 3 - 15 (baseband packet type)\n");
    printf ("         3: DM1\n");
    printf ("         4: DH1 / 2-DH1\n");
    printf ("         8: 3-DH1\n");
    printf ("         10: DM3 / 2-DH3\n");
    printf ("         11: DH3 / 3-DH3\n");
    printf ("         12: EV4 / 2-EV5\n");
    printf ("         13: EV5 / 3-EV5\n");
    printf ("         14: DM5 / 2-DH5\n");
    printf ("         15: DH5 / 3-DH5\n");
    printf ("     packet_length = 0 - 65535 (the length will be limited to the max of the baseband packet type)\n");
    printf ("     tx_power_select = 0 - 9\n");
    printf ("         0 - 7: 0 - -28dBm (4dBm steps)\n");
    printf ("         8: power in dBm\n");
    printf ("         9: index from Power Table\n");
    printf ("     tx_power_dBm_index = -128 - 127dBm, 0 - 7 index from Power Table in FW\n");
    printf ("ex)\n");
    printf ("$ ./mbt tx_test beefbeef0011 2402 3 1 3 0 1 0 (1: -4dBm in steps)\n");
    printf ("$ ./mbt tx_test beefbeef0011 2402 3 1 3 0 8 3 (8: dBm, 3: 3dBm)\n");
    printf ("$ ./mbt tx_test beefbeef0011 2402 3 1 3 0 9 0 (9: Power Table, 0: index 0)\n");
    printf("\n");
}

void print_usage_rx_test(BOOL full)
{
    printf ("Command: mbt rx_test <bdaddr> <frequency> <modulation_type> <logical_channel> <packet_type> <packet_length>\n");
    if (!full) return;
    printf ("     bdaddr = bd address of tx device\n");
    printf ("     frequency = 0 for hopping or 2402 - 2480 MHz\n");
    printf ("         0: hopping 79 channels\n");
    printf ("         2402 - 2480: single frequency, no hopping\n");
    printf ("     modulation_type = 0 - 4,9 (data pattern)\n");
    printf ("         1: 0x00 8-bit Pattern\n");
    printf ("         2: 0xFF 8-bit Pattern\n");
    printf ("         3: 0xAA 8-bit Pattern\n");
    printf ("         9: 0xF0 8-bit Pattern\n");
    printf ("         4: PRBS9 Pattern\n");
    printf ("     logical_channel = 0 - 1 (Basic Rate or Enhanced Data Rate for ACL packets)\n");
    printf ("         0: EDR\n");
    printf ("         1: BR\n");
    printf ("     packet_type = 3 - 15 (baseband packet type)\n");
    printf ("         3: DM1\n");
    printf ("         4: DH1 / 2-DH1\n");
    printf ("         8: 3-DH1\n");
    printf ("         10: DM3 / 2-DH3\n");
    printf ("         11: DH3 / 3-DH3\n");
    printf ("         12: EV4 / 2-EV5\n");
    printf ("         13: EV5 / 3-EV5\n");
    printf ("         14: DM5 / 2-DH5\n");
    printf ("         15: DH5 / 3-DH5\n");
    printf ("     packet_length = 0 - 65535 (the length will be limited to the max of the baseband packet type)\n");
    printf ("ex)\n");
    printf ("$ ./mbt rx_test beefbeef0011 2402 3 1 3 0\n");
    printf("\n");
}

void print_usage_loopback_mode(BOOL full)
{
    printf("Command: mbt loopback_mode\n");
    if (!full) return;
    printf ("ex)\n");
    printf("$ ./mbt loopback_mode\n");
    printf("\n");
}

void print_usage_read_name(BOOL full)
{
    printf("Command: mbt read_name\n");
    if (!full) return;
    printf ("ex)\n");
    printf("$ ./mbt read_name\n");
    printf("\n");
}

void print_usage_update_baudrate(BOOL full)
{
    printf("Command: mbt update_baudrate <baudrate> \n");
    if (!full) return;
    printf ("ex)\n");
    printf("$ ./mbt update_baudrate 3000000 \n");
    printf("\n");
}

void print_usage_read_version(BOOL full)
{
    printf("Command: mbt read_version\n");
    if (!full) return;
    printf ("ex)\n");
    printf("$ ./mbt read_version\n");
    printf("\n");
}

void print_usage_le_rx_test_v3(BOOL full)
{
    printf ("Command: mbt le_rx_test_v3 <rx_channel> <phy>\n");
    if (!full) return;
    printf ("     rx_channel = 0 - 39\n");
    printf ("         0:2402MHz, 39:2480MHz, (F-2402)/2)\n");
    printf ("     phy = 1 - 3\n");
    printf ("         1: LE 1M PHY\n");
    printf ("         2: LE 2M PHY\n");
    printf ("         3: LE CodedvPHY\n");
    printf ("ex)\n");
    printf ("$ ./mbt le_rx_test_v3 24 1\n");
    printf("\n");
}

void print_usage_le_tx_test_v3(BOOL full)
{
    printf ("Command: mbt le_tx_test_v3 <tx_channel> <data_length> <data_pattern> <phy>\n");
    if (!full) return;
    printf ("     tx_channel = 0 - 39\n");
    printf ("         0:2402MHz, 39:2480MHz, (F-2402)/2)\n");
    printf ("     data_length = 0 - 255\n");
    printf ("     data_pattern = 0 - 7\n");
    printf ("         0: Pseudo-Random bit sequence 9\n");
    printf ("         1: Pattern of alternating bits '11110000'\n");
    printf ("         2: Pattern of alternating bits '10101010'\n");
    printf ("         3: Pseudo-Random bit sequence 15\n");
    printf ("         4: Pattern of All '1' bits\n");
    printf ("         5: Pattern of All '0' bits\n");
    printf ("         6: Pattern of alternating bits '00001111'\n");
    printf ("         7: Pattern of alternating bits '0101'\n");
    printf ("     phy = 1 - 3\n");
    printf ("         1: LE 1M PHY\n");
    printf ("         2: LE 2M PHY\n");
    printf ("         3: LE CodedvPHY\n");
    printf ("ex)\n");
    printf ("$ ./mbt mbt le_tx_test_v3 24 255 0 1\n");
    printf("\n");
}

void print_usage_le_enhenced_rx_test(BOOL full)
{
    printf ("Command: mbt le_enhenced_receiver_test <rx_channel> <phy> <mod_idx>\n");
    if (!full) return;
    printf ("     rx_channel = 0 - 39\n");
    printf ("         0:2402MHz, 39:2480MHz, (F-2402)/2)\n");
    printf ("     phy = 1 - 3\n");
    printf ("         1: Receiver set to receive data at 1Ms/s\n");
    printf ("         2: Receiver set to receive at 2Ms/s\n");
    printf ("         3: Receiver set to use Coded PHY\n");
    printf ("     modulation_index = 0 - 1\n");
    printf ("         0: Assume Transmitter will have a standard modulation index\n");
    printf ("         1: Assume Transmitter will have a stable modulation index\n");
    printf ("ex)\n");
    printf ("$ ./mbt le_enhenced_receiver_test 24 1 0\n");
    printf("\n");
}

void print_usage_le_enhenced_tx_test(BOOL full)
{
    printf ("Command: mbt le_enhenced_transmitter_test <tx_channel> <length_of_data> <payload> <phy>\n");
    if (!full) return;
    printf ("     tx_channel = 0 - 39\n");
    printf ("         0:2402MHz, 39:2480MHz, (F-2402)/2)\n");
    printf ("     Length_of_test = 0 - 255\n");
    printf ("     phy = 1 - 4\n");
    printf ("         0: Pseudo=Random bit sequence 9\n");
    printf ("         1: Pattern of alternating bits '11110000'\n");
    printf ("         2: Pattern of alternating bits '10101010'\n");
    printf ("         3: Pseudo-Random bit sequence 15 - Optional\n");
    printf ("         4: Pattern of ALL '1' bits - Optional\n");
    printf ("         5: Pattern of ALL '0' bits - Optional\n");
    printf ("         6: Pattern of alternating bits '00001111' - Optional\n");
    printf ("         7: Pattern of alternating bits '01010101' - Optional\n");
    printf ("     phy = 1 - 4\n");
    printf ("         1: Transmitter set to transmit data at 1Ms/s\n");
    printf ("         2: Transmitter set to transmit data at 2Ms/s\n");
    printf ("         3: Transmitter set to use Coded PHY with S=8\n");
    printf ("         4: Transmitter set to use Coded PHY with S=2\n");
    printf ("ex)\n");
    printf ("$ ./mbt le_enhenced_transmitter_test 24 37 0 1\n");
    printf("\n");
}

void print_usage_get_agc_index(BOOL full)
{
    printf("Command: mbt get_agc_index\n");
    if (!full) return;
    printf ("ex)\n");
    printf("$ ./mbt get_agc_index\n");
    printf("\n");
}

void print_usage_mbt_commands(BOOL full)
{
    //if (full)
    printf("[mbt version: %s]\n", MBT_VERSION_STRING);
    print_usage_help(full);
    print_usage_reset(full);
    print_usage_input_command(full);
    print_usage_download_minidriver(full);
    print_usage_download(full);
    print_usage_write_bdaddr(full);
    print_usage_read_bdaddr(full);
    print_usage_le_receiver_test(full);
    print_usage_le_transmitter_test(full);
    print_usage_le_test_end(full);
    print_usage_set_tx_frequency_arm(full);
    print_usage_receive_only_test(full);
    print_usage_tx_test(full);
    print_usage_rx_test(full);
    print_usage_loopback_mode(full);
    print_usage_read_name(full);
    print_usage_update_baudrate(full);
    print_usage_read_version(full);
    print_usage_le_rx_test_v3(full);
    print_usage_le_tx_test_v3(full);
    print_usage_le_enhenced_rx_test(full);
    print_usage_le_enhenced_tx_test(full);
    print_usage_get_agc_index(full);
    print_usage_otp_write_word(full);


    if (full)
        printf ("-<end help message>-\n");
}

int parse_input_command_args(int argc, char **argv)
{
    if(COMPARE_STRING(argv[1],"help")==0)
    {
        if(argc == 2)
        {
            /*help message short*/
            print_usage_mbt_commands(FALSE);
        }
        if((argc == 3) && COMPARE_STRING(argv[2],"all")==0)
        {
            /*help message full*/
            print_usage_mbt_commands(TRUE);
        }
        return COMMAND_HELP;
    }
    if(COMPARE_STRING(argv[1],"reset")==0)
    {
        if( argc == 2 )
        {
            return COMMAND_RESET;
        }
        /*reset command but arg error - display full reset command usage*/
        print_usage_reset(TRUE);
    }
    else if (COMPARE_STRING(argv[1],"download_minidriver")==0)
    {
        if( argc == 2 )
        {
            return COMMAND_DOWNLOAD_MINIDRIVER;
        }
        print_usage_download_minidriver(TRUE);
    }
    else if (COMPARE_STRING(argv[1],"input_command")==0)
    {
        if( argc == 3 )
        {
            return COMMAND_INPUT_COMMAND;
        }
        print_usage_input_command(TRUE);
    }
    else if(COMPARE_STRING(argv[1],"loopback_mode")==0)
    {
        if( argc == 2 )
        {
            return COMMAND_LOOPBACK_MODE;
        }
        print_usage_loopback_mode(TRUE);
    }
    else if(COMPARE_STRING(argv[1],"download")==0)
    {
        if (argc == 3)
        {
            return COMMAND_DOWNLOAD;
        }
        if((argc == 4) && COMPARE_STRING(argv[3],"--noreset")==0)
        {
            return COMMAND_DOWNLOAD_NO_RESET;
        }
        if((argc == 4) && COMPARE_STRING(argv[3],"--autobaud")==0)
        {
            return COMMAND_DOWNLOAD_AUTOBAUD;
        }
	if((argc == 4) && COMPARE_STRING(argv[3],"--minidriver")==0)
	{
	    return COMMAND_DOWNLOAD_WITH_MINIDRIVER;
	}
	if((argc == 4) && COMPARE_STRING(argv[3],"--autobaud3M")==0)
	{
	    return COMMAND_DOWNLOAD_AUTOBAUD_3M;
	}
        if((argc == 4) && COMPARE_STRING(argv[3],"--soft3M")==0)
	{
	    return COMMAND_DOWNLOAD_SOFT_3M;
	}
        if((argc == 5) && COMPARE_STRING(argv[3],"--launch_ram")==0)
        {
            extern unsigned int asc_to_int(char a);
            extern unsigned int launch_ram;
            int len = strlen(argv[4]);
            int index=0;
            launch_ram = 0;
            while(len)
            {
                int up = asc_to_int(argv[4][index++]);
                int low = asc_to_int(argv[4][index++]);
                len-=2;
                launch_ram |= (((up<<4)+low)<<( 8*(len/2)));
            }
            return COMMAND_DOWNLOAD;
        }
        if((argc == 6) && COMPARE_STRING(argv[3],"--noreset")==0)
        {
            if (COMPARE_STRING(argv[4],"--launch_ram")==0)
            {
                extern unsigned int asc_to_int(char a);
                extern unsigned int launch_ram;
                int len = strlen(argv[5]);
                int index=0;
                launch_ram = 0;
                while(len)
                {
                    int up = asc_to_int(argv[5][index++]);
                    int low = asc_to_int(argv[5][index++]);
                    len-=2;
                    launch_ram |= (((up<<4)+low)<<( 8*(len/2)));
                }
            }
            return COMMAND_DOWNLOAD_NO_RESET;
        }

        print_usage_download(TRUE);
    }
    else if(COMPARE_STRING(argv[1],"write_bdaddr")==0)
    {
        if ((argc == 3) && (strlen(argv[2])==12))
        {
            return COMMAND_WRITE_BDADDR;
        }
        print_usage_write_bdaddr(TRUE);
    }
    else if(COMPARE_STRING(argv[1],"otp_write_word")==0)
    {
        if ((argc == 4) && (strlen(argv[3])==8))
        {
            return COMMAND_OTP_WRITE_WORD;
        }
        print_usage_otp_write_word(TRUE);
    }
    else if(COMPARE_STRING(argv[1],"read_bdaddr")==0)
    {
        if (argc == 2)
        {
            return COMMAND_READ_BDADDR;
        }
        print_usage_read_bdaddr(TRUE);
    }
    else if(COMPARE_STRING(argv[1],"le_receiver_test")==0)
    {
        if( argc == 3 )
        {
            UINT16 channel = atoi(argv[2]);
            if ((channel >= 0) && (channel <= 39))
            {
                return COMMAND_LE_TEST_RECEIVER;
            }
        }
        print_usage_le_receiver_test(TRUE);
    }
    else if(COMPARE_STRING(argv[1],"le_test_end")==0)
    {
        if (argc == 2)
        {
            return COMMAND_LE_TEST_END;
        }
        print_usage_le_test_end(TRUE);
    }
    else if(COMPARE_STRING(argv[1],"le_transmitter_test")==0)
    {
        if (argc == 5)
        {
            UINT16 channel = atoi(argv[2]);
            UINT16 length  = atoi(argv[3]);
            UINT16 pattern = atoi(argv[4]);

            if (((channel >= 0) && (channel <= 39)) && ((length > 0) && (length <= 37)) && ((pattern >= 0) && (pattern < 7)))
            {
                return COMMAND_LE_TEST_TRANSMITTER;
            }
        }
        print_usage_le_transmitter_test(TRUE);
    }
    else if(COMPARE_STRING(argv[1],"set_tx_frequency_arm")==0)
    {
        if (argc >= 3)
        {
            int carrier_on;
            carrier_on = atoi(argv[2]);
            if ((carrier_on == 0) || (carrier_on == 1))
            {
                if (carrier_on == 0)
                {
                    return COMMAND_SET_TX_FREQUENCY_ARM;
                }
                else if (argc == 8)
                {
                    UINT16 tx_frequency = atoi(argv[3]);
                    if ((tx_frequency >= 2402) && (tx_frequency <= 2480))
                    {
                        int tx_mode = atoi(argv[4]);
                        if ((tx_mode >= 0) && (tx_mode <= 5))
                        {
                            int tx_modulation_type = atoi(argv[5]);
                            if ((tx_modulation_type >= 0) && (tx_modulation_type <= 3))
                            {
                                int tx_power_select = atoi(argv[6]); //0x8: Specify Power in dBm, 0x9: Specify Power Table index
                                if ((tx_power_select >= 0) || (tx_power_select <= 7))
                                {
                                    return COMMAND_SET_TX_FREQUENCY_ARM;
                                }
                                else if ((tx_power_select == 8) || (tx_power_select == 9))
                                {
                                    int tx_power_dBm_index = atoi(argv[7]);
                                    if ((tx_power_select == 8) && ((tx_power_dBm_index >= -128) && (tx_power_dBm_index <= 127)))
                                    {
                                        return COMMAND_SET_TX_FREQUENCY_ARM;
                                    }
                                    else if ((tx_power_select == 9) && ((tx_power_dBm_index >= 0) && (tx_power_dBm_index <= 7)))
                                    {
                                        return COMMAND_SET_TX_FREQUENCY_ARM;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        print_usage_set_tx_frequency_arm(TRUE);
    }
    else if(COMPARE_STRING(argv[1],"tx_test")==0)
    {
        if (argc == 10)
        {
            if(strlen(argv[2]) == 12)
            {
                int frequency = atoi(argv[3]);
                if ((frequency == 0) || (frequency >= 2402) && (frequency <= 2480))
                {
                    int modulation_type = atoi(argv[4]);
                    if (((modulation_type >= 1) && (modulation_type <= 4)) || (modulation_type == 9))
                    {
                        int logical_channel = atoi(argv[5]);
                        if ((logical_channel >= 0) && (logical_channel <= 1))
                        {
                             int bb_packet_type = atoi(argv[6]);
                             if ((bb_packet_type >= 3) && (bb_packet_type <= 15))
                             {
                                 int packet_length = atoi(argv[7]);
                                 if ((packet_length >= 0) && (packet_length <= 0xffff))
                                 {
                                    int tx_power_select = atoi(argv[8]); //0x8: Specify Power in dBm, 0x9: Specify Power Table index
                                    if ((tx_power_select >= 0) || (tx_power_select <= 7))
                                    {
                                        return COMMAND_TX_TEST;
                                    }
                                    else if ((tx_power_select == 8) || (tx_power_select == 9))
                                    {
                                        int tx_power_dBm_index = atoi(argv[9]);
                                        if ((tx_power_select == 8) && ((tx_power_dBm_index >= -128) && (tx_power_dBm_index <= 127)))
                                        {
                                            return COMMAND_TX_TEST;
                                        }
                                        else if ((tx_power_select == 9) && ((tx_power_dBm_index >= 0) && (tx_power_dBm_index <= 7)))
                                        {
                                            return COMMAND_TX_TEST;
                                        }
                                    }
                                 }
                             }
                        }
                    }
                }
            }
        }
        print_usage_tx_test(TRUE);
    }
    else if(COMPARE_STRING(argv[1],"rx_test")==0)
    {
        if (argc == 8)
        {
            if(strlen(argv[2]) == 12)
            {
                int frequency = atoi(argv[3]);
                if ((frequency >= 2402) && (frequency <= 2480))
                {
                    int modulation_type = atoi(argv[4]);
                    if (((modulation_type >= 1) && (modulation_type <= 4)) || (modulation_type == 9))
                    {
                        int logical_channel = atoi(argv[5]);
                        if ((logical_channel >= 0) && (logical_channel <= 1))
                        {
                            int bb_packet_type = atoi(argv[6]);
                            if ((bb_packet_type >= 3) && (bb_packet_type <= 15))
                            {
                                int packet_length = atoi(argv[7]);
                                if ((packet_length >= 0) && (packet_length <= 0xffff))
                                {
                                    return COMMAND_RX_TEST;
                                }
                            }
                        }
                    }
                }
            }
        }
        print_usage_rx_test(TRUE);
    }
    else if(COMPARE_STRING(argv[1],"receive_only")==0)
    {
        if (argc == 3)
        {
            UINT16 rx_frequency = atoi(argv[2]);
            if ((rx_frequency >= 2402) && (rx_frequency <= 2480))
            {
                return COMMAND_RECEIVE_ONLY;
            }
        }
        print_usage_receive_only_test(TRUE);
    }
    else if(COMPARE_STRING(argv[1],"read_name")==0)
    {
        return COMMAND_READ_NAME;
    }
    else if(COMPARE_STRING(argv[1],"update_baudrate")==0)
    {
        if (argc == 3)
        {
            return COMMAND_UPDATE_BAUDRATE;
        }
        print_usage_update_baudrate(TRUE);
    }
    else if(COMPARE_STRING(argv[1],"read_version")==0)
    {
        return COMMAND_READ_VERSION;
    }
    else if(COMPARE_STRING(argv[1],"le_rx_test_v3")==0)
    {
        if( argc == 4 )
        {
            UINT8 channel = (UINT8)atoi(argv[2]);
            UINT8 phy     = (UINT8)atoi(argv[3]);

            if ((channel >= 0) && (channel <= 39) && (phy >= 1) && (phy <= 3))
            {
                return COMMAND_LE_RX_TEST_V3;
            }
        }
        print_usage_le_rx_test_v3(TRUE);
    }
    else if(COMPARE_STRING(argv[1],"le_tx_test_v3")==0)
    {
        if (argc == 6)
        {
            UINT8 channel = (UINT8)atoi(argv[2]);
            UINT8 length  = (UINT8)atoi(argv[3]);
            UINT8 pattern = (UINT8)atoi(argv[4]);
            UINT8 phy     = (UINT8)atoi(argv[5]);

            if ((channel >= 0) && (channel <= 39) && (length > 0) && (length <= 255))
            {
                if ((pattern >= 0) && (pattern <= 7) && (phy >= 1) && (phy <= 3))
                {
                    return COMMAND_LE_TX_TEST_V3;
                }
            }
        }
        print_usage_le_tx_test_v3(TRUE);
    }
    else if(COMPARE_STRING(argv[1],"le_enhenced_receiver_test")==0)
    {
        if (argc == 5)
        {
            UINT8 channel = (UINT8)atoi(argv[2]);
            UINT8 phy  = (UINT8)atoi(argv[3]);
            UINT8 mod_index = (UINT8)atoi(argv[4]);

            if ((channel >= 0) && (channel <= 39))
            {
                if ((phy >= 1) && (phy <= 3) && (mod_index >= 0) && (mod_index <= 1))
                {
                    return COMMAND_LE_ENHENCED_RX_TEST;
                }
            }
        }
        print_usage_le_enhenced_rx_test(TRUE);	
    }
    else if(COMPARE_STRING(argv[1],"le_enhenced_transmitter_test")==0)
    {
        if (argc == 6)
        {
            UINT8 channel = (UINT8)atoi(argv[2]);
            UINT8 length_of_data  = (UINT8)atoi(argv[3]);
            UINT8 payload = (UINT8)atoi(argv[4]);
            UINT8 phy     = (UINT8)atoi(argv[5]);

            if ((channel >= 0) && (channel <= 39))
            {
                if ((length_of_data >= 0) && (length_of_data <= 255) && (payload >= 0) && (payload <= 7) && (phy >= 1) && (phy <= 4))
                {
                    return COMMAND_LE_ENHENCED_TX_TEST;
                }
            }
        }
        print_usage_le_enhenced_tx_test(TRUE);
    }
    else if(COMPARE_STRING(argv[1],"get_agc_index")==0)
    {
        return COMMAND_GET_AGC_INDEX;
    }
    else
    {
        return COMMAND_MISMATCH;
    }	
    //printf ("parsing error - argument mismatched\n");
    return COMMAND_ARG_MISMATCH;
}
