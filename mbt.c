/*
 * mbt.c
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


/*****************************************************************************
 *
 *  Name:          mbt.c
 *
 *  Desc
ription:   The manufacturing Bluetooth test tool (MBT) is used to test and verify the RF
 *                 performance of the Cypress family of SoC Bluetooth BR/EDR/BLE standalone and
 *                 combo devices. Each test sends an HCI command to the device and then waits
 *                 for an HCI Command Complete event from the device.
 *
 *                 For usage description, execute:
 *
 *                 ./mbt help
 *
 * Version:    1.32
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/termios.h>
#include <pthread.h>
#include <stdint.h>
#include <stdbool.h>

#include "mbt.h"

/******************************************************
 *               Function Declarations
 ******************************************************/
extern void print_usage_set_environment_variables(BOOL full);
extern void print_usage_mbt_commands(BOOL full);
extern int parse_input_command_args(int argc, char **argv);

static void execute_reset(void);
static void execute_set_event_filter(void);
static void execute_write_scan_enable(void);
static void execute_enable_device_under_test_mode(void);
static void execute_loopback_mode(void);
static void execute_le_receiver_test(UINT16 rx_frequency);
static void execute_le_test_end(void);
static void execute_le_transmitter_test(UINT16 tx_frequency, UINT16 length, UINT16 pattern);
static void execute_set_tx_frequency_arm(BOOL carrier_on, int tx_frequency, int tx_mode, int tx_modulation_type, int tx_power_select, int tx_power_dBm_index);
static void execute_receive_only(UINT16 rx_frequency);
static void execute_radio_tx_test(char *bdaddr, int frequency, int modulation_type, int logical_channel, int packet_type, int packet_length, int tx_power_select, int tx_power_dBm_index);
static void stop_rx_test(int sig);
static void execute_radio_rx_test(char *bdaddr, int frequency, int modulation_type, int logical_channel, int packet_type, int packet_length);
static void execute_otp_write_word(char *offset, char *data);
static void execute_input_command(char *input);
static void execute_download_minidriver(void);
static void wait_for_auto_baud(void);
static void execute_update_baudrate(UINT32 baudrate);
/******************************************************
 *               Variables Definitions
 ******************************************************/
struct termios termios;

int fd_transport          = -1;
int fd_hcdfile       = -1;
FILE *fp_hexfile = NULL;
FILE *logfile = NULL;
UINT8 is_hex = 0;
int debug    = 1;
int check_auto_baud = 1;
UINT8 buffer[1024];
char* mbt_transport = NULL;
UINT8 mbt_transport_type = -1;
UINT32 download_baudrate = 115200;

UINT8 hci_reset[] = { 0x01, 0x03, 0x0c, 0x00 };
UINT8 enter_download_mode[] = {0x01, 0xED, 0xFF, 0x14, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xED, 0xFF, 0x01, 0x00};

/*
UINT8 hci_set_event_filter_cmd_compelete_event[] = { 0x05, 0x0C, 0x03, 0x02, 0x00, 0x02 };
UINT8 hci_write_scan_enable_cmd_compelete_event[] = { 0x01, 0x03, 0x0c, 0x00 };
UINT8 hci_enable_device_under_test_mode_cmd_compelete_event[] = { 0x01, 0x03, 0x0c, 0x00 };
*/

/******************************************************
 *               Function Definitions
 ******************************************************/

/*get transport and type*/
static int get_environment_variables()
{
    mbt_transport = getenv("MBT_TRANSPORT");

    if (mbt_transport == NULL)
    {
        fprintf(stderr, "environtment set MBT_TRANSPORT not found. error %d\n", errno);
        return MBT_TRANSPORT_ERROR;
    }

    printf("[MBT_TRANSPORT: %s]\n", mbt_transport);
    /*get transport type, UART or USB*/
    if (strncmp(mbt_transport, "/dev/ttyUSB", strlen("/dev/ttyUSB")) == 0)
    {
        mbt_transport_type = TRANSPORT_TYPE_UART;
    }
    else if (strncmp(mbt_transport, "/dev/ttymxc", strlen("/dev/ttymxc")) == 0)
    {
        mbt_transport_type = TRANSPORT_TYPE_UART;
    }
    else if (strncmp(mbt_transport, "/dev/btusb", strlen("/dev/btusb")) == 0)
    {
        mbt_transport_type = TRANSPORT_TYPE_USB;
    }
    else if (strncmp(mbt_transport, "/dev/ttySDIO", strlen("/dev/ttySDIO")) == 0)
    {
        mbt_transport_type = TRANSPORT_TYPE_UART;
    }
    else if (strncmp(mbt_transport, "/dev/ttyGS", strlen("/dev/ttyGS")) == 0)
    {
        mbt_transport_type = TRANSPORT_TYPE_UART;
    }
    else if (strncmp(mbt_transport, "/dev/ttyTHS", strlen("/dev/ttyTHS")) == 0)
    {
        mbt_transport_type = TRANSPORT_TYPE_UART;
    }
    else if (strncmp(mbt_transport, "/dev/ttyACM", strlen("/dev/ttyACM")) == 0)
    {
        mbt_transport_type = TRANSPORT_TYPE_UART;
    }
    else if (strncmp(mbt_transport, "/dev/ttyS", strlen("/dev/ttyS")) == 0)
    {
        mbt_transport_type = TRANSPORT_TYPE_UART;
    }
    else if (strncmp(mbt_transport, "/dev/ttyAMA", strlen("/dev/ttyAMA")) == 0)
    {
        mbt_transport_type = TRANSPORT_TYPE_UART;
    }
    else
    {
        /* tranport been set is not supported*/
         return MBT_TRANSPORT_ERROR;

    }
    return MBT_SUCCESS;
}

int parse_patchram(char *optarg)
{
    char *p;
    is_hex = 0;
    fprintf(stderr, "Enter parse_patchram \n");
    if (!(p = strrchr(optarg, '.'))) {
        fprintf(stderr, "file %s not an HCD/hex file\n", optarg);
        exit(3);
    }

    p++;

    if ( (strcasecmp("hcd", p) != 0) && (strcasecmp("hex", p) != 0)) {
        fprintf(stderr, "file %s not an HCD/Hex file\n", optarg);
        exit(4);
    }

    if (strcasecmp("hex", p) == 0)
    {
        is_hex = 1;
        fp_hexfile = fopen(optarg, "r");
        if (fp_hexfile == NULL)
            exit(5);
    }
    else if ((fd_hcdfile = open(optarg, O_RDONLY)) == -1) {
        fprintf(stderr, "file %s could not be opened, error %d\n", optarg, errno);
        exit(5);
    }


    fprintf(stderr, "Exit parse_patchram \n");
    return(0);
}

void init_uart(void)
{

    tcflush(fd_transport, TCIOFLUSH);
    tcgetattr(fd_transport, &termios);

    termios.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                | INLCR | IGNCR | ICRNL | IXON);
    termios.c_oflag &= ~OPOST;
    termios.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    termios.c_cflag &= ~(CSIZE | PARENB);
    termios.c_cflag |= CS8;
    termios.c_cflag |= CRTSCTS;
    //fprintf(stderr, "CTS %d\n", CRTSCTS);
    tcsetattr(fd_transport, TCSANOW, &termios);
    tcflush(fd_transport, TCIOFLUSH);
    tcsetattr(fd_transport, TCSANOW, &termios);
    tcflush(fd_transport, TCIOFLUSH);
    tcflush(fd_transport, TCIOFLUSH);
    cfsetospeed(&termios, B115200);
    cfsetispeed(&termios, B115200);
    tcsetattr(fd_transport, TCSANOW, &termios);
}

void init_uart_change_baudrate(void)
{

    tcflush(fd_transport, TCIOFLUSH);
    tcgetattr(fd_transport, &termios);

    termios.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                | INLCR | IGNCR | ICRNL | IXON);
    termios.c_oflag &= ~OPOST;
    termios.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    termios.c_cflag &= ~(CSIZE | PARENB);
    termios.c_cflag |= CS8;
    termios.c_cflag |= CRTSCTS;
    //fprintf(stderr, "CTS %d\n", CRTSCTS);
    tcsetattr(fd_transport, TCSANOW, &termios);
    tcflush(fd_transport, TCIOFLUSH);
    tcsetattr(fd_transport, TCSANOW, &termios);
    tcflush(fd_transport, TCIOFLUSH);
    tcflush(fd_transport, TCIOFLUSH);
    cfsetospeed(&termios, B921600);
    cfsetispeed(&termios, B921600);
    tcsetattr(fd_transport, TCSANOW, &termios);
}

void init_uart_change_baudrate3M(void)
{

    tcflush(fd_transport, TCIOFLUSH);
    tcgetattr(fd_transport, &termios);

    termios.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                | INLCR | IGNCR | ICRNL | IXON);
    termios.c_oflag &= ~OPOST;
    termios.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    termios.c_cflag &= ~(CSIZE | PARENB);
    termios.c_cflag |= CS8;
    termios.c_cflag |= CRTSCTS;
    //fprintf(stderr, "CTS %d\n", CRTSCTS);
    tcsetattr(fd_transport, TCSANOW, &termios);
    tcflush(fd_transport, TCIOFLUSH);
    tcsetattr(fd_transport, TCSANOW, &termios);
    tcflush(fd_transport, TCIOFLUSH);
    tcflush(fd_transport, TCIOFLUSH);
    cfsetospeed(&termios, B3000000);
    cfsetispeed(&termios, B3000000);
    tcsetattr(fd_transport, TCSANOW, &termios);
}

void cts_low_init(void)    //20220317 pull CTS to LOW //with Chuck Wang
{

    tcflush(fd_transport, TCIOFLUSH);
    tcgetattr(fd_transport, &termios);

    termios.c_iflag &= (IGNBRK | BRKINT | PARMRK | ISTRIP
                | INLCR | IGNCR | ICRNL | IXON);
    termios.c_oflag &= ~OPOST;
    termios.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    termios.c_cflag &= ~(CSIZE | PARENB);
    termios.c_cflag |= CS8;
    termios.c_cflag |= CRTSCTS;
    printf("Reset the chip to autobaud state and press ENTER key!\n");
    getchar();
    tcsetattr(fd_transport, TCSANOW, &termios);
    tcflush(fd_transport, TCIOFLUSH);
    //tcsetattr(fd_transport, TCSANOW, &termios);
    //tcflush(fd_transport, TCIOFLUSH);
    //tcflush(fd_transport, TCIOFLUSH);
    cfsetospeed(&termios, B3000000);
    //cfsetispeed(&termios, B115200);
    //tcsetattr(fd_transport, TCSANOW, &termios);
}

/*https://stackoverflow.com/questions/29335758/using-kbhit-and-getch-on-linux*/
int kbhit()
{
    //struct termios term;
    tcgetattr(0, &termios);

    //termios term2 = termios;
    termios.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &termios);

    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);

    tcsetattr(0, TCSANOW, &termios);

    return byteswaiting > 0;
}

void dump(UINT8 *out, int len)
{
    int i;

    for (i = 0; i < len; i++)
    {
        if (i && !(i % 16))
        {
            fprintf(stderr, "\n");
        }
        fprintf(stderr, " %02x", out[i]);
    }
    fprintf(stderr, "\n");
}

int read_event(int fd, UINT8 *buffer)
{
    int i = 0;
    int len = 3;
    int count;

    while ((count = read(fd, &buffer[i], len)) < len)
    {
        i += count;
        len -= count;
    }

    i += count;
    len = buffer[2];

    while ((count = read(fd, &buffer[i], len)) < len)
    {
        i += count;
        len -= count;
    }

    count += i;

    if(debug)
    {
        fprintf(stderr, "rx: (%d bytes)\n", count);
        dump(buffer, count);
    }
    //if (buffer[count-1]= '00')
    //{
	//fprintf(stderr,"auto-baud \n", len);
    //}

    if (logfile)
    {
        char str[100];
    	int slen = sprintf(str,"rx: (%d bytes)\n", count);
        fwrite(str,slen,1,logfile);
        for(i=0;i<count;i++)
        {
            if ( (i+1) %16 ==0)
                slen = sprintf(str," %02x \n",buffer[i]);
        else
        slen = sprintf(str," %02x ",buffer[i]);
            fwrite(str,slen,1,logfile);
        }
       slen = sprintf(str,"\n");
       fwrite(str,slen,1,logfile);
    }

    return count;
}

void hci_send_cmd(UINT8 *buf, int len)
{
    if(debug)
    {
        fprintf(stderr, "tx: (%d bytes)\n", len);
        dump(buf, len);
    }

    write(fd_transport, buf, len);
/*
    if (logfile)
    {
        char str[100];
    int i;
    int slen = sprintf(str,"tx: (%d bytes)\n", len);
        fwrite(str,slen,1,logfile);
        for(i=0;i<len;i++)
        {
            if ( (i+1) %16 ==0)
                slen = sprintf(str," %02x \n",buf[i]);
        else
        slen = sprintf(str," %02x ",buf[i]);
            fwrite(str,slen,1,logfile);
        }
       slen = sprintf(str,"\n");
       fwrite(str,slen,1,logfile);
    }
*/
}

unsigned int asc_to_int(char a)
{
    if (a >= 'A')
        return (a - 'A') + 10;
    else
        return a - '0';
}
unsigned int hex_to_int(const char *h)
{
    return asc_to_int(*h) * 0x10 + asc_to_int(*(h + 1));
}
static int check_sum(const char *str, int len)
{
    unsigned int sum, cal;
    int i;
    sum = hex_to_int(str + len - 2);
    for (cal = 0, i = 1; i < len - 2; i += 2)
        cal += hex_to_int(str + i);
    cal = 0x100 - cal & 0xFF;
    return sum == cal;
}
static int check_hex_line(const char *str, int len)
{
    if ((str[0] != ':') || (len < 11) || !check_sum(str, len) ||
        (hex_to_int(str + 1) * 2 + 11 != len))
        return 0;
    return 1;
}


static unsigned int lhex_to_int(const char *h)
{
    return hex_to_int(h) * 0x100 + hex_to_int(h + 2);
}
unsigned int launch_ram = 0;
#if 0
unsigned int base_addr=0;
void print_data(UINT8 *buffer,int data_len,unsigned int offaddr)
{
    FILE *temp_fp = logfile;
    if (temp_fp)
    {
        char str[100];
        int i;
        int slen;
        int new_line;
        int rem_len = data_len;
        int count = 0;
        for(i=0;i<data_len;i++)
        {
            if (count == 0)
            {
                if (rem_len >= 16)
                    slen = sprintf(str,"\n:%02X%04X00%02X",16,offaddr,buffer[i]);
                else
                    slen = sprintf(str,"\n:%02X%04X00%02X",rem_len,offaddr,buffer[i]);
                //slen = sprintf(str,"%02X\n",buffer[i]);
                new_line=1;
            }
            else
            {
                slen = sprintf(str,"%02X",buffer[i]);
                new_line=0;
            }
            count++;
            if (count == 16)
            {
                count = 0;
            }

            fwrite(str,slen,1,temp_fp);
            rem_len--;
            offaddr++;
        }
        if (new_line == 0)
        {
           slen = sprintf(str,"\n");
           fwrite(str,slen,1,temp_fp);
        }
    }
}
#endif

void send_buffer_data(ssize_t data_len, unsigned int dest_addr)
{
    buffer[0] = 0x01;
    buffer[1] = 0x4c;
    buffer[2] = 0xfc;
    buffer[3] = data_len + 4;
    buffer[4] = dest_addr;
    buffer[5] = dest_addr >> 8;
    buffer[6] = dest_addr >> 16;
    buffer[7] = dest_addr >> 24;
    //print_data(&buffer[8],data_len, dest_addr-base_addr);
    hci_send_cmd(buffer, data_len + 8);
    read_event(fd_transport, buffer);
    //hci_send_cmd(buffer, data_len + 8);
    //read_event(fd_transport, buffer);
}


void proc_patchram(bool sendMini)
{
    int len;
    if (sendMini == true) {
    	UINT8 hci_download_minidriver[] = { 0x01, 0x2e, 0xfc, 0x00 };

    	fprintf(stderr,"\n Sending Download minidriver \n", len);
    	hci_send_cmd(hci_download_minidriver, sizeof(hci_download_minidriver));
    	read_event(fd_transport, buffer);
    } else {
    	fprintf(stderr,"Skipping minidriver download \n", len);
    }
    int dot_print=0;

    if (is_hex)
    {
        ssize_t len,i,data_len=0;
        char *rbuf = NULL;
        size_t buflen;
        volatile unsigned int addr = 0;
        unsigned int dest_addr = 0,offset_addr = 0,next_offset_addr=0;
        logfile = fopen("log.txt", "w");
        while ((len = getline(&rbuf, &buflen, fp_hexfile)) > 0)
        {
            int type;

            ssize_t curr_len;
            while ((rbuf[len - 1] == '\r') || (rbuf[len - 1] == '\n'))
                len--;
            if (!check_hex_line(rbuf, len))
                break;
            type = hex_to_int(rbuf + 7);
            curr_len = hex_to_int(rbuf + 1);
            printf("currlen %d type %d data_len %d dest_addr 0x%x \n",curr_len, type,data_len,dest_addr);
            if (type == 0)
            {
                offset_addr = lhex_to_int(rbuf + 3);
                if (data_len == 0)
                {
                    dest_addr = addr + lhex_to_int(rbuf + 3);
                }
                else
                {
                    // if offset discontinous, send buffered data
                    if (offset_addr != next_offset_addr)
                    {
                        send_buffer_data(data_len, dest_addr);
                        data_len = 0;
                        dest_addr = addr + lhex_to_int(rbuf + 3);
                    }
                }
                next_offset_addr = offset_addr+curr_len;
                // if data size exeeds MAX SIZE, send buffered data
                if ((curr_len+data_len+8) <255)
                {
                    for (i = 0; i < curr_len; i++)
                    {
                        buffer[data_len+8] = hex_to_int(rbuf + 9 + i * 2);
                        data_len++;
                    }
                    continue;
                }
            }
            // Type has change so send buffered data is any
            if ( data_len)
            {
                send_buffer_data(data_len, dest_addr);
                data_len = 0;
                dest_addr = 0;
            }
            if (type == 0)
            {
                if (data_len == 0)
                {
                    dest_addr = addr + lhex_to_int(rbuf + 3);
                }
                for (i = 0; i < curr_len; i++)
                {
                    buffer[data_len+8] = hex_to_int(rbuf + 9 + i * 2);
                    data_len++;
                }
                continue;
            }
            else if (type == 4)
            {
                addr = lhex_to_int(rbuf + 9) * 0x10000;
                printf("bump addr to 0x%08X\n", addr);
#if 0
                base_addr = addr;
#endif
                dest_addr = 0;
                data_len = 0;
            }
            else if (type == 1)
            {
                printf("Case 1 :) \n");
                dest_addr = 0;
                data_len = 0;
            }
            else
            {
                return;
            }
        }

        UINT8 temp_data [] = {0x01, 0x4e, 0xfc, 0x04, 0xff, 0xff, 0xff, 0xff};
        // if address is given send launch_ram on given address
        if (launch_ram)
        {
            temp_data[4] = launch_ram;
            temp_data[5] = launch_ram >> 8;
            temp_data[6] = launch_ram >> 16;
            temp_data[7] = launch_ram >> 24;
        }
        printf("Launch ram 0x%x",launch_ram);
	debug = 0;
        hci_send_cmd(temp_data, sizeof(temp_data));
        read_event(fd_transport, buffer);
	debug = 1;

        fclose(logfile);
    }
    else
    {
	fprintf(stderr,"... hcd files ... \n", len);
        while( read(fd_hcdfile, &buffer[1], 3) )
        {
        buffer[0] = 0x01;

        len = buffer[3];

        read(fd_hcdfile, &buffer[4], len);

	debug = 0;
        hci_send_cmd(buffer, len + 4);
        //read_event(fd_transport, buffer);
	debug = 1;
        dot_print++;
	fprintf(stderr,".", len);

        if (dot_print%100==0)
        {
        	fprintf(stderr,".\n", len);    
        }
        }
    }
    fprintf(stderr,"\nExit proc_patchram %d \n", len);
}

void execute_download(char* file_name)
{
    char* pathname = file_name;

    fprintf(stderr,"Download BT firmware. This may take a few moments...\n", "\n");  

    execute_reset();

    parse_patchram(pathname);

    proc_patchram(false);

    //init_uart(); //back to 115200

    execute_reset();

    fprintf(stderr,"Current state: Completed successfully", "\n");
}

void execute_download_with_minidriver(char* file_name)
{
    char* pathname = file_name;

    fprintf(stderr,"Download BT firmware. This may take a few moments...\n", "\n");  

    execute_reset();

    parse_patchram(pathname);

    proc_patchram(true);

    //init_uart(); //back to 115200

    execute_reset();

    fprintf(stderr,"Current state: Completed successfully", "\n");
}


void execute_download_autobaud(char* file_name)
{
    char* pathname = file_name;

    cts_low_init();

    init_uart_change_baudrate(); //raise baudrate

    fprintf(stderr,"Download BT firmware. This may take a few moments...", "\n");

    //execute_reset();

    wait_for_auto_baud();

    parse_patchram(pathname);

    proc_patchram(false);

    sleep(1);

    init_uart(); //back to 115200

    execute_reset();

    fprintf(stderr,"Current state: Completed successfully", "\n");
}

void execute_download_auto_threeM(char* file_name)
{
    char* pathname = file_name;

    cts_low_init();

    init_uart_change_baudrate3M(); //raise baudrate

    fprintf(stderr,"Download BT firmware. This may take a few moments...", "\n");

    hci_send_cmd(enter_download_mode, sizeof(enter_download_mode));
    
    read_event(fd_transport, buffer);

    parse_patchram(pathname);

    proc_patchram(false);

    sleep(1);

    init_uart(); //back to 115200

    execute_reset();

    fprintf(stderr,"Current state: Completed successfully", "\n");
}


void execute_download_no_reset(char* file_name)
{
    char* pathname = file_name;

    fprintf(stderr,"Download BT firmware. This may take a few moments...", "\n");
    fprintf(stderr,"NO RESET DOWNLOAD", "\n");

    parse_patchram(pathname);

    proc_patchram(false);

    //execute_reset();
    fprintf(stderr,"Current state: Completed successfully", "\n");
}

/*******************************************************************************
* Function Name: execute_download_threeM
********************************************************************************
* Summary:
* 	This function run for software 3M download firmware, only support in H1 now.
*       H1 need boot in normal mode, not in autobaud mode.
*       After download success, the controller baudrate will keep in 3M
*       1. Send Reset first
*       2. Update Controller baudrate to 3M
*       3. Update Host baudrate to 3M
*       4. Send EnterDownloadMode
*       5. Wirte patch
*       6. Launch ram
*
* Parameters:
*  	char *file_name:  firmware file name
*
* Return:
*  None
*
*******************************************************************************/
static void execute_download_threeM(char* file_name)
{
    char* pathname = file_name;
    int ret = 0;
    int len = 0;

    printf("2. HCI Reset\n");
    hci_send_cmd(hci_reset, sizeof(hci_reset));
    
    ret = read_event(fd_transport, buffer);
    if (ret > 3)
    {
        len = buffer[2];
        if (buffer[3 + len - 1] != 0)
        {
            printf("hci reset fail\n");
            return;
        }
    } else {
        printf("hci reset fail\n");
        return;
    }
    printf("hci reset success\n");

    printf("3. Update Controller Baudrate to 3M\n");
    execute_update_baudrate(3000000);

    printf("4. Update Host baud rate to 3M\n");
    init_uart_change_baudrate3M(); //raise baudrate

    printf("5. Send EnterDownloadMode\n");
    hci_send_cmd(enter_download_mode, sizeof(enter_download_mode));
    ret = read_event(fd_transport, buffer);
    if (ret > 3)
    {
        len = buffer[2];
        if (buffer[3 + len - 1] != 0)
        {
            printf("VSC Enter_Download_Mode fail\n");
            return;
        }
    } else {
        printf("VSC Enter_Download_Mode fail\n");
        return;
    }
    printf("Enter_Download_Mode Success\n");
    sleep(1);
    printf("6. write patch fw\n");
    parse_patchram(pathname);

    printf("7. Launch RAM\n");
    proc_patchram(false);

    printf("Current state: Completed successfully\n");
}



/******************************************************
 *            HCI Function Definitions
 ******************************************************/
void expired(int sig)
{
    hci_send_cmd(hci_reset, sizeof(hci_reset));
    alarm(4);
}

void expired_auto_baud(int sig)
{
    debug=0;
    hci_send_cmd(hci_reset, sizeof(hci_reset));
    debug=1;
    printf("\n\Please reset the chip to autobaud state!\n");
    alarm(2);
}

static void wait_for_auto_baud(void)
{
    signal(SIGALRM, expired_auto_baud);
	
    debug=0;
    hci_send_cmd(hci_reset, sizeof(hci_reset));
    debug=1;

    printf("\n\nPlease reset the chip to autobaud state!\n");

    alarm(2);

    read_event(fd_transport, buffer);

    alarm(0);
}

static void execute_reset(void)
{
    signal(SIGALRM, expired);

    hci_send_cmd(hci_reset, sizeof(hci_reset));
    

    alarm(4);

    read_event(fd_transport, buffer);

    alarm(0);
}

static void execute_download_minidriver(void)
{
    int len;
    UINT8 hci_download_minidriver[] = { 0x01, 0x2e, 0xfc, 0x00 };
    hci_send_cmd(hci_download_minidriver, sizeof(hci_download_minidriver));
    read_event(fd_transport, buffer);
}

static void execute_input_command(char *input)
{

    UINT8 params[strlen(input)/2+1];
    UINT8 input_commands[strlen(input)/2+1];
    memset(&params, 0, sizeof(params));
    memset(&input_commands, 0, sizeof(input_commands));

    printf("input=%s\ninput_length=%d\n", input, strlen(input));
    
    int j = 0;
    for (int i = 0; i < strlen(input)/2; i++)
    {
        sscanf(&input[j], "%02x", &params[i]);
        j = j + 2;
    }

    input_commands[0] = 1;

    for (int i = 0; i < strlen(input)/2; i++)
    {        
        input_commands[i + 1] = params[i];
    }

    printf("Sending HCI Command:\n");


    hci_send_cmd(input_commands, sizeof(input_commands));

    read_event(fd_transport, buffer);

    return 1;
}

static void execute_set_event_filter(void)
{
    UINT8 hci_set_event_filter[] = { 0x01, 0x05, 0x0C, 0x03, 0x02, 0x00, 0x02 };

    hci_send_cmd(hci_set_event_filter, sizeof(hci_set_event_filter));

    read_event(fd_transport, buffer);

}

static void execute_write_scan_enable(void)
{
    UINT8 hci_write_scan_enable[] = { 0x01, 0x1A, 0x0C, 0x01, 0x03 };

    hci_send_cmd(hci_write_scan_enable, sizeof(hci_write_scan_enable));

    read_event(fd_transport, buffer);

}

static void execute_enable_device_under_test_mode(void)
{
    UINT8 hci_enable_device_under_test_mode[] = { 0x01, 0x03, 0x18, 0x00 };

    hci_send_cmd(hci_enable_device_under_test_mode, sizeof(hci_enable_device_under_test_mode));

    read_event(fd_transport, buffer);

}

static void execute_loopback_mode(void)
{
    execute_set_event_filter();
    execute_write_scan_enable();
    execute_enable_device_under_test_mode();
}

static void execute_write_bdaddr(char *bdaddr)
{
    int   params[6];
    int   i;
    UINT8 hci_write_bdaddr[] = { 0x01, 0x01, 0xFC, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    sscanf(bdaddr, "%02x%02x%02x%02x%02x%02x", &params[0], &params[1], &params[2], &params[3], &params[4], &params[5]);
    printf("[bdaddr: %02x%02x%02x%02x%02x%02x]\n", params[0], params[1], params[2], params[3], params[4], params[5]);

    for( i = 0; i < 6; i++ )
    {
        hci_write_bdaddr[i + 4] = params[5 - i];    //bd address
    }

    hci_send_cmd(hci_write_bdaddr, sizeof(hci_write_bdaddr));

    read_event(fd_transport, buffer);
}

static void execute_otp_write_word(char *offset, char *data)
{
    int   params[5];
    int   i;
    UINT8 otp_write_word[] = { 0x01, 0xEC, 0xFD, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    sscanf(offset, "%02x", &params[0]);
    printf("[offset: %02x]\n", params[0]);
    sscanf(data, "%02x%02x%02x%02x", &params[1], &params[2], &params[3], &params[4]);
    printf("[data: %02x%02x%02x%02x]\n", params[1], params[2], params[3], params[4]);

    otp_write_word[4] = params[0];
    
    for( i = 0; i < 4; i++ )
    {
        otp_write_word[i + 8] = params[4 - i];    //otp data
    }

    hci_send_cmd(otp_write_word, sizeof(otp_write_word));

    read_event(fd_transport, buffer);
}

static void execute_read_bdaddr(void)
{
    UINT8 hci_read_bdaddr[] = { 0x01, 0x09, 0x10, 0x00 };

    hci_send_cmd(hci_read_bdaddr, sizeof(hci_read_bdaddr));

    read_event(fd_transport, buffer);
}



static void execute_read_name(void)
{
    UINT8 hci_read_name[] = { 0x01, 0x14, 0x0C, 0x00 };

    hci_send_cmd(hci_read_name, sizeof(hci_read_name));

    read_event(fd_transport, buffer);
}

static void execute_update_baudrate(UINT32 baudrate)
{
    UINT8 hci_update_baud[] = {0x01, 0x18, 0xFC, 0x06, 0x00, 0x00, 0xC0, 0xC6, 0x2D, 0x00};

    hci_update_baud[6] = baudrate & 0xFF;
    hci_update_baud[7] = (baudrate >> 8) & 0xFF;
    hci_update_baud[8] = (baudrate >> 16) & 0xFF;
    hci_update_baud[9] = (baudrate >> 24) & 0xFF;
    hci_send_cmd(hci_update_baud, sizeof(hci_update_baud));
    read_event(fd_transport, buffer);
}

static void execute_read_version(void)
{
    UINT8 hci_read_name[] = { 0x01, 0x01, 0x10, 0x00 };

    hci_send_cmd(hci_read_name, sizeof(hci_read_name));

    read_event(fd_transport, buffer);
}

static void execute_le_receiver_test(UINT16 rx_channel)
{
    UINT8 hci_le_receiver_test[] = { 0x01, 0x01D, 0x20, 0x01, 0x00 };
    printf("[rx_channel: %d]\n", rx_channel);

    hci_le_receiver_test[4] = rx_channel;

    hci_send_cmd(hci_le_receiver_test, sizeof(hci_le_receiver_test));

    read_event(fd_transport, buffer);
}

static void execute_le_test_end(void)
{
    UINT8 hci_le_test_end[] = { 0x01, 0x1f, 0x20, 0x00 };

    hci_send_cmd(hci_le_test_end, sizeof(hci_le_test_end));

    read_event(fd_transport, buffer);

    if( (buffer[4]==0x1F) && (buffer[5]==0x20) )
    {
        printf("Packets received: %d\n", buffer[7] + (buffer[8]<<8) );
    }
}

static void execute_le_transmitter_test(UINT16 tx_channel, UINT16 length, UINT16 pattern)
{
    UINT8 hci_le_transmitter_test[] = { 0x01, 0x01E, 0x20, 0x03, 0x00, 0x00, 0x00 };
    printf("[tx_channel:%d length:%d pattern:%d]\n", tx_channel, length, pattern);

    hci_le_transmitter_test[4] = tx_channel;
    hci_le_transmitter_test[5] = length;
    hci_le_transmitter_test[6] = pattern;

    hci_send_cmd(hci_le_transmitter_test, sizeof(hci_le_transmitter_test));

    read_event(fd_transport, buffer);
}

static void execute_set_tx_frequency_arm(BOOL carrier_on, int tx_frequency, int tx_mode, int tx_modulation_type, int tx_power_select, int tx_power_dBm_index)
{
    INT8 hci_set_tx_frequency_arm[] = { 0x01, 0x014, 0xfc, 0x07, 0, 0, 0, 0, 0, 0, 0};
    printf("[carrier_on:%d tx_frequency:%d tx_mode:%d tx_modulation_type:%d tx_power_select:%d tx_power_dBm_index:%d]\n",
        carrier_on, tx_frequency, tx_mode, tx_modulation_type, tx_power_select, tx_power_dBm_index);

    hci_set_tx_frequency_arm[4] = (carrier_on == 0) ? 1 : 0;
    hci_set_tx_frequency_arm[5] = (carrier_on == 1) ? (tx_frequency - 2400) : 2;
    hci_set_tx_frequency_arm[6] = tx_mode;                    // unmodulated
    hci_set_tx_frequency_arm[7] = tx_modulation_type;         // modulation type
    hci_set_tx_frequency_arm[8] = tx_power_select;       //8:power in dBm, 9:index from table
    if ((tx_power_select >= 0) && (tx_power_select <= 7)) // 0(0dBm) - 7(-28dBm)
    {
        hci_set_tx_frequency_arm[9] = 0;
        hci_set_tx_frequency_arm[10] = 0;
    }
    else if (tx_power_select == 8)
    {
        hci_set_tx_frequency_arm[9] = tx_power_dBm_index; //8: specify Power in dBm
        hci_set_tx_frequency_arm[10] = 0;
    }
    else if (tx_power_select == 9)
    {
        hci_set_tx_frequency_arm[9] = 0;
        hci_set_tx_frequency_arm[10] = tx_power_dBm_index; //9: specify Power Table index
    }
    hci_send_cmd(hci_set_tx_frequency_arm, sizeof(hci_set_tx_frequency_arm));

    read_event(fd_transport, buffer);
}

static void execute_receive_only(UINT16 rx_frequency)
{
    UINT16 chan_num = rx_frequency - 2400;
    UINT8 hci_write_receive_only[] = { 0x01, 0x02b, 0xfc, 0x01, 0x00 };
    printf("[rx_frequency: %dMHz]\n", rx_frequency);

    hci_write_receive_only[4] = chan_num;

    hci_send_cmd(hci_write_receive_only, sizeof(hci_write_receive_only));

    read_event(fd_transport, buffer);
}

static void execute_radio_tx_test(char *bdaddr, int frequency, int modulation_type, int logical_channel, int packet_type, int packet_length, int tx_power_select, int tx_power_dBm_index)
{
    int i;
    int params[6];
    INT8 hci_radio_tx_test[] = {0x01, 0x051, 0xfc, 0x10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    printf("[frequency:%d modulation_type:%d logical_channel:%d packet_type:%d packet_length:%d tx_power_select:%d tx_power_dBm_index:%d]\n",
        frequency, modulation_type, logical_channel, packet_type, packet_length, tx_power_select, tx_power_dBm_index);

    sscanf(bdaddr, "%02x%02x%02x%02x%02x%02x", &params[0], &params[1], &params[2], &params[3], &params[4], &params[5]);
    //printf("bdaddr: %02x%02x%02x%02x%02x%02x\n", params[0], params[1], params[2], params[3], params[4], params[5]);

    for( i = 0; i < 6; i++ )
    {
        hci_radio_tx_test[i+4] = params[5-i];    //bd address
    }
    hci_radio_tx_test[10] = (frequency==0) ? 0 : 1;        //0: hopping, 1: single frequency
    hci_radio_tx_test[11] = (frequency==0) ? 0 : (frequency - 2402);  //0: hopping 0-79:channel number (0: 2402 MHz)
    hci_radio_tx_test[12] = modulation_type;               //data pattern (3: 0xAA  8-bit Pattern)
    hci_radio_tx_test[13] = logical_channel;               //logical_Channel (0:ACL EDR, 1:ACL Basic)
    hci_radio_tx_test[14] = packet_type;                //modulation type (Packet_Type. 3:DM1, 4: DH1 / 2-DH1)
    hci_radio_tx_test[15] = packet_length & 0xff;          //low byte of packet_length
    hci_radio_tx_test[16] = (packet_length>>8) & 0xff;     //high byte of packet_length
    hci_radio_tx_test[17] = tx_power_select;
    if ((tx_power_select >= 0) && (tx_power_select <= 7)) // 0(0dBm) - 7(-28dBm)
    {
        hci_radio_tx_test[18] = 0;
        hci_radio_tx_test[19] = 0;
    }
    else if (tx_power_select == 8)
    {
        hci_radio_tx_test[18] = tx_power_dBm_index; //8: specify Power in dBm
        hci_radio_tx_test[19] = 0;
    }
    else if (tx_power_select == 9)
    {
        hci_radio_tx_test[18] = 0;
        hci_radio_tx_test[19] = tx_power_dBm_index; //9: specify Power Table index
    }
    hci_send_cmd(hci_radio_tx_test, sizeof(hci_radio_tx_test));

    read_event(fd_transport, buffer);
}

static void stop_rx_test(int sig)
{
    UINT8 hci_reset[] = {0x01, 0x03, 0x0c, 0x00};

    hci_send_cmd(hci_reset, sizeof(hci_reset));
    read_event(fd_transport, buffer);

    close(fd_transport);

    printf("stop rx_test.\n");
    exit(EXIT_SUCCESS);
}

static void execute_radio_rx_test(char *bdaddr, int frequency, int modulation_type, int logical_channel, int packet_type, int packet_length)
{
    int i, len;
    int params[6];
    UINT8 hci_radio_rx_test[] = {0x01, 0x52, 0xfc, 0x0e, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    /* register signal 'Ctrl+C' */
    signal(SIGINT, stop_rx_test);

    printf("[frequency:%d modulation_type:%d logical_channel:%d packet_type:%d packet_length:%d]\n",
        frequency, modulation_type, logical_channel, packet_type, packet_length);

    sscanf(bdaddr, "%02x%02x%02x%02x%02x%02x", &params[0], &params[1], &params[2], &params[3], &params[4], &params[5]);
    //printf("bdaddr: %02x%02x%02x%02x%02x%02x\n", params[0], params[1], params[2], params[3], params[4], params[5]);

    for( i = 0; i < 6; i++ )
    {
        hci_radio_rx_test[i+4] = params[5-i];
    }
    hci_radio_rx_test[10] = 0xe8;                          //low byte of report perioe in ms (1sec = 1000ms, 0x03e8)
    hci_radio_rx_test[11] = 0x03;                          //high byte
    hci_radio_rx_test[12] = frequency - 2402;
    hci_radio_rx_test[13] = modulation_type;               //data pattern (3: 0xAA 8-bit Pattern)
    hci_radio_rx_test[14] = logical_channel;               //logical_Channel (0:ACL EDR, 1:ACL Basic)
    hci_radio_rx_test[15] = packet_type;                //modulation type (Packet_Type. 3:DM1, 4: DH1 / 2-DH1)
    hci_radio_rx_test[16] = packet_length & 0xff;          //low byte of packet_length
    hci_radio_rx_test[17] = (packet_length>>8) & 0xff;     //high byte of packet_length

    hci_send_cmd(hci_radio_rx_test, sizeof(hci_radio_rx_test));
    read_event(fd_transport, buffer);
    printf("\nrx_test is running... hit any key or Ctrl+C to stop the test.\n");

    /*loop and handle the rx_test statistics report until any key pressed*/
    while (1)
    {
        i=0;
        len=0;
        memset(&buffer, 0, sizeof(buffer));

        /*read transport*/
        while (1)
        {
            len = read_event(fd_transport, buffer);
            if (len == 36)
            {
                break;
            }
        }

        /*statistics*/
        if ((len == 36) && (buffer[0]==0x04 && buffer[1]==0xFF && buffer[2]==0x21 && buffer[3]==0x07))
        {
            printf ("[rx_test statistics]\n");
            printf ("    Sync_Timeout_Count:     0x%x\n",buffer[4]|buffer[5]<<8|buffer[6]<<16|buffer[7]<<24);
            printf ("    HEC_Error_Count:        0x%x\n",buffer[8]|buffer[9]<<8|buffer[10]<<16|buffer[11]<<24);
            printf ("    Total_Received_Packets: 0x%x\n",buffer[12]|buffer[13]<<8|buffer[14]<<16|buffer[15]<<24);
            printf ("    Good_Packets:           0x%x\n",buffer[16]|buffer[17]<<8|buffer[18]<<16|buffer[19]<<24);
            printf ("    CRC_Error_Packets:      0x%x\n",buffer[20]|buffer[21]<<8|buffer[22]<<16|buffer[23]<<24);
            printf ("    Total_Received_Bits:    0x%x\n",buffer[24]|buffer[25]<<8|buffer[26]<<16|buffer[27]<<24);
            printf ("    Good_Bits:              0x%x\n",buffer[28]|buffer[29]<<8|buffer[30]<<16|buffer[31]<<24);
            printf ("    Error_Bits:             0x%x\n",buffer[32]|buffer[33]<<8|buffer[34]<<16|buffer[35]<<24);
            printf("\n");
        }

        if (kbhit()) // hit any key to stop the rx_test.
        {
            printf("stop rx_test. reset.\n");
            execute_reset();
            return;
        }
    }
}


static void execute_le_receiver_test_v3(UINT8 rx_channel, UINT8 phy)
{
    UINT8 hci_le_rx_test_v3[] = { 0x01, 0x04F, 0x20, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    UINT8 mod_idx, cte_length, cte_type, slot_duration, len_sw_pattern, ant_id;
    printf("[rx_channel:%d phy:%d modulation_index:%d]\n", rx_channel, phy, mod_idx);

    mod_idx = 0;         // 0 for standard modulation, 1 for stable modulation
    cte_length = 0;      // 0 for not use, 20 for maximum duty-cycle
    cte_type = 0;        // 0 for AoA
    slot_duration = 0;
    len_sw_pattern = 0;
    ant_id = 0;

    hci_le_rx_test_v3[4] = rx_channel;
    hci_le_rx_test_v3[5] = phy;
    hci_le_rx_test_v3[6] = mod_idx;
    hci_le_rx_test_v3[7] = cte_length;
    hci_le_rx_test_v3[8] = cte_type;
    hci_le_rx_test_v3[9] = slot_duration;
    hci_le_rx_test_v3[10] = len_sw_pattern;
    hci_le_rx_test_v3[11] = ant_id;

    hci_send_cmd(hci_le_rx_test_v3, sizeof(hci_le_rx_test_v3));

    read_event(fd_transport, buffer);
}

#define MBT_LE_TX_TEST_V3_CMD_HDR       4
#define MBT_LE_TX_TEST_V3_PARAM_LEN     7     // CMD:4, PARAM:7
#define MBT_LE_TX_TEST_V3_ANTID_LEN    12
#define MBT_LE_TX_TEST_V3_CMD_LEN      MBT_LE_TX_TEST_V3_CMD_HDR + MBT_LE_TX_TEST_V3_PARAM_LEN + MBT_LE_TX_TEST_V3_ANTID_LEN



static void execute_le_transmitter_test_v3(UINT8 tx_channel, UINT8 length, UINT8 pattern, UINT8 phy)
{
    UINT8 hci_le_tx_test_v3[MBT_LE_TX_TEST_V3_CMD_LEN] = { 0x01, 0x050, 0x20, 0x13, 0x00, };
    UINT8 cte_length, cte_type, len_sw_pattern, cmd_length;
    UINT8 ant_id[MBT_LE_TX_TEST_V3_ANTID_LEN] = {0x05, 0x1D, 0x05, 0x05, 0x05, 0x0D, 0x05, 0x05, 0x05, 0x15, 0x05, 0x05};
    printf("[tx_channel:%d length:%d pattern:%d phy:%d]\n", tx_channel, length, pattern, phy);

    cte_length = 20;      // 0 for not use, 20 for maximum duty-cycle
    cte_type = 0;        // 0 for AoA
    len_sw_pattern = MBT_LE_TX_TEST_V3_ANTID_LEN;
    cmd_length = MBT_LE_TX_TEST_V3_PARAM_LEN + MBT_LE_TX_TEST_V3_ANTID_LEN;

    hci_le_tx_test_v3[3] = cmd_length;
    hci_le_tx_test_v3[4] = tx_channel;
    hci_le_tx_test_v3[5] = length;
    hci_le_tx_test_v3[6] = pattern;
    hci_le_tx_test_v3[7] = phy;
    hci_le_tx_test_v3[8] = cte_length;
    hci_le_tx_test_v3[9] = cte_type;
    hci_le_tx_test_v3[10] = len_sw_pattern;
    memcpy(hci_le_tx_test_v3+11, ant_id, MBT_LE_TX_TEST_V3_ANTID_LEN);
    hci_send_cmd(hci_le_tx_test_v3, MBT_LE_TX_TEST_V3_CMD_LEN);

    read_event(fd_transport, buffer);
}


static void execute_le_enhanced_receiver_test(UINT8 rx_channel, UINT8 phy, UINT8 mod_idx)
{
    UINT8 hci_le_enhanced_receiver_test[] = { 0x01, 0x33, 0x20, 0x03, 0x00, 0x00, 0x00 };
    printf("[rx_channel:%d phy:%d modulation_index:%d]\n", rx_channel, phy, mod_idx);
    hci_le_enhanced_receiver_test[4] = rx_channel;
    hci_le_enhanced_receiver_test[5] = phy;
    hci_le_enhanced_receiver_test[6] = mod_idx;

    hci_send_cmd(hci_le_enhanced_receiver_test, sizeof(hci_le_enhanced_receiver_test));

    read_event(fd_transport, buffer);
}

static void execute_le_enhanced_transmitter_test(UINT8 tx_channel, UINT8 length_of_data, UINT8 payload, UINT8 phy)
{
    UINT8 hci_le_enhanced_transmitter_test[] = { 0x01, 0x34, 0x20, 0x04, 0x00, 0x00, 0x00, 0x00 };
    printf("[tx_channel:%d length_of_test_data:%d packet_payload:%d phy:%d]\n", tx_channel, length_of_data, payload, phy);
    hci_le_enhanced_transmitter_test[4] = tx_channel;
    hci_le_enhanced_transmitter_test[5] = length_of_data;
    hci_le_enhanced_transmitter_test[6] = payload;
    hci_le_enhanced_transmitter_test[7] = phy;

    hci_send_cmd(hci_le_enhanced_transmitter_test, sizeof(hci_le_enhanced_transmitter_test));

    read_event(fd_transport, buffer);
}

static void stop_get_agc_index(int sig)
{
    UINT8 hci_reset[] = {0x01, 0x03, 0x0c, 0x00};

    //hci_send_cmd(hci_reset, sizeof(hci_reset));
    //read_event(fd_transport, buffer);

    close(fd_transport);

    printf("stop stop_get_agc_index\n");
    exit(EXIT_SUCCESS);
}

static void execute_get_agc_index(void)
{
    int i, len;
    UINT8 gain_index;
    UINT8 hci_cmd[] = {0x01, 0x0C, 0xFC, 0x06, 0x06, 0xEC, 0x05, 0xB2, 0x8F, 0x00}; //  0x8fb205ec Peak Address for RX gain index
    UINT32 agc_status2;

    /* register signal 'Ctrl+C' */
    signal(SIGINT, stop_get_agc_index);
    printf("\nget_agc_index is running... hit any key or Ctrl+C to stop the test.\n");

    /*loop and handle the rx_test statistics report until any key pressed*/
    while (1)
    {
        hci_send_cmd(hci_cmd, sizeof(hci_cmd));
        debug = FALSE;
        read_event(fd_transport, buffer);
        agc_status2 = buffer[13] + (buffer[14] << 8);
        gain_index = (agc_status2 >> 8) & 0x1F;     // 12:8 - gainPtrZsync
        printf("[gain_index:%d], 0x%08x\n", gain_index, agc_status2);

        sleep(1);
    }
}



/******************************************************
 *                        main
 ******************************************************/
int main (int argc, char **argv)
{
    int command, carrier_on;;

    if( argc < 2 )
    {
        print_usage_mbt_commands(FALSE);
        printf("\n");
        exit(EXIT_FAILURE);
    }

    if (get_environment_variables())
    {
        print_usage_set_environment_variables(TRUE);
        exit(EXIT_FAILURE);
    }

    if( (fd_transport = open(mbt_transport, O_RDWR | O_NOCTTY)) == -1 )
    {
        printf("port %s could not be opened, error %d\n", mbt_transport, errno);
        exit(EXIT_FAILURE);
    }
    printf("Wait Controller Detect CTS low\n");
    usleep(50);

    if (mbt_transport_type == TRANSPORT_TYPE_UART)
    {
        /* initialize transport uart */
        init_uart();
	printf("Init UART ..........\n");
    }

    /* parse commands*/
    command = parse_input_command_args(argc, argv);
    switch( command )
    {
    case COMMAND_RESET:
        execute_reset();
        break;

    case COMMAND_LOOPBACK_MODE:
        execute_loopback_mode();
        break;

    case COMMAND_DOWNLOAD:
        execute_download(argv[2]);
        break;

    case COMMAND_DOWNLOAD_NO_RESET:
        execute_download_no_reset(argv[2]);
        break;

    case COMMAND_DOWNLOAD_AUTOBAUD:
        execute_download_autobaud(argv[2]);
        break;

    case COMMAND_WRITE_BDADDR:
        execute_write_bdaddr(argv[2]);
        break;

    case COMMAND_OTP_WRITE_WORD:
        execute_otp_write_word(argv[2],argv[3]);
        break;

    case COMMAND_INPUT_COMMAND:
        execute_input_command(argv[2]);
        break;

    case COMMAND_READ_BDADDR:
        execute_read_bdaddr();
        break;

    case COMMAND_LE_TEST_RECEIVER:
        execute_le_receiver_test(atoi(argv[2]));
        break;

    case COMMAND_LE_TEST_END:
        execute_le_test_end();
        break;

    case COMMAND_LE_TEST_TRANSMITTER:
        execute_le_transmitter_test(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
        break;

    case COMMAND_SET_TX_FREQUENCY_ARM:
        carrier_on = atoi(argv[2]);
        if(carrier_on == 0)
            execute_set_tx_frequency_arm(carrier_on, 2402, 0, 0, 0, 0);
        else
            execute_set_tx_frequency_arm(carrier_on, atoi(argv[3]), atoi(argv[4]), atoi(argv[5]), atoi(argv[6]), atoi(argv[7]));
        break;

    case COMMAND_RECEIVE_ONLY:
        execute_receive_only(atoi(argv[2]));
        break;

    case COMMAND_TX_TEST:
        execute_radio_tx_test(argv[2], atoi(argv[3]), atoi(argv[4]), atoi(argv[5]), atoi(argv[6]), atoi(argv[7]), atoi(argv[8]), atoi(argv[9]));
        break;

    case COMMAND_RX_TEST:
        execute_radio_rx_test(argv[2], atoi(argv[3]), atoi(argv[4]), atoi(argv[5]), atoi(argv[6]), atoi(argv[7]));
        break;

    case COMMAND_MISMATCH:
        print_usage_mbt_commands(FALSE);
        break;

    case COMMAND_READ_NAME:
        execute_read_name();
        break;

    case COMMAND_UPDATE_BAUDRATE:
        execute_update_baudrate(atoi(argv[2]));
        break;

    case COMMAND_READ_VERSION:
        execute_read_version();
        break;

    case COMMAND_LE_RX_TEST_V3:
        execute_le_receiver_test_v3(atoi(argv[2]), atoi(argv[3]));
        break;

    case COMMAND_LE_TX_TEST_V3:
        execute_le_transmitter_test_v3(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), atoi(argv[5]));
        break;
    case COMMAND_LE_ENHENCED_RX_TEST:
        execute_le_enhanced_receiver_test(atoi(argv[2]), atoi(argv[3]),atoi(argv[4]));
        break;
    case COMMAND_LE_ENHENCED_TX_TEST:
        execute_le_enhanced_transmitter_test(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), atoi(argv[5]));
        break;
    case COMMAND_GET_AGC_INDEX:
        execute_get_agc_index();
        break;
    case COMMAND_DOWNLOAD_MINIDRIVER:
	execute_download_minidriver();
	break;
    case COMMAND_HELP:
        break;
    case COMMAND_ARG_MISMATCH:
        break;
    case COMMAND_DOWNLOAD_WITH_MINIDRIVER:
        execute_download_with_minidriver(argv[2]);
        break;
    case COMMAND_DOWNLOAD_AUTOBAUD_3M:
        execute_download_auto_threeM(argv[2]);
        break;
    case COMMAND_DOWNLOAD_SOFT_3M:
        execute_download_threeM(argv[2]);
        break;

    default:
        break;
    }

    printf("\n");
    close(fd_transport);
    exit(EXIT_SUCCESS);
}

