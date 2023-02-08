/*
 * mbt.h
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


#ifndef MBTL_H
#define MBTL_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_MSC_VER)
#include "tchar.h"
#endif

/* data types */
#ifndef NULL
#define NULL     0
#endif

#ifndef FALSE
#define FALSE  0
#endif
#ifndef TRUE
#define TRUE   (!FALSE)
#endif

typedef unsigned char   UINT8;
typedef unsigned char   UBYTE;
typedef unsigned short  UINT16;
typedef unsigned int    UINT32, *PUINT32;
typedef signed int    INT32, *PINT32;
typedef signed char   INT8;
typedef signed short  INT16;
typedef UINT32          TIME_STAMP;
typedef unsigned char   BOOL;


#define MBT_VERSION_STRING "ver 1.32.0 (MBT for Linux)"
#define MBT_MAX_TX_POWER 3 // 3dBm max
#define CRTSCTS 020000000000 /*flow control*/
#define COMPARE_STRING(x,y)     strcmp(x,y)

/* transport types */
enum
{
    TRANSPORT_TYPE_UART,
    TRANSPORT_TYPE_USB
};

/* error codes */
enum
{
    MBT_SUCCESS,
    MBT_TRANSPORT_ERROR,
    MBT_ERROR
};

/* commands */
enum
{
    COMMAND_HELP,
    COMMAND_RESET,
    COMMAND_LOOPBACK_MODE,
    COMMAND_DOWNLOAD,
    COMMAND_DOWNLOAD_NO_RESET,
    COMMAND_DOWNLOAD_AUTOBAUD,
    COMMAND_WRITE_BDADDR,
    COMMAND_READ_BDADDR,
    COMMAND_LE_TEST_RECEIVER,
    COMMAND_LE_TEST_END,
    COMMAND_LE_TEST_TRANSMITTER,
    COMMAND_SET_TX_FREQUENCY_ARM,
    COMMAND_RECEIVE_ONLY,
    COMMAND_TX_TEST,
    COMMAND_RX_TEST,
    COMMAND_INPUT_COMMAND,
    COMMAND_READ_NAME,
    COMMAND_UPDATE_BAUDRATE,
    COMMAND_READ_VERSION,
    COMMAND_LE_RX_TEST_V3,
    COMMAND_LE_TX_TEST_V3,
    COMMAND_LE_ENHENCED_RX_TEST,
    COMMAND_LE_ENHENCED_TX_TEST,
    COMMAND_GET_AGC_INDEX,
    COMMAND_OTP_WRITE_WORD,
    COMMAND_DOWNLOAD_MINIDRIVER,
    COMMAND_MISMATCH = 80,
    COMMAND_DOWNLOAD_WITH_MINIDRIVER,
    COMMAND_DOWNLOAD_AUTOBAUD_3M,
    COMMAND_DOWNLOAD_SOFT_3M,
    COMMAND_ARG_MISMATCH,
};

#ifdef __cplusplus
}
#endif

#endif  //MBTL_H

