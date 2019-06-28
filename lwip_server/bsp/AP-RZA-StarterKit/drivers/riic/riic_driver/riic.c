/*******************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only
* intended for use with Renesas products. No other uses are authorized. This
* software is owned by Renesas Electronics Corporation and is protected under
* all applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT
* LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
* AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR
* ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE
* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software
* and to discontinue the availability of this software. By using this software,
* you agree to the additional terms and conditions found by accessing the
* following link:
* http://www.renesas.com/disclaimer
* Copyright (C) 2012 - 2013 Renesas Electronics Corporation. All rights reserved.
*******************************************************************************/
/*******************************************************************************
* File Name   : riic.c
* $Rev: 703 $
* $Date:: 2014-02-13 11:02:37 +0900#$
* Description : RIIC driver
*******************************************************************************/


/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include "r_typedefs.h"
#include "dev_drv.h"                /* Device Driver common header */
#include "devdrv_riic.h"            /* RIIC Driver header */
#include "iodefine.h"
#include "rza_io_regrw.h"
#include <syslog.h>
#ifdef DEBUG_RIIC
#define TOPPERS_SYSLOG(...) syslog(__VA_ARGS__)
#else
#define TOPPERS_SYSLOG(...) 
#endif

/******************************************************************************
Typedef definitions
******************************************************************************/
/* ==== Reception mode ==== */
typedef enum riic_rx_mode
{
    RIIC_RX_MODE_NORMAL,        /* Normal mode         */
    RIIC_RX_MODE_ACK,           /* ACK mode            */
    RIIC_RX_MODE_LOW_HOLD,      /* Low hold mode       */
    RIIC_RX_MODE_NACK,          /* NACK mode           */
    RIIC_RX_MODE_STOP           /* Stop condition mode */
} riic_rx_mode_t;

/******************************************************************************
Macro definitions
******************************************************************************/
/* ==== RIIC H/W specification ==== */
#define RIIC_CH_TOTAL           (4)     /* Number of RIIC channels */

/******************************************************************************
Imported global variables and functions (from other files)
******************************************************************************/


/******************************************************************************
Exported global variables and functions (to be accessed by other files)
******************************************************************************/


/******************************************************************************
Private global variables and functions
******************************************************************************/
static void    RIIC_Receive(uint32_t channel, riic_rx_mode_t mode, uint8_t * data);
static void    RIIC_ReceiveDummy(uint32_t channel);
static int32_t RIIC_Transmit(uint32_t channel, uint8_t data);
static void    RIIC_TransmitStart(uint32_t channel);
static void    RIIC_TransmitRestart(uint32_t channel);
static void    RIIC_TransmitStop(uint32_t channel);
static void    RIIC_DetectStop(uint32_t channel);
static void    RIIC_ClearNack(volatile struct st_riic * riic);
volatile struct st_riic * RIIC_GetRegAddr(uint32_t channel);

/******************************************************************************
* Function Name: R_RIIC_Init
* Description  : Initializes the RIIC specified by the argument channel by 
*              : using the user-defined function Userdef_RIICn_Init
*              : (n=0 to 3).
* Arguments    : uint32_t channel : RIIC channel (0 to 3)
* Return Value : DEVDRV_SUCCESS   : Success of RIIC initialization
*              : DEVDRV_ERROR     : Failure of RIIC initialization
******************************************************************************/
int32_t R_RIIC_Init(uint32_t channel)
{
TOPPERS_SYSLOG(LOG_INFO, "R_RIIC_Init(%d)_E",channel);
    /* ==== Argument check ==== */
    if (channel >= RIIC_CH_TOTAL)
    {
        return DEVDRV_ERROR;        /* Argument error */
    }

    /* ==== RIIC initialization ==== */
    switch (channel)
    {
        case DEVDRV_CH_0:
            Userdef_RIIC0_Init();
        break;
        case DEVDRV_CH_1:
            Userdef_RIIC1_Init();
        break;
        case DEVDRV_CH_2:
            Userdef_RIIC2_Init();
        break;
        case DEVDRV_CH_3:
            Userdef_RIIC3_Init();
        break;
        default:
            /* Do not reach here based on the assumption */
        break;
    }

TOPPERS_SYSLOG(LOG_INFO, "R_RIIC_Init(%d)_X",channel);
    return DEVDRV_SUCCESS;
}

/******************************************************************************
* Function Name: R_RIIC_SendCond
* Description  : Issues a request to generate the condition specified by the 
*              : argument mode to the RIIC of the channel specified by the 
*              : argument channel.
*              : When the start condition issuance and the restart condition
*              : issuance are requested, the information for the receive data 
*              : full notification, transmit data empty notification, and the 
*              : transmit end notification should be initialized by using the 
*              : user-defined functions (Userdef_RIICn_InitReceiveFull,
*              : Userdef_RIICn_InitTransmitEmpty, and Userdef_RIICn_InitTransmitEnd) (n=0 to 3).
*              : After requesting the stop condition issuance, check the stop 
*              : condition detection by using the API function R_RIIC_DetectStop.
* Arguments    : uint32_t channel : RIIC channel (0 to 3)
*              : uint32_t mode    : Condition mode to be issued
*              :                  :   RIIC_TX_MODE_START   : Start condition issuance
*              :                  :   RIIC_TX_MODE_RESTART : Restart condition issuance
*              :                  :   RIIC_TX_MODE_STOP    : Stop condition issuance
* Return Value : DEVDRV_SUCCESS : Success of RIIC condition issuance request
*              : DEVDRV_ERROR   : Failure of RIIC condition issuance request
******************************************************************************/
int32_t R_RIIC_SendCond(uint32_t channel, uint32_t mode)
{
TOPPERS_SYSLOG(LOG_INFO, "R_RIIC_SendCond(%d)_E",channel);
    /* ==== Argument check ==== */
    if (channel >= RIIC_CH_TOTAL || mode > RIIC_TX_MODE_STOP)
    {
        return DEVDRV_ERROR;        /* Argument error */
    }

    /* ==== Condition issuance request ==== */
    switch (mode)
    {
        case RIIC_TX_MODE_START:
            RIIC_TransmitStart(channel);    /* Start condition issuance request   */
        break;
        case RIIC_TX_MODE_RESTART:
            RIIC_TransmitRestart(channel);  /* Restart condition issuance request */
        break;
        case RIIC_TX_MODE_STOP:
            RIIC_TransmitStop(channel);     /* Stop condition issuance request    */
        break;
        default:
            /* Do not reach here based on the assumption */
        break;
    }

TOPPERS_SYSLOG(LOG_INFO, "R_RIIC_SendCond(%d)_X",channel);
    return DEVDRV_SUCCESS;
}

/******************************************************************************
* Function Name: R_RIIC_WriteSlaveAddr
* Description  : Transmits the slave address (1 byte) specified by the argument
*              : addr using the RIIC master transmission operation of the 
*              : channel specified by the argument channel.
*              : When transmitting the slave address with the
*              : the R/W# bit "0" (write operation) and transmitting the one with 
*              : "1" (read operation), RIIC_TEND_WAIT_TRANSMIT and 
*              : RIIC_TEND_WAIT_RECEIVE should be specified as the argument mode
*              : respectively.
*              : First of all, checks the NACKF bit of ther RIICnSR2 register (n=0 to 3).
*              : If no slave device is recognized by the NACKF bit, 
*              : DEVDRV_ERROR_RIIC_NACK is returned.
*              : Waits until the conditions for the transmit data empty notification 
*              : information are satisfied by using the user-defined function 
*              : Userdef_RIICn_WaitTransmitEmpty. After initializing the transmit
*              : data empty notification information using the user-defined 
*              : function Userdef_RIICn_InitTransmitEmpty (n=0 to 3), transmits 
*              : the salve address.
*              : Then, waits until the conditions for the transmit end notification
*              : information are satisfied and initializes the information using the
*              : user-defined function Userdef_RIICn_InitTransmitEnd (n=0 to 3).
*              : Finally, checks the ACK response with the NACKF bit. If the ACK
*              : response cannot be confirmed, DEVDRV_ERROR_RIIC_NACK is returned.
* Arguments    : uint32_t channel : RIIC channel (0 to 3)
*              : uint16_t addr    : Slave address to be transmitted
*              :                  :   bit7 to bit1  : Slave address
*              :                  :   bit0          : R/W# bit
*              :                  :                 : (Write when 0 is set, read when 1 is set.)
*              : uint32_t mode    : Mode after transmitting slave address
*              :                  :   RIIC_TEND_WAIT_TRANSMIT : Continue transmission mode
*              :                  :   RIIC_TEND_WAIT_RECEIVE  : Transit to reception mode
*              : uint32_t addrbit : Reserved (not in use)
* Return Value : DEVDRV_SUCCESS         : Success of RIIC transmission
*              : DEVDRV_ERROR           : Failure of RIIC transmission
*              : DEVDRV_ERROR_RIIC_NACK : Failure fo RIIC transmission due to 
*              :                        : NACK reception
******************************************************************************/
int32_t R_RIIC_WriteSlaveAddr(uint32_t channel, uint16_t addr, uint32_t mode, uint32_t addrbit)
{
    volatile struct st_riic * riic;
    int32_t ret;
TOPPERS_SYSLOG(LOG_INFO, "R_RIIC_WriteSlaveAddr(%d)_E",channel);

    /* ==== Argument check ==== */
    if (channel >= RIIC_CH_TOTAL || mode > RIIC_TEND_WAIT_RECEIVE)
    {
        return DEVDRV_ERROR;        /* Argument error */
    }

    /* ==== 1-byte transmission ==== */
    ret = RIIC_Transmit(channel, (uint8_t)(addr & 0x00FFu));

    if (DEVDRV_SUCCESS == ret)
    {
        /* ==== Wait until the data transmission end conditions are satisfied, and clear them ==== */
        switch (channel)
        {
            case DEVDRV_CH_0:
                Userdef_RIIC0_WaitTransmitEnd(mode);
                Userdef_RIIC0_InitTransmitEnd(mode);
            break;
            case DEVDRV_CH_1:
                Userdef_RIIC1_WaitTransmitEnd(mode);
                Userdef_RIIC1_InitTransmitEnd(mode);
            break;
            case DEVDRV_CH_2:
                Userdef_RIIC2_WaitTransmitEnd(mode);
                Userdef_RIIC2_InitTransmitEnd(mode);
            break;
            case DEVDRV_CH_3:
                Userdef_RIIC3_WaitTransmitEnd(mode);
                Userdef_RIIC3_InitTransmitEnd(mode);
            break;
            default:
                /* Do not reach here based on the assumption */
            break;
        }

        /* ==== Check ACK write ==== */
        riic = RIIC_GetRegAddr(channel);
        /* ACK */
        if (0 == RZA_IO_RegRead_8(&(riic->RIICnSR2.UINT8[0]), RIICn_RIICnSR2_NACKF_SHIFT, RIICn_RIICnSR2_NACKF))
        {
            ret = DEVDRV_SUCCESS;
        }
        /* NACK */
        else
        {
            ret = DEVDRV_ERROR_RIIC_NACK;
        }
    }
TOPPERS_SYSLOG(LOG_INFO, "R_RIIC_WriteSlaveAddr(%d)_X",channel);

    return ret;
}

/******************************************************************************
* Function Name: R_RIIC_Write
* Description  : Transmits the data specified by the argument *buffer for the 
*              : bytes specified by the argument byte by using the RIIC master
*              : transmit operation of the channel specified by the argument
*              : channel. In this sample, this function is applied to transmit
*              : the memory address and the data to be written.
*              : After checking the NACK receive flag, waits until the conditions
*              : for the transmit data empty notification information are 
*              : satisfied by the user-defined function 
*              : Userdef_RIICn_WaitTransmitEmpty. After initializing the information 
*              : by using the user-defined function Userdef_RIICn_InitTransmitEmpty
*              : (n=0 to 3), transmits the data. This transmit operation repeated 
*              : until the specified byte counts are reached.
*              : The data transmission is interrupted when the NACK is received,
*              : and DEVDRV_ERROR_RIIC_NACK is returned.
*              : After the specified data has been transmitted, waits until the 
*              : conditions for the transmit end notification information are
*              : satisfied by the user-defined function 
*              : Userdef_RIICn_WaitTransmitEnd. Then initializes the information
*              : by using the user-defined function Userdef_RIICn_InitTransmitEnd
*              : (n=0 to 3).
*              : Finally, checks the NACK receive flag. This function returns 
*              : DEVDRV_ERROR_RIIC_NACK when the NACK is received.
* Arguments    : uint32_t channel : RIIC channel (0 to 3)
*              : uint8_t * buffer : Transmit data
*              : uint32_t byte    : Byte counts of transmit data
* Return Value : DEVDRV_SUCCESS         : Success of RIIC transmission
*              : DEVDRV_ERROR           : Failure of RIIC transmission
*              : DEVDRV_ERROR_RIIC_NACK : Failure of RIIC transmission due to 
*              :                        : NACK reception
******************************************************************************/
int32_t R_RIIC_Write(uint32_t channel, const uint8_t * buffer, uint32_t byte)
{
    volatile struct st_riic * riic;
    uint32_t offset;
    int32_t  ret;
TOPPERS_SYSLOG(LOG_INFO, "R_RIIC_Write(%d)_E",channel);

    /* ==== Argument check ==== */
    if (channel >= RIIC_CH_TOTAL)
    {
        return DEVDRV_ERROR;        /* Argument error */
    }

    if (0 == byte)
    {
        return DEVDRV_SUCCESS;      /* Do nothing in this function */
    }

    /* ==== Initialization ==== */
    ret = DEVDRV_SUCCESS;

    /* ==== RIIC transmission ==== */
    for (offset = 0; offset < byte; offset++)
    {
        /* ---- 1-byte transmission ---- */
        ret = RIIC_Transmit(channel, *(buffer + offset));

        /* ---- No ACK from the device ---- */
        if (ret != DEVDRV_SUCCESS)
        {
            break;
        }
    }

    /* ==== Transmission confirmation ==== */
    if (DEVDRV_SUCCESS == ret)
    {
        /* ---- Wait until the data transmission end conditions are satisfied, and clear them ---- */
        switch (channel)
        {
            case DEVDRV_CH_0:
                Userdef_RIIC0_WaitTransmitEnd(RIIC_TEND_WAIT_TRANSMIT);
                Userdef_RIIC0_InitTransmitEnd(RIIC_TEND_WAIT_TRANSMIT);
            break;
            case DEVDRV_CH_1:
                Userdef_RIIC1_WaitTransmitEnd(RIIC_TEND_WAIT_TRANSMIT);
                Userdef_RIIC1_InitTransmitEnd(RIIC_TEND_WAIT_TRANSMIT);
            break;
            case DEVDRV_CH_2:
                Userdef_RIIC2_WaitTransmitEnd(RIIC_TEND_WAIT_TRANSMIT);
                Userdef_RIIC2_InitTransmitEnd(RIIC_TEND_WAIT_TRANSMIT);
            break;
            case DEVDRV_CH_3:
                Userdef_RIIC3_WaitTransmitEnd(RIIC_TEND_WAIT_TRANSMIT);
                Userdef_RIIC3_InitTransmitEnd(RIIC_TEND_WAIT_TRANSMIT);
            break;
            default:
                /* Do not reach here based on the assumption */
            break;
        }

        /* ==== Check ACK for the last transmission ==== */
        riic = RIIC_GetRegAddr(channel);
        /* ACK */
        if (0 == RZA_IO_RegRead_8(&(riic->RIICnSR2.UINT8[0]), RIICn_RIICnSR2_NACKF_SHIFT, RIICn_RIICnSR2_NACKF))
        {
            ret = DEVDRV_SUCCESS;
        }
        /* NACK */
        else
        {
            ret = DEVDRV_ERROR_RIIC_NACK;
        }
    }

TOPPERS_SYSLOG(LOG_INFO, "R_RIIC_Write(%d)_X",channel);
    return ret;
}

/******************************************************************************
* Function Name: R_RIIC_Read
* Description  : Receives the data for the bytes specified by the argument
*              : byte by using the RIIC master receive operation of the channel
*              : specified by the argument channel, and stores the data to the
*              : area specified by the argument buffer.
*              : Waits until the conditions for the receive data full notification
*              : information are satisfied by the user-defined function 
*              : Userdef_RIICn_WaitReceiveFull. After initializing the information
*              : by Userdef_RIICn_InitReceiveFull (n=0 to 3), the data is received.
*              : When the data at the second byte from last is received, 1 is set
*              : to the WAIT bit of the I2C bus mode register 3 (RIICnMR3). This 
*              : holds the period between the ninth clock and the first clock of 
*              : the SCL at the low level until the I2C bus receive data register
*              : (RIICnDRR) is read when the last byte is received, so the state 
*              : is such that issuing a stop condition is possible. 
*              : When the data at the byte before last is received, 1 is set to
*              : the ACKBT bit of the I2C bus mode register 3 (RIICnMR3) and 1
*              : is sent to the acknowledge bit in reception of the last byte to 
*              : make the NACK response.
*              : When the last byte is received, the stop condition issuance is 
*              : required.
*              : Finally, checks the stop condition detection using the user-defined
*              : function Userdef_RIICn_WaitStop (n=0 to 3) to terminate this 
*              : function.
* Arguments    : uint32_t channel : RIIC channel (0 to 3)
*              : uint8_t * buffer : Receive data
*              : uint32_t byte    : Byte counts of receive data
* Return Value : DEVDRV_SUCCESS   : Success of RIIC reception
*              : DEVDRV_ERROR     : Failure of RIIC reception
******************************************************************************/
int32_t R_RIIC_Read(uint32_t channel, uint8_t * buffer, uint32_t byte)
{
    volatile struct st_riic * riic;
    uint32_t  cnt;
    uint8_t * buffer_ptr;
TOPPERS_SYSLOG(LOG_INFO, "R_RIIC_Read(%d)_E",channel);

    /* ==== Argument check ==== */
    if (channel >= RIIC_CH_TOTAL)
    {
        return DEVDRV_ERROR;        /* Argument error */
    }

    if (0 == byte)
    {
        return DEVDRV_SUCCESS;      /* Do nothing in this function */
    }

    /* ==== Initialization ==== */
    buffer_ptr = buffer;
    riic = RIIC_GetRegAddr(channel);

    if (1 == byte)
    {
        /* The period between ninth clock cycle and first clock cycle is held low. */
        RZA_IO_RegWrite_8(&(riic->RIICnMR3.UINT8[0]), 1, RIICn_RIICnMR3_WAIT_SHIFT, RIICn_RIICnMR3_WAIT);

        /* 1 is sent as the acknowledge bit */
        RZA_IO_RegWrite_8(&(riic->RIICnMR3.UINT8[0]), 1, RIICn_RIICnMR3_ACKWP_SHIFT, RIICn_RIICnMR3_ACKWP);
        RZA_IO_RegWrite_8(&(riic->RIICnMR3.UINT8[0]), 1, RIICn_RIICnMR3_ACKBT_SHIFT, RIICn_RIICnMR3_ACKBT);
    }

    /* ==== Dummy read ==== */
    RIIC_ReceiveDummy(channel);

    /* ==== Start reception (only when byte=1) ==== */
    if (1 == byte)
    {
        /* Do Nothing */
    }

    /* ==== Start reception (only when byte=2) ==== */
    else if (2 == byte)
    {
        /* ---- Read the first byte ---- */
        RIIC_Receive(channel, RIIC_RX_MODE_NACK, buffer_ptr);
        buffer_ptr++;
    }

    /* ==== Start reception (when byte=3 or more) ==== */
    else
    {
        /* ==== Read when the remaining byte counts is larger than 3 ==== */
        cnt = byte;
        while (cnt > 3)
        {
            /* ---- Read the first byte to the third byte from last ---- */
            RIIC_Receive(channel, RIIC_RX_MODE_NORMAL, buffer_ptr);
            buffer_ptr++;
            cnt--;
        }

        /* ---- Read the second byte from last ---- */
        RIIC_Receive(channel, RIIC_RX_MODE_LOW_HOLD, buffer_ptr);
        buffer_ptr++;

        /* ---- Read the byte before last ---- */
        RIIC_Receive(channel, RIIC_RX_MODE_NACK, buffer_ptr);
        buffer_ptr++;
    }

    /* ==== Read the last byte === */
    RIIC_Receive(channel, RIIC_RX_MODE_STOP, buffer_ptr);

    /* ==== Processing for the next transfer operation ==== */
    RZA_IO_RegWrite_8(&(riic->RIICnMR3.UINT8[0]), 0, RIICn_RIICnMR3_WAIT_SHIFT, RIICn_RIICnMR3_WAIT);

    /* ==== Check stop condition detection ==== */
    RIIC_DetectStop(channel);

TOPPERS_SYSLOG(LOG_INFO, "R_RIIC_Read(%d)_X",channel);
    return DEVDRV_SUCCESS;
}

/******************************************************************************
* Function Name: R_RIIC_ReadDummy
* Description  : Dummy-reads the RIIC receive data register RIICnDRR (n=0 to 3)
*              : of the channel specified by the argument channel and outputs 
*              : the clock to the I2C bus to start the reception operation.
* Arguments    : uint32_t channel : RIIC channel (0 to 3)
* Return Value : DEVDRV_SUCCESS   : Success of RIIC reception
*              : DEVDRV_ERROR     : Failure of RIIC reception
******************************************************************************/
int32_t R_RIIC_ReadDummy(uint32_t channel)
{
TOPPERS_SYSLOG(LOG_INFO, "R_RIIC_ReadDummy(%d)_E",channel);
    /* ==== Argument check ==== */
    if (channel >= RIIC_CH_TOTAL)
    {
        return DEVDRV_ERROR;        /* Argument error */
    }

    /* ==== Receive 1-byte data ==== */
    RIIC_ReceiveDummy(channel);

TOPPERS_SYSLOG(LOG_INFO, "R_RIIC_ReadDummy(%d)_X",channel);
    return DEVDRV_SUCCESS;
}

/******************************************************************************
* Function Name: R_RIIC_DetectStop
* Description  : Checks the RIIC stop condition detection of the channel 
*              : specified by the argument channel.
*              : Waits until the stop condition is detected by using the user-defined 
*              : function Userdef_RIICn_WaitStop (n=0 to 3).
*              : Clears the NACK receive flag (NACKF) and the stop condition 
*              : detection flag (STOP) for the next transfer operation.
* Arguments    : uint32_t channel : RIIC channel (0 to 3)
* Return Value : DEVDRV_SUCCESS   : Success in checking RIIC stop condition detection
*              : DEVDRV_ERROR     : Failure in checking RIIC stop condition detection
******************************************************************************/
int32_t R_RIIC_DetectStop(uint32_t channel)
{
TOPPERS_SYSLOG(LOG_INFO, "R_RIIC_DetectStop(%d)_E",channel);
    /* ==== Argument check ==== */
    if (channel >= RIIC_CH_TOTAL)
    {
        return DEVDRV_ERROR;        /* Argument error */
    }

    /* ==== Check stop condition detection ==== */
    RIIC_DetectStop(channel);

TOPPERS_SYSLOG(LOG_INFO, "R_RIIC_DetectStop(%d)_X",channel);
    return DEVDRV_SUCCESS;
}

/******************************************************************************
* Function Name: R_RIIC_ClearNack
* Description  : Clears the RIIC NACK receive flag (NACKF) of the channel 
*              : specified by the argument channel.
*              : In the sample code, the NACKF is cleared when the NACK is 
*              : received within the acknowledge polling processing. This function 
*              : is called when the polling processing is continued.
* Arguments    : uint32_t channel : RIIC channel (0 to 3)
* Return Value : DEVDRV_SUCCESS   : Success in clearing RIIC NACK reception flag
*              : DEVDRV_ERROR     : Failure in clearing RIIC NACK reception flag
******************************************************************************/
int32_t R_RIIC_ClearNack(uint32_t channel)
{
    volatile struct st_riic * riic;
TOPPERS_SYSLOG(LOG_INFO, "R_RIIC_ClearNack(%d)_E",channel);

    /* ==== Argument check ==== */
    if (channel >= RIIC_CH_TOTAL)
    {
        return DEVDRV_ERROR;        /* Argument error */
    }

    /* ==== Clear NACK flag ==== */
    riic = RIIC_GetRegAddr(channel);
    RIIC_ClearNack(riic);

TOPPERS_SYSLOG(LOG_INFO, "R_RIIC_ClearNack(%d)_X",channel);
    return DEVDRV_SUCCESS;
}

/******************************************************************************
* Function Name: R_RIIC_RiInterrupt
* Description  : Executes the processing to satisfy the conditions for the RIIC
*              : receive data full notification information of the channel 
*              : specified by the argument channel.
*              : The interrupt issuance can be notified by calling this function
*              : from the RIIC receive data full interrupt handler processing.
*              : The processing to satisfy the conditions for the receive data
*              : full notification information should be executed by using the
*              : user-defined function Userdef_RIICn_SetReceiveFull (n=0 to 3).
* Arguments    : uint32_t channel : RIIC channel (0 to 3)
* Return Value : DEVDRV_SUCCESS   : Success of RIIC receive data full interrupt 
*              :                  : notification
*              : DEVDRV_ERROR     : Failure of RIIC receive data full interrupt 
*              :                  : notification
******************************************************************************/
int32_t R_RIIC_RiInterrupt(uint32_t channel)
{
TOPPERS_SYSLOG(LOG_INFO, "R_RIIC_RiInterrupt(%d)_E",channel);
    /* ==== Argument check ==== */
    if (channel >= RIIC_CH_TOTAL)
    {
        return DEVDRV_ERROR;        /* Argument error */
    }

    /* ==== Set receive data full conditions ==== */
    switch (channel)
    {
        case DEVDRV_CH_0:
            Userdef_RIIC0_SetReceiveFull();
        break;
        case DEVDRV_CH_1:
            Userdef_RIIC1_SetReceiveFull();
        break;
        case DEVDRV_CH_2:
            Userdef_RIIC2_SetReceiveFull();
        break;
        case DEVDRV_CH_3:
            Userdef_RIIC3_SetReceiveFull();
        break;
        default:
            /* Do not reach here based on the assumption */
        break;
    }

TOPPERS_SYSLOG(LOG_INFO, "R_RIIC_RiInterrupt(%d)_X",channel);
    return DEVDRV_SUCCESS;
}

/******************************************************************************
* Function Name: R_RIIC_TiInterrupt
* Description  : Executes the processing to satisfy the conditions for the RIIC
*              : transmit data empty notification information of the channel 
*              : specified by the argument channel.
*              : The interrupt issuance can be notified by calling this function
*              : from the RIIC transmit data empty interrupt handler processing.
*              : The processing to satisfy the conditions of the transmit data 
*              : empty notification information should be executed by using the
*              : user-defined function Userdef_RIICn_SetTransmitEmpty (n=0 to 3).
* Arguments    : uint32_t channel : RIIC channel (0 to 3)
* Return Value : DEVDRV_SUCCESS   : Success of RIIC transmit data empty interrupt 
*              :                  : notification
*              : DEVDRV_ERROR     : Failure of RIIC transmit data empty interrupt 
*              :                  : notification
******************************************************************************/
int32_t R_RIIC_TiInterrupt(uint32_t channel)
{
TOPPERS_SYSLOG(LOG_INFO, "R_RIIC_TiInterrupt(%d)_E",channel);
    /* ==== Argument check ==== */
    if (channel >= RIIC_CH_TOTAL)
    {
        return DEVDRV_ERROR;        /* Argument error */
    }

    /* ==== Set transmit data empty conditions ==== */
    switch (channel)
    {
        case DEVDRV_CH_0:
            Userdef_RIIC0_SetTransmitEmpty();
        break;
        case DEVDRV_CH_1:
            Userdef_RIIC1_SetTransmitEmpty();
        break;
        case DEVDRV_CH_2:
            Userdef_RIIC2_SetTransmitEmpty();
        break;
        case DEVDRV_CH_3:
            Userdef_RIIC3_SetTransmitEmpty();
        break;
        default:
            /* Do not reach here based on the assumption */
        break;
    }

TOPPERS_SYSLOG(LOG_INFO, "R_RIIC_TiInterrupt(%d)_X",channel);
    return DEVDRV_SUCCESS;
}

/******************************************************************************
* Function Name: R_RIIC_TeiInterrupt
* Description  : Executes the processing to satisfy the conditions for the RIIC
*              : transmit end notification information of the channel specified
*              : by the argument channel, and clears the interrupt source flag
*              : (TEND bit).
*              : The interrupt issuance can be notified by calling this function
*              : from the RIIC transmit end interrupt handler processing.
*              : The processing to satisfy the conditions for the transmit end 
*              : notification information should be executed by using the user-defined 
*              : function Userdef_RIICn_SetTransmitEnd (n=0 to 3).
* Arguments    : uint32_t channel : RIIC channel (0 to 3)
* Return Value : DEVDRV_SUCCESS   : Success of RIIC transmission completion 
*              :                  : interrupt notification
*              : DEVDRV_ERROR     : Failure of RIIC transmission completion 
*              :                  : interrupt notification
******************************************************************************/
int32_t R_RIIC_TeiInterrupt(uint32_t channel)
{
    volatile struct  st_riic * riic;
    volatile uint8_t dummy_buf_8b;

TOPPERS_SYSLOG(LOG_INFO, "R_RIIC_TeiInterrupt(%d)_E",channel);
    /* ==== Argument check ==== */
    if (channel >= RIIC_CH_TOTAL)
    {
        return DEVDRV_ERROR;        /* Argument error */
    }

    /* ==== Set transfer end conditions ==== */
    switch (channel)
    {
        case DEVDRV_CH_0:
            Userdef_RIIC0_SetTransmitEnd();
        break;
        case DEVDRV_CH_1:
            Userdef_RIIC1_SetTransmitEnd();
        break;
        case DEVDRV_CH_2:
            Userdef_RIIC2_SetTransmitEnd();
        break;
        case DEVDRV_CH_3:
            Userdef_RIIC3_SetTransmitEnd();
        break;
        default:
            /* Do not reach here based on the assumption */
        break;
    }

    /* ==== Clear transmit end flag ==== */
    riic = RIIC_GetRegAddr(channel);
    RZA_IO_RegWrite_8(&(riic->RIICnSR2.UINT8[0]), 0, RIICn_RIICnSR2_TEND_SHIFT, RIICn_RIICnSR2_TEND);
    dummy_buf_8b = RZA_IO_RegRead_8(&(riic->RIICnSR2.UINT8[0]), RIICn_RIICnSR2_TEND_SHIFT, RIICn_RIICnSR2_TEND);

TOPPERS_SYSLOG(LOG_INFO, "R_RIIC_TeiInterrupt(%d)_X",channel);
    return DEVDRV_SUCCESS;
}

/******************************************************************************
* Function Name: RIIC_Receive
* Description  : Executes the RIIC reception for 1 byte.
* Arguments    : uint32_t channel    : RIIC channel (0 to 3)
*              : riic_rx_mode_t mode : Reception mode
*              :                     :   RIIC_RX_MODE_NORMAL   : Normal mode
*              :                     :   RIIC_RX_MODE_ACK      : ACK mode
*              :                     :   RIIC_RX_MODE_LOW_HOLD : Low hold mode
*              :                     :   RIIC_RX_MODE_NACK     : NACK mode
*              :                     :   RIIC_RX_MODE_STOP     : Stop condition mode
*              : uint8_t * data      : Receive data
* Return Value : none
******************************************************************************/
static void RIIC_Receive(uint32_t channel, riic_rx_mode_t mode, uint8_t * data)
{
    volatile struct st_riic * riic;
TOPPERS_SYSLOG(LOG_INFO, "RIIC_Receive(%d)_E",channel);

    /* ==== Wait until receive data full conditions are satisfied, and clear them ==== */
    switch (channel)
    {
        case DEVDRV_CH_0:
            Userdef_RIIC0_WaitReceiveFull();
            Userdef_RIIC0_InitReceiveFull();
        break;
        case DEVDRV_CH_1:
            Userdef_RIIC1_WaitReceiveFull();
            Userdef_RIIC1_InitReceiveFull();
        break;
        case DEVDRV_CH_2:
            Userdef_RIIC2_WaitReceiveFull();
            Userdef_RIIC2_InitReceiveFull();
        break;
        case DEVDRV_CH_3:
            Userdef_RIIC3_WaitReceiveFull();
            Userdef_RIIC3_InitReceiveFull();
        break;
        default:
            /* Do not reach here based on the assumption */
        break;
    }

    riic = RIIC_GetRegAddr(channel);

    switch (mode)
    {
        /* ---- ACK mode ---- */
        case RIIC_RX_MODE_ACK:
            RZA_IO_RegWrite_8(&(riic->RIICnMR3.UINT8[0]), 1, RIICn_RIICnMR3_ACKWP_SHIFT, RIICn_RIICnMR3_ACKWP);
            /* 0 is sent as the acknowledge bit */
            RZA_IO_RegWrite_8(&(riic->RIICnMR3.UINT8[0]), 0, RIICn_RIICnMR3_ACKBT_SHIFT, RIICn_RIICnMR3_ACKBT);
        break;
        /* ---- Low hold mode ---- */
        case RIIC_RX_MODE_LOW_HOLD:
            /* The period between ninth clock cycle and first clock cycle is held low */
            RZA_IO_RegWrite_8(&(riic->RIICnMR3.UINT8[0]), 1, RIICn_RIICnMR3_WAIT_SHIFT, RIICn_RIICnMR3_WAIT);
        break;
        /* ---- NACK mode ---- */
        case RIIC_RX_MODE_NACK:
            RZA_IO_RegWrite_8(&(riic->RIICnMR3.UINT8[0]), 1, RIICn_RIICnMR3_ACKWP_SHIFT, RIICn_RIICnMR3_ACKWP);
            /* 1 is sent as the acknowledge bit */
            RZA_IO_RegWrite_8(&(riic->RIICnMR3.UINT8[0]), 1, RIICn_RIICnMR3_ACKBT_SHIFT, RIICn_RIICnMR3_ACKBT);
        break;
        /* ---- Stop condition mode ---- */
        case RIIC_RX_MODE_STOP:
            RIIC_TransmitStop(channel);     /* Stop condition issuance request */
        break;
        /* ---- Normal mode ---- */
        case RIIC_RX_MODE_NORMAL:
            /* Do Nothing */
        break;
        default:
            /* Do not reach here based on the assumption */
        break;
    }

    /* ==== Receive data ==== */
    *data = riic->RIICnDRR.UINT8[0];
TOPPERS_SYSLOG(LOG_INFO, "RIIC_Receive(%d)_X",channel);
}

/******************************************************************************
* Function Name: RIIC_ReceiveDummy
* Description  : Executes dummy read for 1 byte.
* Arguments    : uint32_t channel    : RIIC channel (0 to 3)
* Return Value : none
******************************************************************************/
static void RIIC_ReceiveDummy(uint32_t channel)
{
    volatile struct  st_riic * riic;
    volatile uint8_t dummy_buf_8b;
TOPPERS_SYSLOG(LOG_INFO, "RIIC_ReceiveDummy(%d)_E",channel);

    riic = RIIC_GetRegAddr(channel);

    /* ==== Receive 1-byte data ==== */
    dummy_buf_8b = riic->RIICnDRR.UINT8[0];
TOPPERS_SYSLOG(LOG_INFO, "RIIC_ReceiveDummy(%d)_X",channel);
}

/******************************************************************************
* Function Name: RIIC_Transmit
* Description  : Executes the RIIC transmission for 1 byte.
* Arguments    : uint32_t channel : RIIC channel (0 to 3)
*              : uint8_t data     : Transmit data
* Return Value : DEVDRV_SUCCESS         : Success of RIIC transmission
*              : DEVDRV_ERROR_RIIC_NACK : Failure of RIIC transmission due to 
*              :                        : NACK reception
******************************************************************************/
static int32_t RIIC_Transmit(uint32_t channel, uint8_t data)
{
    volatile struct st_riic * riic;
    int32_t ret;

TOPPERS_SYSLOG(LOG_INFO, "RIIC_Transmit(%d)_E",channel);
    riic = RIIC_GetRegAddr(channel);

    /* ==== Check ACK ==== */
    /* ACK */
    if (0 == RZA_IO_RegRead_8(&(riic->RIICnSR2.UINT8[0]), RIICn_RIICnSR2_NACKF_SHIFT, RIICn_RIICnSR2_NACKF))
    {
        ret = DEVDRV_SUCCESS;
    }
    /* NACK */
    else
    {
        ret = DEVDRV_ERROR_RIIC_NACK;
    }

    if (DEVDRV_SUCCESS == ret)
    {
        /* ==== Wait until transmit data empty conditions are satisfied, and clear them ==== */
        switch (channel)
        {
            case DEVDRV_CH_0:
                Userdef_RIIC0_WaitTransmitEmpty();
                Userdef_RIIC0_InitTransmitEmpty();
            break;
            case DEVDRV_CH_1:
                Userdef_RIIC1_WaitTransmitEmpty();
                Userdef_RIIC1_InitTransmitEmpty();
            break;
            case DEVDRV_CH_2:
                Userdef_RIIC2_WaitTransmitEmpty();
                Userdef_RIIC2_InitTransmitEmpty();
            break;
            case DEVDRV_CH_3:
                Userdef_RIIC3_WaitTransmitEmpty();
                Userdef_RIIC3_InitTransmitEmpty();
            break;
            default:
                /* Do not reach here based on the assumption */
            break;
        }

        /* ==== Transmit data ==== */
        riic->RIICnDRT.UINT8[0] = data;
    }

TOPPERS_SYSLOG(LOG_INFO, "RIIC_Transmit(%d)_X",channel);
    return ret;
}

/******************************************************************************
* Function Name: RIIC_TransmitStart
* Description  : Requests the start condition issuance.
* Arguments    : uint32_t channel : RIIC channel (0 to 3)
* Return Value : none
******************************************************************************/
static void RIIC_TransmitStart(uint32_t channel)
{
    volatile struct st_riic * riic;

TOPPERS_SYSLOG(LOG_INFO, "RIIC_TransmitStart(%d)_E",channel);
    /* ==== Wait for bus free ==== */
    /* ==== Initialize receive data full conditions, transfer end conditions, ==== */
    /* ==== and transmit data empty conditions                                ==== */
    switch (channel)
    {
        case DEVDRV_CH_0:
            Userdef_RIIC0_WaitBusMastership(RIIC_BUS_MASTERSHIP_WAIT_FREE);

            Userdef_RIIC0_InitReceiveFull();
            Userdef_RIIC0_InitTransmitEmpty();
            Userdef_RIIC0_InitTransmitEnd(RIIC_TEND_WAIT_TRANSMIT);
        break;
        case DEVDRV_CH_1:
            Userdef_RIIC1_WaitBusMastership(RIIC_BUS_MASTERSHIP_WAIT_FREE);

            Userdef_RIIC1_InitReceiveFull();
            Userdef_RIIC1_InitTransmitEmpty();
            Userdef_RIIC1_InitTransmitEnd(RIIC_TEND_WAIT_TRANSMIT);
        break;
        case DEVDRV_CH_2:
            Userdef_RIIC2_WaitBusMastership(RIIC_BUS_MASTERSHIP_WAIT_FREE);

            Userdef_RIIC2_InitReceiveFull();
            Userdef_RIIC2_InitTransmitEmpty();
            Userdef_RIIC2_InitTransmitEnd(RIIC_TEND_WAIT_TRANSMIT);
        break;
        case DEVDRV_CH_3:
            Userdef_RIIC3_WaitBusMastership(RIIC_BUS_MASTERSHIP_WAIT_FREE);

            Userdef_RIIC3_InitReceiveFull();
            Userdef_RIIC3_InitTransmitEmpty();
            Userdef_RIIC3_InitTransmitEnd(RIIC_TEND_WAIT_TRANSMIT);
        break;
        default:
            /* Do not reach here based on the assumption */
        break;
    }

    /* ==== Start condition issuance request ==== */
    riic = RIIC_GetRegAddr(channel);
    RZA_IO_RegWrite_8(&(riic->RIICnCR2.UINT8[0]), 1, RIICn_RIICnCR2_ST_SHIFT, RIICn_RIICnCR2_ST);
TOPPERS_SYSLOG(LOG_INFO, "RIIC_TransmitStart(%d)_X",channel);
}

/******************************************************************************
* Function Name: RIIC_TransmitRestart
* Description  : Requests the restart condition issuance.
* Arguments    : uint32_t channel : RIIC channel (0 to 3)
* Return Value : none
******************************************************************************/
static void RIIC_TransmitRestart(uint32_t channel)
{
    volatile struct st_riic * riic;
TOPPERS_SYSLOG(LOG_INFO, "RIIC_TransmitRestart(%d)_E",channel);

    /* ==== Wait for bus busy ==== */
    /* ==== Initialze receive data full conditions, transmit end conditions, ==== */
    /* ==== and transmit data empty conditions                               ==== */
    switch (channel)
    {
        case DEVDRV_CH_0:
            Userdef_RIIC0_WaitBusMastership(RIIC_BUS_MASTERSHIP_WAIT_BUSY);

            Userdef_RIIC0_InitReceiveFull();
            Userdef_RIIC0_InitTransmitEmpty();
            Userdef_RIIC0_InitTransmitEnd(RIIC_TEND_WAIT_TRANSMIT);
        break;
        case DEVDRV_CH_1:
            Userdef_RIIC1_WaitBusMastership(RIIC_BUS_MASTERSHIP_WAIT_BUSY);

            Userdef_RIIC1_InitReceiveFull();
            Userdef_RIIC1_InitTransmitEmpty();
            Userdef_RIIC1_InitTransmitEnd(RIIC_TEND_WAIT_TRANSMIT);
        break;
        case DEVDRV_CH_2:
            Userdef_RIIC2_WaitBusMastership(RIIC_BUS_MASTERSHIP_WAIT_BUSY);

            Userdef_RIIC2_InitReceiveFull();
            Userdef_RIIC2_InitTransmitEmpty();
            Userdef_RIIC2_InitTransmitEnd(RIIC_TEND_WAIT_TRANSMIT);
        break;
        case DEVDRV_CH_3:
            Userdef_RIIC3_WaitBusMastership(RIIC_BUS_MASTERSHIP_WAIT_BUSY);

            Userdef_RIIC3_InitReceiveFull();
            Userdef_RIIC3_InitTransmitEmpty();
            Userdef_RIIC3_InitTransmitEnd(RIIC_TEND_WAIT_TRANSMIT);
        break;
        default:
            /* Do not reach here based on the assumption */
        break;
    }

    /* ==== Restart condition issuance request ==== */
    riic = RIIC_GetRegAddr(channel);
    RZA_IO_RegWrite_8(&(riic->RIICnCR2.UINT8[0]), 1, RIICn_RIICnCR2_RS_SHIFT, RIICn_RIICnCR2_RS);
TOPPERS_SYSLOG(LOG_INFO, "RIIC_TransmitRestart(%d)_X",channel);
}

/******************************************************************************
* Function Name: RIIC_TransmitStop
* Description  : Requests the stop condition issuance.
* Arguments    : uint32_t channel : RIIC channel (0 to 3)
* Return Value : none
******************************************************************************/
static void RIIC_TransmitStop(uint32_t channel)
{
    volatile struct st_riic * riic;
TOPPERS_SYSLOG(LOG_INFO, "RIIC_TransmitStop(%d)_E",channel);

    riic = RIIC_GetRegAddr(channel);

    /* ==== Initialize stop condition detection flag ==== */
    RZA_IO_RegWrite_8(&(riic->RIICnSR2.UINT8[0]), 0, RIICn_RIICnSR2_STOP_SHIFT, RIICn_RIICnSR2_STOP);

    /* ==== Stop condition issuance request ==== */
    RZA_IO_RegWrite_8(&(riic->RIICnCR2.UINT8[0]), 1, RIICn_RIICnCR2_SP_SHIFT, RIICn_RIICnCR2_SP);
TOPPERS_SYSLOG(LOG_INFO, "RIIC_TransmitStop(%d)_X",channel);
}

/******************************************************************************
* Function Name: RIIC_DetectStop
* Description  : Waits for the stop condition detection.
* Arguments    : uint32_t channel : RIIC channel (0 to 3)
* Return Value : none
******************************************************************************/
static void RIIC_DetectStop(uint32_t channel)
{
    volatile struct st_riic * riic;
TOPPERS_SYSLOG(LOG_INFO, "RIIC_DetectStop(%d)_E",channel);

    /* ==== Wait for stop condition detection ==== */
    switch (channel)
    {
        case DEVDRV_CH_0:
            Userdef_RIIC0_WaitStop();
        break;
        case DEVDRV_CH_1:
            Userdef_RIIC1_WaitStop();
        break;
        case DEVDRV_CH_2:
            Userdef_RIIC2_WaitStop();
        break;
        case DEVDRV_CH_3:
            Userdef_RIIC3_WaitStop();
        break;
        default:
            /* Do not reach here based on the assumption */
        break;
    }

    riic = RIIC_GetRegAddr(channel);

    /* ==== Processing for the next transfer operation ==== */
    RIIC_ClearNack(riic);
    RZA_IO_RegWrite_8(&(riic->RIICnSR2.UINT8[0]), 0, RIICn_RIICnSR2_STOP_SHIFT, RIICn_RIICnSR2_STOP);
TOPPERS_SYSLOG(LOG_INFO, "RIIC_DetectStop(%d)_X",channel);
}

/******************************************************************************
* Function Name: RIIC_ClearNack
* Description  : Clears the NACK receive flag.
* Arguments    : struct st_riic * riic : Start address of RIIC register by channel
* Return Value : none
******************************************************************************/
static void RIIC_ClearNack(volatile struct st_riic * riic)
{
TOPPERS_SYSLOG(LOG_INFO, "RIIC_CelarNack()_E");
    RZA_IO_RegWrite_8(&(riic->RIICnSR2.UINT8[0]), 0, RIICn_RIICnSR2_NACKF_SHIFT, RIICn_RIICnSR2_NACKF);
TOPPERS_SYSLOG(LOG_INFO, "RIIC_CelarNack()_X");
}

/******************************************************************************
* Function Name: RIIC_GetRegAddr
* Description  : Obtains the start address of the RIIC-related register of the 
*              : specified channel.
* Arguments    : uint32_t channel : RIIC channel (0 to 3)
* Return Value : struct st_riic * : Start address of RIIC register by channel
******************************************************************************/
volatile struct st_riic * RIIC_GetRegAddr(uint32_t channel)
{
    volatile struct st_riic * riic;
TOPPERS_SYSLOG(LOG_INFO, "RIIC_GetRegAddr(%d)_E",channel);

    switch (channel)
    {
        case DEVDRV_CH_0:
            riic = &RIIC0;
        break;
        case DEVDRV_CH_1:
            riic = &RIIC1;
        break;
        case DEVDRV_CH_2:
            riic = &RIIC2;
        break;
        case DEVDRV_CH_3:
            riic = &RIIC3;
        break;
        default:
            /* Do not reach here based on the assumption */
            riic = &RIIC0;
        break;
    }

TOPPERS_SYSLOG(LOG_INFO, "RIIC_GetRegAddr(%d)_X",channel);
    return riic;
}


/* End of File */

