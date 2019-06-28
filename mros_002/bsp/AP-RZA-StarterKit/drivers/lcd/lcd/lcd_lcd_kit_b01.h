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
* Copyright (C) 2014 Renesas Electronics Corporation. All rights reserved.
*******************************************************************************/
/**************************************************************************//**
* @file         lcd_lcd_kit_b01.h
* @version      1.00
* $Rev: 199 $
* $Date:: 2014-05-23 16:33:52 +0900#$
* @brief        LCD panel definition header 
******************************************************************************/

#ifndef LCD_LCD_KIT_B01_H
#define LCD_LCD_KIT_B01_H

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include    <stdlib.h>

#include    "r_typedefs.h"


/******************************************************************************
Macro definitions
******************************************************************************/
#define     LCD_KIT_RIIC_SLAVE_ADDR_R8C     ((uint8_t)0x84u)    /*!< RIIC slave address of control CPU (R8C) */
#define     LCD_KIT_RIIC_RW_BIT_WRITE       ((uint8_t)0x00u)    /*!< RIIC R/W bit low: Write */
#define     LCD_KIT_RIIC_RW_BIT_READ        ((uint8_t)0x01u)    /*!< RIIC R/W bit low: Read */
#define     LCD_KIT_B01_BACKLIGHT           ((uint8_t)100u)     /*!< Backlight luminance: 0 - 100 */


/******************************************************************************
Typedef definitions
******************************************************************************/
/*! LCD-KIT-B01 R8C Command */
typedef enum
{
    LCD_KIT_B01_R8C_CMD_VER = 0x00,     /* Version */
    LCD_KIT_B01_R8C_CMD_INT = 0x01,     /* Interrupt status */
    LCD_KIT_B01_R8C_CMD_MSK = 0x02,     /* Interrupt mask */
    LCD_KIT_B01_R8C_CMD_BL  = 0x03,     /* Backlight */
    LCD_KIT_B01_R8C_CMD_BZ  = 0x04,     /* Buzzer */
    LCD_KIT_B01_R8C_CMD_RST = 0x05      /* Reset */
} lcd_kit_b01_r8c_command_t;


#endif  /* LCD_LCD_KIT_B01_H */
