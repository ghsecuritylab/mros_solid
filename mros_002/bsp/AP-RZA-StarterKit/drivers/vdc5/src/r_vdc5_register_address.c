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
* Copyright (C) 2012 - 2014 Renesas Electronics Corporation. All rights reserved.
*******************************************************************************/
/**************************************************************************//**
* @file         r_vdc5_register_address.c
* @version      1.00
* $Rev: 199 $
* $Date:: 2014-05-23 16:33:52 +0900#$
* @brief        VDC5 driver register address table
******************************************************************************/

/******************************************************************************
Includes   <System Includes> , "Project Includes"
******************************************************************************/
#include    "r_vdc5.h"
#include    "r_vdc5_user.h"
#include    "r_vdc5_register.h"


/******************************************************************************
Macro definitions
******************************************************************************/
#define     VDC5_CH0_GR0_CLUT_TBL           (*(uint32_t*)0xFCFF6000u)
#define     VDC5_CH0_GR1_CLUT_TBL           (*(uint32_t*)0xFCFF6400u)
#define     VDC5_CH0_GR2_CLUT_TBL           (*(uint32_t*)0xFCFF6800u)
#define     VDC5_CH0_GR3_CLUT_TBL           (*(uint32_t*)0xFCFF6C00u)
#define     VDC5_CH0_GR_OIR_CLUT_TBL        (*(uint32_t*)0xFCFF7000u)
#define     VDC5_CH1_GR0_CLUT_TBL           (*(uint32_t*)0xFCFF8000u)
#define     VDC5_CH1_GR1_CLUT_TBL           (*(uint32_t*)0xFCFF8400u)
#define     VDC5_CH1_GR2_CLUT_TBL           (*(uint32_t*)0xFCFF8800u)
#define     VDC5_CH1_GR3_CLUT_TBL           (*(uint32_t*)0xFCFF8C00u)
#define     VDC5_CH1_GR_OIR_CLUT_TBL        (*(uint32_t*)0xFCFF9000u)


/******************************************************************************
Typedef definitions
******************************************************************************/

/** extern definitions **/
static void initialize_vdc5_regaddr_input_ctrl(void);
static void initialize_vdc5_regaddr_scaler(void);
static void initialize_vdc5_regaddr_img_qlty_imp(void);
static void initialize_vdc5_regaddr_color_matrix(void);
static void initialize_vdc5_regaddr_img_synthesizer(void);
static void initialize_vdc5_regaddr_output_ctrl(void);
static void initialize_vdc5_regaddr_gamma(void);
static void initialize_vdc5_regaddr_system_ctrl(void);
static void initialize_vdc5_regaddr_lvds(void);
static void initialize_vdc5_regaddr_clut(void);


void initialize_vdc5_regaddr_tables(void)
{
	initialize_vdc5_regaddr_input_ctrl();
	initialize_vdc5_regaddr_scaler();
	initialize_vdc5_regaddr_img_qlty_imp();
	initialize_vdc5_regaddr_color_matrix();
	initialize_vdc5_regaddr_img_synthesizer();
	initialize_vdc5_regaddr_output_ctrl();
	initialize_vdc5_regaddr_gamma();
	initialize_vdc5_regaddr_system_ctrl();
	initialize_vdc5_regaddr_lvds();
	initialize_vdc5_regaddr_clut();
}



/******************************************************************************
Exported global variables and functions (to be accessed by other files)
******************************************************************************/
/* VDC5 input controller register address list */
#if 0
const vdc5_regaddr_input_ctrl_t vdc5_regaddr_input_ctrl[VDC5_CHANNEL_NUM] =
{
    {   /* Channel 0 */
        &VDC50.INP_UPDATE,
        &VDC50.INP_SEL_CNT,
        &VDC50.INP_EXT_SYNC_CNT,
        &VDC50.INP_VSYNC_PH_ADJ,
        &VDC50.INP_DLY_ADJ,
        &VDC50.IMGCNT_UPDATE,
        &VDC50.IMGCNT_NR_CNT0,
        &VDC50.IMGCNT_NR_CNT1
    },
    {   /* Channel 1 */
        &VDC51.INP_UPDATE,
        &VDC51.INP_SEL_CNT,
        &VDC51.INP_EXT_SYNC_CNT,
        &VDC51.INP_VSYNC_PH_ADJ,
        &VDC51.INP_DLY_ADJ,
        &VDC51.IMGCNT_UPDATE,
        &VDC51.IMGCNT_NR_CNT0,
        &VDC51.IMGCNT_NR_CNT1
    }
};
#else
vdc5_regaddr_input_ctrl_t vdc5_regaddr_input_ctrl[VDC5_CHANNEL_NUM];

static void initialize_vdc5_regaddr_input_ctrl(void)
{
		vdc5_regaddr_input_ctrl[0].inp_update = &VDC50.INP_UPDATE;
		vdc5_regaddr_input_ctrl[0].inp_sel_cnt = &VDC50.INP_SEL_CNT;
		vdc5_regaddr_input_ctrl[0].inp_ext_sync_cnt= &VDC50.INP_EXT_SYNC_CNT;
		vdc5_regaddr_input_ctrl[0].inp_vsync_ph_adj = &VDC50.INP_VSYNC_PH_ADJ;
		vdc5_regaddr_input_ctrl[0].inp_dly_adj = &VDC50.INP_DLY_ADJ;
		vdc5_regaddr_input_ctrl[0].imgcnt_update = &VDC50.IMGCNT_UPDATE;
		vdc5_regaddr_input_ctrl[0].imgcnt_nr_cnt0 = &VDC50.IMGCNT_NR_CNT0;
		vdc5_regaddr_input_ctrl[0].imgcnt_nr_cnt1 = &VDC50.IMGCNT_NR_CNT1;

		vdc5_regaddr_input_ctrl[1].inp_update = &VDC51.INP_UPDATE;
		vdc5_regaddr_input_ctrl[1].inp_sel_cnt = &VDC51.INP_SEL_CNT;
		vdc5_regaddr_input_ctrl[1].inp_ext_sync_cnt = &VDC51.INP_EXT_SYNC_CNT;
		vdc5_regaddr_input_ctrl[1].inp_vsync_ph_adj = &VDC51.INP_VSYNC_PH_ADJ;
		vdc5_regaddr_input_ctrl[1].inp_dly_adj = &VDC51.INP_DLY_ADJ;
		vdc5_regaddr_input_ctrl[1].imgcnt_update = &VDC51.IMGCNT_UPDATE;
		vdc5_regaddr_input_ctrl[1].imgcnt_nr_cnt0 = &VDC51.IMGCNT_NR_CNT0;
		vdc5_regaddr_input_ctrl[1].imgcnt_nr_cnt1 = &VDC51.IMGCNT_NR_CNT1;
	}
#endif

/* VDC5 scaler register address list */
#if 0
const vdc5_regaddr_scaler_t vdc5_regaddr_scaler[VDC5_CHANNEL_NUM][VDC5_SC_TYPE_NUM] =
{
    {   /* Channel 0 */
        {   /* SC0 */
            &VDC50.SC0_SCL0_UPDATE,
            &VDC50.SC0_SCL0_FRC1,
            &VDC50.SC0_SCL0_FRC2,
            &VDC50.SC0_SCL0_FRC3,
            &VDC50.SC0_SCL0_FRC4,
            &VDC50.SC0_SCL0_FRC5,
            &VDC50.SC0_SCL0_FRC6,
            &VDC50.SC0_SCL0_FRC7,
            &VDC50.SC0_SCL0_FRC9,
            &VDC50.SC0_SCL0_MON0,
            &VDC50.SC0_SCL0_INT,
            &VDC50.SC0_SCL0_DS1,
            &VDC50.SC0_SCL0_DS2,
            &VDC50.SC0_SCL0_DS3,
            &VDC50.SC0_SCL0_DS4,
            &VDC50.SC0_SCL0_DS5,
            &VDC50.SC0_SCL0_DS6,
            &VDC50.SC0_SCL0_DS7,
            &VDC50.SC0_SCL0_US1,
            &VDC50.SC0_SCL0_US2,
            &VDC50.SC0_SCL0_US3,
            &VDC50.SC0_SCL0_US4,
            &VDC50.SC0_SCL0_US5,
            &VDC50.SC0_SCL0_US6,
            &VDC50.SC0_SCL0_US7,
            &VDC50.SC0_SCL0_US8,
            &VDC50.SC0_SCL0_OVR1,
            &VDC50.SC0_SCL1_UPDATE,
            &VDC50.SC0_SCL1_WR1,
            &VDC50.SC0_SCL1_WR2,
            &VDC50.SC0_SCL1_WR3,
            &VDC50.SC0_SCL1_WR4,
            &VDC50.SC0_SCL1_WR5,
            &VDC50.SC0_SCL1_WR6,
            &VDC50.SC0_SCL1_WR7,
            &VDC50.SC0_SCL1_WR8,
            &VDC50.SC0_SCL1_WR9,
            &VDC50.SC0_SCL1_WR10,
            &VDC50.SC0_SCL1_WR11,
            &VDC50.SC0_SCL1_MON1,
            &VDC50.SC0_SCL1_PBUF0,
            &VDC50.SC0_SCL1_PBUF1,
            &VDC50.SC0_SCL1_PBUF2,
            &VDC50.SC0_SCL1_PBUF3,
            &VDC50.SC0_SCL1_PBUF_FLD,
            &VDC50.SC0_SCL1_PBUF_CNT
        },
        {   /* SC1 */
            &VDC50.SC1_SCL0_UPDATE,
            &VDC50.SC1_SCL0_FRC1,
            &VDC50.SC1_SCL0_FRC2,
            &VDC50.SC1_SCL0_FRC3,
            &VDC50.SC1_SCL0_FRC4,
            &VDC50.SC1_SCL0_FRC5,
            &VDC50.SC1_SCL0_FRC6,
            &VDC50.SC1_SCL0_FRC7,
            &VDC50.SC1_SCL0_FRC9,
            &VDC50.SC1_SCL0_MON0,
            &VDC50.SC1_SCL0_INT,
            &VDC50.SC1_SCL0_DS1,
            &VDC50.SC1_SCL0_DS2,
            &VDC50.SC1_SCL0_DS3,
            &VDC50.SC1_SCL0_DS4,
            &VDC50.SC1_SCL0_DS5,
            &VDC50.SC1_SCL0_DS6,
            &VDC50.SC1_SCL0_DS7,
            &VDC50.SC1_SCL0_US1,
            &VDC50.SC1_SCL0_US2,
            &VDC50.SC1_SCL0_US3,
            &VDC50.SC1_SCL0_US4,
            &VDC50.SC1_SCL0_US5,
            &VDC50.SC1_SCL0_US6,
            &VDC50.SC1_SCL0_US7,
            &VDC50.SC1_SCL0_US8,
            &VDC50.SC1_SCL0_OVR1,
            &VDC50.SC1_SCL1_UPDATE,
            &VDC50.SC1_SCL1_WR1,
            &VDC50.SC1_SCL1_WR2,
            &VDC50.SC1_SCL1_WR3,
            &VDC50.SC1_SCL1_WR4,
            &VDC50.SC1_SCL1_WR5,
            &VDC50.SC1_SCL1_WR6,
            &VDC50.SC1_SCL1_WR7,
            &VDC50.SC1_SCL1_WR8,
            &VDC50.SC1_SCL1_WR9,
            &VDC50.SC1_SCL1_WR10,
            &VDC50.SC1_SCL1_WR11,
            &VDC50.SC1_SCL1_MON1,
            &VDC50.SC1_SCL1_PBUF0,
            &VDC50.SC1_SCL1_PBUF1,
            &VDC50.SC1_SCL1_PBUF2,
            &VDC50.SC1_SCL1_PBUF3,
            &VDC50.SC1_SCL1_PBUF_FLD,
            &VDC50.SC1_SCL1_PBUF_CNT
        },
        {   /* OIR */
            &VDC50.OIR_SCL0_UPDATE,
            &VDC50.OIR_SCL0_FRC1,
            &VDC50.OIR_SCL0_FRC2,
            &VDC50.OIR_SCL0_FRC3,
            &VDC50.OIR_SCL0_FRC4,
            &VDC50.OIR_SCL0_FRC5,
            &VDC50.OIR_SCL0_FRC6,
            &VDC50.OIR_SCL0_FRC7,
            NULL,
            NULL,
            NULL,
            &VDC50.OIR_SCL0_DS1,
            &VDC50.OIR_SCL0_DS2,
            &VDC50.OIR_SCL0_DS3,
            NULL,
            NULL,
            NULL,
            &VDC50.OIR_SCL0_DS7,
            &VDC50.OIR_SCL0_US1,
            &VDC50.OIR_SCL0_US2,
            &VDC50.OIR_SCL0_US3,
            NULL,
            NULL,
            NULL,
            NULL,
            &VDC50.OIR_SCL0_US8,
            &VDC50.OIR_SCL0_OVR1,
            &VDC50.OIR_SCL1_UPDATE,
            &VDC50.OIR_SCL1_WR1,
            &VDC50.OIR_SCL1_WR2,
            &VDC50.OIR_SCL1_WR3,
            &VDC50.OIR_SCL1_WR4,
            &VDC50.OIR_SCL1_WR5,
            &VDC50.OIR_SCL1_WR6,
            &VDC50.OIR_SCL1_WR7,
            NULL,
            NULL,
            NULL,
            NULL,
            NULL,
            NULL,
            NULL,
            NULL,
            NULL,
            NULL,
            NULL
        }
    },
    {   /* Channel 1 */
        {   /* SC0 */
            &VDC51.SC0_SCL0_UPDATE,
            &VDC51.SC0_SCL0_FRC1,
            &VDC51.SC0_SCL0_FRC2,
            &VDC51.SC0_SCL0_FRC3,
            &VDC51.SC0_SCL0_FRC4,
            &VDC51.SC0_SCL0_FRC5,
            &VDC51.SC0_SCL0_FRC6,
            &VDC51.SC0_SCL0_FRC7,
            &VDC51.SC0_SCL0_FRC9,
            &VDC51.SC0_SCL0_MON0,
            &VDC51.SC0_SCL0_INT,
            &VDC51.SC0_SCL0_DS1,
            &VDC51.SC0_SCL0_DS2,
            &VDC51.SC0_SCL0_DS3,
            &VDC51.SC0_SCL0_DS4,
            &VDC51.SC0_SCL0_DS5,
            &VDC51.SC0_SCL0_DS6,
            &VDC51.SC0_SCL0_DS7,
            &VDC51.SC0_SCL0_US1,
            &VDC51.SC0_SCL0_US2,
            &VDC51.SC0_SCL0_US3,
            &VDC51.SC0_SCL0_US4,
            &VDC51.SC0_SCL0_US5,
            &VDC51.SC0_SCL0_US6,
            &VDC51.SC0_SCL0_US7,
            &VDC51.SC0_SCL0_US8,
            &VDC51.SC0_SCL0_OVR1,
            &VDC51.SC0_SCL1_UPDATE,
            &VDC51.SC0_SCL1_WR1,
            &VDC51.SC0_SCL1_WR2,
            &VDC51.SC0_SCL1_WR3,
            &VDC51.SC0_SCL1_WR4,
            &VDC51.SC0_SCL1_WR5,
            &VDC51.SC0_SCL1_WR6,
            &VDC51.SC0_SCL1_WR7,
            &VDC51.SC0_SCL1_WR8,
            &VDC51.SC0_SCL1_WR9,
            &VDC51.SC0_SCL1_WR10,
            &VDC51.SC0_SCL1_WR11,
            &VDC51.SC0_SCL1_MON1,
            &VDC51.SC0_SCL1_PBUF0,
            &VDC51.SC0_SCL1_PBUF1,
            &VDC51.SC0_SCL1_PBUF2,
            &VDC51.SC0_SCL1_PBUF3,
            &VDC51.SC0_SCL1_PBUF_FLD,
            &VDC51.SC0_SCL1_PBUF_CNT
        },
        {   /* SC1 */
            &VDC51.SC1_SCL0_UPDATE,
            &VDC51.SC1_SCL0_FRC1,
            &VDC51.SC1_SCL0_FRC2,
            &VDC51.SC1_SCL0_FRC3,
            &VDC51.SC1_SCL0_FRC4,
            &VDC51.SC1_SCL0_FRC5,
            &VDC51.SC1_SCL0_FRC6,
            &VDC51.SC1_SCL0_FRC7,
            &VDC51.SC1_SCL0_FRC9,
            &VDC51.SC1_SCL0_MON0,
            &VDC51.SC1_SCL0_INT,
            &VDC51.SC1_SCL0_DS1,
            &VDC51.SC1_SCL0_DS2,
            &VDC51.SC1_SCL0_DS3,
            &VDC51.SC1_SCL0_DS4,
            &VDC51.SC1_SCL0_DS5,
            &VDC51.SC1_SCL0_DS6,
            &VDC51.SC1_SCL0_DS7,
            &VDC51.SC1_SCL0_US1,
            &VDC51.SC1_SCL0_US2,
            &VDC51.SC1_SCL0_US3,
            &VDC51.SC1_SCL0_US4,
            &VDC51.SC1_SCL0_US5,
            &VDC51.SC1_SCL0_US6,
            &VDC51.SC1_SCL0_US7,
            &VDC51.SC1_SCL0_US8,
            &VDC51.SC1_SCL0_OVR1,
            &VDC51.SC1_SCL1_UPDATE,
            &VDC51.SC1_SCL1_WR1,
            &VDC51.SC1_SCL1_WR2,
            &VDC51.SC1_SCL1_WR3,
            &VDC51.SC1_SCL1_WR4,
            &VDC51.SC1_SCL1_WR5,
            &VDC51.SC1_SCL1_WR6,
            &VDC51.SC1_SCL1_WR7,
            &VDC51.SC1_SCL1_WR8,
            &VDC51.SC1_SCL1_WR9,
            &VDC51.SC1_SCL1_WR10,
            &VDC51.SC1_SCL1_WR11,
            &VDC51.SC1_SCL1_MON1,
            &VDC51.SC1_SCL1_PBUF0,
            &VDC51.SC1_SCL1_PBUF1,
            &VDC51.SC1_SCL1_PBUF2,
            &VDC51.SC1_SCL1_PBUF3,
            &VDC51.SC1_SCL1_PBUF_FLD,
            &VDC51.SC1_SCL1_PBUF_CNT
        },
        {   /* OIR */
            &VDC51.OIR_SCL0_UPDATE,
            &VDC51.OIR_SCL0_FRC1,
            &VDC51.OIR_SCL0_FRC2,
            &VDC51.OIR_SCL0_FRC3,
            &VDC51.OIR_SCL0_FRC4,
            &VDC51.OIR_SCL0_FRC5,
            &VDC51.OIR_SCL0_FRC6,
            &VDC51.OIR_SCL0_FRC7,
            NULL,
            NULL,
            NULL,
            &VDC51.OIR_SCL0_DS1,
            &VDC51.OIR_SCL0_DS2,
            &VDC51.OIR_SCL0_DS3,
            NULL,
            NULL,
            NULL,
            &VDC51.OIR_SCL0_DS7,
            &VDC51.OIR_SCL0_US1,
            &VDC51.OIR_SCL0_US2,
            &VDC51.OIR_SCL0_US3,
            NULL,
            NULL,
            NULL,
            NULL,
            &VDC51.OIR_SCL0_US8,
            &VDC51.OIR_SCL0_OVR1,
            &VDC51.OIR_SCL1_UPDATE,
            &VDC51.OIR_SCL1_WR1,
            &VDC51.OIR_SCL1_WR2,
            &VDC51.OIR_SCL1_WR3,
            &VDC51.OIR_SCL1_WR4,
            &VDC51.OIR_SCL1_WR5,
            &VDC51.OIR_SCL1_WR6,
            &VDC51.OIR_SCL1_WR7,
            NULL,
            NULL,
            NULL,
            NULL,
            NULL,
            NULL,
            NULL,
            NULL,
            NULL,
            NULL,
            NULL
        }
    }
};
#else
vdc5_regaddr_scaler_t vdc5_regaddr_scaler[VDC5_CHANNEL_NUM][VDC5_SC_TYPE_NUM];

static void initialize_vdc5_regaddr_scaler(void)
{
	/* Channel 0 */
	/* SC0 */
	vdc5_regaddr_scaler[0][0].scl0_update = &VDC50.SC0_SCL0_UPDATE;
	vdc5_regaddr_scaler[0][0].scl0_frc1 = &VDC50.SC0_SCL0_FRC1;
	vdc5_regaddr_scaler[0][0].scl0_frc2 = &VDC50.SC0_SCL0_FRC2;
	vdc5_regaddr_scaler[0][0].scl0_frc3 = &VDC50.SC0_SCL0_FRC3;
	vdc5_regaddr_scaler[0][0].scl0_frc4 = &VDC50.SC0_SCL0_FRC4;
	vdc5_regaddr_scaler[0][0].scl0_frc5 = &VDC50.SC0_SCL0_FRC5;
	vdc5_regaddr_scaler[0][0].scl0_frc6 = &VDC50.SC0_SCL0_FRC6;
	vdc5_regaddr_scaler[0][0].scl0_frc7 = &VDC50.SC0_SCL0_FRC7;
	vdc5_regaddr_scaler[0][0].scl0_frc9 = &VDC50.SC0_SCL0_FRC9;
	vdc5_regaddr_scaler[0][0].scl0_mon0 = &VDC50.SC0_SCL0_MON0;
	vdc5_regaddr_scaler[0][0].scl0_int = &VDC50.SC0_SCL0_INT;
	vdc5_regaddr_scaler[0][0].scl0_ds1 = &VDC50.SC0_SCL0_DS1;
	vdc5_regaddr_scaler[0][0].scl0_ds2 = &VDC50.SC0_SCL0_DS2;
	vdc5_regaddr_scaler[0][0].scl0_ds3 = &VDC50.SC0_SCL0_DS3;
	vdc5_regaddr_scaler[0][0].scl0_ds4 = &VDC50.SC0_SCL0_DS4;
	vdc5_regaddr_scaler[0][0].scl0_ds5 = &VDC50.SC0_SCL0_DS5;
	vdc5_regaddr_scaler[0][0].scl0_ds6 = &VDC50.SC0_SCL0_DS6;
	vdc5_regaddr_scaler[0][0].scl0_ds7 = &VDC50.SC0_SCL0_DS7;
	vdc5_regaddr_scaler[0][0].scl0_us1 = &VDC50.SC0_SCL0_US1;
	vdc5_regaddr_scaler[0][0].scl0_us2 = &VDC50.SC0_SCL0_US2;
	vdc5_regaddr_scaler[0][0].scl0_us3 = &VDC50.SC0_SCL0_US3;
	vdc5_regaddr_scaler[0][0].scl0_us4 = &VDC50.SC0_SCL0_US4;
	vdc5_regaddr_scaler[0][0].scl0_us5 = &VDC50.SC0_SCL0_US5;
	vdc5_regaddr_scaler[0][0].scl0_us6 = &VDC50.SC0_SCL0_US6;
	vdc5_regaddr_scaler[0][0].scl0_us7 = &VDC50.SC0_SCL0_US7;
	vdc5_regaddr_scaler[0][0].scl0_us8 = &VDC50.SC0_SCL0_US8;
	vdc5_regaddr_scaler[0][0].scl0_ovr1 = &VDC50.SC0_SCL0_OVR1;
	vdc5_regaddr_scaler[0][0].scl1_update = &VDC50.SC0_SCL1_UPDATE;
	vdc5_regaddr_scaler[0][0].scl1_wr1 = &VDC50.SC0_SCL1_WR1;
	vdc5_regaddr_scaler[0][0].scl1_wr2 = &VDC50.SC0_SCL1_WR2;
	vdc5_regaddr_scaler[0][0].scl1_wr3 = &VDC50.SC0_SCL1_WR3;
	vdc5_regaddr_scaler[0][0].scl1_wr4 = &VDC50.SC0_SCL1_WR4;
	vdc5_regaddr_scaler[0][0].scl1_wr5 = &VDC50.SC0_SCL1_WR5;
	vdc5_regaddr_scaler[0][0].scl1_wr6 = &VDC50.SC0_SCL1_WR6;
	vdc5_regaddr_scaler[0][0].scl1_wr7 = &VDC50.SC0_SCL1_WR7;
	vdc5_regaddr_scaler[0][0].scl1_wr8 = &VDC50.SC0_SCL1_WR8;
	vdc5_regaddr_scaler[0][0].scl1_wr9 = &VDC50.SC0_SCL1_WR9;
	vdc5_regaddr_scaler[0][0].scl1_wr10 = &VDC50.SC0_SCL1_WR10;
	vdc5_regaddr_scaler[0][0].scl1_wr11 = &VDC50.SC0_SCL1_WR11;
	vdc5_regaddr_scaler[0][0].scl1_mon1 = &VDC50.SC0_SCL1_MON1;
	vdc5_regaddr_scaler[0][0].scl1_pbuf0 = &VDC50.SC0_SCL1_PBUF0;
	vdc5_regaddr_scaler[0][0].scl1_pbuf1 = &VDC50.SC0_SCL1_PBUF1;
	vdc5_regaddr_scaler[0][0].scl1_pbuf2 = &VDC50.SC0_SCL1_PBUF2;
	vdc5_regaddr_scaler[0][0].scl1_pbuf3 = &VDC50.SC0_SCL1_PBUF3;
	vdc5_regaddr_scaler[0][0].scl1_pbuf_fld = &VDC50.SC0_SCL1_PBUF_FLD;
	vdc5_regaddr_scaler[0][0].scl1_pbuf_cnt = &VDC50.SC0_SCL1_PBUF_CNT;
	/* SC1 */
	vdc5_regaddr_scaler[0][1].scl0_update = &VDC50.SC1_SCL0_UPDATE;
	vdc5_regaddr_scaler[0][1].scl0_frc1 = &VDC50.SC1_SCL0_FRC1;
	vdc5_regaddr_scaler[0][1].scl0_frc2 = &VDC50.SC1_SCL0_FRC2;
	vdc5_regaddr_scaler[0][1].scl0_frc3 = &VDC50.SC1_SCL0_FRC3;
	vdc5_regaddr_scaler[0][1].scl0_frc4 = &VDC50.SC1_SCL0_FRC4;
	vdc5_regaddr_scaler[0][1].scl0_frc5 = &VDC50.SC1_SCL0_FRC5;
	vdc5_regaddr_scaler[0][1].scl0_frc6 = &VDC50.SC1_SCL0_FRC6;
	vdc5_regaddr_scaler[0][1].scl0_frc7 = &VDC50.SC1_SCL0_FRC7;
	vdc5_regaddr_scaler[0][1].scl0_frc9 = &VDC50.SC1_SCL0_FRC9;
	vdc5_regaddr_scaler[0][1].scl0_mon0 = &VDC50.SC1_SCL0_MON0;
	vdc5_regaddr_scaler[0][1].scl0_int = &VDC50.SC1_SCL0_INT;
	vdc5_regaddr_scaler[0][1].scl0_ds1 = &VDC50.SC1_SCL0_DS1;
	vdc5_regaddr_scaler[0][1].scl0_ds2 = &VDC50.SC1_SCL0_DS2;
	vdc5_regaddr_scaler[0][1].scl0_ds3 = &VDC50.SC1_SCL0_DS3;
	vdc5_regaddr_scaler[0][1].scl0_ds4 = &VDC50.SC1_SCL0_DS4;
	vdc5_regaddr_scaler[0][1].scl0_ds5 = &VDC50.SC1_SCL0_DS5;
	vdc5_regaddr_scaler[0][1].scl0_ds6 = &VDC50.SC1_SCL0_DS6;
	vdc5_regaddr_scaler[0][1].scl0_ds7 = &VDC50.SC1_SCL0_DS7;
	vdc5_regaddr_scaler[0][1].scl0_us1 = &VDC50.SC1_SCL0_US1;
	vdc5_regaddr_scaler[0][1].scl0_us2 = &VDC50.SC1_SCL0_US2;
	vdc5_regaddr_scaler[0][1].scl0_us3 = &VDC50.SC1_SCL0_US3;
	vdc5_regaddr_scaler[0][1].scl0_us4 = &VDC50.SC1_SCL0_US4;
	vdc5_regaddr_scaler[0][1].scl0_us5 = &VDC50.SC1_SCL0_US5;
	vdc5_regaddr_scaler[0][1].scl0_us6 = &VDC50.SC1_SCL0_US6;
	vdc5_regaddr_scaler[0][1].scl0_us7 = &VDC50.SC1_SCL0_US7;
	vdc5_regaddr_scaler[0][1].scl0_us8 = &VDC50.SC1_SCL0_US8;
	vdc5_regaddr_scaler[0][1].scl0_ovr1 = &VDC50.SC1_SCL0_OVR1;
	vdc5_regaddr_scaler[0][1].scl1_update = &VDC50.SC1_SCL1_UPDATE;
	vdc5_regaddr_scaler[0][1].scl1_wr1 = &VDC50.SC1_SCL1_WR1;
	vdc5_regaddr_scaler[0][1].scl1_wr2 = &VDC50.SC1_SCL1_WR2;
	vdc5_regaddr_scaler[0][1].scl1_wr3 = &VDC50.SC1_SCL1_WR3;
	vdc5_regaddr_scaler[0][1].scl1_wr4 = &VDC50.SC1_SCL1_WR4;
	vdc5_regaddr_scaler[0][1].scl1_wr5 = &VDC50.SC1_SCL1_WR5;
	vdc5_regaddr_scaler[0][1].scl1_wr6 = &VDC50.SC1_SCL1_WR6;
	vdc5_regaddr_scaler[0][1].scl1_wr7 = &VDC50.SC1_SCL1_WR7;
	vdc5_regaddr_scaler[0][1].scl1_wr8 = &VDC50.SC1_SCL1_WR8;
	vdc5_regaddr_scaler[0][1].scl1_wr9 = &VDC50.SC1_SCL1_WR9;
	vdc5_regaddr_scaler[0][1].scl1_wr10 = &VDC50.SC1_SCL1_WR10;
	vdc5_regaddr_scaler[0][1].scl1_wr11 = &VDC50.SC1_SCL1_WR11;
	vdc5_regaddr_scaler[0][1].scl1_mon1 = &VDC50.SC1_SCL1_MON1;
	vdc5_regaddr_scaler[0][1].scl1_pbuf0 = &VDC50.SC1_SCL1_PBUF0;
	vdc5_regaddr_scaler[0][1].scl1_pbuf1 = &VDC50.SC1_SCL1_PBUF1;
	vdc5_regaddr_scaler[0][1].scl1_pbuf2 = &VDC50.SC1_SCL1_PBUF2;
	vdc5_regaddr_scaler[0][1].scl1_pbuf3 = &VDC50.SC1_SCL1_PBUF3;
	vdc5_regaddr_scaler[0][1].scl1_pbuf_fld = &VDC50.SC1_SCL1_PBUF_FLD;
	vdc5_regaddr_scaler[0][1].scl1_pbuf_cnt = &VDC50.SC1_SCL1_PBUF_CNT;
	 /* OIR */
	vdc5_regaddr_scaler[0][2].scl0_update = &VDC50.OIR_SCL0_UPDATE;
	vdc5_regaddr_scaler[0][2].scl0_frc1 = &VDC50.OIR_SCL0_FRC1;
	vdc5_regaddr_scaler[0][2].scl0_frc2 = &VDC50.OIR_SCL0_FRC2;
	vdc5_regaddr_scaler[0][2].scl0_frc3 = &VDC50.OIR_SCL0_FRC3;
	vdc5_regaddr_scaler[0][2].scl0_frc4 = &VDC50.OIR_SCL0_FRC4;
	vdc5_regaddr_scaler[0][2].scl0_frc5 = &VDC50.OIR_SCL0_FRC5;
	vdc5_regaddr_scaler[0][2].scl0_frc6 = &VDC50.OIR_SCL0_FRC6;
	vdc5_regaddr_scaler[0][2].scl0_frc7 = &VDC50.OIR_SCL0_FRC7;
	vdc5_regaddr_scaler[0][2].scl0_frc9 = NULL;
	vdc5_regaddr_scaler[0][2].scl0_mon0 = NULL;
	vdc5_regaddr_scaler[0][2].scl0_int = NULL;
	vdc5_regaddr_scaler[0][2].scl0_ds1 = &VDC50.OIR_SCL0_DS1;
	vdc5_regaddr_scaler[0][2].scl0_ds2 = &VDC50.OIR_SCL0_DS2;
	vdc5_regaddr_scaler[0][2].scl0_ds3 = &VDC50.OIR_SCL0_DS3;
	vdc5_regaddr_scaler[0][2].scl0_ds4 = NULL;
	vdc5_regaddr_scaler[0][2].scl0_ds5 = NULL;
	vdc5_regaddr_scaler[0][2].scl0_ds6 = NULL;
	vdc5_regaddr_scaler[0][2].scl0_ds7 = &VDC50.OIR_SCL0_DS7;
	vdc5_regaddr_scaler[0][2].scl0_us1 = &VDC50.OIR_SCL0_US1;
	vdc5_regaddr_scaler[0][2].scl0_us2 = &VDC50.OIR_SCL0_US2;
	vdc5_regaddr_scaler[0][2].scl0_us3 = &VDC50.OIR_SCL0_US3;
	vdc5_regaddr_scaler[0][2].scl0_us4 = NULL;
	vdc5_regaddr_scaler[0][2].scl0_us5 = NULL;
	vdc5_regaddr_scaler[0][2].scl0_us6 = NULL;
	vdc5_regaddr_scaler[0][2].scl0_us7 = NULL;
	vdc5_regaddr_scaler[0][2].scl0_us8 = &VDC50.OIR_SCL0_US8;
	vdc5_regaddr_scaler[0][2].scl0_ovr1 = &VDC50.OIR_SCL0_OVR1;
	vdc5_regaddr_scaler[0][2].scl1_update = &VDC50.OIR_SCL1_UPDATE;
	vdc5_regaddr_scaler[0][2].scl1_wr1 = &VDC50.OIR_SCL1_WR1;
	vdc5_regaddr_scaler[0][2].scl1_wr2 = &VDC50.OIR_SCL1_WR2;
	vdc5_regaddr_scaler[0][2].scl1_wr3 = &VDC50.OIR_SCL1_WR3;
	vdc5_regaddr_scaler[0][2].scl1_wr4 = &VDC50.OIR_SCL1_WR4;
	vdc5_regaddr_scaler[0][2].scl1_wr5 = &VDC50.OIR_SCL1_WR5;
	vdc5_regaddr_scaler[0][2].scl1_wr6 = &VDC50.OIR_SCL1_WR6;
	vdc5_regaddr_scaler[0][2].scl1_wr7 = &VDC50.OIR_SCL1_WR7;
	vdc5_regaddr_scaler[0][2].scl1_wr8 = NULL;
	vdc5_regaddr_scaler[0][2].scl1_wr9 = NULL;
	vdc5_regaddr_scaler[0][2].scl1_wr10 = NULL;
	vdc5_regaddr_scaler[0][2].scl1_wr11 = NULL;
	vdc5_regaddr_scaler[0][2].scl1_mon1 = NULL;
	vdc5_regaddr_scaler[0][2].scl1_pbuf0 = NULL;
	vdc5_regaddr_scaler[0][2].scl1_pbuf1 = NULL;
	vdc5_regaddr_scaler[0][2].scl1_pbuf2 = NULL;
	vdc5_regaddr_scaler[0][2].scl1_pbuf3 = NULL;
	vdc5_regaddr_scaler[0][2].scl1_pbuf_fld = NULL;
	vdc5_regaddr_scaler[0][2].scl1_pbuf_cnt = NULL;

	/* Channel 1 */
	/* SC0 */
	vdc5_regaddr_scaler[1][0].scl0_update = &VDC51.SC0_SCL0_UPDATE;
	vdc5_regaddr_scaler[1][0].scl0_frc1 = &VDC51.SC0_SCL0_FRC1;
	vdc5_regaddr_scaler[1][0].scl0_frc2 = &VDC51.SC0_SCL0_FRC2;
	vdc5_regaddr_scaler[1][0].scl0_frc3 = &VDC51.SC0_SCL0_FRC3;
	vdc5_regaddr_scaler[1][0].scl0_frc4 = &VDC51.SC0_SCL0_FRC4;
	vdc5_regaddr_scaler[1][0].scl0_frc5 = &VDC51.SC0_SCL0_FRC5;
	vdc5_regaddr_scaler[1][0].scl0_frc6 = &VDC51.SC0_SCL0_FRC6;
	vdc5_regaddr_scaler[1][0].scl0_frc7 = &VDC51.SC0_SCL0_FRC7;
	vdc5_regaddr_scaler[1][0].scl0_frc9 = &VDC51.SC0_SCL0_FRC9;
	vdc5_regaddr_scaler[1][0].scl0_mon0 = &VDC51.SC0_SCL0_MON0;
	vdc5_regaddr_scaler[1][0].scl0_int = &VDC51.SC0_SCL0_INT;
	vdc5_regaddr_scaler[1][0].scl0_ds1 = &VDC51.SC0_SCL0_DS1;
	vdc5_regaddr_scaler[1][0].scl0_ds2 = &VDC51.SC0_SCL0_DS2;
	vdc5_regaddr_scaler[1][0].scl0_ds3 = &VDC51.SC0_SCL0_DS3;
	vdc5_regaddr_scaler[1][0].scl0_ds4 = &VDC51.SC0_SCL0_DS4;
	vdc5_regaddr_scaler[1][0].scl0_ds5 = &VDC51.SC0_SCL0_DS5;
	vdc5_regaddr_scaler[1][0].scl0_ds6 = &VDC51.SC0_SCL0_DS6;
	vdc5_regaddr_scaler[1][0].scl0_ds7 = &VDC51.SC0_SCL0_DS7;
	vdc5_regaddr_scaler[1][0].scl0_us1 = &VDC51.SC0_SCL0_US1;
	vdc5_regaddr_scaler[1][0].scl0_us2 = &VDC51.SC0_SCL0_US2;
	vdc5_regaddr_scaler[1][0].scl0_us3 = &VDC51.SC0_SCL0_US3;
	vdc5_regaddr_scaler[1][0].scl0_us4 = &VDC51.SC0_SCL0_US4;
	vdc5_regaddr_scaler[1][0].scl0_us5 = &VDC51.SC0_SCL0_US5;
	vdc5_regaddr_scaler[1][0].scl0_us6 = &VDC51.SC0_SCL0_US6;
	vdc5_regaddr_scaler[1][0].scl0_us7 = &VDC51.SC0_SCL0_US7;
	vdc5_regaddr_scaler[1][0].scl0_us8 = &VDC51.SC0_SCL0_US8;
	vdc5_regaddr_scaler[1][0].scl0_ovr1 = &VDC51.SC0_SCL0_OVR1;
	vdc5_regaddr_scaler[1][0].scl1_update = &VDC51.SC0_SCL1_UPDATE;
	vdc5_regaddr_scaler[1][0].scl1_wr1 = &VDC51.SC0_SCL1_WR1;
	vdc5_regaddr_scaler[1][0].scl1_wr2 = &VDC51.SC0_SCL1_WR2;
	vdc5_regaddr_scaler[1][0].scl1_wr3 = &VDC51.SC0_SCL1_WR3;
	vdc5_regaddr_scaler[1][0].scl1_wr4 = &VDC51.SC0_SCL1_WR4;
	vdc5_regaddr_scaler[1][0].scl1_wr5 = &VDC51.SC0_SCL1_WR5;
	vdc5_regaddr_scaler[1][0].scl1_wr6 = &VDC51.SC0_SCL1_WR6;
	vdc5_regaddr_scaler[1][0].scl1_wr7 = &VDC51.SC0_SCL1_WR7;
	vdc5_regaddr_scaler[1][0].scl1_wr8 = &VDC51.SC0_SCL1_WR8;
	vdc5_regaddr_scaler[1][0].scl1_wr9 = &VDC51.SC0_SCL1_WR9;
	vdc5_regaddr_scaler[1][0].scl1_wr10 = &VDC51.SC0_SCL1_WR10;
	vdc5_regaddr_scaler[1][0].scl1_wr11 = &VDC51.SC0_SCL1_WR11;
	vdc5_regaddr_scaler[1][0].scl1_mon1 = &VDC51.SC0_SCL1_MON1;
	vdc5_regaddr_scaler[1][0].scl1_pbuf0 = &VDC51.SC0_SCL1_PBUF0;
	vdc5_regaddr_scaler[1][0].scl1_pbuf1 = &VDC51.SC0_SCL1_PBUF1;
	vdc5_regaddr_scaler[1][0].scl1_pbuf2 = &VDC51.SC0_SCL1_PBUF2;
	vdc5_regaddr_scaler[1][0].scl1_pbuf3 = &VDC51.SC0_SCL1_PBUF3;
	vdc5_regaddr_scaler[1][0].scl1_pbuf_fld = &VDC51.SC0_SCL1_PBUF_FLD;
	vdc5_regaddr_scaler[1][0].scl1_pbuf_cnt = &VDC51.SC0_SCL1_PBUF_CNT;
	/* SC1 */
	vdc5_regaddr_scaler[1][1].scl0_update = &VDC51.SC1_SCL0_UPDATE;
	vdc5_regaddr_scaler[1][1].scl0_frc1 = &VDC51.SC1_SCL0_FRC1;
	vdc5_regaddr_scaler[1][1].scl0_frc2 = &VDC51.SC1_SCL0_FRC2;
	vdc5_regaddr_scaler[1][1].scl0_frc3 = &VDC51.SC1_SCL0_FRC3;
	vdc5_regaddr_scaler[1][1].scl0_frc4 = &VDC51.SC1_SCL0_FRC4;
	vdc5_regaddr_scaler[1][1].scl0_frc5 = &VDC51.SC1_SCL0_FRC5;
	vdc5_regaddr_scaler[1][1].scl0_frc6 = &VDC51.SC1_SCL0_FRC6;
	vdc5_regaddr_scaler[1][1].scl0_frc7 = &VDC51.SC1_SCL0_FRC7;
	vdc5_regaddr_scaler[1][1].scl0_frc9 = &VDC51.SC1_SCL0_FRC9;
	vdc5_regaddr_scaler[1][1].scl0_mon0 = &VDC51.SC1_SCL0_MON0;
	vdc5_regaddr_scaler[1][1].scl0_int = &VDC51.SC1_SCL0_INT;
	vdc5_regaddr_scaler[1][1].scl0_ds1 = &VDC51.SC1_SCL0_DS1;
	vdc5_regaddr_scaler[1][1].scl0_ds2 = &VDC51.SC1_SCL0_DS2;
	vdc5_regaddr_scaler[1][1].scl0_ds3 = &VDC51.SC1_SCL0_DS3;
	vdc5_regaddr_scaler[1][1].scl0_ds4 = &VDC51.SC1_SCL0_DS4;
	vdc5_regaddr_scaler[1][1].scl0_ds5 = &VDC51.SC1_SCL0_DS5;
	vdc5_regaddr_scaler[1][1].scl0_ds6 = &VDC51.SC1_SCL0_DS6;
	vdc5_regaddr_scaler[1][1].scl0_ds7 = &VDC51.SC1_SCL0_DS7;
	vdc5_regaddr_scaler[1][1].scl0_us1 = &VDC51.SC1_SCL0_US1;
	vdc5_regaddr_scaler[1][1].scl0_us2 = &VDC51.SC1_SCL0_US2;
	vdc5_regaddr_scaler[1][1].scl0_us3 = &VDC51.SC1_SCL0_US3;
	vdc5_regaddr_scaler[1][1].scl0_us4 = &VDC51.SC1_SCL0_US4;
	vdc5_regaddr_scaler[1][1].scl0_us5 = &VDC51.SC1_SCL0_US5;
	vdc5_regaddr_scaler[1][1].scl0_us6 = &VDC51.SC1_SCL0_US6;
	vdc5_regaddr_scaler[1][1].scl0_us7 = &VDC51.SC1_SCL0_US7;
	vdc5_regaddr_scaler[1][1].scl0_us8 = &VDC51.SC1_SCL0_US8;
	vdc5_regaddr_scaler[1][1].scl0_ovr1 = &VDC51.SC1_SCL0_OVR1;
	vdc5_regaddr_scaler[1][1].scl1_update = &VDC51.SC1_SCL1_UPDATE;
	vdc5_regaddr_scaler[1][1].scl1_wr1 = &VDC51.SC1_SCL1_WR1;
	vdc5_regaddr_scaler[1][1].scl1_wr2 = &VDC51.SC1_SCL1_WR2;
	vdc5_regaddr_scaler[1][1].scl1_wr3 = &VDC51.SC1_SCL1_WR3;
	vdc5_regaddr_scaler[1][1].scl1_wr4 = &VDC51.SC1_SCL1_WR4;
	vdc5_regaddr_scaler[1][1].scl1_wr5 = &VDC51.SC1_SCL1_WR5;
	vdc5_regaddr_scaler[1][1].scl1_wr6 = &VDC51.SC1_SCL1_WR6;
	vdc5_regaddr_scaler[1][1].scl1_wr7 = &VDC51.SC1_SCL1_WR7;
	vdc5_regaddr_scaler[1][1].scl1_wr8 = &VDC51.SC1_SCL1_WR8;
	vdc5_regaddr_scaler[1][1].scl1_wr9 = &VDC51.SC1_SCL1_WR9;
	vdc5_regaddr_scaler[1][1].scl1_wr10 = &VDC51.SC1_SCL1_WR10;
	vdc5_regaddr_scaler[1][1].scl1_wr11 = &VDC51.SC1_SCL1_WR11;
	vdc5_regaddr_scaler[1][1].scl1_mon1 = &VDC51.SC1_SCL1_MON1;
	vdc5_regaddr_scaler[1][1].scl1_pbuf0 = &VDC51.SC1_SCL1_PBUF0;
	vdc5_regaddr_scaler[1][1].scl1_pbuf1 = &VDC51.SC1_SCL1_PBUF1;
	vdc5_regaddr_scaler[1][1].scl1_pbuf2 = &VDC51.SC1_SCL1_PBUF2;
	vdc5_regaddr_scaler[1][1].scl1_pbuf3 = &VDC51.SC1_SCL1_PBUF3;
	vdc5_regaddr_scaler[1][1].scl1_pbuf_fld = &VDC51.SC1_SCL1_PBUF_FLD;
	vdc5_regaddr_scaler[1][1].scl1_pbuf_cnt = &VDC51.SC1_SCL1_PBUF_CNT;
	/* OIR */
	vdc5_regaddr_scaler[1][2].scl0_update = &VDC51.OIR_SCL0_UPDATE;
	vdc5_regaddr_scaler[1][2].scl0_frc1 = &VDC51.OIR_SCL0_FRC1;
	vdc5_regaddr_scaler[1][2].scl0_frc2 = &VDC51.OIR_SCL0_FRC2;
	vdc5_regaddr_scaler[1][2].scl0_frc3 = &VDC51.OIR_SCL0_FRC3;
	vdc5_regaddr_scaler[1][2].scl0_frc4 = &VDC51.OIR_SCL0_FRC4;
	vdc5_regaddr_scaler[1][2].scl0_frc5 = &VDC51.OIR_SCL0_FRC5;
	vdc5_regaddr_scaler[1][2].scl0_frc6 = &VDC51.OIR_SCL0_FRC6;
	vdc5_regaddr_scaler[1][2].scl0_frc7 = &VDC51.OIR_SCL0_FRC7;
	vdc5_regaddr_scaler[1][2].scl0_frc9 = NULL;
	vdc5_regaddr_scaler[1][2].scl0_mon0 = NULL;
	vdc5_regaddr_scaler[1][2].scl0_int = NULL;
	vdc5_regaddr_scaler[1][2].scl0_ds1 = &VDC51.OIR_SCL0_DS1;
	vdc5_regaddr_scaler[1][2].scl0_ds2 = &VDC51.OIR_SCL0_DS2;
	vdc5_regaddr_scaler[1][2].scl0_ds3 = &VDC51.OIR_SCL0_DS3;
	vdc5_regaddr_scaler[1][2].scl0_ds4 = NULL;
	vdc5_regaddr_scaler[1][2].scl0_ds5 = NULL;
	vdc5_regaddr_scaler[1][2].scl0_ds6 = NULL;
	vdc5_regaddr_scaler[1][2].scl0_ds7 = &VDC51.OIR_SCL0_DS7;
	vdc5_regaddr_scaler[1][2].scl0_us1 = &VDC51.OIR_SCL0_US1;
	vdc5_regaddr_scaler[1][2].scl0_us2 = &VDC51.OIR_SCL0_US2;
	vdc5_regaddr_scaler[1][2].scl0_us3 = &VDC51.OIR_SCL0_US3;
	vdc5_regaddr_scaler[1][2].scl0_us4 = NULL;
	vdc5_regaddr_scaler[1][2].scl0_us5 = NULL;
	vdc5_regaddr_scaler[1][2].scl0_us6 = NULL;
	vdc5_regaddr_scaler[1][2].scl0_us7 = NULL;
	vdc5_regaddr_scaler[1][2].scl0_us8 = &VDC51.OIR_SCL0_US8;
	vdc5_regaddr_scaler[1][2].scl0_ovr1 = &VDC51.OIR_SCL0_OVR1;
	vdc5_regaddr_scaler[1][2].scl1_update = &VDC51.OIR_SCL1_UPDATE;
	vdc5_regaddr_scaler[1][2].scl1_wr1 = &VDC51.OIR_SCL1_WR1;
	vdc5_regaddr_scaler[1][2].scl1_wr2 = &VDC51.OIR_SCL1_WR2;
	vdc5_regaddr_scaler[1][2].scl1_wr3 = &VDC51.OIR_SCL1_WR3;
	vdc5_regaddr_scaler[1][2].scl1_wr4 = &VDC51.OIR_SCL1_WR4;
	vdc5_regaddr_scaler[1][2].scl1_wr5 = &VDC51.OIR_SCL1_WR5;
	vdc5_regaddr_scaler[1][2].scl1_wr6 = &VDC51.OIR_SCL1_WR6;
	vdc5_regaddr_scaler[1][2].scl1_wr7 = &VDC51.OIR_SCL1_WR7;
	vdc5_regaddr_scaler[1][2].scl1_wr8 = NULL;
	vdc5_regaddr_scaler[1][2].scl1_wr9 = NULL;
	vdc5_regaddr_scaler[1][2].scl1_wr10 = NULL;
	vdc5_regaddr_scaler[1][2].scl1_wr11 = NULL;
	vdc5_regaddr_scaler[1][2].scl1_mon1 = NULL;
	vdc5_regaddr_scaler[1][2].scl1_pbuf0 = NULL;
	vdc5_regaddr_scaler[1][2].scl1_pbuf1 = NULL;
	vdc5_regaddr_scaler[1][2].scl1_pbuf2 = NULL;
	vdc5_regaddr_scaler[1][2].scl1_pbuf3 = NULL;
	vdc5_regaddr_scaler[1][2].scl1_pbuf_fld = NULL;
	vdc5_regaddr_scaler[1][2].scl1_pbuf_cnt = NULL;
}
#endif

/* VDC5 image quality improver register address list */
#if 0
const vdc5_regaddr_img_qlty_imp_t vdc5_regaddr_img_qlty_imp[VDC5_CHANNEL_NUM][VDC5_IMG_IMPRV_NUM] =
{
    {   /* Channel 0 */
        {   /* SC0 */
            &VDC50.ADJ0_UPDATE,
            &VDC50.ADJ0_BKSTR_SET,
            &VDC50.ADJ0_ENH_TIM1,
            &VDC50.ADJ0_ENH_TIM2,
            &VDC50.ADJ0_ENH_TIM3,
            &VDC50.ADJ0_ENH_SHP1,
            &VDC50.ADJ0_ENH_SHP2,
            &VDC50.ADJ0_ENH_SHP3,
            &VDC50.ADJ0_ENH_SHP4,
            &VDC50.ADJ0_ENH_SHP5,
            &VDC50.ADJ0_ENH_SHP6,
            &VDC50.ADJ0_ENH_LTI1,
            &VDC50.ADJ0_ENH_LTI2
        },
        {   /* SC1 */
            &VDC50.ADJ1_UPDATE,
            &VDC50.ADJ1_BKSTR_SET,
            &VDC50.ADJ1_ENH_TIM1,
            &VDC50.ADJ1_ENH_TIM2,
            &VDC50.ADJ1_ENH_TIM3,
            &VDC50.ADJ1_ENH_SHP1,
            &VDC50.ADJ1_ENH_SHP2,
            &VDC50.ADJ1_ENH_SHP3,
            &VDC50.ADJ1_ENH_SHP4,
            &VDC50.ADJ1_ENH_SHP5,
            &VDC50.ADJ1_ENH_SHP6,
            &VDC50.ADJ1_ENH_LTI1,
            &VDC50.ADJ1_ENH_LTI2
        }
    },
    {   /* Channel 1 */
        {   /* SC0 */
            &VDC51.ADJ0_UPDATE,
            &VDC51.ADJ0_BKSTR_SET,
            &VDC51.ADJ0_ENH_TIM1,
            &VDC51.ADJ0_ENH_TIM2,
            &VDC51.ADJ0_ENH_TIM3,
            &VDC51.ADJ0_ENH_SHP1,
            &VDC51.ADJ0_ENH_SHP2,
            &VDC51.ADJ0_ENH_SHP3,
            &VDC51.ADJ0_ENH_SHP4,
            &VDC51.ADJ0_ENH_SHP5,
            &VDC51.ADJ0_ENH_SHP6,
            &VDC51.ADJ0_ENH_LTI1,
            &VDC51.ADJ0_ENH_LTI2
        },
        {   /* SC1 */
            &VDC51.ADJ1_UPDATE,
            &VDC51.ADJ1_BKSTR_SET,
            &VDC51.ADJ1_ENH_TIM1,
            &VDC51.ADJ1_ENH_TIM2,
            &VDC51.ADJ1_ENH_TIM3,
            &VDC51.ADJ1_ENH_SHP1,
            &VDC51.ADJ1_ENH_SHP2,
            &VDC51.ADJ1_ENH_SHP3,
            &VDC51.ADJ1_ENH_SHP4,
            &VDC51.ADJ1_ENH_SHP5,
            &VDC51.ADJ1_ENH_SHP6,
            &VDC51.ADJ1_ENH_LTI1,
            &VDC51.ADJ1_ENH_LTI2
        }
    }
};
#else
vdc5_regaddr_img_qlty_imp_t vdc5_regaddr_img_qlty_imp[VDC5_CHANNEL_NUM][VDC5_IMG_IMPRV_NUM];
static void initialize_vdc5_regaddr_img_qlty_imp(void)
{
	 /* Channel 0 */
	/* SC0 */
	vdc5_regaddr_img_qlty_imp[0][0].adj_update = &VDC50.ADJ0_UPDATE;
	vdc5_regaddr_img_qlty_imp[0][0].adj_bkstr_set = &VDC50.ADJ0_BKSTR_SET;
	vdc5_regaddr_img_qlty_imp[0][0].adj_enh_tim1 = &VDC50.ADJ0_ENH_TIM1;
	vdc5_regaddr_img_qlty_imp[0][0].adj_enh_tim2 = &VDC50.ADJ0_ENH_TIM2;
	vdc5_regaddr_img_qlty_imp[0][0].adj_enh_tim3 = &VDC50.ADJ0_ENH_TIM3;
	vdc5_regaddr_img_qlty_imp[0][0].adj_enh_shp1 = &VDC50.ADJ0_ENH_SHP1;
	vdc5_regaddr_img_qlty_imp[0][0].adj_enh_shp2 = &VDC50.ADJ0_ENH_SHP2;
	vdc5_regaddr_img_qlty_imp[0][0].adj_enh_shp3 = &VDC50.ADJ0_ENH_SHP3;
	vdc5_regaddr_img_qlty_imp[0][0].adj_enh_shp4 = &VDC50.ADJ0_ENH_SHP4;
	vdc5_regaddr_img_qlty_imp[0][0].adj_enh_shp5 = &VDC50.ADJ0_ENH_SHP5;
	vdc5_regaddr_img_qlty_imp[0][0].adj_enh_shp6 = &VDC50.ADJ0_ENH_SHP6;
	vdc5_regaddr_img_qlty_imp[0][0].adj_enh_lti1 = &VDC50.ADJ0_ENH_LTI1;
	vdc5_regaddr_img_qlty_imp[0][0].adj_enh_lti2 = &VDC50.ADJ0_ENH_LTI2;
	/* SC1 */
	vdc5_regaddr_img_qlty_imp[0][1].adj_update = &VDC50.ADJ1_UPDATE;
	vdc5_regaddr_img_qlty_imp[0][1].adj_bkstr_set = &VDC50.ADJ1_BKSTR_SET;
	vdc5_regaddr_img_qlty_imp[0][1].adj_enh_tim1 = &VDC50.ADJ1_ENH_TIM1;
	vdc5_regaddr_img_qlty_imp[0][1].adj_enh_tim2 = &VDC50.ADJ1_ENH_TIM2;
	vdc5_regaddr_img_qlty_imp[0][1].adj_enh_tim3 = &VDC50.ADJ1_ENH_TIM3;
	vdc5_regaddr_img_qlty_imp[0][1].adj_enh_shp1 = &VDC50.ADJ1_ENH_SHP1;
	vdc5_regaddr_img_qlty_imp[0][1].adj_enh_shp2 = &VDC50.ADJ1_ENH_SHP2;
	vdc5_regaddr_img_qlty_imp[0][1].adj_enh_shp3 = &VDC50.ADJ1_ENH_SHP3;
	vdc5_regaddr_img_qlty_imp[0][1].adj_enh_shp4 = &VDC50.ADJ1_ENH_SHP4;
	vdc5_regaddr_img_qlty_imp[0][1].adj_enh_shp5 = &VDC50.ADJ1_ENH_SHP5;
	vdc5_regaddr_img_qlty_imp[0][1].adj_enh_shp6 = &VDC50.ADJ1_ENH_SHP6;
	vdc5_regaddr_img_qlty_imp[0][1].adj_enh_lti1 = &VDC50.ADJ1_ENH_LTI1;
	vdc5_regaddr_img_qlty_imp[0][1].adj_enh_lti2 = &VDC50.ADJ1_ENH_LTI2;

	/* Channel 1 */
	/* SC0 */
	vdc5_regaddr_img_qlty_imp[1][0].adj_update = &VDC51.ADJ0_UPDATE;
	vdc5_regaddr_img_qlty_imp[1][0].adj_bkstr_set = &VDC51.ADJ0_BKSTR_SET;
	vdc5_regaddr_img_qlty_imp[1][0].adj_enh_tim1 = &VDC51.ADJ0_ENH_TIM1;
	vdc5_regaddr_img_qlty_imp[1][0].adj_enh_tim2 = &VDC51.ADJ0_ENH_TIM2;
	vdc5_regaddr_img_qlty_imp[1][0].adj_enh_tim3 = &VDC51.ADJ0_ENH_TIM3;
	vdc5_regaddr_img_qlty_imp[1][0].adj_enh_shp1 = &VDC51.ADJ0_ENH_SHP1;
	vdc5_regaddr_img_qlty_imp[1][0].adj_enh_shp2 = &VDC51.ADJ0_ENH_SHP2;
	vdc5_regaddr_img_qlty_imp[1][0].adj_enh_shp3 = &VDC51.ADJ0_ENH_SHP3;
	vdc5_regaddr_img_qlty_imp[1][0].adj_enh_shp4 = &VDC51.ADJ0_ENH_SHP4;
	vdc5_regaddr_img_qlty_imp[1][0].adj_enh_shp5 = &VDC51.ADJ0_ENH_SHP5;
	vdc5_regaddr_img_qlty_imp[1][0].adj_enh_shp6 = &VDC51.ADJ0_ENH_SHP6;
	vdc5_regaddr_img_qlty_imp[1][0].adj_enh_lti1 = &VDC51.ADJ0_ENH_LTI1;
	vdc5_regaddr_img_qlty_imp[1][0].adj_enh_lti2 = &VDC51.ADJ0_ENH_LTI2;
	/* SC1 */
	vdc5_regaddr_img_qlty_imp[1][1].adj_update = &VDC51.ADJ1_UPDATE;
	vdc5_regaddr_img_qlty_imp[1][1].adj_bkstr_set = &VDC51.ADJ1_BKSTR_SET;
	vdc5_regaddr_img_qlty_imp[1][1].adj_enh_tim1 = &VDC51.ADJ1_ENH_TIM1;
	vdc5_regaddr_img_qlty_imp[1][1].adj_enh_tim2 = &VDC51.ADJ1_ENH_TIM2;
	vdc5_regaddr_img_qlty_imp[1][1].adj_enh_tim3 = &VDC51.ADJ1_ENH_TIM3;
	vdc5_regaddr_img_qlty_imp[1][1].adj_enh_shp1 = &VDC51.ADJ1_ENH_SHP1;
	vdc5_regaddr_img_qlty_imp[1][1].adj_enh_shp2 = &VDC51.ADJ1_ENH_SHP2;
	vdc5_regaddr_img_qlty_imp[1][1].adj_enh_shp3 = &VDC51.ADJ1_ENH_SHP3;
	vdc5_regaddr_img_qlty_imp[1][1].adj_enh_shp4 = &VDC51.ADJ1_ENH_SHP4;
	vdc5_regaddr_img_qlty_imp[1][1].adj_enh_shp5 = &VDC51.ADJ1_ENH_SHP5;
	vdc5_regaddr_img_qlty_imp[1][1].adj_enh_shp6 = &VDC51.ADJ1_ENH_SHP6;
	vdc5_regaddr_img_qlty_imp[1][1].adj_enh_lti1 = &VDC51.ADJ1_ENH_LTI1;
	vdc5_regaddr_img_qlty_imp[1][1].adj_enh_lti2 = &VDC51.ADJ1_ENH_LTI2;
}
#endif

/* VDC5 color matrix register address list */
#if 0
const vdc5_regaddr_color_matrix_t vdc5_regaddr_color_matrix[VDC5_CHANNEL_NUM][VDC5_COLORMTX_NUM] =
{
    {   /* Channel 0 */
        {   /* Input Controller */
            &VDC50.IMGCNT_UPDATE,
            &VDC50.IMGCNT_MTX_MODE,
            &VDC50.IMGCNT_MTX_YG_ADJ0,
            &VDC50.IMGCNT_MTX_YG_ADJ1,
            &VDC50.IMGCNT_MTX_CBB_ADJ0,
            &VDC50.IMGCNT_MTX_CBB_ADJ1,
            &VDC50.IMGCNT_MTX_CRR_ADJ0,
            &VDC50.IMGCNT_MTX_CRR_ADJ1
        },
        {   /* Image quality improver 0 */
            &VDC50.ADJ0_UPDATE,
            &VDC50.ADJ0_MTX_MODE,
            &VDC50.ADJ0_MTX_YG_ADJ0,
            &VDC50.ADJ0_MTX_YG_ADJ1,
            &VDC50.ADJ0_MTX_CBB_ADJ0,
            &VDC50.ADJ0_MTX_CBB_ADJ1,
            &VDC50.ADJ0_MTX_CRR_ADJ0,
            &VDC50.ADJ0_MTX_CRR_ADJ1
        },
        {   /* Image quality improver 1 */
            &VDC50.ADJ1_UPDATE,
            &VDC50.ADJ1_MTX_MODE,
            &VDC50.ADJ1_MTX_YG_ADJ0,
            &VDC50.ADJ1_MTX_YG_ADJ1,
            &VDC50.ADJ1_MTX_CBB_ADJ0,
            &VDC50.ADJ1_MTX_CBB_ADJ1,
            &VDC50.ADJ1_MTX_CRR_ADJ0,
            &VDC50.ADJ1_MTX_CRR_ADJ1
        }
    },
    {   /* Channel 1 */
        {   /* Input Controller */
            &VDC51.IMGCNT_UPDATE,
            &VDC51.IMGCNT_MTX_MODE,
            &VDC51.IMGCNT_MTX_YG_ADJ0,
            &VDC51.IMGCNT_MTX_YG_ADJ1,
            &VDC51.IMGCNT_MTX_CBB_ADJ0,
            &VDC51.IMGCNT_MTX_CBB_ADJ1,
            &VDC51.IMGCNT_MTX_CRR_ADJ0,
            &VDC51.IMGCNT_MTX_CRR_ADJ1
        },
        {   /* Image quality improver 0 */
            &VDC51.ADJ0_UPDATE,
            &VDC51.ADJ0_MTX_MODE,
            &VDC51.ADJ0_MTX_YG_ADJ0,
            &VDC51.ADJ0_MTX_YG_ADJ1,
            &VDC51.ADJ0_MTX_CBB_ADJ0,
            &VDC51.ADJ0_MTX_CBB_ADJ1,
            &VDC51.ADJ0_MTX_CRR_ADJ0,
            &VDC51.ADJ0_MTX_CRR_ADJ1
        },
        {   /* Image quality improver 1 */
            &VDC51.ADJ1_UPDATE,
            &VDC51.ADJ1_MTX_MODE,
            &VDC51.ADJ1_MTX_YG_ADJ0,
            &VDC51.ADJ1_MTX_YG_ADJ1,
            &VDC51.ADJ1_MTX_CBB_ADJ0,
            &VDC51.ADJ1_MTX_CBB_ADJ1,
            &VDC51.ADJ1_MTX_CRR_ADJ0,
            &VDC51.ADJ1_MTX_CRR_ADJ1
        }
    }
};
#else
vdc5_regaddr_color_matrix_t vdc5_regaddr_color_matrix[VDC5_CHANNEL_NUM][VDC5_COLORMTX_NUM];

static void initialize_vdc5_regaddr_color_matrix(void)
{
	/* Channel 0 */
	/* Input Controller */
	vdc5_regaddr_color_matrix[0][0].mtx_update = &VDC50.IMGCNT_UPDATE;
	vdc5_regaddr_color_matrix[0][0].mtx_mode = &VDC50.IMGCNT_MTX_MODE;
	vdc5_regaddr_color_matrix[0][0].mtx_yg_adj0 = &VDC50.IMGCNT_MTX_YG_ADJ0;
	vdc5_regaddr_color_matrix[0][0].mtx_yg_adj1 = &VDC50.IMGCNT_MTX_YG_ADJ1;
	vdc5_regaddr_color_matrix[0][0].mtx_cbb_adj0 = &VDC50.IMGCNT_MTX_CBB_ADJ0;
	vdc5_regaddr_color_matrix[0][0].mtx_cbb_adj1 = &VDC50.IMGCNT_MTX_CBB_ADJ1;
	vdc5_regaddr_color_matrix[0][0].mtx_crr_adj0 = &VDC50.IMGCNT_MTX_CRR_ADJ0;
	vdc5_regaddr_color_matrix[0][0].mtx_crr_adj1 = &VDC50.IMGCNT_MTX_CRR_ADJ1;
	/* Image quality improver 0 */
	vdc5_regaddr_color_matrix[0][1].mtx_update = &VDC50.ADJ0_UPDATE;
	vdc5_regaddr_color_matrix[0][1].mtx_mode = &VDC50.ADJ0_MTX_MODE;
	vdc5_regaddr_color_matrix[0][1].mtx_yg_adj0 = &VDC50.ADJ0_MTX_YG_ADJ0;
	vdc5_regaddr_color_matrix[0][1].mtx_yg_adj1 = &VDC50.ADJ0_MTX_YG_ADJ1;
	vdc5_regaddr_color_matrix[0][1].mtx_cbb_adj0 = &VDC50.ADJ0_MTX_CBB_ADJ0;
	vdc5_regaddr_color_matrix[0][1].mtx_cbb_adj1 = &VDC50.ADJ0_MTX_CBB_ADJ1;
	vdc5_regaddr_color_matrix[0][1].mtx_crr_adj0 = &VDC50.ADJ0_MTX_CRR_ADJ0;
	vdc5_regaddr_color_matrix[0][1].mtx_crr_adj1 = &VDC50.ADJ0_MTX_CRR_ADJ1;
	/* Image quality improver 1 */
	vdc5_regaddr_color_matrix[0][2].mtx_update = &VDC50.ADJ1_UPDATE;
	vdc5_regaddr_color_matrix[0][2].mtx_mode = &VDC50.ADJ1_MTX_MODE;
	vdc5_regaddr_color_matrix[0][2].mtx_yg_adj0 = &VDC50.ADJ1_MTX_YG_ADJ0;
	vdc5_regaddr_color_matrix[0][2].mtx_yg_adj1 = &VDC50.ADJ1_MTX_YG_ADJ1;
	vdc5_regaddr_color_matrix[0][2].mtx_cbb_adj0 = &VDC50.ADJ1_MTX_CBB_ADJ0;
	vdc5_regaddr_color_matrix[0][2].mtx_cbb_adj1 = &VDC50.ADJ1_MTX_CBB_ADJ1;
	vdc5_regaddr_color_matrix[0][2].mtx_crr_adj0 = &VDC50.ADJ1_MTX_CRR_ADJ0;
	vdc5_regaddr_color_matrix[0][2].mtx_crr_adj1 = &VDC50.ADJ1_MTX_CRR_ADJ1;

	/* Channel 1 */
	/* Input Controller */
	vdc5_regaddr_color_matrix[1][0].mtx_update = &VDC51.IMGCNT_UPDATE;
	vdc5_regaddr_color_matrix[1][0].mtx_mode = &VDC51.IMGCNT_MTX_MODE;
	vdc5_regaddr_color_matrix[1][0].mtx_yg_adj0 = &VDC51.IMGCNT_MTX_YG_ADJ0;
	vdc5_regaddr_color_matrix[1][0].mtx_yg_adj1 = &VDC51.IMGCNT_MTX_YG_ADJ1;
	vdc5_regaddr_color_matrix[1][0].mtx_cbb_adj0 = &VDC51.IMGCNT_MTX_CBB_ADJ0;
	vdc5_regaddr_color_matrix[1][0].mtx_cbb_adj1 = &VDC51.IMGCNT_MTX_CBB_ADJ1;
	vdc5_regaddr_color_matrix[1][0].mtx_crr_adj0 = &VDC51.IMGCNT_MTX_CRR_ADJ0;
	vdc5_regaddr_color_matrix[1][0].mtx_crr_adj1 = &VDC51.IMGCNT_MTX_CRR_ADJ1;
	/* Image quality improver 0 */
	vdc5_regaddr_color_matrix[1][1].mtx_update = &VDC51.ADJ0_UPDATE;
	vdc5_regaddr_color_matrix[1][1].mtx_mode = &VDC51.ADJ0_MTX_MODE;
	vdc5_regaddr_color_matrix[1][1].mtx_yg_adj0 = &VDC51.ADJ0_MTX_YG_ADJ0;
	vdc5_regaddr_color_matrix[1][1].mtx_yg_adj1 = &VDC51.ADJ0_MTX_YG_ADJ1;
	vdc5_regaddr_color_matrix[1][1].mtx_cbb_adj0 = &VDC51.ADJ0_MTX_CBB_ADJ0;
	vdc5_regaddr_color_matrix[1][1].mtx_cbb_adj1 = &VDC51.ADJ0_MTX_CBB_ADJ1;
	vdc5_regaddr_color_matrix[1][1].mtx_crr_adj0 = &VDC51.ADJ0_MTX_CRR_ADJ0;
	vdc5_regaddr_color_matrix[1][1].mtx_crr_adj1 = &VDC51.ADJ0_MTX_CRR_ADJ1;
	/* Image quality improver 1 */
	vdc5_regaddr_color_matrix[1][2].mtx_update = &VDC51.ADJ1_UPDATE;
	vdc5_regaddr_color_matrix[1][2].mtx_mode = &VDC51.ADJ1_MTX_MODE;
	vdc5_regaddr_color_matrix[1][2].mtx_yg_adj0 = &VDC51.ADJ1_MTX_YG_ADJ0;
	vdc5_regaddr_color_matrix[1][2].mtx_yg_adj1 = &VDC51.ADJ1_MTX_YG_ADJ1;
	vdc5_regaddr_color_matrix[1][2].mtx_cbb_adj0 = &VDC51.ADJ1_MTX_CBB_ADJ0;
	vdc5_regaddr_color_matrix[1][2].mtx_cbb_adj1 = &VDC51.ADJ1_MTX_CBB_ADJ1;
	vdc5_regaddr_color_matrix[1][2].mtx_crr_adj0 = &VDC51.ADJ1_MTX_CRR_ADJ0;
	vdc5_regaddr_color_matrix[1][2].mtx_crr_adj1 = &VDC51.ADJ1_MTX_CRR_ADJ1;
}
#endif
/* VDC5 image synthesizer register address list */
#if 0
const vdc5_regaddr_img_synthesizer_t vdc5_regaddr_img_synthesizer[VDC5_CHANNEL_NUM][VDC5_GR_TYPE_NUM] =
{
    {   /* Channel 0 */
        {   /* GR0 */
            &VDC50.GR0_UPDATE,
            &VDC50.GR0_FLM_RD,
            &VDC50.GR0_FLM1,
            &VDC50.GR0_FLM2,
            &VDC50.GR0_FLM3,
            &VDC50.GR0_FLM4,
            &VDC50.GR0_FLM5,
            &VDC50.GR0_FLM6,
            &VDC50.GR0_AB1,
            &VDC50.GR0_AB2,
            &VDC50.GR0_AB3,
            NULL,
            NULL,
            NULL,
            &VDC50.GR0_AB7,
            &VDC50.GR0_AB8,
            &VDC50.GR0_AB9,
            &VDC50.GR0_AB10,
            &VDC50.GR0_AB11,
            &VDC50.GR0_BASE,
            &VDC50.GR0_CLUT,
            NULL
        },
        {   /* GR1 */
            &VDC50.GR1_UPDATE,
            &VDC50.GR1_FLM_RD,
            &VDC50.GR1_FLM1,
            &VDC50.GR1_FLM2,
            &VDC50.GR1_FLM3,
            &VDC50.GR1_FLM4,
            &VDC50.GR1_FLM5,
            &VDC50.GR1_FLM6,
            &VDC50.GR1_AB1,
            &VDC50.GR1_AB2,
            &VDC50.GR1_AB3,
            &VDC50.GR1_AB4,
            &VDC50.GR1_AB5,
            &VDC50.GR1_AB6,
            &VDC50.GR1_AB7,
            &VDC50.GR1_AB8,
            &VDC50.GR1_AB9,
            &VDC50.GR1_AB10,
            &VDC50.GR1_AB11,
            &VDC50.GR1_BASE,
            &VDC50.GR1_CLUT,
            &VDC50.GR1_MON
        },
        {   /* GR2 */
            &VDC50.GR2_UPDATE,
            &VDC50.GR2_FLM_RD,
            &VDC50.GR2_FLM1,
            &VDC50.GR2_FLM2,
            &VDC50.GR2_FLM3,
            &VDC50.GR2_FLM4,
            &VDC50.GR2_FLM5,
            &VDC50.GR2_FLM6,
            &VDC50.GR2_AB1,
            &VDC50.GR2_AB2,
            &VDC50.GR2_AB3,
            &VDC50.GR2_AB4,
            &VDC50.GR2_AB5,
            &VDC50.GR2_AB6,
            &VDC50.GR2_AB7,
            &VDC50.GR2_AB8,
            &VDC50.GR2_AB9,
            &VDC50.GR2_AB10,
            &VDC50.GR2_AB11,
            &VDC50.GR2_BASE,
            &VDC50.GR2_CLUT,
            &VDC50.GR2_MON
        },
        {   /* GR3 */
            &VDC50.GR3_UPDATE,
            &VDC50.GR3_FLM_RD,
            &VDC50.GR3_FLM1,
            &VDC50.GR3_FLM2,
            &VDC50.GR3_FLM3,
            &VDC50.GR3_FLM4,
            &VDC50.GR3_FLM5,
            &VDC50.GR3_FLM6,
            &VDC50.GR3_AB1,
            &VDC50.GR3_AB2,
            &VDC50.GR3_AB3,
            &VDC50.GR3_AB4,
            &VDC50.GR3_AB5,
            &VDC50.GR3_AB6,
            &VDC50.GR3_AB7,
            &VDC50.GR3_AB8,
            &VDC50.GR3_AB9,
            &VDC50.GR3_AB10,
            &VDC50.GR3_AB11,
            &VDC50.GR3_BASE,
            &VDC50.GR3_CLUT_INT,
            &VDC50.GR3_MON
        },
        {   /* VIN */
            &VDC50.GR_VIN_UPDATE,
            NULL,
            NULL,
            NULL,
            NULL,
            NULL,
            NULL,
            NULL,
            &VDC50.GR_VIN_AB1,
            &VDC50.GR_VIN_AB2,
            &VDC50.GR_VIN_AB3,
            &VDC50.GR_VIN_AB4,
            &VDC50.GR_VIN_AB5,
            &VDC50.GR_VIN_AB6,
            &VDC50.GR_VIN_AB7,
            NULL,
            NULL,
            NULL,
            NULL,
            &VDC50.GR_VIN_BASE,
            NULL,
            &VDC50.GR_VIN_MON
        },
        {   /* OIR */
            &VDC50.GR_OIR_UPDATE,
            &VDC50.GR_OIR_FLM_RD,
            &VDC50.GR_OIR_FLM1,
            &VDC50.GR_OIR_FLM2,
            &VDC50.GR_OIR_FLM3,
            &VDC50.GR_OIR_FLM4,
            &VDC50.GR_OIR_FLM5,
            &VDC50.GR_OIR_FLM6,
            &VDC50.GR_OIR_AB1,
            &VDC50.GR_OIR_AB2,
            &VDC50.GR_OIR_AB3,
            NULL,
            NULL,
            NULL,
            &VDC50.GR_OIR_AB7,
            &VDC50.GR_OIR_AB8,
            &VDC50.GR_OIR_AB9,
            &VDC50.GR_OIR_AB10,
            &VDC50.GR_OIR_AB11,
            &VDC50.GR_OIR_BASE,
            &VDC50.GR_OIR_CLUT,
            &VDC50.GR_OIR_MON
        }
    },
    {   /* Channel 1 */
        {   /* GR0 */
            &VDC51.GR0_UPDATE,
            &VDC51.GR0_FLM_RD,
            &VDC51.GR0_FLM1,
            &VDC51.GR0_FLM2,
            &VDC51.GR0_FLM3,
            &VDC51.GR0_FLM4,
            &VDC51.GR0_FLM5,
            &VDC51.GR0_FLM6,
            &VDC51.GR0_AB1,
            &VDC51.GR0_AB2,
            &VDC51.GR0_AB3,
            NULL,
            NULL,
            NULL,
            &VDC51.GR0_AB7,
            &VDC51.GR0_AB8,
            &VDC51.GR0_AB9,
            &VDC51.GR0_AB10,
            &VDC51.GR0_AB11,
            &VDC51.GR0_BASE,
            &VDC51.GR0_CLUT,
            NULL
        },
        {   /* GR1 */
            &VDC51.GR1_UPDATE,
            &VDC51.GR1_FLM_RD,
            &VDC51.GR1_FLM1,
            &VDC51.GR1_FLM2,
            &VDC51.GR1_FLM3,
            &VDC51.GR1_FLM4,
            &VDC51.GR1_FLM5,
            &VDC51.GR1_FLM6,
            &VDC51.GR1_AB1,
            &VDC51.GR1_AB2,
            &VDC51.GR1_AB3,
            &VDC51.GR1_AB4,
            &VDC51.GR1_AB5,
            &VDC51.GR1_AB6,
            &VDC51.GR1_AB7,
            &VDC51.GR1_AB8,
            &VDC51.GR1_AB9,
            &VDC51.GR1_AB10,
            &VDC51.GR1_AB11,
            &VDC51.GR1_BASE,
            &VDC51.GR1_CLUT,
            &VDC51.GR1_MON
        },
        {   /* GR2 */
            &VDC51.GR2_UPDATE,
            &VDC51.GR2_FLM_RD,
            &VDC51.GR2_FLM1,
            &VDC51.GR2_FLM2,
            &VDC51.GR2_FLM3,
            &VDC51.GR2_FLM4,
            &VDC51.GR2_FLM5,
            &VDC51.GR2_FLM6,
            &VDC51.GR2_AB1,
            &VDC51.GR2_AB2,
            &VDC51.GR2_AB3,
            &VDC51.GR2_AB4,
            &VDC51.GR2_AB5,
            &VDC51.GR2_AB6,
            &VDC51.GR2_AB7,
            &VDC51.GR2_AB8,
            &VDC51.GR2_AB9,
            &VDC51.GR2_AB10,
            &VDC51.GR2_AB11,
            &VDC51.GR2_BASE,
            &VDC51.GR2_CLUT,
            &VDC51.GR2_MON
        },
        {   /* GR3 */
            &VDC51.GR3_UPDATE,
            &VDC51.GR3_FLM_RD,
            &VDC51.GR3_FLM1,
            &VDC51.GR3_FLM2,
            &VDC51.GR3_FLM3,
            &VDC51.GR3_FLM4,
            &VDC51.GR3_FLM5,
            &VDC51.GR3_FLM6,
            &VDC51.GR3_AB1,
            &VDC51.GR3_AB2,
            &VDC51.GR3_AB3,
            &VDC51.GR3_AB4,
            &VDC51.GR3_AB5,
            &VDC51.GR3_AB6,
            &VDC51.GR3_AB7,
            &VDC51.GR3_AB8,
            &VDC51.GR3_AB9,
            &VDC51.GR3_AB10,
            &VDC51.GR3_AB11,
            &VDC51.GR3_BASE,
            &VDC51.GR3_CLUT_INT,
            &VDC51.GR3_MON
        },
        {   /* VIN */
            &VDC51.GR_VIN_UPDATE,
            NULL,
            NULL,
            NULL,
            NULL,
            NULL,
            NULL,
            NULL,
            &VDC51.GR_VIN_AB1,
            &VDC51.GR_VIN_AB2,
            &VDC51.GR_VIN_AB3,
            &VDC51.GR_VIN_AB4,
            &VDC51.GR_VIN_AB5,
            &VDC51.GR_VIN_AB6,
            &VDC51.GR_VIN_AB7,
            NULL,
            NULL,
            NULL,
            NULL,
            &VDC51.GR_VIN_BASE,
            NULL,
            &VDC51.GR_VIN_MON
        },
        {   /* OIR */
            &VDC51.GR_OIR_UPDATE,
            &VDC51.GR_OIR_FLM_RD,
            &VDC51.GR_OIR_FLM1,
            &VDC51.GR_OIR_FLM2,
            &VDC51.GR_OIR_FLM3,
            &VDC51.GR_OIR_FLM4,
            &VDC51.GR_OIR_FLM5,
            &VDC51.GR_OIR_FLM6,
            &VDC51.GR_OIR_AB1,
            &VDC51.GR_OIR_AB2,
            &VDC51.GR_OIR_AB3,
            NULL,
            NULL,
            NULL,
            &VDC51.GR_OIR_AB7,
            &VDC51.GR_OIR_AB8,
            &VDC51.GR_OIR_AB9,
            &VDC51.GR_OIR_AB10,
            &VDC51.GR_OIR_AB11,
            &VDC51.GR_OIR_BASE,
            &VDC51.GR_OIR_CLUT,
            &VDC51.GR_OIR_MON
        }
    }
};
#else
vdc5_regaddr_img_synthesizer_t vdc5_regaddr_img_synthesizer[VDC5_CHANNEL_NUM][VDC5_GR_TYPE_NUM];
static void initialize_vdc5_regaddr_img_synthesizer(void)
{
	/* Channel 0 */
	/* GR0 */
	vdc5_regaddr_img_synthesizer[0][0].gr_update = &VDC50.GR0_UPDATE;
	vdc5_regaddr_img_synthesizer[0][0].gr_flm_rd = &VDC50.GR0_FLM_RD;
	vdc5_regaddr_img_synthesizer[0][0].gr_flm1 = &VDC50.GR0_FLM1;
	vdc5_regaddr_img_synthesizer[0][0].gr_flm2 = &VDC50.GR0_FLM2;
	vdc5_regaddr_img_synthesizer[0][0].gr_flm3 = &VDC50.GR0_FLM3;
	vdc5_regaddr_img_synthesizer[0][0].gr_flm4 = &VDC50.GR0_FLM4;
	vdc5_regaddr_img_synthesizer[0][0].gr_flm5 = &VDC50.GR0_FLM5;
	vdc5_regaddr_img_synthesizer[0][0].gr_flm6 = &VDC50.GR0_FLM6;
	vdc5_regaddr_img_synthesizer[0][0].gr_ab1 = &VDC50.GR0_AB1;
	vdc5_regaddr_img_synthesizer[0][0].gr_ab2 = &VDC50.GR0_AB2;
	vdc5_regaddr_img_synthesizer[0][0].gr_ab3 = &VDC50.GR0_AB3;
	vdc5_regaddr_img_synthesizer[0][0].gr_ab4 = NULL;
	vdc5_regaddr_img_synthesizer[0][0].gr_ab5 = NULL;
	vdc5_regaddr_img_synthesizer[0][0].gr_ab6 = NULL;
	vdc5_regaddr_img_synthesizer[0][0].gr_ab7 = &VDC50.GR0_AB7;
	vdc5_regaddr_img_synthesizer[0][0].gr_ab8 = &VDC50.GR0_AB8;
	vdc5_regaddr_img_synthesizer[0][0].gr_ab9 = &VDC50.GR0_AB9;
	vdc5_regaddr_img_synthesizer[0][0].gr_ab10 = &VDC50.GR0_AB10;
	vdc5_regaddr_img_synthesizer[0][0].gr_ab11 = &VDC50.GR0_AB11;
	vdc5_regaddr_img_synthesizer[0][0].gr_base = &VDC50.GR0_BASE;
	vdc5_regaddr_img_synthesizer[0][0].gr_clut = &VDC50.GR0_CLUT;
	vdc5_regaddr_img_synthesizer[0][0].gr_mon = NULL;
	/* GR1 */
	vdc5_regaddr_img_synthesizer[0][1].gr_update = &VDC50.GR1_UPDATE;
	vdc5_regaddr_img_synthesizer[0][1].gr_flm_rd = &VDC50.GR1_FLM_RD;
	vdc5_regaddr_img_synthesizer[0][1].gr_flm1 = &VDC50.GR1_FLM1;
	vdc5_regaddr_img_synthesizer[0][1].gr_flm2 = &VDC50.GR1_FLM2;
	vdc5_regaddr_img_synthesizer[0][1].gr_flm3 = &VDC50.GR1_FLM3;
	vdc5_regaddr_img_synthesizer[0][1].gr_flm4 = &VDC50.GR1_FLM4;
	vdc5_regaddr_img_synthesizer[0][1].gr_flm5 = &VDC50.GR1_FLM5;
	vdc5_regaddr_img_synthesizer[0][1].gr_flm6 = &VDC50.GR1_FLM6;
	vdc5_regaddr_img_synthesizer[0][1].gr_ab1 = &VDC50.GR1_AB1;
	vdc5_regaddr_img_synthesizer[0][1].gr_ab2 = &VDC50.GR1_AB2;
	vdc5_regaddr_img_synthesizer[0][1].gr_ab3 = &VDC50.GR1_AB3;
	vdc5_regaddr_img_synthesizer[0][1].gr_ab4 = &VDC50.GR1_AB4;
	vdc5_regaddr_img_synthesizer[0][1].gr_ab5 = &VDC50.GR1_AB5;
	vdc5_regaddr_img_synthesizer[0][1].gr_ab6 = &VDC50.GR1_AB6;
	vdc5_regaddr_img_synthesizer[0][1].gr_ab7 = &VDC50.GR1_AB7;
	vdc5_regaddr_img_synthesizer[0][1].gr_ab8 = &VDC50.GR1_AB8;
	vdc5_regaddr_img_synthesizer[0][1].gr_ab9 = &VDC50.GR1_AB9;
	vdc5_regaddr_img_synthesizer[0][1].gr_ab10 = &VDC50.GR1_AB10;
	vdc5_regaddr_img_synthesizer[0][1].gr_ab11 = &VDC50.GR1_AB11;
	vdc5_regaddr_img_synthesizer[0][1].gr_base = &VDC50.GR1_BASE;
	vdc5_regaddr_img_synthesizer[0][1].gr_clut = &VDC50.GR1_CLUT;
	vdc5_regaddr_img_synthesizer[0][1].gr_mon = &VDC50.GR1_MON;
	/* GR2 */
	vdc5_regaddr_img_synthesizer[0][2].gr_update = &VDC50.GR2_UPDATE;
	vdc5_regaddr_img_synthesizer[0][2].gr_flm_rd = &VDC50.GR2_FLM_RD;
	vdc5_regaddr_img_synthesizer[0][2].gr_flm1 = &VDC50.GR2_FLM1;
	vdc5_regaddr_img_synthesizer[0][2].gr_flm2 = &VDC50.GR2_FLM2;
	vdc5_regaddr_img_synthesizer[0][2].gr_flm3 = &VDC50.GR2_FLM3;
	vdc5_regaddr_img_synthesizer[0][2].gr_flm4 = &VDC50.GR2_FLM4;
	vdc5_regaddr_img_synthesizer[0][2].gr_flm5 = &VDC50.GR2_FLM5;
	vdc5_regaddr_img_synthesizer[0][2].gr_flm6 = &VDC50.GR2_FLM6;
	vdc5_regaddr_img_synthesizer[0][2].gr_ab1 = &VDC50.GR2_AB1;
	vdc5_regaddr_img_synthesizer[0][2].gr_ab2 = &VDC50.GR2_AB2;
	vdc5_regaddr_img_synthesizer[0][2].gr_ab3 = &VDC50.GR2_AB3;
	vdc5_regaddr_img_synthesizer[0][2].gr_ab4 = &VDC50.GR2_AB4;
	vdc5_regaddr_img_synthesizer[0][2].gr_ab5 = &VDC50.GR2_AB5;
	vdc5_regaddr_img_synthesizer[0][2].gr_ab6 = &VDC50.GR2_AB6;
	vdc5_regaddr_img_synthesizer[0][2].gr_ab7 = &VDC50.GR2_AB7;
	vdc5_regaddr_img_synthesizer[0][2].gr_ab8 = &VDC50.GR2_AB8;
	vdc5_regaddr_img_synthesizer[0][2].gr_ab9 = &VDC50.GR2_AB9;
	vdc5_regaddr_img_synthesizer[0][2].gr_ab10 = &VDC50.GR2_AB10;
	vdc5_regaddr_img_synthesizer[0][2].gr_ab11 = &VDC50.GR2_AB11;
	vdc5_regaddr_img_synthesizer[0][2].gr_base = &VDC50.GR2_BASE;
	vdc5_regaddr_img_synthesizer[0][2].gr_clut = &VDC50.GR2_CLUT;
	vdc5_regaddr_img_synthesizer[0][2].gr_mon = &VDC50.GR2_MON;
	/* GR3 */
	vdc5_regaddr_img_synthesizer[0][3].gr_update = &VDC50.GR3_UPDATE;
	vdc5_regaddr_img_synthesizer[0][3].gr_flm_rd = &VDC50.GR3_FLM_RD;
	vdc5_regaddr_img_synthesizer[0][3].gr_flm1 = &VDC50.GR3_FLM1;
	vdc5_regaddr_img_synthesizer[0][3].gr_flm2 = &VDC50.GR3_FLM2;
	vdc5_regaddr_img_synthesizer[0][3].gr_flm3 = &VDC50.GR3_FLM3;
	vdc5_regaddr_img_synthesizer[0][3].gr_flm4 = &VDC50.GR3_FLM4;
	vdc5_regaddr_img_synthesizer[0][3].gr_flm5 = &VDC50.GR3_FLM5;
	vdc5_regaddr_img_synthesizer[0][3].gr_flm6 = &VDC50.GR3_FLM6;
	vdc5_regaddr_img_synthesizer[0][3].gr_ab1 = &VDC50.GR3_AB1;
	vdc5_regaddr_img_synthesizer[0][3].gr_ab2 = &VDC50.GR3_AB2;
	vdc5_regaddr_img_synthesizer[0][3].gr_ab3 = &VDC50.GR3_AB3;
	vdc5_regaddr_img_synthesizer[0][3].gr_ab4 = &VDC50.GR3_AB4;
	vdc5_regaddr_img_synthesizer[0][3].gr_ab5 = &VDC50.GR3_AB5;
	vdc5_regaddr_img_synthesizer[0][3].gr_ab6 = &VDC50.GR3_AB6;
	vdc5_regaddr_img_synthesizer[0][3].gr_ab7 = &VDC50.GR3_AB7;
	vdc5_regaddr_img_synthesizer[0][3].gr_ab8 = &VDC50.GR3_AB8;
	vdc5_regaddr_img_synthesizer[0][3].gr_ab9 = &VDC50.GR3_AB9;
	vdc5_regaddr_img_synthesizer[0][3].gr_ab10 = &VDC50.GR3_AB10;
	vdc5_regaddr_img_synthesizer[0][3].gr_ab11 = &VDC50.GR3_AB11;
	vdc5_regaddr_img_synthesizer[0][3].gr_base = &VDC50.GR3_BASE;
	vdc5_regaddr_img_synthesizer[0][3].gr_clut = &VDC50.GR3_CLUT_INT;
	vdc5_regaddr_img_synthesizer[0][3].gr_mon = &VDC50.GR3_MON;
	/* VIN */
	vdc5_regaddr_img_synthesizer[0][4].gr_update = &VDC50.GR_VIN_UPDATE;
	vdc5_regaddr_img_synthesizer[0][4].gr_flm_rd = NULL;
	vdc5_regaddr_img_synthesizer[0][4].gr_flm1 = NULL;
	vdc5_regaddr_img_synthesizer[0][4].gr_flm2 = NULL;
	vdc5_regaddr_img_synthesizer[0][4].gr_flm3 = NULL;
	vdc5_regaddr_img_synthesizer[0][4].gr_flm4 = NULL;
	vdc5_regaddr_img_synthesizer[0][4].gr_flm5 = NULL;
	vdc5_regaddr_img_synthesizer[0][4].gr_flm6 = NULL;
	vdc5_regaddr_img_synthesizer[0][4].gr_ab1 = &VDC50.GR_VIN_AB1;
	vdc5_regaddr_img_synthesizer[0][4].gr_ab2 = &VDC50.GR_VIN_AB2;
	vdc5_regaddr_img_synthesizer[0][4].gr_ab3 = &VDC50.GR_VIN_AB3;
	vdc5_regaddr_img_synthesizer[0][4].gr_ab4 = &VDC50.GR_VIN_AB4;
	vdc5_regaddr_img_synthesizer[0][4].gr_ab5 = &VDC50.GR_VIN_AB5;
	vdc5_regaddr_img_synthesizer[0][4].gr_ab6 = &VDC50.GR_VIN_AB6;
	vdc5_regaddr_img_synthesizer[0][4].gr_ab7 = &VDC50.GR_VIN_AB7;
	vdc5_regaddr_img_synthesizer[0][4].gr_ab8 = NULL;
	vdc5_regaddr_img_synthesizer[0][4].gr_ab9 = NULL;
	vdc5_regaddr_img_synthesizer[0][4].gr_ab10 = NULL;
	vdc5_regaddr_img_synthesizer[0][4].gr_ab11 = NULL;
	vdc5_regaddr_img_synthesizer[0][4].gr_base = &VDC50.GR_VIN_BASE;
	vdc5_regaddr_img_synthesizer[0][4].gr_clut = NULL;
	vdc5_regaddr_img_synthesizer[0][4].gr_mon = &VDC50.GR_VIN_MON;
	/* OIR */
	vdc5_regaddr_img_synthesizer[0][5].gr_update = &VDC50.GR_OIR_UPDATE;
	vdc5_regaddr_img_synthesizer[0][5].gr_flm_rd = &VDC50.GR_OIR_FLM_RD;
	vdc5_regaddr_img_synthesizer[0][5].gr_flm1 = &VDC50.GR_OIR_FLM1;
	vdc5_regaddr_img_synthesizer[0][5].gr_flm2 = &VDC50.GR_OIR_FLM2;
	vdc5_regaddr_img_synthesizer[0][5].gr_flm3 = &VDC50.GR_OIR_FLM3;
	vdc5_regaddr_img_synthesizer[0][5].gr_flm4 = &VDC50.GR_OIR_FLM4;
	vdc5_regaddr_img_synthesizer[0][5].gr_flm5 = &VDC50.GR_OIR_FLM5;
	vdc5_regaddr_img_synthesizer[0][5].gr_flm6 = &VDC50.GR_OIR_FLM6;
	vdc5_regaddr_img_synthesizer[0][5].gr_ab1 = &VDC50.GR_OIR_AB1;
	vdc5_regaddr_img_synthesizer[0][5].gr_ab2 = &VDC50.GR_OIR_AB2;
	vdc5_regaddr_img_synthesizer[0][5].gr_ab3 = &VDC50.GR_OIR_AB3;
	vdc5_regaddr_img_synthesizer[0][5].gr_ab4 = NULL;
	vdc5_regaddr_img_synthesizer[0][5].gr_ab5 = NULL;
	vdc5_regaddr_img_synthesizer[0][5].gr_ab6 = NULL;
	vdc5_regaddr_img_synthesizer[0][5].gr_ab7 = &VDC50.GR_OIR_AB7;
	vdc5_regaddr_img_synthesizer[0][5].gr_ab8 = &VDC50.GR_OIR_AB8;
	vdc5_regaddr_img_synthesizer[0][5].gr_ab9 = &VDC50.GR_OIR_AB9;
	vdc5_regaddr_img_synthesizer[0][5].gr_ab10 = &VDC50.GR_OIR_AB10;
	vdc5_regaddr_img_synthesizer[0][5].gr_ab11 = &VDC50.GR_OIR_AB11;
	vdc5_regaddr_img_synthesizer[0][5].gr_base = &VDC50.GR_OIR_BASE;
	vdc5_regaddr_img_synthesizer[0][5].gr_clut = &VDC50.GR_OIR_CLUT;
	vdc5_regaddr_img_synthesizer[0][5].gr_mon = &VDC50.GR_OIR_MON;

	/* Channel 1 */
	/* GR0 */
	vdc5_regaddr_img_synthesizer[1][0].gr_update = &VDC51.GR0_UPDATE;
	vdc5_regaddr_img_synthesizer[1][0].gr_flm_rd = &VDC51.GR0_FLM_RD;
	vdc5_regaddr_img_synthesizer[1][0].gr_flm1 = &VDC51.GR0_FLM1;
	vdc5_regaddr_img_synthesizer[1][0].gr_flm2 = &VDC51.GR0_FLM2;
	vdc5_regaddr_img_synthesizer[1][0].gr_flm3 = &VDC51.GR0_FLM3;
	vdc5_regaddr_img_synthesizer[1][0].gr_flm4 = &VDC51.GR0_FLM4;
	vdc5_regaddr_img_synthesizer[1][0].gr_flm5 = &VDC51.GR0_FLM5;
	vdc5_regaddr_img_synthesizer[1][0].gr_flm6 = &VDC51.GR0_FLM6;
	vdc5_regaddr_img_synthesizer[1][0].gr_ab1 = &VDC51.GR0_AB1;
	vdc5_regaddr_img_synthesizer[1][0].gr_ab2 = &VDC51.GR0_AB2;
	vdc5_regaddr_img_synthesizer[1][0].gr_ab3 = &VDC51.GR0_AB3;
	vdc5_regaddr_img_synthesizer[1][0].gr_ab4 = NULL;
	vdc5_regaddr_img_synthesizer[1][0].gr_ab5 = NULL;
	vdc5_regaddr_img_synthesizer[1][0].gr_ab6 = NULL;
	vdc5_regaddr_img_synthesizer[1][0].gr_ab7 = &VDC51.GR0_AB7;
	vdc5_regaddr_img_synthesizer[1][0].gr_ab8 = &VDC51.GR0_AB8;
	vdc5_regaddr_img_synthesizer[1][0].gr_ab9 = &VDC51.GR0_AB9;
	vdc5_regaddr_img_synthesizer[1][0].gr_ab10 = &VDC51.GR0_AB10;
	vdc5_regaddr_img_synthesizer[1][0].gr_ab11 = &VDC51.GR0_AB11;
	vdc5_regaddr_img_synthesizer[1][0].gr_base = &VDC51.GR0_BASE;
	vdc5_regaddr_img_synthesizer[1][0].gr_clut = &VDC51.GR0_CLUT;
	vdc5_regaddr_img_synthesizer[1][0].gr_mon = NULL;
	/* GR1 */
	vdc5_regaddr_img_synthesizer[1][1].gr_update = &VDC51.GR1_UPDATE;
	vdc5_regaddr_img_synthesizer[1][1].gr_flm_rd = &VDC51.GR1_FLM_RD;
	vdc5_regaddr_img_synthesizer[1][1].gr_flm1 = &VDC51.GR1_FLM1;
	vdc5_regaddr_img_synthesizer[1][1].gr_flm2 = &VDC51.GR1_FLM2;
	vdc5_regaddr_img_synthesizer[1][1].gr_flm3 = &VDC51.GR1_FLM3;
	vdc5_regaddr_img_synthesizer[1][1].gr_flm4 = &VDC51.GR1_FLM4;
	vdc5_regaddr_img_synthesizer[1][1].gr_flm5 = &VDC51.GR1_FLM5;
	vdc5_regaddr_img_synthesizer[1][1].gr_flm6 = &VDC51.GR1_FLM6;
	vdc5_regaddr_img_synthesizer[1][1].gr_ab1 = &VDC51.GR1_AB1;
	vdc5_regaddr_img_synthesizer[1][1].gr_ab2 = &VDC51.GR1_AB2;
	vdc5_regaddr_img_synthesizer[1][1].gr_ab3 = &VDC51.GR1_AB3;
	vdc5_regaddr_img_synthesizer[1][1].gr_ab4 = &VDC51.GR1_AB4;
	vdc5_regaddr_img_synthesizer[1][1].gr_ab5 = &VDC51.GR1_AB5;
	vdc5_regaddr_img_synthesizer[1][1].gr_ab6 = &VDC51.GR1_AB6;
	vdc5_regaddr_img_synthesizer[1][1].gr_ab7 = &VDC51.GR1_AB7;
	vdc5_regaddr_img_synthesizer[1][1].gr_ab8 = &VDC51.GR1_AB8;
	vdc5_regaddr_img_synthesizer[1][1].gr_ab9 = &VDC51.GR1_AB9;
	vdc5_regaddr_img_synthesizer[1][1].gr_ab10 = &VDC51.GR1_AB10;
	vdc5_regaddr_img_synthesizer[1][1].gr_ab11 = &VDC51.GR1_AB11;
	vdc5_regaddr_img_synthesizer[1][1].gr_base = &VDC51.GR1_BASE;
	vdc5_regaddr_img_synthesizer[1][1].gr_clut = &VDC51.GR1_CLUT;
	vdc5_regaddr_img_synthesizer[1][1].gr_mon = &VDC51.GR1_MON;
	/* GR2 */
	vdc5_regaddr_img_synthesizer[1][2].gr_update = &VDC51.GR2_UPDATE;
	vdc5_regaddr_img_synthesizer[1][2].gr_flm_rd = &VDC51.GR2_FLM_RD;
	vdc5_regaddr_img_synthesizer[1][2].gr_flm1 = &VDC51.GR2_FLM1;
	vdc5_regaddr_img_synthesizer[1][2].gr_flm2 = &VDC51.GR2_FLM2;
	vdc5_regaddr_img_synthesizer[1][2].gr_flm3 = &VDC51.GR2_FLM3;
	vdc5_regaddr_img_synthesizer[1][2].gr_flm4 = &VDC51.GR2_FLM4;
	vdc5_regaddr_img_synthesizer[1][2].gr_flm5 = &VDC51.GR2_FLM5;
	vdc5_regaddr_img_synthesizer[1][2].gr_flm6 = &VDC51.GR2_FLM6;
	vdc5_regaddr_img_synthesizer[1][2].gr_ab1 = &VDC51.GR2_AB1;
	vdc5_regaddr_img_synthesizer[1][2].gr_ab2 = &VDC51.GR2_AB2;
	vdc5_regaddr_img_synthesizer[1][2].gr_ab3 = &VDC51.GR2_AB3;
	vdc5_regaddr_img_synthesizer[1][2].gr_ab4 = &VDC51.GR2_AB4;
	vdc5_regaddr_img_synthesizer[1][2].gr_ab5 = &VDC51.GR2_AB5;
	vdc5_regaddr_img_synthesizer[1][2].gr_ab6 = &VDC51.GR2_AB6;
	vdc5_regaddr_img_synthesizer[1][2].gr_ab7 = &VDC51.GR2_AB7;
	vdc5_regaddr_img_synthesizer[1][2].gr_ab8 = &VDC51.GR2_AB8;
	vdc5_regaddr_img_synthesizer[1][2].gr_ab9 = &VDC51.GR2_AB9;
	vdc5_regaddr_img_synthesizer[1][2].gr_ab10 = &VDC51.GR2_AB10;
	vdc5_regaddr_img_synthesizer[1][2].gr_ab11 = &VDC51.GR2_AB11;
	vdc5_regaddr_img_synthesizer[1][2].gr_base = &VDC51.GR2_BASE;
	vdc5_regaddr_img_synthesizer[1][2].gr_clut = &VDC51.GR2_CLUT;
	vdc5_regaddr_img_synthesizer[1][2].gr_mon = &VDC51.GR2_MON;
	/* GR3 */
	vdc5_regaddr_img_synthesizer[1][3].gr_update = &VDC51.GR3_UPDATE;
	vdc5_regaddr_img_synthesizer[1][3].gr_flm_rd = &VDC51.GR3_FLM_RD;
	vdc5_regaddr_img_synthesizer[1][3].gr_flm1 = &VDC51.GR3_FLM1;
	vdc5_regaddr_img_synthesizer[1][3].gr_flm2 = &VDC51.GR3_FLM2;
	vdc5_regaddr_img_synthesizer[1][3].gr_flm3 = &VDC51.GR3_FLM3;
	vdc5_regaddr_img_synthesizer[1][3].gr_flm4 = &VDC51.GR3_FLM4;
	vdc5_regaddr_img_synthesizer[1][3].gr_flm5 = &VDC51.GR3_FLM5;
	vdc5_regaddr_img_synthesizer[1][3].gr_flm6 = &VDC51.GR3_FLM6;
	vdc5_regaddr_img_synthesizer[1][3].gr_ab1 = &VDC51.GR3_AB1;
	vdc5_regaddr_img_synthesizer[1][3].gr_ab2 = &VDC51.GR3_AB2;
	vdc5_regaddr_img_synthesizer[1][3].gr_ab3 = &VDC51.GR3_AB3;
	vdc5_regaddr_img_synthesizer[1][3].gr_ab4 = &VDC51.GR3_AB4;
	vdc5_regaddr_img_synthesizer[1][3].gr_ab5 = &VDC51.GR3_AB5;
	vdc5_regaddr_img_synthesizer[1][3].gr_ab6 = &VDC51.GR3_AB6;
	vdc5_regaddr_img_synthesizer[1][3].gr_ab7 = &VDC51.GR3_AB7;
	vdc5_regaddr_img_synthesizer[1][3].gr_ab8 = &VDC51.GR3_AB8;
	vdc5_regaddr_img_synthesizer[1][3].gr_ab9 = &VDC51.GR3_AB9;
	vdc5_regaddr_img_synthesizer[1][3].gr_ab10 = &VDC51.GR3_AB10;
	vdc5_regaddr_img_synthesizer[1][3].gr_ab11 = &VDC51.GR3_AB11;
	vdc5_regaddr_img_synthesizer[1][3].gr_base = &VDC51.GR3_BASE;
	vdc5_regaddr_img_synthesizer[1][3].gr_clut = &VDC51.GR3_CLUT_INT;
	vdc5_regaddr_img_synthesizer[1][3].gr_mon = &VDC51.GR3_MON;
	/* VIN */
	vdc5_regaddr_img_synthesizer[1][4].gr_update = &VDC51.GR_VIN_UPDATE;
	vdc5_regaddr_img_synthesizer[1][4].gr_flm_rd = NULL;
	vdc5_regaddr_img_synthesizer[1][4].gr_flm1 = NULL;
	vdc5_regaddr_img_synthesizer[1][4].gr_flm2 = NULL;
	vdc5_regaddr_img_synthesizer[1][4].gr_flm3 = NULL;
	vdc5_regaddr_img_synthesizer[1][4].gr_flm4 = NULL;
	vdc5_regaddr_img_synthesizer[1][4].gr_flm5 = NULL;
	vdc5_regaddr_img_synthesizer[1][4].gr_flm6 = NULL;
	vdc5_regaddr_img_synthesizer[1][4].gr_ab1 = &VDC51.GR_VIN_AB1;
	vdc5_regaddr_img_synthesizer[1][4].gr_ab2 = &VDC51.GR_VIN_AB2;
	vdc5_regaddr_img_synthesizer[1][4].gr_ab3 = &VDC51.GR_VIN_AB3;
	vdc5_regaddr_img_synthesizer[1][4].gr_ab4 = &VDC51.GR_VIN_AB4;
	vdc5_regaddr_img_synthesizer[1][4].gr_ab5 = &VDC51.GR_VIN_AB5;
	vdc5_regaddr_img_synthesizer[1][4].gr_ab6 = &VDC51.GR_VIN_AB6;
	vdc5_regaddr_img_synthesizer[1][4].gr_ab7 = &VDC51.GR_VIN_AB7;
	vdc5_regaddr_img_synthesizer[1][4].gr_ab8 = NULL;
	vdc5_regaddr_img_synthesizer[1][4].gr_ab9 = NULL;
	vdc5_regaddr_img_synthesizer[1][4].gr_ab10 = NULL;
	vdc5_regaddr_img_synthesizer[1][4].gr_ab11 = NULL;
	vdc5_regaddr_img_synthesizer[1][4].gr_base = &VDC51.GR_VIN_BASE;
	vdc5_regaddr_img_synthesizer[1][4].gr_clut = NULL;
	vdc5_regaddr_img_synthesizer[1][4].gr_mon = &VDC51.GR_VIN_MON;
	/* OIR */
	vdc5_regaddr_img_synthesizer[1][5].gr_update = &VDC51.GR_OIR_UPDATE;
	vdc5_regaddr_img_synthesizer[1][5].gr_flm_rd = &VDC51.GR_OIR_FLM_RD;
	vdc5_regaddr_img_synthesizer[1][5].gr_flm1 = &VDC51.GR_OIR_FLM1;
	vdc5_regaddr_img_synthesizer[1][5].gr_flm2 = &VDC51.GR_OIR_FLM2;
	vdc5_regaddr_img_synthesizer[1][5].gr_flm3 = &VDC51.GR_OIR_FLM3;
	vdc5_regaddr_img_synthesizer[1][5].gr_flm4 = &VDC51.GR_OIR_FLM4;
	vdc5_regaddr_img_synthesizer[1][5].gr_flm5 = &VDC51.GR_OIR_FLM5;
	vdc5_regaddr_img_synthesizer[1][5].gr_flm6 = &VDC51.GR_OIR_FLM6;
	vdc5_regaddr_img_synthesizer[1][5].gr_ab1 = &VDC51.GR_OIR_AB1;
	vdc5_regaddr_img_synthesizer[1][5].gr_ab2 = &VDC51.GR_OIR_AB2;
	vdc5_regaddr_img_synthesizer[1][5].gr_ab3 = &VDC51.GR_OIR_AB3;
	vdc5_regaddr_img_synthesizer[1][5].gr_ab4 = NULL;
	vdc5_regaddr_img_synthesizer[1][5].gr_ab5 = NULL;
	vdc5_regaddr_img_synthesizer[1][5].gr_ab6 = NULL;
	vdc5_regaddr_img_synthesizer[1][5].gr_ab7 = &VDC51.GR_OIR_AB7;
	vdc5_regaddr_img_synthesizer[1][5].gr_ab8 = &VDC51.GR_OIR_AB8;
	vdc5_regaddr_img_synthesizer[1][5].gr_ab9 = &VDC51.GR_OIR_AB9;
	vdc5_regaddr_img_synthesizer[1][5].gr_ab10 = &VDC51.GR_OIR_AB10;
	vdc5_regaddr_img_synthesizer[1][5].gr_ab11 = &VDC51.GR_OIR_AB11;
	vdc5_regaddr_img_synthesizer[1][5].gr_base = &VDC51.GR_OIR_BASE;
	vdc5_regaddr_img_synthesizer[1][5].gr_clut = &VDC51.GR_OIR_CLUT;
	vdc5_regaddr_img_synthesizer[1][5].gr_mon = &VDC51.GR_OIR_MON;
}
#endif

/* VDC5 CLUT register address list */
#if 0
uint32_t * const vdc5_regaddr_clut[VDC5_CHANNEL_NUM][VDC5_GR_TYPE_NUM] =
{
    {   /* Channel 0 */
        &VDC5_CH0_GR0_CLUT_TBL,
        &VDC5_CH0_GR1_CLUT_TBL,
        &VDC5_CH0_GR2_CLUT_TBL,
        &VDC5_CH0_GR3_CLUT_TBL,
        NULL,
        &VDC5_CH0_GR_OIR_CLUT_TBL
    },
    {   /* Channel 1 */
        &VDC5_CH1_GR0_CLUT_TBL,
        &VDC5_CH1_GR1_CLUT_TBL,
        &VDC5_CH1_GR2_CLUT_TBL,
        &VDC5_CH1_GR3_CLUT_TBL,
        NULL,
        &VDC5_CH1_GR_OIR_CLUT_TBL
    }
};
#else
uint32_t *vdc5_regaddr_clut[VDC5_CHANNEL_NUM][VDC5_GR_TYPE_NUM];

static void initialize_vdc5_regaddr_clut(void)
{
	/* Channel 0 */
	vdc5_regaddr_clut[0][0] = &VDC5_CH0_GR0_CLUT_TBL;
	vdc5_regaddr_clut[0][1] = &VDC5_CH0_GR1_CLUT_TBL;
	vdc5_regaddr_clut[0][2] = &VDC5_CH0_GR2_CLUT_TBL;
	vdc5_regaddr_clut[0][3] = &VDC5_CH0_GR3_CLUT_TBL;
	vdc5_regaddr_clut[0][4] = NULL;
	vdc5_regaddr_clut[0][5] = &VDC5_CH0_GR_OIR_CLUT_TBL;
	/* Channel 1 */
	vdc5_regaddr_clut[1][0] = &VDC5_CH1_GR0_CLUT_TBL;
	vdc5_regaddr_clut[1][1] = &VDC5_CH1_GR1_CLUT_TBL;
	vdc5_regaddr_clut[1][2] = &VDC5_CH1_GR2_CLUT_TBL;
	vdc5_regaddr_clut[1][3] = &VDC5_CH1_GR3_CLUT_TBL;
	vdc5_regaddr_clut[1][4] = NULL;
	vdc5_regaddr_clut[1][5] = &VDC5_CH1_GR_OIR_CLUT_TBL;
}
#endif

/* VDC5 output controller register address list */
#if 0
const vdc5_regaddr_output_ctrl_t vdc5_regaddr_output_ctrl[VDC5_CHANNEL_NUM] =
{
    {   /* Channel 0 */
        &VDC50.TCON_UPDATE,
        &VDC50.TCON_TIM,
        &VDC50.TCON_TIM_STVA1,
        &VDC50.TCON_TIM_STVA2,
        &VDC50.TCON_TIM_STVB1,
        &VDC50.TCON_TIM_STVB2,
        &VDC50.TCON_TIM_STH1,
        &VDC50.TCON_TIM_STH2,
        &VDC50.TCON_TIM_STB1,
        &VDC50.TCON_TIM_STB2,
        &VDC50.TCON_TIM_CPV1,
        &VDC50.TCON_TIM_CPV2,
        &VDC50.TCON_TIM_POLA1,
        &VDC50.TCON_TIM_POLA2,
        &VDC50.TCON_TIM_POLB1,
        &VDC50.TCON_TIM_POLB2,
        &VDC50.TCON_TIM_DE,
        &VDC50.OUT_UPDATE,
        &VDC50.OUT_SET,
        &VDC50.OUT_BRIGHT1,
        &VDC50.OUT_BRIGHT2,
        &VDC50.OUT_CONTRAST,
        &VDC50.OUT_PDTHA,
        &VDC50.OUT_CLK_PHASE
    },
    {   /* Channel 1 */
        &VDC51.TCON_UPDATE,
        &VDC51.TCON_TIM,
        &VDC51.TCON_TIM_STVA1,
        &VDC51.TCON_TIM_STVA2,
        &VDC51.TCON_TIM_STVB1,
        &VDC51.TCON_TIM_STVB2,
        &VDC51.TCON_TIM_STH1,
        &VDC51.TCON_TIM_STH2,
        &VDC51.TCON_TIM_STB1,
        &VDC51.TCON_TIM_STB2,
        &VDC51.TCON_TIM_CPV1,
        &VDC51.TCON_TIM_CPV2,
        &VDC51.TCON_TIM_POLA1,
        &VDC51.TCON_TIM_POLA2,
        &VDC51.TCON_TIM_POLB1,
        &VDC51.TCON_TIM_POLB2,
        &VDC51.TCON_TIM_DE,
        &VDC51.OUT_UPDATE,
        &VDC51.OUT_SET,
        &VDC51.OUT_BRIGHT1,
        &VDC51.OUT_BRIGHT2,
        &VDC51.OUT_CONTRAST,
        &VDC51.OUT_PDTHA,
        &VDC51.OUT_CLK_PHASE
    }
};
#else
vdc5_regaddr_output_ctrl_t vdc5_regaddr_output_ctrl[VDC5_CHANNEL_NUM];
static void initialize_vdc5_regaddr_output_ctrl(void)
{
	/* Channel 0 */
	vdc5_regaddr_output_ctrl[0].tcon_update = &VDC50.TCON_UPDATE;
	vdc5_regaddr_output_ctrl[0].tcon_tim = &VDC50.TCON_TIM;
	vdc5_regaddr_output_ctrl[0].tcon_tim_stva1 = &VDC50.TCON_TIM_STVA1;
	vdc5_regaddr_output_ctrl[0].tcon_tim_stva2 = &VDC50.TCON_TIM_STVA2;
	vdc5_regaddr_output_ctrl[0].tcon_tim_stvb1 = &VDC50.TCON_TIM_STVB1;
	vdc5_regaddr_output_ctrl[0].tcon_tim_stvb2 = &VDC50.TCON_TIM_STVB2;
	vdc5_regaddr_output_ctrl[0].tcon_tim_sth1 = &VDC50.TCON_TIM_STH1;
	vdc5_regaddr_output_ctrl[0].tcon_tim_sth2 = &VDC50.TCON_TIM_STH2;
	vdc5_regaddr_output_ctrl[0].tcon_tim_stb1 = &VDC50.TCON_TIM_STB1;
	vdc5_regaddr_output_ctrl[0].tcon_tim_stb2 = &VDC50.TCON_TIM_STB2;
	vdc5_regaddr_output_ctrl[0].tcon_tim_cpv1 = &VDC50.TCON_TIM_CPV1;
	vdc5_regaddr_output_ctrl[0].tcon_tim_cpv2 = &VDC50.TCON_TIM_CPV2;
	vdc5_regaddr_output_ctrl[0].tcon_tim_pola1= &VDC50.TCON_TIM_POLA1;
	vdc5_regaddr_output_ctrl[0].tcon_tim_pola2 = &VDC50.TCON_TIM_POLA2;
	vdc5_regaddr_output_ctrl[0].tcon_tim_polb1 = &VDC50.TCON_TIM_POLB1;
	vdc5_regaddr_output_ctrl[0].tcon_tim_polb2 = &VDC50.TCON_TIM_POLB2;
	vdc5_regaddr_output_ctrl[0].tcon_tim_de = &VDC50.TCON_TIM_DE;
	vdc5_regaddr_output_ctrl[0].out_update = &VDC50.OUT_UPDATE;
	vdc5_regaddr_output_ctrl[0].out_set = &VDC50.OUT_SET;
	vdc5_regaddr_output_ctrl[0].out_bright1 = &VDC50.OUT_BRIGHT1;
	vdc5_regaddr_output_ctrl[0].out_bright2 = &VDC50.OUT_BRIGHT2;
	vdc5_regaddr_output_ctrl[0].out_contrast = &VDC50.OUT_CONTRAST;
	vdc5_regaddr_output_ctrl[0].out_pdtha = &VDC50.OUT_PDTHA;
	vdc5_regaddr_output_ctrl[0].out_clk_phase = &VDC50.OUT_CLK_PHASE;
	/* Channel 1 */
	vdc5_regaddr_output_ctrl[1].tcon_update = &VDC51.TCON_UPDATE;
	vdc5_regaddr_output_ctrl[1].tcon_tim = &VDC51.TCON_TIM;
	vdc5_regaddr_output_ctrl[1].tcon_tim_stva1 = &VDC51.TCON_TIM_STVA1;
	vdc5_regaddr_output_ctrl[1].tcon_tim_stva2 = &VDC51.TCON_TIM_STVA2;
	vdc5_regaddr_output_ctrl[1].tcon_tim_stvb1 = &VDC51.TCON_TIM_STVB1;
	vdc5_regaddr_output_ctrl[1].tcon_tim_stvb2 = &VDC51.TCON_TIM_STVB2;
	vdc5_regaddr_output_ctrl[1].tcon_tim_sth1 = &VDC51.TCON_TIM_STH1;
	vdc5_regaddr_output_ctrl[1].tcon_tim_sth2 = &VDC51.TCON_TIM_STH2;
	vdc5_regaddr_output_ctrl[1].tcon_tim_stb1 = &VDC51.TCON_TIM_STB1;
	vdc5_regaddr_output_ctrl[1].tcon_tim_stb2 = &VDC51.TCON_TIM_STB2;
	vdc5_regaddr_output_ctrl[1].tcon_tim_cpv1 = &VDC51.TCON_TIM_CPV1;
	vdc5_regaddr_output_ctrl[1].tcon_tim_cpv2 = &VDC51.TCON_TIM_CPV2;
	vdc5_regaddr_output_ctrl[1].tcon_tim_pola1 = &VDC51.TCON_TIM_POLA1;
	vdc5_regaddr_output_ctrl[1].tcon_tim_pola2 = &VDC51.TCON_TIM_POLA2;
	vdc5_regaddr_output_ctrl[1].tcon_tim_polb1 = &VDC51.TCON_TIM_POLB1;
	vdc5_regaddr_output_ctrl[1].tcon_tim_polb2 = &VDC51.TCON_TIM_POLB2;
	vdc5_regaddr_output_ctrl[1].tcon_tim_de = &VDC51.TCON_TIM_DE;
	vdc5_regaddr_output_ctrl[1].out_update = &VDC51.OUT_UPDATE;
	vdc5_regaddr_output_ctrl[1].out_set = &VDC51.OUT_SET;
	vdc5_regaddr_output_ctrl[1].out_bright1 = &VDC51.OUT_BRIGHT1;
	vdc5_regaddr_output_ctrl[1].out_bright2 = &VDC51.OUT_BRIGHT2;
	vdc5_regaddr_output_ctrl[1].out_contrast = &VDC51.OUT_CONTRAST;
	vdc5_regaddr_output_ctrl[1].out_pdtha = &VDC51.OUT_PDTHA;
	vdc5_regaddr_output_ctrl[1].out_clk_phase = &VDC51.OUT_CLK_PHASE;
}
#endif
/* VDC5 gamma correction register address list */
#if 0
const vdc5_regaddr_gamma_t vdc5_regaddr_gamma[VDC5_CHANNEL_NUM] =
{
    {   /* Channel 0 */
        &VDC50.GAM_SW,
        &VDC50.GAM_G_UPDATE,
        {
            &VDC50.GAM_G_LUT1,
            &VDC50.GAM_G_LUT2,
            &VDC50.GAM_G_LUT3,
            &VDC50.GAM_G_LUT4,
            &VDC50.GAM_G_LUT5,
            &VDC50.GAM_G_LUT6,
            &VDC50.GAM_G_LUT7,
            &VDC50.GAM_G_LUT8,
            &VDC50.GAM_G_LUT9,
            &VDC50.GAM_G_LUT10,
            &VDC50.GAM_G_LUT11,
            &VDC50.GAM_G_LUT12,
            &VDC50.GAM_G_LUT13,
            &VDC50.GAM_G_LUT14,
            &VDC50.GAM_G_LUT15,
            &VDC50.GAM_G_LUT16
        },
        {
            &VDC50.GAM_G_AREA1,
            &VDC50.GAM_G_AREA2,
            &VDC50.GAM_G_AREA3,
            &VDC50.GAM_G_AREA4,
            &VDC50.GAM_G_AREA5,
            &VDC50.GAM_G_AREA6,
            &VDC50.GAM_G_AREA7,
            &VDC50.GAM_G_AREA8
        },
        &VDC50.GAM_B_UPDATE,
        {
            &VDC50.GAM_B_LUT1,
            &VDC50.GAM_B_LUT2,
            &VDC50.GAM_B_LUT3,
            &VDC50.GAM_B_LUT4,
            &VDC50.GAM_B_LUT5,
            &VDC50.GAM_B_LUT6,
            &VDC50.GAM_B_LUT7,
            &VDC50.GAM_B_LUT8,
            &VDC50.GAM_B_LUT9,
            &VDC50.GAM_B_LUT10,
            &VDC50.GAM_B_LUT11,
            &VDC50.GAM_B_LUT12,
            &VDC50.GAM_B_LUT13,
            &VDC50.GAM_B_LUT14,
            &VDC50.GAM_B_LUT15,
            &VDC50.GAM_B_LUT16
        },
        {
            &VDC50.GAM_B_AREA1,
            &VDC50.GAM_B_AREA2,
            &VDC50.GAM_B_AREA3,
            &VDC50.GAM_B_AREA4,
            &VDC50.GAM_B_AREA5,
            &VDC50.GAM_B_AREA6,
            &VDC50.GAM_B_AREA7,
            &VDC50.GAM_B_AREA8
        },
        &VDC50.GAM_R_UPDATE,
        {
            &VDC50.GAM_R_LUT1,
            &VDC50.GAM_R_LUT2,
            &VDC50.GAM_R_LUT3,
            &VDC50.GAM_R_LUT4,
            &VDC50.GAM_R_LUT5,
            &VDC50.GAM_R_LUT6,
            &VDC50.GAM_R_LUT7,
            &VDC50.GAM_R_LUT8,
            &VDC50.GAM_R_LUT9,
            &VDC50.GAM_R_LUT10,
            &VDC50.GAM_R_LUT11,
            &VDC50.GAM_R_LUT12,
            &VDC50.GAM_R_LUT13,
            &VDC50.GAM_R_LUT14,
            &VDC50.GAM_R_LUT15,
            &VDC50.GAM_R_LUT16
        },
        {
            &VDC50.GAM_R_AREA1,
            &VDC50.GAM_R_AREA2,
            &VDC50.GAM_R_AREA3,
            &VDC50.GAM_R_AREA4,
            &VDC50.GAM_R_AREA5,
            &VDC50.GAM_R_AREA6,
            &VDC50.GAM_R_AREA7,
            &VDC50.GAM_R_AREA8
        }
    },
    {   /* Channel 1 */
        &VDC51.GAM_SW,
        &VDC51.GAM_G_UPDATE,
        {
            &VDC51.GAM_G_LUT1,
            &VDC51.GAM_G_LUT2,
            &VDC51.GAM_G_LUT3,
            &VDC51.GAM_G_LUT4,
            &VDC51.GAM_G_LUT5,
            &VDC51.GAM_G_LUT6,
            &VDC51.GAM_G_LUT7,
            &VDC51.GAM_G_LUT8,
            &VDC51.GAM_G_LUT9,
            &VDC51.GAM_G_LUT10,
            &VDC51.GAM_G_LUT11,
            &VDC51.GAM_G_LUT12,
            &VDC51.GAM_G_LUT13,
            &VDC51.GAM_G_LUT14,
            &VDC51.GAM_G_LUT15,
            &VDC51.GAM_G_LUT16
        },
        {
            &VDC51.GAM_G_AREA1,
            &VDC51.GAM_G_AREA2,
            &VDC51.GAM_G_AREA3,
            &VDC51.GAM_G_AREA4,
            &VDC51.GAM_G_AREA5,
            &VDC51.GAM_G_AREA6,
            &VDC51.GAM_G_AREA7,
            &VDC51.GAM_G_AREA8
        },
        &VDC51.GAM_B_UPDATE,
        {
            &VDC51.GAM_B_LUT1,
            &VDC51.GAM_B_LUT2,
            &VDC51.GAM_B_LUT3,
            &VDC51.GAM_B_LUT4,
            &VDC51.GAM_B_LUT5,
            &VDC51.GAM_B_LUT6,
            &VDC51.GAM_B_LUT7,
            &VDC51.GAM_B_LUT8,
            &VDC51.GAM_B_LUT9,
            &VDC51.GAM_B_LUT10,
            &VDC51.GAM_B_LUT11,
            &VDC51.GAM_B_LUT12,
            &VDC51.GAM_B_LUT13,
            &VDC51.GAM_B_LUT14,
            &VDC51.GAM_B_LUT15,
            &VDC51.GAM_B_LUT16
        },
        {
            &VDC51.GAM_B_AREA1,
            &VDC51.GAM_B_AREA2,
            &VDC51.GAM_B_AREA3,
            &VDC51.GAM_B_AREA4,
            &VDC51.GAM_B_AREA5,
            &VDC51.GAM_B_AREA6,
            &VDC51.GAM_B_AREA7,
            &VDC51.GAM_B_AREA8
        },
        &VDC51.GAM_R_UPDATE,
        {
            &VDC51.GAM_R_LUT1,
            &VDC51.GAM_R_LUT2,
            &VDC51.GAM_R_LUT3,
            &VDC51.GAM_R_LUT4,
            &VDC51.GAM_R_LUT5,
            &VDC51.GAM_R_LUT6,
            &VDC51.GAM_R_LUT7,
            &VDC51.GAM_R_LUT8,
            &VDC51.GAM_R_LUT9,
            &VDC51.GAM_R_LUT10,
            &VDC51.GAM_R_LUT11,
            &VDC51.GAM_R_LUT12,
            &VDC51.GAM_R_LUT13,
            &VDC51.GAM_R_LUT14,
            &VDC51.GAM_R_LUT15,
            &VDC51.GAM_R_LUT16
        },
        {
            &VDC51.GAM_R_AREA1,
            &VDC51.GAM_R_AREA2,
            &VDC51.GAM_R_AREA3,
            &VDC51.GAM_R_AREA4,
            &VDC51.GAM_R_AREA5,
            &VDC51.GAM_R_AREA6,
            &VDC51.GAM_R_AREA7,
            &VDC51.GAM_R_AREA8
        }
    }
};
#else
vdc5_regaddr_gamma_t vdc5_regaddr_gamma[VDC5_CHANNEL_NUM];

static void initialize_vdc5_regaddr_gamma(void)
{
	/* Channel 0 */
	vdc5_regaddr_gamma[0].gam_sw = &VDC50.GAM_SW;
	vdc5_regaddr_gamma[0].gam_g_update = &VDC50.GAM_G_UPDATE;
	vdc5_regaddr_gamma[0].gam_g_lut[0] = &VDC50.GAM_G_LUT1;
	vdc5_regaddr_gamma[0].gam_g_lut[1] = &VDC50.GAM_G_LUT2;
	vdc5_regaddr_gamma[0].gam_g_lut[2] = &VDC50.GAM_G_LUT3;
	vdc5_regaddr_gamma[0].gam_g_lut[3] = &VDC50.GAM_G_LUT4;
	vdc5_regaddr_gamma[0].gam_g_lut[4] = &VDC50.GAM_G_LUT5;
	vdc5_regaddr_gamma[0].gam_g_lut[5] = &VDC50.GAM_G_LUT6;
	vdc5_regaddr_gamma[0].gam_g_lut[6] = &VDC50.GAM_G_LUT7;
	vdc5_regaddr_gamma[0].gam_g_lut[7] = &VDC50.GAM_G_LUT8;
	vdc5_regaddr_gamma[0].gam_g_lut[8] = &VDC50.GAM_G_LUT9;
	vdc5_regaddr_gamma[0].gam_g_lut[9] = &VDC50.GAM_G_LUT10;
	vdc5_regaddr_gamma[0].gam_g_lut[10] = &VDC50.GAM_G_LUT11;
	vdc5_regaddr_gamma[0].gam_g_lut[11] = &VDC50.GAM_G_LUT12;
	vdc5_regaddr_gamma[0].gam_g_lut[12] = &VDC50.GAM_G_LUT13;
	vdc5_regaddr_gamma[0].gam_g_lut[13] = &VDC50.GAM_G_LUT14;
	vdc5_regaddr_gamma[0].gam_g_lut[14] = &VDC50.GAM_G_LUT15;
	vdc5_regaddr_gamma[0].gam_g_lut[15] = &VDC50.GAM_G_LUT16;
	vdc5_regaddr_gamma[0].gam_g_area[0] = &VDC50.GAM_G_AREA1;
	vdc5_regaddr_gamma[0].gam_g_area[1] = &VDC50.GAM_G_AREA2;
	vdc5_regaddr_gamma[0].gam_g_area[2] = &VDC50.GAM_G_AREA3;
	vdc5_regaddr_gamma[0].gam_g_area[3] = &VDC50.GAM_G_AREA4;
	vdc5_regaddr_gamma[0].gam_g_area[4] = &VDC50.GAM_G_AREA5;
	vdc5_regaddr_gamma[0].gam_g_area[5] = &VDC50.GAM_G_AREA6;
	vdc5_regaddr_gamma[0].gam_g_area[6] = &VDC50.GAM_G_AREA7;
	vdc5_regaddr_gamma[0].gam_g_area[7] = &VDC50.GAM_G_AREA8;
	vdc5_regaddr_gamma[0].gam_b_update = &VDC50.GAM_B_UPDATE;
	vdc5_regaddr_gamma[0].gam_b_lut[0] = &VDC50.GAM_B_LUT1;
	vdc5_regaddr_gamma[0].gam_b_lut[1] = &VDC50.GAM_B_LUT2;
	vdc5_regaddr_gamma[0].gam_b_lut[2] = &VDC50.GAM_B_LUT3;
	vdc5_regaddr_gamma[0].gam_b_lut[3] = &VDC50.GAM_B_LUT4;
	vdc5_regaddr_gamma[0].gam_b_lut[4] = &VDC50.GAM_B_LUT5;
	vdc5_regaddr_gamma[0].gam_b_lut[5] = &VDC50.GAM_B_LUT6;
	vdc5_regaddr_gamma[0].gam_b_lut[6] = &VDC50.GAM_B_LUT7;
	vdc5_regaddr_gamma[0].gam_b_lut[7] = &VDC50.GAM_B_LUT8;
	vdc5_regaddr_gamma[0].gam_b_lut[8] = &VDC50.GAM_B_LUT9;
	vdc5_regaddr_gamma[0].gam_b_lut[9] = &VDC50.GAM_B_LUT10;
	vdc5_regaddr_gamma[0].gam_b_lut[10] = &VDC50.GAM_B_LUT11;
	vdc5_regaddr_gamma[0].gam_b_lut[11] = &VDC50.GAM_B_LUT12;
	vdc5_regaddr_gamma[0].gam_b_lut[12] = &VDC50.GAM_B_LUT13;
	vdc5_regaddr_gamma[0].gam_b_lut[13] = &VDC50.GAM_B_LUT14;
	vdc5_regaddr_gamma[0].gam_b_lut[14] = &VDC50.GAM_B_LUT15;
	vdc5_regaddr_gamma[0].gam_b_lut[15] = &VDC50.GAM_B_LUT16;
	vdc5_regaddr_gamma[0].gam_b_area[0] = &VDC50.GAM_B_AREA1;
	vdc5_regaddr_gamma[0].gam_b_area[1] = &VDC50.GAM_B_AREA2;
	vdc5_regaddr_gamma[0].gam_b_area[2] = &VDC50.GAM_B_AREA3;
	vdc5_regaddr_gamma[0].gam_b_area[3] = &VDC50.GAM_B_AREA4;
	vdc5_regaddr_gamma[0].gam_b_area[4] = &VDC50.GAM_B_AREA5;
	vdc5_regaddr_gamma[0].gam_b_area[5] = &VDC50.GAM_B_AREA6;
	vdc5_regaddr_gamma[0].gam_b_area[6] = &VDC50.GAM_B_AREA7;
	vdc5_regaddr_gamma[0].gam_b_area[7] = &VDC50.GAM_B_AREA8;
	vdc5_regaddr_gamma[0].gam_r_update = &VDC50.GAM_R_UPDATE;
	vdc5_regaddr_gamma[0].gam_r_lut[0] = &VDC50.GAM_R_LUT1;
	vdc5_regaddr_gamma[0].gam_r_lut[1] = &VDC50.GAM_R_LUT2;
	vdc5_regaddr_gamma[0].gam_r_lut[2] = &VDC50.GAM_R_LUT3;
	vdc5_regaddr_gamma[0].gam_r_lut[3] = &VDC50.GAM_R_LUT4;
	vdc5_regaddr_gamma[0].gam_r_lut[4] = &VDC50.GAM_R_LUT5;
	vdc5_regaddr_gamma[0].gam_r_lut[5] = &VDC50.GAM_R_LUT6;
	vdc5_regaddr_gamma[0].gam_r_lut[6] = &VDC50.GAM_R_LUT7;
	vdc5_regaddr_gamma[0].gam_r_lut[7] = &VDC50.GAM_R_LUT8;
	vdc5_regaddr_gamma[0].gam_r_lut[8] = &VDC50.GAM_R_LUT9;
	vdc5_regaddr_gamma[0].gam_r_lut[9] = &VDC50.GAM_R_LUT10;
	vdc5_regaddr_gamma[0].gam_r_lut[10] = &VDC50.GAM_R_LUT11;
	vdc5_regaddr_gamma[0].gam_r_lut[11] = &VDC50.GAM_R_LUT12;
	vdc5_regaddr_gamma[0].gam_r_lut[12] = &VDC50.GAM_R_LUT13;
	vdc5_regaddr_gamma[0].gam_r_lut[13] = &VDC50.GAM_R_LUT14;
	vdc5_regaddr_gamma[0].gam_r_lut[14] = &VDC50.GAM_R_LUT15;
	vdc5_regaddr_gamma[0].gam_r_lut[15] = &VDC50.GAM_R_LUT16;
	vdc5_regaddr_gamma[0].gam_r_area[0] = &VDC50.GAM_R_AREA1;
	vdc5_regaddr_gamma[0].gam_r_area[1] = &VDC50.GAM_R_AREA2;
	vdc5_regaddr_gamma[0].gam_r_area[2] = &VDC50.GAM_R_AREA3;
	vdc5_regaddr_gamma[0].gam_r_area[3] = &VDC50.GAM_R_AREA4;
	vdc5_regaddr_gamma[0].gam_r_area[4] = &VDC50.GAM_R_AREA5;
	vdc5_regaddr_gamma[0].gam_r_area[5] = &VDC50.GAM_R_AREA6;
	vdc5_regaddr_gamma[0].gam_r_area[6] = &VDC50.GAM_R_AREA7;
	vdc5_regaddr_gamma[0].gam_r_area[7] = &VDC50.GAM_R_AREA8;

	/* Channel 1 */
	vdc5_regaddr_gamma[1].gam_sw = &VDC51.GAM_SW;
	vdc5_regaddr_gamma[1].gam_g_update = &VDC51.GAM_G_UPDATE;
	vdc5_regaddr_gamma[1].gam_g_lut[0] = &VDC51.GAM_G_LUT1;
	vdc5_regaddr_gamma[1].gam_g_lut[1] = &VDC51.GAM_G_LUT2;
	vdc5_regaddr_gamma[1].gam_g_lut[2] = &VDC51.GAM_G_LUT3;
	vdc5_regaddr_gamma[1].gam_g_lut[3] = &VDC51.GAM_G_LUT4;
	vdc5_regaddr_gamma[1].gam_g_lut[4] = &VDC51.GAM_G_LUT5;
	vdc5_regaddr_gamma[1].gam_g_lut[5] = &VDC51.GAM_G_LUT6;
	vdc5_regaddr_gamma[1].gam_g_lut[6] = &VDC51.GAM_G_LUT7;
	vdc5_regaddr_gamma[1].gam_g_lut[7] = &VDC51.GAM_G_LUT8;
	vdc5_regaddr_gamma[1].gam_g_lut[8] = &VDC51.GAM_G_LUT9;
	vdc5_regaddr_gamma[1].gam_g_lut[9] = &VDC51.GAM_G_LUT10;
	vdc5_regaddr_gamma[1].gam_g_lut[10] = &VDC51.GAM_G_LUT11;
	vdc5_regaddr_gamma[1].gam_g_lut[11] = &VDC51.GAM_G_LUT12;
	vdc5_regaddr_gamma[1].gam_g_lut[12] = &VDC51.GAM_G_LUT13;
	vdc5_regaddr_gamma[1].gam_g_lut[13] = &VDC51.GAM_G_LUT14;
	vdc5_regaddr_gamma[1].gam_g_lut[14] = &VDC51.GAM_G_LUT15;
	vdc5_regaddr_gamma[1].gam_g_lut[15] = &VDC51.GAM_G_LUT16;
	vdc5_regaddr_gamma[1].gam_g_area[0] = &VDC51.GAM_G_AREA1;
	vdc5_regaddr_gamma[1].gam_g_area[1] = &VDC51.GAM_G_AREA2;
	vdc5_regaddr_gamma[1].gam_g_area[2] = &VDC51.GAM_G_AREA3;
	vdc5_regaddr_gamma[1].gam_g_area[3] = &VDC51.GAM_G_AREA4;
	vdc5_regaddr_gamma[1].gam_g_area[4] = &VDC51.GAM_G_AREA5;
	vdc5_regaddr_gamma[1].gam_g_area[5] = &VDC51.GAM_G_AREA6;
	vdc5_regaddr_gamma[1].gam_g_area[6] = &VDC51.GAM_G_AREA7;
	vdc5_regaddr_gamma[1].gam_g_area[7] = &VDC51.GAM_G_AREA8;
	vdc5_regaddr_gamma[1].gam_b_update = &VDC51.GAM_B_UPDATE;
	vdc5_regaddr_gamma[1].gam_b_lut[0] = &VDC51.GAM_B_LUT1;
	vdc5_regaddr_gamma[1].gam_b_lut[1] = &VDC51.GAM_B_LUT2;
	vdc5_regaddr_gamma[1].gam_b_lut[2] = &VDC51.GAM_B_LUT3;
	vdc5_regaddr_gamma[1].gam_b_lut[3] = &VDC51.GAM_B_LUT4;
	vdc5_regaddr_gamma[1].gam_b_lut[4] = &VDC51.GAM_B_LUT5;
	vdc5_regaddr_gamma[1].gam_b_lut[5] = &VDC51.GAM_B_LUT6;
	vdc5_regaddr_gamma[1].gam_b_lut[6] = &VDC51.GAM_B_LUT7;
	vdc5_regaddr_gamma[1].gam_b_lut[7] = &VDC51.GAM_B_LUT8;
	vdc5_regaddr_gamma[1].gam_b_lut[8] = &VDC51.GAM_B_LUT9;
	vdc5_regaddr_gamma[1].gam_b_lut[9] = &VDC51.GAM_B_LUT10;
	vdc5_regaddr_gamma[1].gam_b_lut[10] = &VDC51.GAM_B_LUT11;
	vdc5_regaddr_gamma[1].gam_b_lut[11] = &VDC51.GAM_B_LUT12;
	vdc5_regaddr_gamma[1].gam_b_lut[12] = &VDC51.GAM_B_LUT13;
	vdc5_regaddr_gamma[1].gam_b_lut[13] = &VDC51.GAM_B_LUT14;
	vdc5_regaddr_gamma[1].gam_b_lut[14] = &VDC51.GAM_B_LUT15;
	vdc5_regaddr_gamma[1].gam_b_lut[15] = &VDC51.GAM_B_LUT16;
	vdc5_regaddr_gamma[1].gam_b_area[0] = &VDC51.GAM_B_AREA1;
	vdc5_regaddr_gamma[1].gam_b_area[1] = &VDC51.GAM_B_AREA2;
	vdc5_regaddr_gamma[1].gam_b_area[2] = &VDC51.GAM_B_AREA3;
	vdc5_regaddr_gamma[1].gam_b_area[3] = &VDC51.GAM_B_AREA4;
	vdc5_regaddr_gamma[1].gam_b_area[4] = &VDC51.GAM_B_AREA5;
	vdc5_regaddr_gamma[1].gam_b_area[5] = &VDC51.GAM_B_AREA6;
	vdc5_regaddr_gamma[1].gam_b_area[6] = &VDC51.GAM_B_AREA7;
	vdc5_regaddr_gamma[1].gam_b_area[7] = &VDC51.GAM_B_AREA8;
	vdc5_regaddr_gamma[1].gam_r_update = &VDC51.GAM_R_UPDATE;
	vdc5_regaddr_gamma[1].gam_r_lut[0] = &VDC51.GAM_R_LUT1;
	vdc5_regaddr_gamma[1].gam_r_lut[1] = &VDC51.GAM_R_LUT2;
	vdc5_regaddr_gamma[1].gam_r_lut[2] = &VDC51.GAM_R_LUT3;
	vdc5_regaddr_gamma[1].gam_r_lut[3] = &VDC51.GAM_R_LUT4;
	vdc5_regaddr_gamma[1].gam_r_lut[4] = &VDC51.GAM_R_LUT5;
	vdc5_regaddr_gamma[1].gam_r_lut[5] = &VDC51.GAM_R_LUT6;
	vdc5_regaddr_gamma[1].gam_r_lut[6] = &VDC51.GAM_R_LUT7;
	vdc5_regaddr_gamma[1].gam_r_lut[7] = &VDC51.GAM_R_LUT8;
	vdc5_regaddr_gamma[1].gam_r_lut[8] = &VDC51.GAM_R_LUT9;
	vdc5_regaddr_gamma[1].gam_r_lut[9] = &VDC51.GAM_R_LUT10;
	vdc5_regaddr_gamma[1].gam_r_lut[10] = &VDC51.GAM_R_LUT11;
	vdc5_regaddr_gamma[1].gam_r_lut[11] = &VDC51.GAM_R_LUT12;
	vdc5_regaddr_gamma[1].gam_r_lut[12] = &VDC51.GAM_R_LUT13;
	vdc5_regaddr_gamma[1].gam_r_lut[13] = &VDC51.GAM_R_LUT14;
	vdc5_regaddr_gamma[1].gam_r_lut[14] = &VDC51.GAM_R_LUT15;
	vdc5_regaddr_gamma[1].gam_r_lut[15] = &VDC51.GAM_R_LUT16;
	vdc5_regaddr_gamma[1].gam_r_area[0] = &VDC51.GAM_R_AREA1;
	vdc5_regaddr_gamma[1].gam_r_area[1] = &VDC51.GAM_R_AREA2;
	vdc5_regaddr_gamma[1].gam_r_area[2] = &VDC51.GAM_R_AREA3;
	vdc5_regaddr_gamma[1].gam_r_area[3] = &VDC51.GAM_R_AREA4;
	vdc5_regaddr_gamma[1].gam_r_area[4] = &VDC51.GAM_R_AREA5;
	vdc5_regaddr_gamma[1].gam_r_area[5] = &VDC51.GAM_R_AREA6;
	vdc5_regaddr_gamma[1].gam_r_area[6] = &VDC51.GAM_R_AREA7;
	vdc5_regaddr_gamma[1].gam_r_area[7] = &VDC51.GAM_R_AREA8;
}
#endif

/* VDC5 system controller register address list */
#if 0
const vdc5_regaddr_system_ctrl_t vdc5_regaddr_system_ctrl[VDC5_CHANNEL_NUM] =
{
    {   /* Channel 0 */
        &VDC50.SYSCNT_INT1,
        &VDC50.SYSCNT_INT2,
        &VDC50.SYSCNT_INT3,
        &VDC50.SYSCNT_INT4,
        &VDC50.SYSCNT_INT5,
        &VDC50.SYSCNT_INT6,
        &VDC50.SYSCNT_PANEL_CLK,
        &VDC50.SYSCNT_CLUT
    },
    {   /* Channel 1 */
        &VDC51.SYSCNT_INT1,
        &VDC51.SYSCNT_INT2,
        &VDC51.SYSCNT_INT3,
        &VDC51.SYSCNT_INT4,
        &VDC51.SYSCNT_INT5,
        &VDC51.SYSCNT_INT6,
        &VDC51.SYSCNT_PANEL_CLK,
        &VDC51.SYSCNT_CLUT
    }
};
#else
vdc5_regaddr_system_ctrl_t vdc5_regaddr_system_ctrl[VDC5_CHANNEL_NUM];

static void initialize_vdc5_regaddr_system_ctrl(void)
{
	/* Channel 0 */
	vdc5_regaddr_system_ctrl[0].syscnt_int1 = &VDC50.SYSCNT_INT1;
	vdc5_regaddr_system_ctrl[0].syscnt_int2 = &VDC50.SYSCNT_INT2;
	vdc5_regaddr_system_ctrl[0].syscnt_int3 = &VDC50.SYSCNT_INT3;
	vdc5_regaddr_system_ctrl[0].syscnt_int4 = &VDC50.SYSCNT_INT4;
	vdc5_regaddr_system_ctrl[0].syscnt_int5 = &VDC50.SYSCNT_INT5;
	vdc5_regaddr_system_ctrl[0].syscnt_int6 = &VDC50.SYSCNT_INT6;
	vdc5_regaddr_system_ctrl[0].syscnt_panel_clk = &VDC50.SYSCNT_PANEL_CLK;
	vdc5_regaddr_system_ctrl[0].syscnt_clut = &VDC50.SYSCNT_CLUT;
	/* Channel 1 */
	vdc5_regaddr_system_ctrl[1].syscnt_int1 = &VDC51.SYSCNT_INT1;
	vdc5_regaddr_system_ctrl[1].syscnt_int2 = &VDC51.SYSCNT_INT2;
	vdc5_regaddr_system_ctrl[1].syscnt_int3 = &VDC51.SYSCNT_INT3;
	vdc5_regaddr_system_ctrl[1].syscnt_int4 = &VDC51.SYSCNT_INT4;
	vdc5_regaddr_system_ctrl[1].syscnt_int5 = &VDC51.SYSCNT_INT5;
	vdc5_regaddr_system_ctrl[1].syscnt_int6 = &VDC51.SYSCNT_INT6;
	vdc5_regaddr_system_ctrl[1].syscnt_panel_clk = &VDC51.SYSCNT_PANEL_CLK;
	vdc5_regaddr_system_ctrl[1].syscnt_clut = &VDC51.SYSCNT_CLUT;
}
#endif
/* LVDS register address list */
#if 0
const vdc5_regaddr_lvds_t vdc5_regaddr_lvds =
{
    &LVDS.LVDS_UPDATE,
    &LVDS.LVDSFCL,
    &LVDS.LCLKSELR,
    &LVDS.LPLLSETR,
    &LVDS.LPHYACC
};
#else
vdc5_regaddr_lvds_t vdc5_regaddr_lvds;

static void initialize_vdc5_regaddr_lvds(void)
{
	vdc5_regaddr_lvds.lvds_update = &LVDS.LVDS_UPDATE;
	vdc5_regaddr_lvds.lvdsfcl = &LVDS.LVDSFCL;
	vdc5_regaddr_lvds.lclkselr = &LVDS.LCLKSELR;
	vdc5_regaddr_lvds.lpllsetr = &LVDS.LPLLSETR;
	vdc5_regaddr_lvds.lphyacc = &LVDS.LPHYACC;
};
#endif
