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
* http://www.renesas.com/disclaimer*
* Copyright (C) 2013-2014 Renesas Electronics Corporation. All rights reserved.
*******************************************************************************/
/*******************************************************************************
* File Name : iodefine.h
* $Rev: 1275 $
* $Date:: 2014-11-07 15:16:20 +0900#$
* Description : Definition of I/O Register (V1.01a)
******************************************************************************/
#ifndef R7S72100_IODEFINE_H
#define R7S72100_IODEFINE_H
//#define IODEFINE_H_VERSION  101

#include "solid_type.h"
#include "solid_iores.h"

// SOLID ?øΩ?øΩ typedef ?øΩ?øΩ conflict ?øΩ?øΩ?øΩ?øΩ?øΩBOSPL ?øΩ≈ÇÕég?øΩ?øΩ?øΩ?øΩ?øΩƒÇÔøΩ?øΩ»ÇÔøΩ?øΩÊÇ§?øΩ»ÇÃÇ≈ÅA?øΩR?øΩ?øΩ?øΩ?øΩ?øΩg?øΩA?øΩE?øΩg?øΩ≈ëŒèÔøΩ tokuyama@kmg 2016.09.14
#if 0 //tokuyama@kmg 2016.09.14
enum iodefine_byte_select_t
{
    L = 0, H = 1,
    LL= 0, LH = 1, HL = 2, HH = 3
};
#endif /* 0 */

/***********************************************************************
  <<< [iodefine_reg32_t] >>> 
- Padding : sizeof(iodefine_reg32_t) == 4
- Alignment(Offset) : &UINT32==0, &UINT16[0]==0, &UINT16[1]==2
-                     &UINT8[0]==0, &UINT8[1]==1, &UINT8[2]==2, &UINT8[3]==3
- Endian : Independent (Same as CPU endian as register endian)
- Bit-Order : Independent
************************************************************************/
/* ->MISRA 18.4 : Pack unpack union */ /* ->SEC M1.6.2 */
/* ->SEC M1.10.1 : Not magic number */
union iodefine_reg32_t
{
    volatile uint32_t  UINT32;                                  /*  32-bit Access   */
    volatile uint16_t  UINT16[2];                               /*  16-bit Access   */
    volatile uint8_t   UINT8[4];                                /*  8-bit Access    */
};
/* <-SEC M1.10.1 */
/* <-MISRA 18.4 */ /* <-SEC M1.6.2 */

/***********************************************************************
  <<< [iodefine_reg32_16_t] >>> 
- Padding : sizeof(iodefine_reg32_16_t) == 4
- Alignment(Offset) : &UINT32==0, &UINT16[0]==0, &UINT16[1]==2
- Endian : Independent (Same as CPU endian as register endian)
- Bit-Order : Independent
************************************************************************/
/* ->MISRA 18.4 : Pack unpack union */ /* ->SEC M1.6.2 */
/* ->SEC M1.10.1 : Not magic number */
union iodefine_reg32_16_t
{
    volatile uint32_t  UINT32;                                  /*  32-bit Access   */
    volatile uint16_t  UINT16[2];                               /*  16-bit Access   */
};
/* <-SEC M1.10.1 */
/* <-MISRA 18.4 */ /* <-SEC M1.6.2 */

/***********************************************************************
  <<< [iodefine_reg16_8_t] >>> 
- Padding : sizeof(iodefine_reg16_8_t) == 2
- Alignment(Offset) : &UINT16==0, &UINT8[0]==0, &UINT8[1]==1
- Endian : Independent (Same as CPU endian as register endian)
- Bit-Order : Independent
************************************************************************/
/* ->MISRA 18.4 : Pack unpack union */ /* ->SEC M1.6.2 */
/* ->SEC M1.10.1 : Not magic number */
union iodefine_reg16_8_t
{
    volatile uint16_t  UINT16;                                  /*  16-bit Access   */
    volatile uint8_t   UINT8[2];                                /*  8-bit Access    */
};
/* <-SEC M1.10.1 */
/* <-MISRA 18.4 */ /* <-SEC M1.6.2 */






#include "iodefines/adc_iodefine.h"                             /* (V1.00a) */
#include "iodefines/bsc_iodefine.h"                             /* (V1.00a) */
#include "iodefines/ceu_iodefine.h"                             /* (V1.00a) */
#include "iodefines/cpg_iodefine.h"                             /* (V1.00a) */
#include "iodefines/disc_iodefine.h"                            /* (V1.00a) */
#include "iodefines/dmac_iodefine.h"                            /* (V1.00a) */
#include "iodefines/dvdec_iodefine.h"                           /* (V1.00a) */
//#include "iodefines/ether_iodefine.h"                           /* (V1.00a) */
#include "iodefines/flctl_iodefine.h"                           /* (V1.00a) */
#include "iodefines/gpio_iodefine.h"                            /* (V1.00a) */
#include "iodefines/ieb_iodefine.h"                             /* (V1.00a) */
#include "iodefines/inb_iodefine.h"                             /* (V1.00a) */
//#include "iodefines/intc_iodefine.h"                            /* (V1.00a) */
#include "iodefines/irda_iodefine.h"                            /* (V1.00a) */
#include "iodefines/jcu_iodefine.h"                             /* (V1.00a) */
#include "iodefines/l2c_iodefine.h"                             /* (V1.00a) */
#include "iodefines/lin_iodefine.h"                             /* (V1.00a) */
#include "iodefines/lvds_iodefine.h"                            /* (V1.01a) */
#include "iodefines/mlb_iodefine.h"                             /* (V1.00a) */
//#include "iodefines/mmc_iodefine.h"                             /* (V1.00a) */
#include "iodefines/mtu2_iodefine.h"                            /* (V1.00a) */
#include "iodefines/ostm_iodefine.h"                            /* (V1.00a) */
#include "iodefines/pfv_iodefine.h"                             /* (V1.00a) */
#include "iodefines/pwm_iodefine.h"                             /* (V1.00a) */
#include "iodefines/riic_iodefine.h"                            /* (V1.00a) */
#include "iodefines/romdec_iodefine.h"                          /* (V1.00a) */
#include "iodefines/rscan0_iodefine.h"                          /* (V1.00a) */
#include "iodefines/rspi_iodefine.h"                            /* (V1.00a) */
#include "iodefines/rtc_iodefine.h"                             /* (V1.00a) */
//#include "iodefines/scif_iodefine.h"                            /* (V1.00a) */
//#include "iodefines/scim_iodefine.h"                            /* (V1.00a) */
#include "iodefines/scux_iodefine.h"                            /* (V1.00a) */
#include "iodefines/sdg_iodefine.h"                             /* (V1.00a) */
#include "iodefines/spdif_iodefine.h"                           /* (V1.00a) */
#include "iodefines/spibsc_iodefine.h"                          /* (V1.01a) */
#include "iodefines/ssif_iodefine.h"                            /* (V1.00a) */
#include "iodefines/usb20_iodefine.h"                           /* (V1.00a) */
#include "iodefines/vdc5_iodefine.h"                            /* (V1.00a) */
#include "iodefines/wdt_iodefine.h"                             /* (V1.00a) */

// extern
extern SOLID_IORES_INFO g_CPGRegs;
extern SOLID_IORES_INFO g_GPIORegs;
extern SOLID_IORES_INFO g_JCURegs;
extern SOLID_IORES_INFO g_LVDSRegs;
extern SOLID_IORES_INFO g_MTU2Regs;
extern SOLID_IORES_INFO g_OSTM0Regs;
extern SOLID_IORES_INFO g_OSTM1Regs;
extern SOLID_IORES_INFO g_RIIC0Regs;
extern SOLID_IORES_INFO g_RIIC1Regs;
extern SOLID_IORES_INFO g_RIIC2Regs;
extern SOLID_IORES_INFO g_RIIC3Regs;
extern SOLID_IORES_INFO g_VDC50Regs;
extern SOLID_IORES_INFO g_VDC51Regs;
extern SOLID_IORES_INFO g_RTCRegs;

extern SOLID_IORES_INFO g_ADCRegs;
extern SOLID_IORES_INFO g_BSCRegs;
extern SOLID_IORES_INFO g_CEURegs;
extern SOLID_IORES_INFO g_DISC0Regs;
extern SOLID_IORES_INFO g_DISC1Regs;
extern SOLID_IORES_INFO g_DMACRegs;
extern SOLID_IORES_INFO g_DMARSRegs;
extern SOLID_IORES_INFO g_DVDEC0Regs;
extern SOLID_IORES_INFO g_DVDEC1Regs;
extern SOLID_IORES_INFO g_FLCTLRegs;
extern SOLID_IORES_INFO g_IEBRegs;
extern SOLID_IORES_INFO g_INBRegs;
extern SOLID_IORES_INFO g_IRDARegs;
extern SOLID_IORES_INFO g_LIN0Regs;
extern SOLID_IORES_INFO g_LIN1Regs;
extern SOLID_IORES_INFO g_MLBRegs;
extern SOLID_IORES_INFO g_PFV0Regs;
extern SOLID_IORES_INFO g_PFV1Regs;
extern SOLID_IORES_INFO g_PWMRegs;
extern SOLID_IORES_INFO g_ROMDECRegs;
extern SOLID_IORES_INFO g_RSCAN0Regs;
extern SOLID_IORES_INFO g_RSPI0Regs;
extern SOLID_IORES_INFO g_RSPI1Regs;
extern SOLID_IORES_INFO g_RSPI2Regs;
extern SOLID_IORES_INFO g_RSPI3Regs;
extern SOLID_IORES_INFO g_RSPI4Regs;
extern SOLID_IORES_INFO g_SCUXRegs;
extern SOLID_IORES_INFO g_SDG0Regs;
extern SOLID_IORES_INFO g_SDG1Regs;
extern SOLID_IORES_INFO g_SDG2Regs;
extern SOLID_IORES_INFO g_SDG3Regs;
extern SOLID_IORES_INFO g_SPDIFRegs;
extern SOLID_IORES_INFO g_SPIBSC0Regs;
extern SOLID_IORES_INFO g_SPIBSC1Regs;
extern SOLID_IORES_INFO g_SSIF0Regs;
extern SOLID_IORES_INFO g_SSIF1Regs;
extern SOLID_IORES_INFO g_SSIF2Regs;
extern SOLID_IORES_INFO g_SSIF3Regs;
extern SOLID_IORES_INFO g_SSIF4Regs;
extern SOLID_IORES_INFO g_SSIF5Regs;
extern SOLID_IORES_INFO g_USB200Regs;
extern SOLID_IORES_INFO g_USB201Regs;
extern SOLID_IORES_INFO g_WDTRegs;



#endif
