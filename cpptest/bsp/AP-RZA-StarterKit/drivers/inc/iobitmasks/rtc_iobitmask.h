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
/*******************************************************************************
* File Name   : rtc_iobitmask.h
* $Rev: 1131 $
* $Date:: 2014-08-07 15:23:35 +0900#$
* Description : Realtime Clock register define header
*******************************************************************************/
#ifndef RTC_IOBITMASK_H
#define RTC_IOBITMASK_H


/* ==== Mask values for IO registers ==== */
#define RTC_R64CNT_64Hz                     (0x01u)
#define RTC_R64CNT_32Hz                     (0x02u)
#define RTC_R64CNT_16Hz                     (0x04u)
#define RTC_R64CNT_8Hz                      (0x08u)
#define RTC_R64CNT_4Hz                      (0x10u)
#define RTC_R64CNT_2Hz                      (0x20u)
#define RTC_R64CNT_1Hz                      (0x40u)

#define RTC_RSECCNT_1SEC                    (0x0Fu)
#define RTC_RSECCNT_10SEC                   (0x70u)
#define RTC_RSECCNT_SEC                     (RTC_RSECCNT_10SEC | RTC_RSECCNT_1SEC)

#define RTC_RMINCNT_1MIN                    (0x0Fu)
#define RTC_RMINCNT_10MIN                   (0x70u)
#define RTC_RMINCNT_MIN                     (RTC_RMINCNT_10MIN | RTC_RMINCNT_1MIN)

#define RTC_RHRCNT_1HR                      (0x0Fu)
#define RTC_RHRCNT_10HR                     (0x30u)
#define RTC_RHRCNT_HR                       (RTC_RHRCNT_10HR | RTC_RHRCNT_1HR)

#define RTC_RWKCNT_WK                       (0x07u)

#define RTC_RDAYCNT_1DAY                    (0x0Fu)
#define RTC_RDAYCNT_10DAY                   (0x30u)
#define RTC_RDAYCNT_DAY                     (RTC_RDAYCNT_10DAY | RTC_RDAYCNT_1DAY)

#define RTC_RMONCNT_1MON                    (0x0Fu)
#define RTC_RMONCNT_10MON                   (0x10u)
#define RTC_RMONCNT_MON                     (RTC_RMONCNT_10MON | RTC_RMONCNT_1MON)

#define RTC_RYRCNT_1YR                      (0x000Fu)
#define RTC_RYRCNT_10YR                     (0x00F0u)
#define RTC_RYRCNT_100YR                    (0x0F00u)
#define RTC_RYRCNT_1000YR                   (0xF000u)
#define RTC_RYRCNT_YR                       (RTC_RYRCNT_1000YR | RTC_RYRCNT_100YR | RTC_RYRCNT_10YR | RTC_RYRCNT_1YR)

#define RTC_RSECAR_1SEC                     (0x0Fu)
#define RTC_RSECAR_10SEC                    (0x70u)
#define RTC_RSECAR_SEC                      (RTC_RSECAR_10SEC | RTC_RSECAR_1SEC)
#define RTC_RSECAR_ENB                      (0x80u)

#define RTC_RMINAR_1MIN                     (0x0Fu)
#define RTC_RMINAR_10MIN                    (0x70u)
#define RTC_RMINAR_MIN                      (RTC_RMINAR_10MIN | RTC_RMINAR_1MIN)
#define RTC_RMINAR_ENB                      (0x80u)

#define RTC_RHRAR_1HR                       (0x0Fu)
#define RTC_RHRAR_10HR                      (0x30u)
#define RTC_RHRAR_HR                        (RTC_RHRAR_10HR | RTC_RHRAR_1HR)
#define RTC_RHRAR_ENB                       (0x80u)

#define RTC_RWKAR_WK                        (0x07u)
#define RTC_RWKAR_ENB                       (0x80u)

#define RTC_RDAYAR_1DAY                     (0x0Fu)
#define RTC_RDAYAR_10DAY                    (0x30u)
#define RTC_RDAYAR_DAY                      (RTC_RDAYAR_10DAY | RTC_RDAYAR_1DAY)
#define RTC_RDAYAR_ENB                      (0x80u)

#define RTC_RMONAR_1MON                     (0x0Fu)
#define RTC_RMONAR_10MON                    (0x10u)
#define RTC_RMONAR_MON                      (RTC_RMONAR_10MON | RTC_RMONAR_1MON)
#define RTC_RMONAR_ENB                      (0x80u)

#define RTC_RYRAR_1YR                       (0x000Fu)
#define RTC_RYRAR_10YR                      (0x00F0u)
#define RTC_RYRAR_100YR                     (0x0F00u)
#define RTC_RYRAR_1000YR                    (0xF000u)
#define RTC_RYRAR_YR                        (RTC_RYRAR_1000YR | RTC_RYRAR_100YR | RTC_RYRAR_10YR | RTC_RYRAR_1YR)

#define RTC_RCR1_AF                         (0x01u)
#define RTC_RCR1_AIE                        (0x08u)
#define RTC_RCR1_CIE                        (0x10u)
#define RTC_RCR1_CF                         (0x80u)

#define RTC_RCR2_START                      (0x01u)
#define RTC_RCR2_RESET                      (0x02u)
#define RTC_RCR2_ADJ                        (0x04u)
#define RTC_RCR2_RTCEN                      (0x08u)
#define RTC_RCR2_PES                        (0x70u)
#define RTC_RCR2_PEF                        (0x80u)

#define RTC_RCR3_ENB                        (0x80u)

#define RTC_RCR5_RCKSEL                     (0x03u)

#define RTC_RFRH_L_RFC                      (0x0007FFFFuL)
#define RTC_RFRH_L_SEL64                    (0x80000000uL)


/* ==== Shift values for IO registers ==== */
#define RTC_R64CNT_64Hz_SHIFT               (0u)
#define RTC_R64CNT_32Hz_SHIFT               (1u)
#define RTC_R64CNT_16Hz_SHIFT               (2u)
#define RTC_R64CNT_8Hz_SHIFT                (3u)
#define RTC_R64CNT_4Hz_SHIFT                (4u)
#define RTC_R64CNT_2Hz_SHIFT                (5u)
#define RTC_R64CNT_1Hz_SHIFT                (6u)

#define RTC_RSECCNT_1SEC_SHIFT              (0u)
#define RTC_RSECCNT_10SEC_SHIFT             (4u)
#define RTC_RSECCNT_SEC_SHIFT               (RTC_RSECCNT_1SEC_SHIFT)

#define RTC_RMINCNT_1MIN_SHIFT              (0u)
#define RTC_RMINCNT_10MIN_SHIFT             (4u)
#define RTC_RMINCNT_MIN_SHIFT               (RTC_RMINCNT_1MIN_SHIFT)

#define RTC_RHRCNT_1HR_SHIFT                (0u)
#define RTC_RHRCNT_10HR_SHIFT               (4u)
#define RTC_RHRCNT_HR_SHIFT                 (RTC_RHRCNT_1HR_SHIFT)

#define RTC_RWKCNT_WK_SHIFT                 (0u)

#define RTC_RDAYCNT_1DAY_SHIFT              (0u)
#define RTC_RDAYCNT_10DAY_SHIFT             (4u)
#define RTC_RDAYCNT_DAY_SHIFT               (RTC_RDAYCNT_1DAY_SHIFT)

#define RTC_RMONCNT_1MON_SHIFT              (0u)
#define RTC_RMONCNT_10MON_SHIFT             (4u)
#define RTC_RMONCNT_MON_SHIFT               (RTC_RMONCNT_1MON_SHIFT)

#define RTC_RYRCNT_1YR_SHIFT                (0u)
#define RTC_RYRCNT_10YR_SHIFT               (4u)
#define RTC_RYRCNT_100YR_SHIFT              (8u)
#define RTC_RYRCNT_1000YR_SHIFT             (12u)
#define RTC_RYRCNT_YR_SHIFT                 (RTC_RYRCNT_1YR_SHIFT)

#define RTC_RSECAR_1SEC_SHIFT               (0u)
#define RTC_RSECAR_10SEC_SHIFT              (4u)
#define RTC_RSECAR_SEC_SHIFT                (RTC_RSECAR_1SEC_SHIFT)
#define RTC_RSECAR_ENB_SHIFT                (7u)

#define RTC_RMINAR_1MIN_SHIFT               (0u)
#define RTC_RMINAR_10MIN_SHIFT              (4u)
#define RTC_RMINAR_MIN_SHIFT                (RTC_RMINAR_1MIN_SHIFT)
#define RTC_RMINAR_ENB_SHIFT                (7u)

#define RTC_RHRAR_1HR_SHIFT                 (0u)
#define RTC_RHRAR_10HR_SHIFT                (4u)
#define RTC_RHRAR_HR_SHIFT                  (RTC_RHRAR_1HR_SHIFT)
#define RTC_RHRAR_ENB_SHIFT                 (7u)

#define RTC_RWKAR_WK_SHIFT                  (0u)
#define RTC_RWKAR_ENB_SHIFT                 (7u)

#define RTC_RDAYAR_1DAY_SHIFT               (0u)
#define RTC_RDAYAR_10DAY_SHIFT              (4u)
#define RTC_RDAYAR_DAY_SHIFT                (RTC_RDAYAR_1DAY_SHIFT)
#define RTC_RDAYAR_ENB_SHIFT                (7u)

#define RTC_RMONAR_1MON_SHIFT               (0u)
#define RTC_RMONAR_10MON_SHIFT              (4u)
#define RTC_RMONAR_MON_SHIFT                (RTC_RMONAR_1MON_SHIFT)
#define RTC_RMONAR_ENB_SHIFT                (7u)

#define RTC_RYRAR_1YR_SHIFT                 (0u)
#define RTC_RYRAR_10YR_SHIFT                (4u)
#define RTC_RYRAR_100YR_SHIFT               (8u)
#define RTC_RYRAR_1000YR_SHIFT              (12u)
#define RTC_RYRAR_YR_SHIFT                  (RTC_RYRAR_1YR_SHIFT)

#define RTC_RCR1_AF_SHIFT                   (0u)
#define RTC_RCR1_AIE_SHIFT                  (3u)
#define RTC_RCR1_CIE_SHIFT                  (4u)
#define RTC_RCR1_CF_SHIFT                   (7u)

#define RTC_RCR2_START_SHIFT                (0u)
#define RTC_RCR2_RESET_SHIFT                (1u)
#define RTC_RCR2_ADJ_SHIFT                  (2u)
#define RTC_RCR2_RTCEN_SHIFT                (3u)
#define RTC_RCR2_PES_SHIFT                  (4u)
#define RTC_RCR2_PEF_SHIFT                  (7u)

#define RTC_RCR3_ENB_SHIFT                  (7u)

#define RTC_RCR5_RCKSEL_SHIFT               (0u)

#define RTC_RFRH_L_RFC_SHIFT                (0u)
#define RTC_RFRH_L_SEL64_SHIFT              (31u)


#endif /* RTC_IOBITMASK_H */
/* End of File */
