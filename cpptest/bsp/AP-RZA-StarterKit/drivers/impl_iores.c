#include "iodefine.h"
#include "impl_iores.h"
#include "impl_rza1_eth.h"

static SOLID_IORES_REGINFO ioInfo[] = {
    //  const char* pName, SOLID_PHYADDRESS addr, size_t width, int extra
#ifdef BOARD_AP_RZA_0A
    {"SD_SPI", 0xE800C800, 0x24, -1},	// SPI ch.0
    {"SD_CD", 0xFCFE3204, 2, 6},	// P1_6
    {"LOG_UART", 0xe8009000, 0x30, -1},	// UART ch.4
#else // RZA_1A
    {"SD_SPI", 0xE800D800, 0x24, -1},	// SPI ch.2
    {"SD_CD",  0xFCFE3220, 2, 2},	// P8_2
#ifdef SOLID_USE_MONITORDEBUG
    {"MON_UART", 0xe8009800, 0x30, 243},
#else
    {"LOG_UART", 0xe8009800, 0x30, -1}, // UART ch.5
#endif
#endif
    {"ETH1", RZETH_REG_BASE1_PA, RZETH_REG_AREA1_SIZE, INTNO_ETHERI}, // Ethernet Reg 1st,: extra=interrupt No.
    {"ETH2", RZETH_REG_BASE2_PA, RZETH_REG_AREA2_SIZE, -1}, // Ethernet Reg 2nd
    {"ETH3", RZETH_REG_BASE3_PA, RZETH_REG_AREA3_SIZE, -1}, // Ethernet Reg 3rd
    {SOLID_IORES_NAME_GICD,  0xE8201000, 0x1000, 586}, // ARM GIC Distributor : extra = max intno
    {SOLID_IORES_NAME_GICC,  0xE8202000, 0x2000, 32}, // ARM GIC CPU Interface : extra = priority level (2-256)
    {"OS_TIMER", 0xFCFEC000, 0x21, 134},    // OS Timer ch.0
    {"OS_TIMER1", 0xFCFEC400, 0x21, 135},    // OS Timer ch.1

    {"CPG", CPG_PA, sizeof(struct st_cpg), -1},
    {"GPIO", GPIO_PA, sizeof(struct st_gpio), -1},
    {"JCU", JCU_PA, sizeof(struct st_jcu), -1},
    {"LVDS", LVDS_PA, sizeof(struct st_lvds), -1},
    {"MTU2", MTU2_PA, sizeof(struct st_mtu2), -1},
    {"OSTM0", OSTM0_PA, sizeof(struct st_ostm), -1},
    {"OSTM1", OSTM1_PA, sizeof(struct st_ostm), -1},
    {"RIIC0", RIIC0_PA, sizeof(struct st_riic), -1},
    {"RIIC1", RIIC1_PA, sizeof(struct st_riic), -1},
    {"RIIC2", RIIC2_PA, sizeof(struct st_riic), -1},
    {"RIIC3", RIIC3_PA, sizeof(struct st_riic), -1},
    {"VDC50", VDC50_PA, sizeof(struct st_vdc5), -1},
    {"VDC51", VDC51_PA, sizeof(struct st_vdc5), -1},
    {"RTC", RTC_PA, sizeof(struct st_rtc), -1},
    {"ADC", ADC_PA, sizeof(struct st_adc), -1},
    {"BSC", BSC_PA, sizeof(struct st_bsc), -1},
    {"CEU", CEU_PA, sizeof(struct st_ceu), -1},
    {"DISC0", DISC0_PA, sizeof(struct st_disc), -1},
    {"DISC1", DISC0_PA, sizeof(struct st_disc), -1},
	{"DMAC", DMAC_PA, sizeof(struct st_dmac), 41},
    {"DMACRS", DMACRS_PA, sizeof(struct st_dmars), -1},
    {"DVDEC0", DVDEC0_PA, sizeof(struct st_dvdec), -1},
    {"DVDEC1", DVDEC1_PA, sizeof(struct st_dvdec), -1},
    {"FLCTL", FLCTL_PA, sizeof(struct st_flctl), -1},
    {"IEB", IEB_PA, sizeof(struct st_ieb), -1},
    {"INB", INB_PA, sizeof(struct st_inb), -1},
    {"IRDA", IRDA_PA, sizeof(struct st_irda), -1},
    {"LIN0", LIN0_PA, sizeof(struct st_lin), -1},
    {"LIN1", LIN1_PA, sizeof(struct st_lin), -1},
    {"MLB", MLB_PA, sizeof(struct st_mlb), -1},
    {"PFV0", PFV0_PA, sizeof(struct st_pfv), -1},
    {"PFV1", PFV1_PA, sizeof(struct st_pfv), -1},
    {"PWM", PWM_PA, sizeof(struct st_pwm), -1},
    {"ROMDEC", ROMDEC_PA, sizeof(struct st_romdec), -1},
    {"RSCAN0", RSCAN0_PA, sizeof(struct st_rscan0), -1},
    {"RSPI0", RSPI0_PA, sizeof(struct st_rspi), -1},
    {"RSPI1", RSPI1_PA, sizeof(struct st_rspi), -1},
    {"RSPI2", RSPI2_PA, sizeof(struct st_rspi), -1},
    {"RSPI3", RSPI3_PA, sizeof(struct st_rspi), -1},
    {"RSPI4", RSPI4_PA, sizeof(struct st_rspi), -1},
    {"SCUX", SCUX_PA, sizeof(struct st_scux), -1},
    {"SDG0", SDG0_PA, sizeof(struct st_sdg), -1},
    {"SDG1", SDG1_PA, sizeof(struct st_sdg), -1},
    {"SDG2", SDG2_PA, sizeof(struct st_sdg), -1},
    {"SDG3", SDG3_PA, sizeof(struct st_sdg), -1},
    {"SPDIF", SPDIF_PA, sizeof(struct st_spdif), -1},
    {"SPIBSC0", SPIBSC0_PA, sizeof(struct st_spibsc), -1},
    {"SPIBSC1", SPIBSC1_PA, sizeof(struct st_spibsc), -1},
    {"SSIF0", SSIF0_PA, sizeof(struct st_ssif), -1},
    {"SSIF1", SSIF1_PA, sizeof(struct st_ssif), -1},
    {"SSIF2", SSIF2_PA, sizeof(struct st_ssif), -1},
    {"SSIF3", SSIF3_PA, sizeof(struct st_ssif), -1},
    {"SSIF4", SSIF4_PA, sizeof(struct st_ssif), -1},
    {"SSIF5", SSIF5_PA, sizeof(struct st_ssif), -1},
    {"USB200", USB200_PA, sizeof(struct st_usb20), -1},
    {"USB201", USB201_PA, sizeof(struct st_usb20), -1},
    {"WDT", WDT_PA, sizeof(struct st_wdt), -1},
};

// IOÉäÉ\Å[ÉXèÓïÒÇ≈ÅAèâä˙âªéûÇ…ìoò^Ç∑ÇÈÇ‡ÇÃÇéÊìæÇ∑ÇÈ
void IMPL_IORES_GetConfig(SOLID_IORES_REGINFO **ppInfo, int *pNum)
{
    *ppInfo = ioInfo;
    *pNum = sizeof(ioInfo) / sizeof(SOLID_IORES_REGINFO);
}
