#include <string.h>

#include "impl_dmac.h"
#include "solid_iores.h"
#include "iodefine.h"
#include "solid_intc.h"

#define MAX_CHANNEL (16)

SOLID_IORES_INFO g_DMACRegs;
SOLID_IORES_INFO g_DMARSRegs;

static SOLID_INTC_HANDLER g_DmacHandler[MAX_CHANNEL+1];

#define CHSTAT_EN   (0x00000001) // Statusレジスタ: Enble
#define CHSTAT_RQST (0x00000002) // Statusレジスタ: Request
#define CHSTAT_TACT (0x00000004) // Statusレジスタ: Transaction Active
#define CHSTAT_SUS  (0x00000008) // Statusレジスタ: Suspend
#define CHSTAT_ER   (0x00000010) // Statusレジスタ: Error
#define CHSTAT_END  (0x00000020) // Statusレジスタ: DMA END Interrupt

#define CHCFG_DAD   (1 << 21)
#define CHCFG_SAD   (1 << 20)

#define CHCTRL_SETEN     (1 << 0)
#define CHCTRL_CLREN     (1 << 1)
#define CHCTRL_STG       (1 << 2)
#define CHCTRL_SWST      (1 << 3)
#define CHCTRL_CLRRQ     (1 << 4)
#define CHCTRL_CLREND    (1 << 5)
#define CHCTRL_CLRTC     (1 << 6)
#define CHCTRL_SETSUS    (1 << 8)
#define CHCTRL_CLRSUS    (1 << 9)
#define CHCTRL_SETINTMSK (1 << 16)
#define CHCTRL_CLRINTMSK (1 << 17)

static struct _IMPL_DMAC_CONFIG_ {
    const char* pName;	     // 設定名
    unsigned long config;    // CFGレジスタの設定
    unsigned long interval;  // Intervalレジスタの設定
    unsigned long extension; // extensionレジスタの設定
    unsigned short dmars;    // DMARSレジスタの設定
} DmaConfig[] = {
//    Name,  CFG,        ITVL,       EXT,        DMARS
    {"TXI0", 0x00000268, 0x00000000, 0x00000000, 0x0061},
    {"RXI0", 0x00000260, 0x00000000, 0x00000000, 0x0062}
};

static int DmacCallback(void* param, SOLID_CPU_CONTEXT* pContext)
{
    int ch = (int)(param);

    struct st_dmac_n* pRegs = (struct st_dmac_n*)(g_DMACRegs.addr + 0x40*ch);

    unsigned long status = pRegs->CHSTAT_n;

    if (status & CHSTAT_ER) {
	SOLID_DMAC_Notify(ch, SOLID_DMAC_CALLBACK_STATUS_FAILED);
    } else if (status & CHSTAT_END) {
	SOLID_DMAC_Notify(ch, SOLID_DMAC_CALLBACK_STATUS_SUCCESS);
    }

    return SOLID_ERR_OK;
}

static int DmacErrorCallback(void* param, SOLID_CPU_CONTEXT* pContext)
{
    // error
    return SOLID_ERR_OK;
}

static void SetDMACRS(int ch, unsigned short val)
{
    volatile unsigned long *pRegs = (unsigned long*)(g_DMARSRegs.addr) + ch/2;
    unsigned long tmp = *pRegs;
    if ((ch % 2) ==0) {
	tmp = (tmp & 0xFFFF0000) | val;
    } else {
	tmp = ((unsigned long)val) << 16 | (tmp & 0x0000FFFF);
    }
    *pRegs = tmp;
}

// DMAコントローラの初期化
int IMPL_DMAC_Init(SOLID_DMAC_DEVICEINFO* pDevInfo)
{
    SOLID_IORES_GetInfo("DMAC", &g_DMACRegs);
    SOLID_IORES_GetInfo("DMACRS", &g_DMARSRegs);

    pDevInfo->maxChannel = MAX_CHANNEL;
    pDevInfo->maxTransferSize = 0xFFFFFFFF;
    pDevInfo->maxTransferWidth = 1024/8;

    // エラーハンドラの登録
    g_DmacHandler[MAX_CHANNEL].intno    = g_DMACRegs.extra + MAX_CHANNEL;
    g_DmacHandler[MAX_CHANNEL].priority = SOLID_INTC_GetPriorityLevel()/2;
    g_DmacHandler[MAX_CHANNEL].config   = 0x03; // edge trigger
    g_DmacHandler[MAX_CHANNEL].func     = DmacErrorCallback;
    g_DmacHandler[MAX_CHANNEL].param    = NULL;
    SOLID_INTC_Register(&g_DmacHandler[MAX_CHANNEL]);
    SOLID_INTC_Enable(g_DmacHandler[MAX_CHANNEL].intno);

    return SOLID_ERR_OK;
}

// DMAのチャネル占有開始
int IMPL_DMAC_Use(int ch)
{
    g_DmacHandler[ch].intno    = g_DMACRegs.extra + ch;
    g_DmacHandler[ch].priority = SOLID_INTC_GetPriorityLevel()/2;
    g_DmacHandler[ch].config   = 0x03; // edge trigger
    g_DmacHandler[ch].func     = DmacCallback;
    g_DmacHandler[ch].param    = (void*)(ch);
    SOLID_INTC_Register(&g_DmacHandler[ch]);
    SOLID_INTC_Enable(g_DmacHandler[ch].intno);

    return SOLID_ERR_OK;
}

// DMAのチャネル占有終了
int IMPL_DMAC_UnUse(int ch)
{
    SOLID_INTC_Disable(g_DmacHandler[ch].intno);
    SOLID_INTC_UnRegister(&g_DmacHandler[ch]);

    return SOLID_ERR_OK;
}

// DMA転送の開始
int IMPL_DMAC_StartTransfer(int ch, SOLID_DMAC_TRANSACTION* pTx)
{
    volatile struct st_dmac_n* pRegs = (struct st_dmac_n*)(g_DMACRegs.addr + 0x40*ch);

    pRegs->N0SA_n = pTx->src.phyAddr;
    pRegs->N0DA_n = pTx->dest.phyAddr;
    pRegs->N0TB_n = pTx->src.size;

    unsigned long cfg = (ch & 0x07);
    if (pTx->src.bufType != SOLID_DMAC_BUF_MEMORY) {
	cfg |= CHCFG_SAD;
    }
    switch (pTx->src.width) {
    case   1: cfg |= (0 << 12); break;
    case   2: cfg |= (1 << 12); break;
    case   4: cfg |= (2 << 12); break;
    case  16: cfg |= (3 << 12); break;
    case  32: cfg |= (4 << 12); break;
    case  64: cfg |= (5 << 12); break;
    case 128: cfg |= (6 << 12); break;
    case 256: cfg |= (7 << 12); break;
    }

    if (pTx->dest.bufType != SOLID_DMAC_BUF_MEMORY) {
	cfg |= CHCFG_DAD;
    }
    switch (pTx->dest.width) {
    case   1: cfg |= (0 << 16); break;
    case   2: cfg |= (1 << 16); break;
    case   4: cfg |= (2 << 16); break;
    case  16: cfg |= (3 << 16); break;
    case  32: cfg |= (4 << 16); break;
    case  64: cfg |= (5 << 16); break;
    case 128: cfg |= (6 << 16); break;
    case 256: cfg |= (7 << 16); break;
    }

    if (pTx->configIdx == -1) {
	cfg |= 0x00400400;   // block transfer, auto request
	pRegs->CHITVL_n = 0;
	pRegs->CHEXT_n  = 0;
	SetDMACRS(ch, 0);
    } else if (pTx->configIdx < sizeof(DmaConfig)/sizeof(DmaConfig[0])) {
	struct _IMPL_DMAC_CONFIG_* pCfg = &DmaConfig[pTx->configIdx];
	cfg |= pCfg->config;
	pRegs->CHITVL_n = pCfg->interval;
	pRegs->CHEXT_n  = pCfg->extension;
	SetDMACRS(ch, pCfg->dmars);
    } else {
	return SOLID_ERR_BADSEQUENCE;
    }

    pRegs->CHCFG_n = cfg;

    // start
    pRegs->CHCTRL_n = CHCTRL_SWST; // ステータスクリア
    pRegs->CHCTRL_n = CHCTRL_SETEN | ((pTx->configIdx == -1) ? CHCTRL_STG : 0); // DMA転送許可

    return 1;
}

// DMA転送の中断
int IMPL_DMAC_AbortTransfer(int ch)
{
    volatile struct st_dmac_n* pRegs = (struct st_dmac_n*)(g_DMACRegs.addr + 0x40*ch);

    if ((pRegs->CHSTAT_n & CHSTAT_EN) == 0) return SOLID_ERR_OK;

    // SUSPEND送信
    pRegs->CHCTRL_n = CHCTRL_SETSUS;

    while (1) {
        unsigned long sts = pRegs->CHSTAT_n;
	if ((sts & CHSTAT_EN) == 0) return SOLID_ERR_OK;
	if ((sts & CHSTAT_SUS) != 0) break;
    }

    // 停止処理
    pRegs->CHCTRL_n = CHCTRL_CLREN;

    while (1) {
        unsigned long sts = pRegs->CHSTAT_n;
	if ((sts & CHSTAT_TACT) != 0) break;
    }

    return SOLID_ERR_OK;
}

// DMA設定のインデックスを取得
int IMPL_DMAC_GetConfigIdx(const char* pName, int* pConfigIdx)
{
    int i;
    for (i = 0; i < sizeof(DmaConfig)/sizeof(DmaConfig[0]); i++) {
	if (strcmp(pName, DmaConfig[i].pName) == 0) {
	    *pConfigIdx = i;
	    return SOLID_ERR_OK;
	}
    }

    *pConfigIdx = -1;
    return SOLID_ERR_NOTFOUND;
}
