#include "solid_type.h"
#include "solid_mem.h"
#include "solid_timer.h"
#include "solid_iores.h"
#include "solid_fatfs_driver.h"
#include "impl_sd_spi.h"


#define SD_CMD_RETRY	(100) /* SDのコマンドのリトライ回数: 100回 */
#define SD_POLL_TIMEOUT	(3000) /* SDのコマンドタイムアウト: 3sec */

static SOLID_FATFS_DRIVER drv;
static enum { MMC, SDCv1, SDCv2, SDHC} card_type;

static void WaitMsec(int msec)
{
    unsigned long long past;
    unsigned long long curr = SOLID_TIMER_GetCurrentTick();

    do {
	past = SOLID_TIMER_ToUsec(SOLID_TIMER_GetCurrentTick() - curr) / 1000;
    }
    while (past < msec);
}

void IMPL_SDSPI_Init()
{
    IMPL_SDSPI_DeviceInit();

    drv.path = "SD";
    drv.param = NULL;
    drv.open  = &IMPL_SDSPI_open;
    drv.close = &IMPL_SDSPI_close;
    drv.getDeviceInfo = &IMPL_SDSPI_getDeviceInfo;
    drv.isAvailable = &IMPL_SDSPI_isAvailable;
    drv.read = &IMPL_SDSPI_read;
    drv.write = &IMPL_SDSPI_write;
    drv.flush = &IMPL_SDSPI_flush;

    SOLID_FatFS_RegisterDriver(&drv);

}

// 上位レイヤ
#define R1_IDLE_STATE (1 << 0)
#define R1_ERASE_RESET (1 << 1)
#define R1_ILLEGAL_COMMAND (1 << 2)
#define R1_COM_CRC_ERROR (1 << 3)
#define R1_ERASE_SEQUENCE_ERROR (1 << 4)
#define R1_ADDRESS_ERROR (1 << 5)
#define R1_PARAMETER_ERROR (1 << 6)

int IMPL_SDSPI_open(void *param)
{
    int i, res, retry;

    if (!IMPL_SDSPI_CheckCD()) {
	return SOLID_ERR_NOTFOUND;
    }

    retry = 0;
    do {
        // 初期化前に16BYTE分CLKを進める
	IMPL_SDSPI_SetCS(1);
	for (i = 0; i < 16; i++) {
	    IMPL_SDSPI_Write(0xff);
	}
	IMPL_SDSPI_SetCS(0);

	// CMD0発行
	res = IMPL_SDSPI_cmd(0, 0, NULL, 0);
	IMPL_SDSPI_SetCS(0);
	retry++;
    } while ((res != R1_IDLE_STATE)&&(retry < SD_CMD_RETRY));
    if (res != R1_IDLE_STATE) {
	return SOLID_ERR_NOTFOUND;
    }

    // CMD8発行
    res = IMPL_SDSPI_cmd(8, 0x000001AA, NULL, 4);
    IMPL_SDSPI_SetCS(0);
    if (res == R1_IDLE_STATE)
    {
	// v2 or SDHC,SDXC
	unsigned char ocr[4];
	retry = 0;
	do {
	    WaitMsec(50);
    	    res = IMPL_SDSPI_cmd(58, 0, ocr, 4);
	    IMPL_SDSPI_SetCS(0);
	    if (res < 0) return SOLID_ERR_BADSEQUENCE;
    	    res = IMPL_SDSPI_cmd(41, 0x40000000, NULL, 0);  // Accept High Capacity
	    IMPL_SDSPI_SetCS(0);
	    if (res < 0) return SOLID_ERR_BADSEQUENCE;
	    retry++;
	} while ((res == R1_IDLE_STATE)&&(retry < SD_CMD_RETRY));
	if (res == R1_IDLE_STATE) return SOLID_ERR_TIMEOUT;
	res = IMPL_SDSPI_cmd(58, 0, ocr, 4);
        IMPL_SDSPI_SetCS(0);
	if (res != 0) return SOLID_ERR_BADSEQUENCE;
	if (ocr[0] & 0x40) {
	    card_type = SDHC;
	} else {
	    card_type = SDCv2;
	}
    }
    else if (res == (R1_IDLE_STATE | R1_ILLEGAL_COMMAND))
    {
	// v1 or MMC
	retry = 0;
	do {
	    res = IMPL_SDSPI_cmd(41, 0, NULL, 0);
	    if (res < 0) return SOLID_ERR_BADSEQUENCE;
	    IMPL_SDSPI_SetCS(0);
	    retry++;
	} while ((res == R1_IDLE_STATE)&&(retry < SD_CMD_RETRY));
	if (res == R1_IDLE_STATE) return SOLID_ERR_TIMEOUT;
	if (res == 0) {
	    card_type = SDCv1;
	} else {
	    card_type = MMC;
		// CMD1による初期化
		retry = 0;
		do
		{
			IMPL_SDSPI_cmd(1, 0, NULL, 0);
			if (res < 0) return SOLID_ERR_BADSEQUENCE;
			IMPL_SDSPI_SetCS(0);
			retry++;
		} while ((res == R1_IDLE_STATE) && (retry < SD_CMD_RETRY));
		if (res == R1_IDLE_STATE) return SOLID_ERR_TIMEOUT;
		if (res != 0) return SOLID_ERR_BADSEQUENCE;
	}
    }
    else
    {
	// エラー
	return SOLID_ERR_NOTSUPPORTED;
    }

    if (card_type != SDHC) {
        // CMD16発行
	IMPL_SDSPI_cmd(16, 512, NULL, 0);
	IMPL_SDSPI_SetCS(0);
    }

    return SOLID_ERR_OK;
}

int IMPL_SDSPI_close(void *param)
{
    IMPL_SDSPI_SetCS(0);
    IMPL_SDSPI_WaitWrite();

    return SOLID_ERR_OK;
}

int IMPL_SDSPI_isAvailable(void *param)
{

    return SOLID_ERR_OK;
}

static int ext_bits(unsigned char *data, int msb, int lsb)
{
    int i;
    int bits = 0;
    int size = 1 + msb - lsb;
    for (i = 0; i < size; i++)
    {
	int position = lsb + i;
	int byte = 15 - (position >> 3);
	int bit = position & 0x7;
	int value = (data[byte] >> bit) & 1;
	bits |= value << i;
    }
    return bits;
}

int IMPL_SDSPI_getDeviceInfo(void *param, SOLID_FATFS_DRIVER_INFO *info)
{
    if (info == NULL) return SOLID_ERR_PAR;

    int res;
    unsigned char csd[16];

    res = IMPL_SDSPI_cmd(9, 0, NULL, 0);
    if (res != 0) {
	return SOLID_ERR_BADSEQUENCE;
    }
    IMPL_SDSPI_WaitResponse(0xFF, 0xFE);
    IMPL_SDSPI_Read(csd, sizeof(csd));
    IMPL_SDSPI_SetCS(0);

    // csd_structure : csd[127:126]
    // c_size        : csd[73:62]
    // c_size_mult   : csd[49:47]
    // read_bl_len   : csd[83:80] - the *maximum* read block length

    int csd_structure = ext_bits(csd, 127, 126);
    int c_size = ext_bits(csd, 73, 62);
    int perm_write_protect = ext_bits(csd, 13, 13);
    int tmp_write_protect = ext_bits(csd, 12, 12);


    switch (csd_structure) {
    case 0:
	{
	    int c_size_mult = ext_bits(csd, 49, 47);
	    int read_bl_len = ext_bits(csd, 83, 80);

	    // memory capacity = BLOCKNR * BLOCK_LEN
	    // where
	    //  BLOCKNR = (C_SIZE+1) * MULT
	    //  MULT = 2^(C_SIZE_MULT+2) (C_SIZE_MULT < 8)
	    //  BLOCK_LEN = 2^READ_BL_LEN, (READ_BL_LEN < 12)

	    unsigned long block_len = 1 << read_bl_len;
	    unsigned long mult = 1 << (c_size_mult + 2);
	    unsigned long blocknr = (c_size + 1) * mult;
	    unsigned long capacity = blocknr * block_len;

	    info->SectorSize = 512;
	    info->SectorCount = capacity / 512;
	    info->EraseSize = 512;

	    info->bReadOnly = ( perm_write_protect || tmp_write_protect);
	} break;
    case 1:
	{
	    unsigned long hc_c_size = ext_bits(csd, 69, 48);
	    info->SectorSize = 512;
	    info->SectorCount = (hc_c_size+1)*1024;
	    info->EraseSize = 512;

            info->bReadOnly = ( perm_write_protect || tmp_write_protect);
	} break;
    default:
	return SOLID_ERR_UNKNOWNDEVICE;
    }


    return SOLID_ERR_OK;
}

int IMPL_SDSPI_read(void *param, unsigned long addr, void *buf, size_t count)
{
    int i;
    for (i = 0; i < count; i++) {
	int retCode = IMPL_SDSPI_readblk(addr, buf);
	if (retCode < 0) return retCode;
	addr++;
	buf = (void*)((char*)buf + 512);
    }
    return SOLID_ERR_OK;
}

int IMPL_SDSPI_write(void *param, unsigned long addr, const void *buf, size_t count)
{
    int i;
    for (i = 0; i < count; i++)
    {
	int retCode = IMPL_SDSPI_writeblk(addr, buf);
	if (retCode < 0) return retCode;
	addr++;
	buf = (void *)((char *)buf + 512);
    }
    return SOLID_ERR_OK;
}

int IMPL_SDSPI_flush(void * param)
{
    IMPL_SDSPI_WaitWrite();

    return SOLID_ERR_OK;
}

// SDコマンド用
int IMPL_SDSPI_cmd(unsigned char cmd, int arg, unsigned char *res, int reslen)
{
    int response;

    if (cmd == 41) {  // ACMD
	response = IMPL_SDSPI_cmd(55, 0, NULL, 0);
	if ((response != 0x00) && (response != R1_IDLE_STATE)) return response;
	IMPL_SDSPI_SetCS(0);
    }

    IMPL_SDSPI_SetCS(1);
    IMPL_SDSPI_Write(0x40 | cmd); // CMD
    IMPL_SDSPI_Write(arg >> 24);  // ARG
    IMPL_SDSPI_Write(arg >> 16);
    IMPL_SDSPI_Write(arg >> 8);
    IMPL_SDSPI_Write(arg);
    IMPL_SDSPI_Write((cmd == 8) ? 0x87 : 0x95);		    // CRC
    response = IMPL_SDSPI_WaitResponse(0x80, 0);	    // response待ち
    if (response < 0) return response;
    if (reslen > 0) {
	IMPL_SDSPI_Read(res, reslen);
    }
    return response;
}

int IMPL_SDSPI_readblk(unsigned long addr, void *buf)
{
    if (card_type != SDHC) {
	addr *= 512;
    }
    int retry = 0;
    int response;
retry_loop:
    response = IMPL_SDSPI_cmd(17, addr, NULL, 0);
    if (response < 0) {
	return response;
    } else if (response != 0x00) {
	retry++;
	if (retry < SD_CMD_RETRY) goto retry_loop;
	return SOLID_ERR_BUSY;
    }

    response = IMPL_SDSPI_WaitResponse(0xFF, 0xFE);    // data token
    if (response < 0) {
	return response;
    }
    IMPL_SDSPI_Read((unsigned char *)buf, 512);
    IMPL_SDSPI_Read(NULL, 2);		    // CRC
    IMPL_SDSPI_SetCS(0);

    return SOLID_ERR_OK;
}

int IMPL_SDSPI_writeblk(unsigned long addr, const void *buf)
{
    int i;
    unsigned char *pData = (unsigned char *)buf;
    if (card_type != SDHC) {
	addr *= 512;
    }
    int retry = 0;
    int response;
retry_loop:
    response = IMPL_SDSPI_cmd(24, addr, NULL, 0);
    if (response < 0) {
	return response;
    } else if (response != 0x00) {
	retry++;
	if (retry < SD_CMD_RETRY) goto retry_loop;
	return SOLID_ERR_BUSY;
    }

    IMPL_SDSPI_Write(0xFF); // CMD-DATA間は1byte以上あける
    IMPL_SDSPI_Write(0xFE); // START
    for (i = 0; i < 512; i++)
    {
	IMPL_SDSPI_Write(pData[i]);
    }
    IMPL_SDSPI_Write(0xFF); // CRC
    IMPL_SDSPI_Write(0xFF); // CRC

    response = IMPL_SDSPI_WaitResponse(0x11, 0x01);
    if (response < 0) {
	return response;
    } else if ((response & 0x1F) != 0x05) {
	return SOLID_ERR_BADSEQUENCE;
    }
    IMPL_SDSPI_WaitResponse(0xFF, 0xFF); // BUSY終了待ち
    IMPL_SDSPI_SetCS(0);

    return SOLID_ERR_OK;
}


// 下位レイヤ
#define SDSPI_TXDUMMY	(0xFF)

static const char* SPIRegName = "SD_SPI";
static SOLID_IORES_INFO SPIRegs;

#define SPCR (volatile unsigned char*)(SPIRegs.addr+0x00)
#define SSLP (volatile unsigned char *)(SPIRegs.addr + 0x01)
#define SPPCR (volatile unsigned char *)(SPIRegs.addr + 0x02)
#define SPSR (volatile unsigned char *)(SPIRegs.addr + 0x03)
#define SPDR (volatile unsigned char *)(SPIRegs.addr + 0x04)
#define SPSCR (volatile unsigned char *)(SPIRegs.addr + 0x08)
#define SPSSR (volatile unsigned char *)(SPIRegs.addr + 0x09)
#define SPBR (volatile unsigned char *)(SPIRegs.addr + 0x0A)
#define SPDCR (volatile unsigned char *)(SPIRegs.addr + 0x0B)
#define SPCKD (volatile unsigned char *)(SPIRegs.addr + 0x0C)
#define SSLND (volatile unsigned char *)(SPIRegs.addr + 0x0D)
#define SPND (volatile unsigned char *)(SPIRegs.addr + 0x0E)
#define SPCMD0 (volatile unsigned short *)(SPIRegs.addr + 0x10)
#define SPCMD1 (volatile unsigned short *)(SPIRegs.addr + 0x12)
#define SPCMD2 (volatile unsigned short *)(SPIRegs.addr + 0x14)
#define SPCMD3 (volatile unsigned short *)(SPIRegs.addr + 0x16)
#define SPBFCR (volatile unsigned char *)(SPIRegs.addr + 0x20)
#define SPBFDR (volatile unsigned short *)(SPIRegs.addr + 0x22)

static const char* CDPinName = "SD_CD";
static SOLID_IORES_INFO CDPin;

#define CDPIN ((*(volatile unsigned short *)(CDPin.addr)) & (0x0001 << CDPin.extra))

void IMPL_SDSPI_DeviceInit()
{
    SOLID_IORES_Use(SPIRegName);
    SOLID_IORES_GetInfo(SPIRegName, &SPIRegs);

    *SPCR = 0x48;   // 機能enable, 割り込みは使用しない, マスタ設定
    *SSLP = 0x00;   // SSLは0アクティブ
    *SPPCR = 0x30;  // MOSIはアイドル時1固定、通常モード
    *SPSCR = 0x00;  // シーケンスはCMD0のみを使用
    *SPBR = 79;	    // CLK : 64MHz / 2x(79+1)x2^n = 400Kbps (nはBRDVビットで変更可能)
    *SPDCR = 0x20;  // ダミー送信なし、データアクセス幅8bit
//    *SPCKD = 0x00;  // クロック発信遅延: 1CLK
//    *SSLND = 0x00;  // SSLネゲート遅延: 1CLK
//    *SPND  = 0x00;  // 次アクセス遅延: 1CLK
    *SPCMD0 = 0x0780; // MSB 1st, Data長8bit, SSL保持, SRDV:1分周 SPI;Mode0
    *SPBFCR = 0xC0; // バッファリセット
    *SPBFCR = 0x00; // バッファクリア

    SOLID_IORES_Use(CDPinName);
    SOLID_IORES_GetInfo(CDPinName, &CDPin);
}

int IMPL_SDSPI_CheckCD()
{
    return (CDPIN ? 0: 1);  // CSは0で挿入状態, 1で未挿入状態
}

void IMPL_SDSPI_SetCS(int bAssert)
{
    IMPL_SDSPI_WaitWrite();
    if (bAssert) {
	*SPCMD0 |= 0x0080;
    } else {
	*SPCMD0 &= ~0x0080;
	*SPDR = 0xff;	// ダミーを送信して終了時にCSをネゲートする
    }
    // データを空読み
    while ((*SPBFDR & 0x00FF) > 0) {
	*SPDR;
    }

}

unsigned char IMPL_SDSPI_Write(unsigned char d)
{
    while ((*SPBFDR & 0xFF00) == 0x0800) ; // バッファフルの場合は待ち
    *SPDR = d;

    // 受信データを空読み
    unsigned char res = 0xff;
    while ((*SPBFDR & 0x00FF) > 0) {
	 res = *SPDR;
    }
    return res;
}

// データが送信されるまで待つ
void IMPL_SDSPI_WaitWrite()
{
    while ((*SPSR & 0x40) == 0x00) ; // 送信終了待ち
}

int IMPL_SDSPI_WaitResponse(unsigned char mask, unsigned char data)
{
    unsigned char res;
    unsigned long long tStart = SOLID_TIMER_GetCurrentTick();

    do {
	while ((*SPBFDR & 0x00FF) == 0) { // 受信データなし
	    if ((*SPBFDR & 0xFF00) == 0) { // データが無い場合 dummyを送信
		unsigned long long tPast = SOLID_TIMER_ToUsec(SOLID_TIMER_GetCurrentTick() - tStart);
		if (tPast > SD_POLL_TIMEOUT * 1000) {
		    return SOLID_ERR_TIMEOUT;
		}
		*SPDR = SDSPI_TXDUMMY;
	    }
	}
	res = *SPDR;
    } while ((res & mask) != data);

    return res;
}


void IMPL_SDSPI_Read(unsigned char *buf, int length)
{
    while (length > 0) {
	while ((*SPBFDR & 0x00FF) == 0)	{ // 受信データなし
	    if ((*SPBFDR & 0xFF00) == 0) {
		// データが無い場合 dummyを送信
		*SPDR = SDSPI_TXDUMMY;
	    }
	}
	char ch = *SPDR;
	if (buf != NULL) {
	    *buf = ch;
	    buf++;
	}
	length--;
    }
}

void IMPL_SDSPI_SetSpeed(unsigned long kbps)
{
    int n = (32000 / kbps) -1;

    if (n < 0) {
	n = 0;
    } else if (n > 255) {
	n = 255;
    }

    IMPL_SDSPI_WaitWrite();

    *SPBR = (unsigned char)n;
}


#include "solid_fs.h"
#include <string.h>
void IMPL_SDSPI_Test()
{
    int fd;

    if (SOLID_FS_Open(&fd, "\\FATFS\\SD\\TEST.TXT", O_RDWR | O_CREAT) >= 0) {
	const char *pStr = "Test from solid\n";
	size_t retlen;
	if (SOLID_FS_Write(fd, pStr, strlen(pStr), &retlen) < 0) goto close;
close:
	SOLID_FS_Close(fd);
    }
}
