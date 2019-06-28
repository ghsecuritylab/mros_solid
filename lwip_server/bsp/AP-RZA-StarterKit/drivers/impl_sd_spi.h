#ifndef _IMPL_SD_SPI_H_
#define _IMPL_SD_SPI_H_

#include "solid_fatfs_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

void IMPL_SDSPI_Init();

// 上位レイヤ
int IMPL_SDSPI_open(void* param);

int IMPL_SDSPI_close(void* param);

int IMPL_SDSPI_isAvailable(void* param);

int IMPL_SDSPI_getDeviceInfo(void *param, SOLID_FATFS_DRIVER_INFO* info);

int IMPL_SDSPI_read(void *param, unsigned long addr, void* buf, size_t count);

int IMPL_SDSPI_write(void *param, unsigned long addr, const void *buf, size_t count);

int IMPL_SDSPI_flush(void *param);

// SDコマンド用
int IMPL_SDSPI_cmd(unsigned char cmd, int arg, unsigned char* res, int reslen);

int IMPL_SDSPI_readblk(unsigned long addr, void *buf);

int IMPL_SDSPI_writeblk(unsigned long addr, const void *buf);

// 下位レイヤ
// SD用SPIを初期化する
void IMPL_SDSPI_DeviceInit();

// CD信号をチェックする
int IMPL_SDSPI_CheckCD();

// CSをアサート/ネゲートする
void IMPL_SDSPI_SetCS(int bAssert);

// データをスレーブに送信する
unsigned char IMPL_SDSPI_Write(unsigned char d);

// データが送信されるまで待つ
void IMPL_SDSPI_WaitWrite();

// レスポンスを受信する
int IMPL_SDSPI_WaitResponse(unsigned char rxMask, unsigned char rxData);

// データを受信する
void IMPL_SDSPI_Read(unsigned char* buf, int length);

// 送受信速度を変更する
void IMPL_SDSPI_SetSpeed(unsigned long kbps);


// テスト用コード
void IMPL_SDSPI_Test();


#ifdef  __cplusplus
}
#endif

#endif