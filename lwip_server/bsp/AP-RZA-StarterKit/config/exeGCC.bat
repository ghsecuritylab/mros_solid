@echo off
set GCCDIR=%EXEGCC_STD_ARM_ROOTDIR%
set GCC_CPU=armv7aLai
set GCC_DEFAULT=armv7aLain
set GCCSW= -mcpu=cortex-a9 -mthumb-interwork -mfloat-abi=soft -D_SOFT_FLOAT -DENDIAN_BIG=0 -D__LITTLE_ENDIAN -D__MSL_NO_IO_SUPPORT__ -DFAST -fsigned-char -fomit-frame-pointer -D__armv7aLai__ -finput-charset=utf-8
set GCC_MODE=ARM
set EXEGCC=ON
set SYSROOT=%GCCDIR%
set COMPILER_PATH=%GCCDIR%\bin
set PATH=%COMPILER_PATH%;%PATH%
set LANG=C-SJIS
set GCCREL=4

echo プロセッサ : Cortex-A9
echo 命令モード : ARM
echo 呼び出しモード : Interwork
echo VFP : VFP/FPUを使用しない(soft)
echo エンディアン : Little endian
echo VLINK : OFF
title exeGCC4 armv7aLain
%ComSpec%
