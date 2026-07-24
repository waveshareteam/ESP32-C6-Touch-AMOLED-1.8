#pragma once

#define XPOWERS_CHIP_AXP2101

#define LCD_SDIO0 1
#define LCD_SDIO1 2
#define LCD_SDIO2 3
#define LCD_SDIO3 4
#define LCD_SCLK  0
#define LCD_CS 5
#define LCD_WIDTH 368
#define LCD_HEIGHT 448

// TOUCH
#define IIC_SDA 8
#define IIC_SCL 7
#define TP_INT 15

// ES8311
#define I2S_MCK_IO 19
#define I2S_BCK_IO 20
#define I2S_DI_IO 21
#define I2S_WS_IO 22
#define I2S_DO_IO 23


#define MCLKPIN             19
#define BCLKPIN             20
#define WSPIN               22
#define DOPIN               21
#define DIPIN               23

// SD
const int SDMMC_CLK = 11;
const int SDMMC_CMD = 10;
const int SDMMC_DATA = 18;
const int SDMMC_CS   = 6;