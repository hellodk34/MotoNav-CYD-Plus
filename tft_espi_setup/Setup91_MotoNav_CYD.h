// TFT_eSPI 配置 —— MotoNav-CYD2USB（ESP32-2432S028R 双 USB 版）
// 屏幕：ST7789，240x320 竖屏原生，固件里用 tft.setRotation(1) 转成 320x240 横屏。
// 触摸(XPT2046)和 microSD 由固件自己驱动（见 config.h），不经 TFT_eSPI，这里只配 TFT。

#define USER_SETUP_INFO "MotoNav-CYD2USB (ESP32-2432S028R) ST7789 240x320"

#define ST7789_DRIVER

#define TFT_WIDTH  240
#define TFT_HEIGHT 320

// TFT SPI 引脚（与原版 CYD 一致）
#define TFT_MISO 12
#define TFT_MOSI 13
#define TFT_SCLK 14
#define TFT_CS   15
#define TFT_DC   2
#define TFT_RST  -1   // 此板复位脚未接到 ESP32
#define TFT_BL   21   // 背光 LED（GPIO21）
#define TFT_BACKLIGHT_ON HIGH  // 背光高电平点亮（必须定义，否则 TFT_eSPI 不会驱动背光）

// ST7789 面板一般是 BGR 颜色顺序
#define TFT_RGB_ORDER TFT_BGR

// 字体：必须开 GFXFF，固件用了 FreeSansBold（由 TFT_eSPI 自带，勿在 sketch 里重复 include）
#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define LOAD_GFXFF
#define SMOOTH_FONT

#define SPI_FREQUENCY        40000000
#define SPI_READ_FREQUENCY   20000000
#define SPI_TOUCH_FREQUENCY   2500000
