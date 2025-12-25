#include <Arduino.h>
#include "Arduino_GFX_Library.h"
#include <Adafruit_XCA9554.h>
#include "pin_config.h"
#include <Wire.h>

Adafruit_XCA9554 expander;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS /* CS */, LCD_SCLK /* SCK */, LCD_SDIO0 /* SDIO0 */, LCD_SDIO1 /* SDIO1 */,
  LCD_SDIO2 /* SDIO2 */, LCD_SDIO3 /* SDIO3 */);

Arduino_SH8601 *gfx = new Arduino_SH8601(
    bus, GFX_NOT_DEFINED /* RST */, 0 /* rotation */, LCD_WIDTH /* width */, LCD_HEIGHT /* height */);

void setup(void) {
  Serial.begin(115200);
  // Serial.setDebugOutput(true);
  // while(!Serial);

  Wire.begin(IIC_SDA, IIC_SCL);

  if (!expander.begin(0x20)) {  // Replace with actual I2C address if different
    Serial.println("Failed to find XCA9554 chip");
    while (1)
      ;
  }

  expander.pinMode(4, OUTPUT);
  expander.pinMode(5, OUTPUT);
  expander.digitalWrite(4, 1);
  expander.digitalWrite(5, 1);
  delay(500);
  Serial.println("Arduino_GFX Hello World example");

  // Init Display
  if (!gfx->begin()) {
    Serial.println("gfx->begin() failed!");
  }

  gfx->fillScreen(RGB565_BLACK);

  gfx->setBrightness(255);

  gfx->setCursor(10, 10);
  gfx->setTextColor(RGB565_RED);
  gfx->println("Hello World!");

  delay(5000);  // 5 seconds
}

void loop() {
  gfx->setCursor(random(gfx->width()), random(gfx->height()));
  gfx->setTextColor(random(0xffff), random(0xffff));
  gfx->setTextSize(random(6) /* x scale */, random(6) /* y scale */, random(2) /* pixel_margin */);
  gfx->println("Hello World!");
  Serial.println("loop");
  delay(200);
}
