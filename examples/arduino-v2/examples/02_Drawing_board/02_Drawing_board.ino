#include <Wire.h>
#include <Arduino.h>
#include "pin_config.h"
#include "Arduino_GFX_Library.h"
#include "Arduino_DriveBus_Library.h"
#include <Adafruit_XCA9554.h>
#include "HWCDC.h"
HWCDC USBSerial;

Adafruit_XCA9554 expander;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS /* CS */, LCD_SCLK /* SCK */, LCD_SDIO0 /* SDIO0 */, LCD_SDIO1 /* SDIO1 */,
  LCD_SDIO2 /* SDIO2 */, LCD_SDIO3 /* SDIO3 */);

Arduino_CO5300 *gfx = new Arduino_CO5300(
    bus, GFX_NOT_DEFINED /* RST */, 0 /* rotation */, LCD_WIDTH /* width */, LCD_HEIGHT /* height */, 16, 0, 0, 0);

std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus =
  std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);

void Arduino_IIC_Touch_Interrupt(void);

// CST820 is handled by the compatible Arduino_CST816x driver.
std::unique_ptr<Arduino_IIC> CST820(new Arduino_CST816x(IIC_Bus, CST816T_DEVICE_ADDRESS,
                                                       DRIVEBUS_DEFAULT_VALUE, TP_INT, Arduino_IIC_Touch_Interrupt));

void Arduino_IIC_Touch_Interrupt(void) {
  CST820->IIC_Interrupt_Flag = true;
}

void setup() {
  USBSerial.begin(115200);
  USBSerial.setTxTimeoutMs(0);  // Prevent debug output from blocking the sketch.
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
  while (CST820->begin() == false) {
    USBSerial.println("CST820 initialization fail");
    delay(2000);
  }
  USBSerial.println("CST820 initialization successfully");

  CST820->IIC_Write_Device_State(CST820->Arduino_IIC_Touch::Device::TOUCH_DEVICE_INTERRUPT_MODE,
                                 CST820->Arduino_IIC_Touch::Device_Mode::TOUCH_DEVICE_INTERRUPT_PERIODIC);

  gfx->begin();
  gfx->fillScreen(RGB565_WHITE);

  for (int i = 0; i <= 255; i++)  //0-255
  {
    gfx->setBrightness(i);
    gfx->setCursor(30, 150);
    gfx->setTextColor(RGB565_BLUE);
    gfx->setTextSize(4);
    gfx->println("Loading board");
    delay(3);
  }
  delay(500);
  gfx->fillScreen(RGB565_WHITE);
}

void loop() {
  static uint32_t lastTouchRead = 0;
  static int32_t lastTouchX = -1;
  static int32_t lastTouchY = -1;
  const uint32_t now = millis();
  if (now - lastTouchRead < 10) {
    delay(1);
    return;
  }
  lastTouchRead = now;

  const int32_t touchPoints =
    CST820->IIC_Read_Device_Value(CST820->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);
  if (touchPoints > 0) {
    const int32_t touchX =
      CST820->IIC_Read_Device_Value(CST820->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
    const int32_t touchY =
      CST820->IIC_Read_Device_Value(CST820->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);
    if (USBSerial) {
      USBSerial.printf("Touch X:%d Y:%d\n", touchX, touchY);
    }
    if (touchX > 20 && touchY > 20 && (touchX != lastTouchX || touchY != lastTouchY)) {
      if (lastTouchX >= 0 && lastTouchY >= 0) {
        gfx->drawLine(lastTouchX, lastTouchY, touchX, touchY, RGB565_BLUE);
      }
      gfx->fillCircle(touchX, touchY, 5, RGB565_BLUE);
      lastTouchX = touchX;
      lastTouchY = touchY;
    }
  } else {
    lastTouchX = -1;
    lastTouchY = -1;
  }
}
