/*Using LVGL with Arduino requires some extra steps:
 *Be sure to read the docs here: https://docs.lvgl.io/master/get-started/platforms/arduino.html  */


//USB CDC On Boot: Enabled
//Core Debug Level: Debug
//CPU Frequency: 240MHz
//PSRAM: OPI PSRAM
//Flash size: 16MB
//Flash mode: QIO 80MHz
//Partition scheme: 16MB Flash / 3MB APP
//


#include "Wireless.h"
#include "Gyro_QMI8658.h"
#include "RTC_PCF85063.h"
#include "SD_Card.h"
#include "LVGL_Driver.h"
#include "LVGL_Example.h"
#include "BAT_Driver.h"


void Driver_Loop(void *parameter)
{
  while(1)
  {
    QMI8658_Loop();
    RTC_Loop();
    BAT_Get_Volts();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
void Driver_Init()
{
  Flash_test();
  BAT_Init();
  I2C_Init();
  TCA9554PWR_Init(0x00);   
  Set_EXIO(EXIO_PIN8,Low);
  PCF85063_Init();
  QMI8658_Init(); 
  
  xTaskCreatePinnedToCore(
    Driver_Loop,     
    "Other Driver task",   
    4096,                
    NULL,                 
    3,                    
    NULL,                
    0                    
  );
}

 // If you later reinitialize the LCD, you must initialize the SD card again !!!!!!!!!!
 // It must be initialized after the LCD, and if the LCD is reinitialized later, the SD also needs to be reinitialized
void setup()
{
  Serial.begin(115200);
  delay(2000);
  Serial.println("1 start");
  Wireless_Test2();
  Serial.println("2 wireless ok");
  Driver_Init();
  Serial.println("3 driver ok");
  LCD_Init();
  Serial.println("4 lcd ok");
  SD_Init();
  Serial.println("5 sd ok");
  Lvgl_Init();
  Serial.println("6 lvgl ok");
  Lvgl_Example1();
  Serial.println("7 main budík ok");
}

void loop()
{
  Lvgl_Loop();
  vTaskDelay(pdMS_TO_TICKS(5));
}
