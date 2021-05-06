/** @file main.cpp
 *  @brief Demonstrates/tests use of Lolin 2.13 inch ePaper Display
 *  
 */
#include <Arduino.h>
//#include <driver/can.h>
//#include <driver/gpio.h>
//#include <esp_system.h>
#include <stdio.h>
#include <stdlib.h>
//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
//#include "freertos/queue.h"
//#include "freertos/semphr.h"
#include <LOLIN_EPD.h>
#include <Adafruit_GFX.h>
#include "BackgroundImage.h"  //monochrome pixel array exported by GIMP as .h file

/*Pin assignments: D1 mini*/
#define EPD_CS 2
#define EPD_DC 5
#define EPD_RST 4  // can set to -1 and share with microcontroller Reset!
#define EPD_BUSY 12 // can set to -1 to not use a pin (will wait a fixed delay)
// Use Hardware SPI with EPD instantiation. Default d1_mini pins are:
// MOSI GPIO13 
// CLK  GPIO14 
#define SCREEN_WIDTH_PIXELS     250
#define SCREEN_HEIGHT_PIXELS    122

//=================== Globals ==============================
LOLIN_IL3897 EPD(SCREEN_WIDTH_PIXELS, SCREEN_HEIGHT_PIXELS, EPD_DC, EPD_RST, EPD_CS, EPD_BUSY); //hardware SPI

uint32_t lastTxNMEATimestamp;
const uint32_t interval = 1000;

//=================== Functions ==============================
/** 
 * @brief Renders an array of pixel values (1 = White, 0 = Black) into display buffer
 * 
 * Uses the EPD class in the LOLIN IL3897 library (modified).
 * @param pixelArray Pointer to array of pixel values
 * @param width Horizontal dimension of array
 * @param height Vertical dimension of array
 */
void DrawBitmap( const unsigned char *pixelArray, uint16_t width, uint16_t height ) {
  for (uint16_t x = 0; x < width; x++) {
    for (uint16_t y = 0; y < height; y++) {
      if (0 == pixelArray[x + width * y]) {
        EPD.drawPixel(x, y, EPD_BLACK);
      }
    }
  }
}//end DrawBitmap()

enum Status { okay, noComms, disconnected };

/**
 * @brief Render the graphical interface containing battery information
 *
 * Fills a working buffer in RAM with graphics and text.
 * Upload to display is performed elsewhere.
 * Uses the EPD class in the LOLIN IL3897 library (modified), and
 * the AdaFruit GFX library.
 * 
 * @param voltage Battery Voltage, in V
 * @param temperature Battery temperature, in K
 * @param chargeLevel Battery state of charge, as ratio [0..1]
 */
void RenderBatteryUI(float voltage, float temperature, float chargeLevel, Status batteryStatus ) {

  EPD.fillScreen(EPD_WHITE);  // floods screen buffer with white pixels

  // load graphics from #included GIMP-exported .h file
  DrawBitmap(header_data, SCREEN_WIDTH_PIXELS, SCREEN_HEIGHT_PIXELS);

  // Display voltage
  EPD.setCursor(4, 46);
  EPD.setTextColor(EPD_BLACK);
  EPD.setTextSize(5);
  EPD.printf("%4.1f", voltage);

  // Display battery temperature
  EPD.setCursor(75, 106);
  EPD.setTextSize(2);
  EPD.printf("%4.1f", temperature);

  // Display Charge Percent
  uint8_t chargePercent = (uint8_t)(chargeLevel * 100.0 + 0.5);
  uint8_t x, y;

  if (100 == chargePercent) {
    x = 160;
    EPD.setTextSize(4);
  } else if (10 > chargePercent) {
    x = 186;
    EPD.setTextSize(5);
  } else {
    x = 170;
    EPD.setTextSize(5);
  }
  y = 24;
  EPD.setCursor(x, y);
  EPD.print(chargePercent);

  // Fill charge percent bargraph
  uint8_t barTop = 2 + (uint8_t)((100.0 - (float)chargePercent) * 1.17);
  EPD.fillRect(233, barTop, 15, 119 - barTop, EPD_BLACK);

    EPD.setTextSize(1);
    EPD.setCursor(3, 88);
  switch( batteryStatus ) {
      case Status::okay:
        break;
      case Status::disconnected:  
        EPD.print("Battery disconnected");
        break;
    case Status::noComms:
        EPD.print("No communications");
        break;
    default:
        EPD.print("Unknown status");
        break;
  }
}//end RenderBatteryUI()


void SetupCanDriver()
{
    /*
 can_general_config_t general_config = {
        .mode = CAN_MODE_LISTEN_ONLY, //CAN_MODE_NORMAL,
        .tx_io = (gpio_num_t)GPIO_NUM_5,
        .rx_io = (gpio_num_t)GPIO_NUM_4,
        .clkout_io = (gpio_num_t)CAN_IO_UNUSED,
        .bus_off_io = (gpio_num_t)CAN_IO_UNUSED,
        .tx_queue_len = 5,
        .rx_queue_len = 5,
        .alerts_enabled = CAN_ALERT_NONE,
        .clkout_divider = 0};
    can_timing_config_t timing_config = CAN_TIMING_CONFIG_1MBITS();
    can_filter_config_t filter_config = CAN_FILTER_CONFIG_ACCEPT_ALL();
    esp_err_t error;

    error = can_driver_install(&general_config, &timing_config, &filter_config);
    if (error == ESP_OK)
    {
        Serial.println("CAN Driver installation success...");
    }
    else
    {
        Serial.println("CAN Driver installation fail...");
        return;
    }

    // start CAN driver
    error = can_start();
    if (error == ESP_OK)
    {
        Serial.println("CAN Driver start success...");
    }
    else
    {
        Serial.println("CAN Driver start FAILED...");
        return;
    }
    Serial.print("Running main loop()");
    */
}//end SetupCanDriver()

/*
Method to print the reason by which ESP
has been awoken from sleep
*/
void print_wakeup_reason(){
/*  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
  */
}

//=======================================================================
const uint32_t updateRateus = 30e6; //microseconds between updates

void setup() {
  Serial.begin(115200);
  Serial.println("\nBattery Monitor v1.1.0");

  SetupCanDriver();

  EPD.begin();

  Serial.printf("Providing status updates every %d seconds...\n", updateRateus / (uint32_t)1e6 );

}  // end setup()

/***
 * Stores the most-recently-displayed battery parameters.  Allows
 * us to not bother re-displaying the same numbers.  The contents
 * are stored in the user-section of RTC RAM, which persists through
 * resets (but not power-cycles). A flag with a magic number indicates
 * whether the other values are intentionally stored, or random data
 * following a power-cycle.
 */
struct BatteryDetails {
    uint32_t dataValidFlag;    //whether contents are valid (i.e. not fresh from powerup)
    uint32_t loopCount;    //incremented each time coming out of sleep
    float voltage;
    float chargeFraction;
    float temperature;
    Status status;
    bool isChanged; //whether any of data values newly read are different than previous
} batteryDetails;

const uint32_t magicValidValue = 0xabba0420; //presence indicates valid data
const uint32_t userRTCMemOffset = 68;  // start of user RAM in RTC peripheral

void loop() {
  // check whether this is a fresh start, or we have valid saved values from
  // previous runs
  if (true == ESP.rtcUserMemoryRead(userRTCMemOffset, (uint32 *)&batteryDetails,
                                    sizeof(struct BatteryDetails))) {
    if (batteryDetails.dataValidFlag == magicValidValue) {
      Serial.printf("RTC RAM seems valid:  ");
      Serial.printf("read loopCount = %d\n", batteryDetails.loopCount);

    } else {
      Serial.println("Just came out of power-on");
      batteryDetails.loopCount = 0;
      batteryDetails.dataValidFlag = magicValidValue;
      batteryDetails.isChanged = true;
    }
  } else {
    Serial.println("Couldn't read stored values.");
  }

  Serial.printf("System time: %d us\n", micros());
  batteryDetails.loopCount++;  // record that we're starting another loop

  //read the battery details from CAN bus
  batteryDetails.chargeFraction = 0.80;
  batteryDetails.temperature = 18.0;
  batteryDetails.voltage = 11.99;
  batteryDetails.status = Status::disconnected;
  //compare new values to old, and update -> batteryDetails.isChanged = true;

  if (batteryDetails.isChanged) {
    RenderBatteryUI(batteryDetails.voltage, batteryDetails.temperature,
                    batteryDetails.chargeFraction, batteryDetails.status);
    // upload buffer to Display
    EPD.display();
  }

  batteryDetails.isChanged = false; //display reflects current values
  if (true == ESP.rtcUserMemoryWrite(userRTCMemOffset, (uint32 *)&batteryDetails,
                                    sizeof(struct BatteryDetails))) {
    Serial.printf("incremented and saved batt details. Loop %d\n", batteryDetails.loopCount);
    }else {
    Serial.printf("Problem saving details. loopCount: %d\n", batteryDetails.loopCount);
    }

    ESP.deepSleep(updateRateus, RFMode::RF_DISABLED );
}  // end loop()