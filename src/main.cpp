#include <Arduino.h>
#include <driver/can.h>
#include <driver/gpio.h>
#include <esp_system.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// define your CAN messages here, OR you can define them locally...
// standard 11-bit frame ID = 0
// extended 29-bit frame ID = 1
// format:   can_message_t (name of your message) = {std/ext frame, message ID, message DLC, {data bytes here}};

can_message_t myMessageToSend = {0, 0x123, 8, {0x01, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x99}};

uint32_t lastTxNMEATimestamp;
const uint32_t interval = 1000;

//=================================================
void SetupCanDriver();


void setup()
{
  Serial.begin(115200);
  // anything else you might need to do....
  SetupCanDriver();
}

void SetupCanDriver()
{
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
}

//=======================================================================

void loop()
{
     can_message_t rx_frame;
    if (can_receive(&rx_frame, pdMS_TO_TICKS(1000)) == ESP_OK)
    {
      Serial.printf("ID=0x%03X  Data:", rx_frame.identifier);
      for( int i = 0; i < rx_frame.data_length_code; i++ ) 
      { Serial.printf("%02X ", rx_frame.data[i]);
      }
      Serial.println("");
      // Serial.print("R");
    }
    
     if (millis() - lastTxNMEATimestamp > interval)  // send out your CAN frame once every second
        { 
            lastTxNMEATimestamp = millis();
            can_transmit(&myMessageToSend, pdMS_TO_TICKS(1000));
            Serial.print(".");
      }
}