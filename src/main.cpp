#include <Arduino.h>
#include <SPIFFS.h>

#include "FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"

#include "mil_lib.hpp"
#include "iso_tp.hpp"
#include "obd.hpp"
#include "web.hpp"

#define OBD2_ACCEPTANCE_CODE (0x7E8 << 21)
#define OBD2_ACCEPTANCE_MASK 0x1FFFFF
#define CAN_PIN_TX (gpio_num_t)5
#define CAN_PIN_RX (gpio_num_t)6

TaskHandle_t WEB_TASK;

void halt() {
  while(1) {vTaskDelay(pdMS_TO_TICKS(1000));}
}

void setup() {
	
  Serial.begin(9600);
  delay(1000);

  // Initialize SPIFFS filesystem
  if(!SPIFFS.begin()) {
    Serial.printf("[MAIN] Failed to initialize SPIFFS\n");
    halt();
  }

  esp_err_t INIT_STATUS;

  // I/O Pin config and operation mode
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_PIN_TX, CAN_PIN_RX, TWAI_MODE_NORMAL);

  // Baud rate config
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  
  // Message filter config
  twai_filter_config_t f_config = {0};
  f_config.single_filter = true;
  f_config.acceptance_code = OBD2_ACCEPTANCE_CODE;
  f_config.acceptance_mask = OBD2_ACCEPTANCE_MASK;

  // Install configs
  INIT_STATUS = twai_driver_install( &g_config, &t_config, &f_config );
  if(INIT_STATUS != ESP_OK) {
    Serial.printf("[MAIN] Failed to install driver with error code 0x%04X %s\n", INIT_STATUS, esp_err_to_name(INIT_STATUS));
    halt();
  }

  // Start peripheral driver
  INIT_STATUS = twai_start();
  if(INIT_STATUS != ESP_OK) {
    Serial.printf("[MAIN] Failed to start driver with error code 0x%04X %s\n", INIT_STATUS, esp_err_to_name(INIT_STATUS));
    halt();
  }

  Serial.printf("[MAIN] CAN peripheral initialized!\n");

  // Init libs
  tp_lib_init();
  mil_init();

  // Testing this function eventually
  esp_err_t result = poll_all_pids();
  Serial.printf("[MAIN] PID Support polling returned 0x%04X %s\n", result, esp_err_to_name(result));

  // Disabled temporarily while Evan works on OBD lib
  // xTaskCreate((TaskFunction_t)web_main, "WEB", 65535, (void*)1, 10, &WEB_TASK);

}

void loop() {
  return;
}
