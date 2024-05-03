#include <Arduino.h>

#pragma once

void tp_lib_init();
esp_err_t tp_rx(uint8_t* data_out, uint32_t* data_len);