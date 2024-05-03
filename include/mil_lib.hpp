#include <Arduino.h>

#pragma once

void mil_init();
bool mil_description(uint16_t plain_dtc, char* out);
void decode_dtc(uint16_t plain_dtc, char str_dtc[6]);
esp_err_t bytes_to_dtcs(const uint8_t* in, uint16_t* out, uint32_t in_len, uint32_t* out_len);