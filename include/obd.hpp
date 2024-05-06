#include <Arduino.h>

#pragma once

typedef struct {
    uint8_t PIDS[3];
    uint32_t VALS[3];
} obd_readings_t;

void remove_front(uint8_t* data, uint32_t* len);
bool obd_errchk(uint8_t* data);
esp_err_t poll_all_pids();