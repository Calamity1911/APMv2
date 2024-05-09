#include <Arduino.h>

#pragma once

typedef struct {
    uint8_t PIDS[3];
    uint32_t VALS[3];
} obd_readings_t;

void obd_dict_check();
void remove_front(uint8_t* data, uint32_t* len);
bool obd_errchk(uint8_t* data);
esp_err_t poll_all_pids();

bool get_pid_name(uint8_t PID, char out[64]);
char get_pid_formula(uint8_t PID);

/* All of the formulas for converting raw ISO-TP data below */

int32_t pid_formula_a(uint8_t* data, uint32_t len);
int32_t pid_formula_b(uint8_t* data, uint32_t len);
float   pid_formula_c(uint8_t* data, uint32_t len);
int32_t pid_formula_d(uint8_t* data, uint32_t len);
float   pid_formula_e(uint8_t* data, uint32_t len);
float   pid_formula_f(uint8_t* data, uint32_t len);
int32_t pid_formula_g(uint8_t* data, uint32_t len);
float   pid_formula_h(uint8_t* data, uint32_t len);
float   pid_formula_i(uint8_t* data, uint32_t len);
float   pid_formula_j(uint8_t* data, uint32_t len);
float   pid_formula_k(uint8_t* data, uint32_t len);
float   pid_formula_l(uint8_t* data, uint32_t len);