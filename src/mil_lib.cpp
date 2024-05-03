#include <Arduino.h>
#include <SPIFFS.h>

#include "freertos/task.h"

// #define MIL_VERBOSE

void mil_init() {

    if(!SPIFFS.exists("/dtc/p_00.txt")) {
        Serial.printf("[MIL] DTC/MIL library for PIDs P00XX do not exist\n");
        while(1) {vTaskDelay(pdMS_TO_TICKS(1000));}
    }

    if(!SPIFFS.exists("/dtc/p_01.txt")) {
        Serial.printf("[MIL] DTC/MIL library for PIDs P01XX do not exist\n");
        while(1) {vTaskDelay(pdMS_TO_TICKS(1000));}
    }

    if(!SPIFFS.exists("/dtc/p_02.txt")) {
        Serial.printf("[MIL] DTC/MIL library for PIDs P02XX do not exist\n");
        while(1) {vTaskDelay(pdMS_TO_TICKS(1000));}
    }

    if(!SPIFFS.exists("/dtc/p_03.txt")) {
        Serial.printf("[MIL] DTC/MIL library for PIDs P03XX do not exist\n");
        while(1) {vTaskDelay(pdMS_TO_TICKS(1000));}
    }

    if(!SPIFFS.exists("/dtc/p_04.txt")) {
        Serial.printf("[MIL] DTC/MIL library for PIDs P04XX do not exist\n");
        while(1) {vTaskDelay(pdMS_TO_TICKS(1000));}
    }

    if(!SPIFFS.exists("/dtc/p_05.txt")) {
        Serial.printf("[MIL] DTC/MIL library for PIDs P05XX do not exist\n");
        while(1) {vTaskDelay(pdMS_TO_TICKS(1000));}
    }

    if(!SPIFFS.exists("/dtc/p_06.txt")) {
        Serial.printf("[MIL] DTC/MIL library for PIDs P06XX do not exist\n");
        while(1) {vTaskDelay(pdMS_TO_TICKS(1000));}
    }

    if(!SPIFFS.exists("/dtc/p_07.txt")) {
        Serial.printf("[MIL] DTC/MIL library for PIDs P07XX do not exist\n");
        while(1) {vTaskDelay(pdMS_TO_TICKS(1000));}
    }

    if(!SPIFFS.exists("/dtc/p_08.txt")) {
        Serial.printf("[MIL] DTC/MIL library for PIDs P08XX do not exist\n");
        while(1) {vTaskDelay(pdMS_TO_TICKS(1000));}
    }

    if(!SPIFFS.exists("/dtc/p_09.txt")) {
        Serial.printf("[MIL] DTC/MIL library for PIDs P09XX do not exist\n");
        while(1) {vTaskDelay(pdMS_TO_TICKS(1000));}
    }

    Serial.printf("[MIL] Library checks passed!\n");

}

/**
 * Reads the DTC dictionary files stored locally on SPIFFS
 * 
 * @param in 5-character DTC with a null terminator (6 bytes total)
 * @param out 256-byte buffer where the decoded DTC description will go
 * 
 */
void read_dictionary_p(const char* in, char* out) {

    char category = in[2];

    File dictionary_page;

    // Open the correct dictionary page based on the category of DTC
    // P00XX, P01XX, P02XX, ... , P09XX
    switch(category) {

        case '0':
            dictionary_page = SPIFFS.open("/dtc/p_00.txt");
            break;
        
        case '1':
            dictionary_page = SPIFFS.open("/dtc/p_01.txt");
            break;
        
        case '2':
            dictionary_page = SPIFFS.open("/dtc/p_02.txt");
            break;
        
        case '3':
            dictionary_page = SPIFFS.open("/dtc/p_03.txt");
            break;

        case '4':
            dictionary_page = SPIFFS.open("/dtc/p_04.txt");
            break;
        
        case '5': 
            dictionary_page = SPIFFS.open("/dtc/p_05.txt");
            break;

        case '6':
            dictionary_page = SPIFFS.open("/dtc/p_06.txt");
            break;
        
        case '7':
            dictionary_page = SPIFFS.open("/dtc/p_07.txt");
            break;
        
        case '8':
            dictionary_page = SPIFFS.open("/dtc/p_08.txt");
            break;
        
        case '9':
            dictionary_page = SPIFFS.open("/dtc/p_09.txt");
            break;

        default:
            Serial.printf("[MIL] DTC \"%s\" appears to be valid, but does not exist in the dictionary\n", in);
            sprintf(out, "%s: Unrecognized code", in);
            return;
    }

    // Read one line at a time
    String line = dictionary_page.readStringUntil('\n');

    // Line will be empty when we hit the end of the file
    while(!line.isEmpty()) {
        
        // Extract the DTC from the description
        String dictionary_dtc = line.substring(0, 5);

        // Compare to provided DTC, break from the loop if equal
        if(strcmp(in, dictionary_dtc.c_str()) == 0) {
            break;
        }

        // Otherwise advance to next line
        line = dictionary_page.readStringUntil('\n');
    }

    // Close the file you dummy!
    dictionary_page.close();

    // If the line is empty, a match wasn't found
    if( !line.isEmpty() ) {

        #ifdef MIL_VERBOSE
        Serial.printf("[MIL] Found a description for DTC %s\n", in);
        #endif

        strncpy(out, line.c_str(), 256);
    } else {
        Serial.printf("[MIL] DTC \"%s\" appears to be valid, but does not exist in the dictionary\n", in);
        sprintf(out, "%s: Unrecognized code", in);
    }

}

void decode_dtc(uint16_t plain_dtc, char str_dtc[6]) {

    char letter = 0x00;
    char thousands = 0x00;
    char hundreds = 0x00;
    char tens = 0x00;
    char ones = 0x00;

    uint8_t mask = (plain_dtc & 0xC000) >> 14;
    switch(mask) {
        case 0x00:
            letter = 'P';
            break;
        
        case 0x01:
            letter = 'C';
            break;
        
        case 0x02:
            letter = 'B';
            break;
        
        case 0x03:
            letter = 'U';
            break;
        
        default:
            Serial.printf("[MIL] Bitmask for DTC letter was invalid somehow, %02X\n", mask);
            letter = '~';
    }

    mask = (plain_dtc & 0x3000) >> 12;
    switch(mask) {
        case 0x00:
            thousands = '0';
            break;
        
        case 0x01:
            thousands = '1';
            break;
        
        case 0x02:
            thousands = '2';
            break;
        
        case 0x03:
            thousands = '3';
            break;
        
        default:
            Serial.printf("[MIL] Bitmask for thousands place digit was invalid somehow, %02X\n", mask);
            thousands = '~';
    }

    // I feel like twhat follows is illegal
    mask = (plain_dtc & 0xF00) >> 8;
    if(mask < 10) {
        hundreds = '0' + mask;
    } else {
        mask -= 10;
        hundreds = 'A' + mask;
    }

    mask = (plain_dtc & 0xF0) >> 4;
    if(mask < 10) {
        tens = '0' + mask;
    } else {
        mask -= 10;
        tens = 'A' + mask;
    }

    mask = (plain_dtc & 0xF) >> 0;
    if(mask < 10) {
        ones = '0' + mask;
    } else {
        mask -= 10;
        ones = 'A' + mask;
    }

    sprintf(str_dtc, "%c%c%c%c%c", letter, thousands, hundreds, tens, ones);
    
    #ifdef MIL_VERBOSE
    Serial.printf("[MIL] Decoded DTC %s from 0x%04X\n", str_dtc, plain_dtc);
    #endif

}

bool mil_description(uint16_t plain_dtc, char* out) {

    // All DTCs are 5 chars plus a null
    char str_dtc[6] = {0};
    decode_dtc(plain_dtc, str_dtc);

    // 'P', 'C', 'B', 'U' 
    char origin = str_dtc[0];

    // '1', '0'
    char mfg_specific = str_dtc[1];

    // Standard DTCs can only contain P, C, B, or U as the origin
    if( !(origin == 'P' || origin == 'C' || origin == 'B' || origin == 'U') ) {
        Serial.printf("[MIL] Origin of provided DTC is %c\n", origin);
        return false;
    }

    // DTCs in the range X1XXX are manufacturer-specific
    if(mfg_specific == '1') {

        #ifdef MIL_VERBOSE
        Serial.printf("[MIL] DTC \"%s\" appears to be valid but is manufacturer specific\n", str_dtc);
        #endif
        
        sprintf(out, "%s: Manufacturer specific", str_dtc);
        return true;
    }

    switch(origin) {
        case 'P':
            read_dictionary_p(str_dtc, out);
            break;
        
        default:
            Serial.printf("[MIL] Invalid state, this means that the origin of the DTC was not P, C, B, or U but it made it past the checks. Provided DTC was %s\n", str_dtc);
            return false;
    }

    return true;
}

esp_err_t bytes_to_dtcs(const uint8_t* in, uint16_t* out, uint32_t in_len, uint32_t* out_len) {

    if(in_len == 0) {
        *out_len = 0;
        return ESP_OK;
    }

    if(in_len % 2 != 0) {
        Serial.printf("[MIL] Provided input data isn't an even length, and DTCs are represented as two bytes\n");
        return ESP_FAIL;
    }

    uint32_t dtc_count = in_len / 2;
    if(dtc_count > *out_len) {
        Serial.printf("[MIL] DTC output buffer is too small for provided data. %d DTCs in the input, output is %d long.\n", dtc_count, *out_len);
        return ESP_FAIL;
    }

    for(int i = 0; i < in_len; i+=2) {
        out[i/2] = (in[i+0] << 8) | in[i+1];
    }

    *out_len = dtc_count;
    return ESP_OK;

}