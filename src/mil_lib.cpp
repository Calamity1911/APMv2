#include <Arduino.h>
#include <SPIFFS.h>

#include "freertos/task.h"

// #define MIL_VERBOSE

// Checks for the presence of the dictionaries for the P0XXX codes
// Halts execution on a fail, not allowing the UI to boot
void check_dict_p() {
    
    // Check for the dictionary containing DTCs in the range P00XX
    if(!SPIFFS.exists("/dtc/p_00.txt")) {
        Serial.printf("[MIL] DTC/MIL library for PIDs P00XX doesn't exist\n");
        while(1) {vTaskDelay(pdMS_TO_TICKS(1000));}
    }

    // Check for the dictionary containing DTCs in the range P01XX
    if(!SPIFFS.exists("/dtc/p_01.txt")) {
        Serial.printf("[MIL] DTC/MIL library for PIDs P01XX doesn't exist\n");
        while(1) {vTaskDelay(pdMS_TO_TICKS(1000));}
    }

    // Check for the dictionary containing DTCs in the range P02XX
    if(!SPIFFS.exists("/dtc/p_02.txt")) {
        Serial.printf("[MIL] DTC/MIL library for PIDs P02XX doesn't exist\n");
        while(1) {vTaskDelay(pdMS_TO_TICKS(1000));}
    }

    // Check for the dictionary containing DTCs in the range P03XX
    if(!SPIFFS.exists("/dtc/p_03.txt")) {
        Serial.printf("[MIL] DTC/MIL library for PIDs P03XX doesn't exist\n");
        while(1) {vTaskDelay(pdMS_TO_TICKS(1000));}
    }

    // Check for the dictionary containing DTCs in the range P04XX
    if(!SPIFFS.exists("/dtc/p_04.txt")) {
        Serial.printf("[MIL] DTC/MIL library for PIDs P04XX doesn't exist\n");
        while(1) {vTaskDelay(pdMS_TO_TICKS(1000));}
    }

    // Check for the dictionary containing DTCs in the range P05XX
    if(!SPIFFS.exists("/dtc/p_05.txt")) {
        Serial.printf("[MIL] DTC/MIL library for PIDs P05XX doesn't exist\n");
        while(1) {vTaskDelay(pdMS_TO_TICKS(1000));}
    }

    // Check for the dictionary containing DTCs in the range P06XX
    if(!SPIFFS.exists("/dtc/p_06.txt")) {
        Serial.printf("[MIL] DTC/MIL library for PIDs P06XX doesn't exist\n");
        while(1) {vTaskDelay(pdMS_TO_TICKS(1000));}
    }

    // Check for the dictionary containing DTCs in the range P07XX
    if(!SPIFFS.exists("/dtc/p_07.txt")) {
        Serial.printf("[MIL] DTC/MIL library for PIDs P07XX doesn't exist\n");
        while(1) {vTaskDelay(pdMS_TO_TICKS(1000));}
    }

    // Check for the dictionary containing DTCs in the range P08XX
    if(!SPIFFS.exists("/dtc/p_08.txt")) {
        Serial.printf("[MIL] DTC/MIL library for PIDs P08XX doesn't exist\n");
        while(1) {vTaskDelay(pdMS_TO_TICKS(1000));}
    }

    // Check for the dictionary containing DTCs in the range P09XX
    if(!SPIFFS.exists("/dtc/p_09.txt")) {
        Serial.printf("[MIL] DTC/MIL library for PIDs P09XX doesn't exist\n");
        while(1) {vTaskDelay(pdMS_TO_TICKS(1000));}
    }
}

void mil_init() {

    check_dict_p();

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

// Converts a DTC being represented as a uint16 to a DTC string, ex P0123
void decode_dtc(uint16_t plain_dtc, char str_dtc[6]) {

    // Characters of the DTC string
    char letter = 0x00;
    char thousands = 0x00;
    char hundreds = 0x00;
    char tens = 0x00;
    char ones = 0x00;

    // Determine the letter at the beginning of the DTC
    // Only 4 legal cases b/c the mask is 2 bits, so a switch statement is fine to write
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

    // Determine the thousands digit of the DTC. Any number besides 0 is mfgr specific
    // Only 4 legal cases b/c the mask is 2 bits, so a switch statement is fine to write
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
    // This is Evan's jank way to convert from BCD to Hex characters
    // The mask is actually just a hex value and usually is only decimal
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

    // Read from the matching dictionary to see if we have a description
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

// Condenses pairs of bytes in a provided list into 16-bit DTCs
esp_err_t bytes_to_dtcs(const uint8_t* in, uint16_t* out, uint32_t in_len, uint32_t* out_len) {

    // No DTCs
    if(in_len == 0) {
        *out_len = 0;
        return ESP_OK;
    }

    // Not a multiple of 2 bytes, so not a whole number of DTCs
    if(in_len % 2 != 0) {
        Serial.printf("[MIL] Provided input data isn't an even length, and DTCs are represented as two bytes\n");
        return ESP_FAIL;
    }

    // Check if all DTCs will fit in output buffer
    uint32_t dtc_count = in_len / 2;
    if(dtc_count > *out_len) {
        Serial.printf("[MIL] DTC output buffer is too small for provided data. %d DTCs in the input, output is %d long.\n", dtc_count, *out_len);
        return ESP_FAIL;
    }

    // Do the actual condensing
    for(int i = 0; i < in_len; i+=2) {
        out[i/2] = (in[i+0] << 8) | in[i+1];
    }

    // Update output length
    *out_len = dtc_count;
    
    return ESP_OK;

}