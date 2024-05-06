#include <Arduino.h>
#include "driver/twai.h"
#include "iso_tp.hpp"
#include "obd.hpp"

#define OBD_BROADCAST_ID 0x7DF

uint8_t POLL_PIDS[3];

// Only supporting PIDs 0x00 thru 0x7F = 128 PIDs
uint8_t SUPPORTED_PIDS[256];

// Removes 0th element and shifts remaining data over
void remove_front(uint8_t* data, uint32_t* len) {

    // Don't shift, just set values
    if(*len <= 1) {
        data[0] = 0;
        *len = 0;
        return;
    }

  // Create a temporary buffer whose size is the same as the input
  uint8_t temp[(*len)] = {0};

  // Copy data starting at the first element into the temp buffer (adjust size to ignore the 0th element)
  memcpy(temp, &(data[1]), (*len)-1);

  // Clear the original
  memset(data, 0, *len);

  // Copy in the shifted data
  memcpy(data, temp, (*len)-1);

  // Decrease the size of the output by one
  *len -= 1;

}

// Reverses bit order of a provided byte by using a lookup table
uint8_t reverse_byte(uint8_t x) {
    static const uint8_t table[] = {
        0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0,
        0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
        0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8,
        0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
        0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4,
        0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
        0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec,
        0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
        0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2,
        0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
        0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea,
        0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
        0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6,
        0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
        0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee,
        0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
        0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1,
        0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
        0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9,
        0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
        0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5,
        0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
        0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed,
        0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
        0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3,
        0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
        0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb,
        0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
        0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7,
        0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
        0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef,
        0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff,
    };
    return table[x];
}

// For now, just checks the 0th element for a positive response indicator
bool obd_errchk(uint8_t* data) {
    
    // Positive response check
    return (data[0] & 0xF0 == 0x40);
}

// Uses PID 0x00 to poll the vehicle for supported PIDs in the range [0x01, 0x20] inclusive
esp_err_t poll_supported_00() {

    // Always assume PID 0x00 is supported, as this is used to poll
    // for other supported PIDs
    memset(SUPPORTED_PIDS, 0, 256);
    SUPPORTED_PIDS[0x00] = 1;

    // data[0] = length of valid data
    // data[1] = obd2 mode (01 for current data)
    // data[2] = PID
    static const uint8_t REQUEST_PAYLOAD[] = {0x02, 0x01, 0x00, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC};

    // Build a response buffer, oversized just in case. Should only need to be 6 bytes
    uint32_t RESPONSE_LEN = 16;
    uint8_t RESPONSE_DATA[RESPONSE_LEN] = {0};

    // Create the request message
    twai_message_t request = {0};
    request.data_length_code = 8;
    request.identifier = OBD_BROADCAST_ID;
    memcpy(request.data, REQUEST_PAYLOAD, 8);

    // TX the request
    esp_err_t result = twai_transmit(&request, pdMS_TO_TICKS(500));
    if(result != ESP_OK) {
        Serial.printf("[OBD] Failed to transmit request, 0x%04X %s\n", result, esp_err_to_name(result));
        return result;
    }

    // Assuming TX success, immediately start RX
    result = tp_rx(RESPONSE_DATA, &RESPONSE_LEN);
    if(result != ESP_OK) {
        Serial.printf("[OBD] Failed to receive request response, 0x%04X %s\n", result, esp_err_to_name(result));
        return result;
    }

    // Check for a PRSP indicator
    if(!obd_errchk(RESPONSE_DATA)) {
        Serial.printf("[OBD] Response error check failed\n");
        return ESP_FAIL;
    }

    // If the response is positive, there are two protocol bytes at the beginning which can
    // be discarded/disregarded
    remove_front(RESPONSE_DATA, &RESPONSE_LEN);
    remove_front(RESPONSE_DATA, &RESPONSE_LEN);

    // There should only be 4 bytes left at this point, check that
    if(RESPONSE_LEN != 4) {
        Serial.printf("[OBD] Not enough bytes, expected 4 but  got %d\n", RESPONSE_LEN);
        return ESP_FAIL;
    }

    // At this point, swap bit orders of the remaining bytes to make it easier
    // for me as a human to decode

    uint8_t A = reverse_byte(RESPONSE_DATA[0]);
    uint8_t B = reverse_byte(RESPONSE_DATA[1]);
    uint8_t C = reverse_byte(RESPONSE_DATA[2]);
    uint8_t D = reverse_byte(RESPONSE_DATA[3]);

    uint32_t support_mask = (D << 24) | (C << 16) | (B << 8) | (A << 0);

    // Extract all bits and update the corresponding cell in the SUPPORTED_PIDS list
    for(int i = 0x00; i < 0x20; i++) {
        // 0x01 comes from the fact that PID 0x00 is assumed to be true
        SUPPORTED_PIDS[i+0x01] = (support_mask & (1 << i)) >> i;
    }

    return ESP_OK;
}

// Uses PID 0x20 to poll the vehicle for supported PIDs in the range [0x21, 0x40] inclusive
// poll_supported_00() should be executed first, as the support for PID 0x20 itself is determined there
esp_err_t poll_supported_20() {

    // Check if 0x20 is supported
    if(!SUPPORTED_PIDS[0x20]) {
        Serial.printf("[OBD] PID 0x20 is not supported by this vehicle\n");
        return ESP_FAIL;
    }

    // data[0] = length of valid data
    // data[1] = obd2 mode (01 for current data)
    // data[2] = PID
    static const uint8_t REQUEST_PAYLOAD[] = {0x02, 0x01, 0x20, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC};

    // Build a response buffer, oversized just in case. Should only need to be 6 bytes
    uint32_t RESPONSE_LEN = 16;
    uint8_t RESPONSE_DATA[RESPONSE_LEN] = {0};

    // Create the request message
    twai_message_t request = {0};
    request.data_length_code = 8;
    request.identifier = OBD_BROADCAST_ID;
    memcpy(request.data, REQUEST_PAYLOAD, 8);

    // TX the request
    esp_err_t result = twai_transmit(&request, pdMS_TO_TICKS(500));
    if(result != ESP_OK) {
        Serial.printf("[OBD] Failed to transmit request, 0x%04X %s\n", result, esp_err_to_name(result));
        return result;
    }

    // Assuming TX success, immediately start RX
    result = tp_rx(RESPONSE_DATA, &RESPONSE_LEN);
    if(result != ESP_OK) {
        Serial.printf("[OBD] Failed to receive request response, 0x%04X %s\n", result, esp_err_to_name(result));
        return result;
    }

    // Check for a PRSP indicator
    if(!obd_errchk(RESPONSE_DATA)) {
        Serial.printf("[OBD] Response error check failed\n");
        return ESP_FAIL;
    }

    // If the response is positive, there are two protocol bytes at the beginning which can
    // be discarded/disregarded
    remove_front(RESPONSE_DATA, &RESPONSE_LEN);
    remove_front(RESPONSE_DATA, &RESPONSE_LEN);

    // There should only be 4 bytes left at this point, check that
    if(RESPONSE_LEN != 4) {
        Serial.printf("[OBD] Not enough bytes, expected 4 but  got %d\n", RESPONSE_LEN);
        return ESP_FAIL;
    }

    // At this point, swap bit orders of the remaining bytes to make it easier
    // for me as a human to decode

    uint8_t A = reverse_byte(RESPONSE_DATA[0]);
    uint8_t B = reverse_byte(RESPONSE_DATA[1]);
    uint8_t C = reverse_byte(RESPONSE_DATA[2]);
    uint8_t D = reverse_byte(RESPONSE_DATA[3]);

    uint32_t support_mask = (D << 24) | (C << 16) | (B << 8) | (A << 0);

    // Extract all bits and update the corresponding cell in the SUPPORTED_PIDS list
    for(int i = 0x00; i < 0x20; i++) {
        // 0x21 comes from the fact that PID 0x00 is assumed to be true, and that 0x20 is the original request
        SUPPORTED_PIDS[i+0x21] = (support_mask & (1 << i)) >> i;
    }

    return ESP_OK;
}

// Uses PID 0x40 to poll the vehicle for supported PIDs in the range [0x41, 0x60] inclusive
// poll_supported_20() should be executed first, as the support for PID 0x40 itself is determined there
esp_err_t poll_supported_40() {

    // Check if 0x40 is supported
    if(!SUPPORTED_PIDS[0x40]) {
        Serial.printf("[OBD] PID 0x40 is not supported by this vehicle\n");
        return ESP_FAIL;
    }

    // data[0] = length of valid data
    // data[1] = obd2 mode (01 for current data)
    // data[2] = PID
    static const uint8_t REQUEST_PAYLOAD[] = {0x02, 0x01, 0x40, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC};

    // Build a response buffer, oversized just in case. Should only need to be 6 bytes
    uint32_t RESPONSE_LEN = 16;
    uint8_t RESPONSE_DATA[RESPONSE_LEN] = {0};

    // Create the request message
    twai_message_t request = {0};
    request.data_length_code = 8;
    request.identifier = OBD_BROADCAST_ID;
    memcpy(request.data, REQUEST_PAYLOAD, 8);

    // TX the request
    esp_err_t result = twai_transmit(&request, pdMS_TO_TICKS(500));
    if(result != ESP_OK) {
        Serial.printf("[OBD] Failed to transmit request, 0x%04X %s\n", result, esp_err_to_name(result));
        return result;
    }

    // Assuming TX success, immediately start RX
    result = tp_rx(RESPONSE_DATA, &RESPONSE_LEN);
    if(result != ESP_OK) {
        Serial.printf("[OBD] Failed to receive request response, 0x%04X %s\n", result, esp_err_to_name(result));
        return result;
    }

    // Check for a PRSP indicator
    if(!obd_errchk(RESPONSE_DATA)) {
        Serial.printf("[OBD] Response error check failed\n");
        return ESP_FAIL;
    }

    // If the response is positive, there are two protocol bytes at the beginning which can
    // be discarded/disregarded
    remove_front(RESPONSE_DATA, &RESPONSE_LEN);
    remove_front(RESPONSE_DATA, &RESPONSE_LEN);

    // There should only be 4 bytes left at this point, check that
    if(RESPONSE_LEN != 4) {
        Serial.printf("[OBD] Not enough bytes, expected 4 but  got %d\n", RESPONSE_LEN);
        return ESP_FAIL;
    }

    // At this point, swap bit orders of the remaining bytes to make it easier
    // for me as a human to decode

    uint8_t A = reverse_byte(RESPONSE_DATA[0]);
    uint8_t B = reverse_byte(RESPONSE_DATA[1]);
    uint8_t C = reverse_byte(RESPONSE_DATA[2]);
    uint8_t D = reverse_byte(RESPONSE_DATA[3]);

    uint32_t support_mask = (D << 24) | (C << 16) | (B << 8) | (A << 0);

    // Extract all bits and update the corresponding cell in the SUPPORTED_PIDS list
    for(int i = 0x00; i < 0x20; i++) {
        // 0x41 comes from the fact that PID 0x00 is assumed to be true, and that 0x40 is the original request
        SUPPORTED_PIDS[i+0x41] = (support_mask & (1 << i)) >> i;
    }

    return ESP_OK;
}

// Uses PID 0x60 to poll the vehicle for supported PIDs in the range [0x61, 0x80] inclusive
// poll_supported_40() should be executed first, as the support for PID 0x60 itself is determined there
esp_err_t poll_supported_60() {

    // Check if 0x60 is supported
    if(!SUPPORTED_PIDS[0x60]) {
        Serial.printf("[OBD] PID 0x60 is not supported by this vehicle\n");
        return ESP_FAIL;
    }

    // data[0] = length of valid data
    // data[1] = obd2 mode (01 for current data)
    // data[2] = PID
    static const uint8_t REQUEST_PAYLOAD[] = {0x02, 0x01, 0x60, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC};

    // Build a response buffer, oversized just in case. Should only need to be 6 bytes
    uint32_t RESPONSE_LEN = 16;
    uint8_t RESPONSE_DATA[RESPONSE_LEN] = {0};

    // Create the request message
    twai_message_t request = {0};
    request.data_length_code = 8;
    request.identifier = OBD_BROADCAST_ID;
    memcpy(request.data, REQUEST_PAYLOAD, 8);

    // TX the request
    esp_err_t result = twai_transmit(&request, pdMS_TO_TICKS(500));
    if(result != ESP_OK) {
        Serial.printf("[OBD] Failed to transmit request, 0x%04X %s\n", result, esp_err_to_name(result));
        return result;
    }

    // Assuming TX success, immediately start RX
    result = tp_rx(RESPONSE_DATA, &RESPONSE_LEN);
    if(result != ESP_OK) {
        Serial.printf("[OBD] Failed to receive request response, 0x%04X %s\n", result, esp_err_to_name(result));
        return result;
    }

    // Check for a PRSP indicator
    if(!obd_errchk(RESPONSE_DATA)) {
        Serial.printf("[OBD] Response error check failed\n");
        return ESP_FAIL;
    }

    // If the response is positive, there are two protocol bytes at the beginning which can
    // be discarded/disregarded
    remove_front(RESPONSE_DATA, &RESPONSE_LEN);
    remove_front(RESPONSE_DATA, &RESPONSE_LEN);

    // There should only be 4 bytes left at this point, check that
    if(RESPONSE_LEN != 4) {
        Serial.printf("[OBD] Not enough bytes, expected 4 but  got %d\n", RESPONSE_LEN);
        return ESP_FAIL;
    }

    // At this point, swap bit orders of the remaining bytes to make it easier
    // for me as a human to decode

    uint8_t A = reverse_byte(RESPONSE_DATA[0]);
    uint8_t B = reverse_byte(RESPONSE_DATA[1]);
    uint8_t C = reverse_byte(RESPONSE_DATA[2]);
    uint8_t D = reverse_byte(RESPONSE_DATA[3]);

    uint32_t support_mask = (D << 24) | (C << 16) | (B << 8) | (A << 0);

    // Extract all bits and update the corresponding cell in the SUPPORTED_PIDS list
    for(int i = 0x00; i < 0x20; i++) {
        // 0x61 comes from the fact that PID 0x00 is assumed to be true, and that 0x60 is the original request
        SUPPORTED_PIDS[i+0x61] = (support_mask & (1 << i)) >> i;
    }

    return ESP_OK;
}

// Uses helper functions to poll the connected vehicle for what PIDs are supported. I only really care about
// 0x00 thru 0x7F as above is the newer WWH-OBD standard
// Also prints the availability of PIDs to the serial port initialized in main
esp_err_t poll_all_pids() {

    // Assume PID 0x00 is always available, this will update
    // availability of 0x01 - 0x20 where 0x20 is used to poll
    // the next bunch of PIDs
    esp_err_t result = poll_supported_00();
    if(result != ESP_OK) {
        Serial.printf("[OBD] Failed to update availability of PIDs 0x01 - 0x20\n");
        
        for(int i = 0; i < 0x21; i++) {
            Serial.printf("\tPID 0x%02X: ", i);
            if(SUPPORTED_PIDS[i]) {
                Serial.printf("True\n");
            } else {
                Serial.printf("False\n");
            }
        }

        return result;
    }

    // Poll availability of PIDs 0x21 - 0x40
    result = poll_supported_20();
    if(result != ESP_OK) {
        Serial.printf("[OBD] Failed to update availability of PIDs 0x21 - 0x40\n");

        for(int i = 0; i < 0x41; i++) {
            Serial.printf("\tPID 0x%02X: ", i);
            if(SUPPORTED_PIDS[i]) {
                Serial.printf("True\n");
            } else {
                Serial.printf("False\n");
            }
        }

        return result;
    }

    // Poll availability of PIDs 0x41 - 0x60
    result = poll_supported_40();
    if(result != ESP_OK) {
        Serial.printf("[OBD] Failed to update availability of PIDs 0x41 - 0x60\n");

        for(int i = 0; i < 0x61; i++) {
            Serial.printf("\tPID 0x%02X: ", i);
            if(SUPPORTED_PIDS[i]) {
                Serial.printf("True\n");
            } else {
                Serial.printf("False\n");
            }
        }

        return result;
    }

    // Poll availability of PIDs 0x61 - 0x80
    result = poll_supported_60();
    if(result != ESP_OK) {
        Serial.printf("[OBD] Failed to update availability of PIDs 0x61 - 0x80\n");

        for(int i = 0; i < 0x81; i++) {
            Serial.printf("\tPID 0x%02X: ", i);
            if(SUPPORTED_PIDS[i]) {
                Serial.printf("True\n");
            } else {
                Serial.printf("False\n");
            }
        }

        return result;
    }

    // If we make it this far, print all supported PIDs
    Serial.printf("[OBD] Supported PIDs:\n");
    for(int i = 0; i < 128; i++) {
        Serial.printf("\tPID 0x%02X: ", i);
        if(SUPPORTED_PIDS[i]) {
            Serial.printf("True\n");
        } else {
            Serial.printf("False\n");
        }
    }

    return ESP_OK;

}

// Request a PID value
// Dumps raw OBD/ISO-TP data into the provided output buffer and delegates decoding said data to who/whatever calls this function.
// Also assumes that no data needs to be sent other than the PID itself (and the mode, which should always be 0x01 for reading PIDs)
esp_err_t obd_get_pid(uint8_t PID, uint8_t* rx_data, uint32_t* rx_len) {
    return ESP_OK;
}

