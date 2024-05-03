#include <Arduino.h>
#include <cppQueue.h>
#include <driver/twai.h>

// #define TP_TEST

#define TP_MAX_LEN  4095
#define BYTES_QUEUE    1

#define TP_SINGLE 0x00
#define TP_FIRST  0x10
#define TP_CONSEC 0x20
#define TP_CTRL   0x30

uint32_t rx_msg_len;
uint8_t last_bsc;
cppQueue rx_queue(BYTES_QUEUE, TP_MAX_LEN);

cppQueue tx_queue(BYTES_QUEUE, TP_MAX_LEN);

void tp_lib_init() {
    
    // Clear the TX/RX queues
    rx_queue.clean();
    tx_queue.clean();

}

void tp_return_vals(uint8_t* data, uint32_t* len) {
    
    // Update data length
    *len = rx_msg_len;

    // Copy data
    for(int i = 0; i < rx_msg_len; i++) {
        rx_queue.pop( &(data[i]) );
    }

}

esp_err_t tp_process_sf(twai_message_t* msg) {
    
    // Empty any old data
    rx_queue.clean();
    rx_msg_len = 0;

    rx_msg_len = msg->data[0] & 0x0F;

    for(int i = 0; i < rx_msg_len; i++) {
        
        if( !rx_queue.push(&(msg->data[i])) ) {
            Serial.printf("[ISO] Failed to push data into RX queue\n");
            return ESP_FAIL;
        }

    }

    return ESP_OK;
}

esp_err_t tp_process_ff(twai_message_t* msg) { 

    static const uint8_t fc_data[] = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    // Transmit a flow control frame immediately
    twai_message_t fc_frame = {0};
    memcpy(fc_frame.data, fc_data, 8);
    fc_frame.data_length_code = 8;
    fc_frame.identifier = 0x7E0;

    esp_err_t result = twai_transmit(&fc_frame, pdMS_TO_TICKS(250));
    if(result != ESP_OK) {
        Serial.printf("[ISO] Failed to transmit flow control frame, error 0x%04X %s\n", result, esp_err_to_name(result));
        return result;
    }

    // Update RX'd message length
    rx_msg_len = ((msg->data[0] & 0x0F) << 8) | msg->data[1];

    // Prepare last BSC value, a first frame implies consecutive frames with sequence counters will follow
    last_bsc = 0;

    // Ignore the first two bytes and push the rest of the message
    for(int i = 2; i < 8; i++) {
        if( !rx_queue.push(&(msg->data[i])) ) {
            Serial.printf("[ISO] Failed to push data into RX queue\n");
            return ESP_FAIL;
        }
    }

    return ESP_OK;

}

esp_err_t tp_process_cf(twai_message_t* msg) {

    // Get the sequence count of this message
    uint8_t sequence_count = msg->data[0] & 0x0F;

    // Compare the message sequence to the expected sequence
    if( sequence_count != (++last_bsc & 0x0F) ) {
        Serial.printf("[ISO] Sequence violation, expected sequence %d but got %d\n", last_bsc, sequence_count);
        return ESP_FAIL;
    }
    
    // Determine how many bytes are left in the entire message
    uint32_t remaining_length = rx_msg_len - rx_queue.getCount();

    // Push 7 bytes if there are 7 or more bytes left
    if( remaining_length >= 7 ) {

        // Discard first byte as that's protocol stuff
        for(int i = 1; i < 8; i++) {
            if( !rx_queue.push( &(msg->data[i])) ) {
                Serial.printf("[ISO] Failed to push data into RX queue\n");
                return ESP_FAIL;
            }
        }

    // Just push what's left
    } else {

        // Discard first byte as that's protocol stuff
        for(int i = 1; i < remaining_length+1; i++) {
            if( !rx_queue.push( &(msg->data[i])) ) {
                Serial.printf("[ISO] Failed to push data into RX queue\n");
                return ESP_FAIL;
            }
        }
        
    }

    return ESP_OK;

}

esp_err_t tp_process_frame(twai_message_t* msg) {

    // Extract frame type
    uint8_t msg_type = msg->data[0] & 0xF0;

    // Process message according to frame type
    switch(msg_type) {

        case TP_SINGLE:
            return tp_process_sf(msg);
            break;
        
        case TP_FIRST:
            return tp_process_ff(msg);
            break;

        case TP_CONSEC:
            return tp_process_cf(msg);
            break;

        case TP_CTRL:
            Serial.printf("[ISO] Received an unexpected control frame while in RX loop\n");
            return ESP_FAIL;
            break;

        default:
            Serial.printf("[ISO] Invalid TP frame type 0x%02X received\n", msg_type);
            return ESP_FAIL;
    }

}

esp_err_t tp_rx(uint8_t* data_out, uint32_t* data_len) {

    static const uint8_t msg_0_data[] = {0x10, 0x0C, 0x43, 0x05, 0x04, 0x20, 0x01, 0x02};
    static const uint8_t msg_1_data[] = {0x21, 0x01, 0x13, 0x03, 0x00, 0x10, 0x00, 0xCC};

    // Null output buffer
    if(data_out == NULL) {
        Serial.printf("[ISO] Provided input data is null\n");
        return ESP_ERR_INVALID_ARG;
    }

    // Zero output length
    if( *data_len < 1 ) {
        Serial.printf("[ISO] Provided message size is zero\n");
        return ESP_ERR_INVALID_SIZE;
    }

    // Clear any data from old/failed messages
    rx_queue.flush();
    rx_msg_len = 0;

    // Create an empty message
    twai_message_t rx_msg;
    memset(&rx_msg, 0, sizeof(twai_message_t));

    // Number of loops
    int ticks = 0;

    // Initial result
    esp_err_t result = ESP_FAIL;

    #ifdef TP_TEST
    // Test messages
    twai_message_t msg_0 = {0};
    twai_message_t msg_1 = {0};

    // Populate first test message
    memcpy(msg_0.data, msg_0_data, 8);
    msg_0.data_length_code = 8;
    msg_0.identifier = 0x7E8;

    // Populate second test message
    memcpy(msg_1.data, msg_1_data, 8);
    msg_1.data_length_code = 8;
    msg_1.identifier = 0x7E8;

    // Process first test message
    result = tp_process_frame(&msg_0);
    if( result != ESP_OK) {
        return result;
    }

    // Process second test message
    result = tp_process_frame(&msg_1);
    if( result != ESP_OK) {
        return result;
    }

    Serial.printf("[ISO] Messages processed! Decoded %d bytes while expecting %d\n", rx_queue.getCount(), rx_msg_len);

    // Were all bytes processed?
    if( rx_queue.getCount() == rx_msg_len ) {
        tp_return_vals(data_out, data_len);
        return ESP_OK;
    }
    #endif

    #ifndef TP_TEST
    while(ticks < 5) {
        
        // If a message was RX'd
        if(result == ESP_OK) {

            // Reset timeout
            ticks = 0;

            // Attempt to process the new message
            result = tp_process_frame(&rx_msg);

            // If it was processed successfully
            if(result == ESP_OK) {
                
                // Will the message not fit into the output buffer?
                if(rx_msg_len > *data_len) {
                    Serial.printf("[ISO] Response will be too large for the provided output buffer\n");
                    while(twai_receive(&rx_msg, pdMS_TO_TICKS(500)) == ESP_OK) {}   // Wait until all messages are sent
                    return ESP_ERR_INVALID_SIZE;    // Return an error code
                }

                // Is the message complete?
                if(rx_msg_len == rx_queue.getCount()) {
                    tp_return_vals(data_out, data_len);
                    return ESP_OK;
                }

            // Message didn't decode properly
            } else {
                Serial.printf("[ISO] Encountered an error processing a RX'd frame\n");
                return result;
            }

        } 

        // Update message and receive status
        vTaskDelay(pdMS_TO_TICKS(250));
        result = twai_receive(&rx_msg, pdMS_TO_TICKS(500));

        // Increment loop count
        ticks++;
        Serial.printf("[ISO] Loop %d\n", ticks);
    }
    #endif

    Serial.printf("[ISO] Timed out while listening for messages\n");
    return ESP_FAIL;

}