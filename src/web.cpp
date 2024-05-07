#include <Arduino.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "driver/twai.h"
#include "ESPTrueRandom.h"

#include "iso_tp.hpp"
#include "obd.hpp"
#include "mil_lib.hpp"

#define MAX_CLIENTS 1
#define WIFI_CHANNEL 9
#define DNS_INTERVAL_MS 30

// SSID and Password
char ssid[32];
const char* password = NULL;
char redirect_addr[32];

DNSServer dns_server;
AsyncWebServer web_server(80);
AsyncWebSocket dtc_ws("/dtc-ws");
AsyncWebSocket obd_ws("/obd-ws");

JsonDocument json_document;

#define JSON_STR_LEN 1024
char json_string[JSON_STR_LEN] = {0};

// Initializes DNS server. Unsure if this is needed for the captive portal, HTTP server, or both.
void init_dns() {
    dns_server.setErrorReplyCode(DNSReplyCode::NoError);
	dns_server.setTTL(300);
    dns_server.start(53, "*", WiFi.softAPIP());
}

// Initializes the actual wireless interface
void init_softAP() {

	// Generate SSID from the chip's MAC address
	uint8_t mac_addr[6] = {0};
	esp_read_mac(mac_addr, ESP_MAC_WIFI_SOFTAP);
	sprintf(ssid, "APM_%02X%02X%02X", mac_addr[3], mac_addr[4], mac_addr[5]);

	// Create the soft AP
	if(!WiFi.softAP(ssid, password)) {
		Serial.printf("[WEB] Failed to initialize AP\n");
		while(1){vTaskDelay(pdMS_TO_TICKS(1000));}
	}
	vTaskDelay(pdMS_TO_TICKS(100));

	// Generate the redirect URL
	sprintf(redirect_addr, "http://%s", WiFi.softAPIP().toString());
	Serial.printf("[WEB] Generated redirect URL %s\n", redirect_addr);

	// For debugging purposes, print all the important info
	Serial.printf("[WEB] AP SSID = %s\n", WiFi.softAPSSID().c_str());
    Serial.printf("[WEB] AP IP Address = %s\n", WiFi.softAPIP().toString());
    Serial.printf("[WEB] AP MAC Address = %s\n", WiFi.softAPmacAddress().c_str());

    vTaskDelay(pdMS_TO_TICKS(100));

}

// Builds a response to a "dtc_list" command from the client and sends it over the websocket
// Uses the OBD-II library and ISO-TP library to get a list of DTCs and encapsulate it in the expected JSON format
void ws_send_dtcs() {

    json_document.clear();
    memset(json_string, 0, JSON_STR_LEN);
    json_document["rsp"] = "dtc_list";

    // Create and add an element to the JSON document which is an array
    JsonArray dtc_list = json_document["dtc"].to<JsonArray>();

    // Raw OBD-II Data goes here
    uint32_t raw_len = 256;
    uint8_t raw_data[raw_len] = {0};

    // DTCs derrived from raw OBD-II data go here
    uint32_t dtc_count = 32;
    uint16_t raw_dtcs[dtc_count] = {0};

    // Reused string for getting DTC description
    uint32_t dtc_str_len = 256;
    char dtc_str[dtc_str_len] = {0};

    // Build OBD-II DTC Request
    static const uint8_t REQUEST_PAYLOAD[] = {0x01, 0x03, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC};
    twai_message_t request = {0};
    request.data_length_code = 8;
    request.identifier = 0x7DF;
    memcpy(request.data, REQUEST_PAYLOAD, 8);

    // Transmit OBD-II DTC Request
    esp_err_t result = twai_transmit(&request, pdMS_TO_TICKS(250));
    if(result != ESP_OK) {
        // Add an error code and skip the rest of this function
        dtc_list.add("Failed to read codes");
        dtc_list.add("Error 0x01: Failed to transmit request");
        goto serialize;
    }

    // Receive the raw OBD-II data using the ISO-TP protocol
    result = tp_rx(raw_data, &raw_len);
    if(result != ESP_OK) {
        // Add an error code and skip the rest of this function
        dtc_list.add("Failed to read codes");
        dtc_list.add("Error 0x02: Failed to receive codes");
        goto serialize;
    }

    // If we received any DTCs, length should be greater than two
    if(raw_len >= 2) {
        // First two bytes of raw data are protocol info which should be checked
        // but discard for now
        remove_front(raw_data, &raw_len);
        remove_front(raw_data, &raw_len);
    }

    // Attempt to decode raw OBD-II data as DTCs
    result = bytes_to_dtcs(raw_data, raw_dtcs, raw_len, &dtc_count);
    if(result != ESP_OK) {
        // Add an error code and skip the rest of this function
        dtc_list.add("Failed to convert raw bytes to DTCs");
        dtc_list.add("Error 0x03: Failed to decode data into DTCs");
        goto serialize;
    }

    // We received no codes?
    if( dtc_count == 0 ) {
        // P0000 = No DTC
        mil_description(0x0000, dtc_str);
        dtc_list.add(dtc_str);
        goto serialize;
    }

    // For each DTC we got, get its description and append it to the WS message
    for(int i = 0; i < dtc_count; i++) {
        memset(dtc_str, 0, dtc_str_len);
        mil_description(raw_dtcs[i], dtc_str);
        dtc_list.add(dtc_str);
    }
    
    // Serialize WS message and send
    serialize:
    if( serializeJson(json_document, json_string, JSON_STR_LEN) > 0 ) {
        dtc_ws.textAll(json_string);
    }

}

// Builds a response to a "dtc_clear" command from the client and sends it over the websocket
// Uses the OBD-II library to request the clearing of DTCs and encapsulates a response in the expected JSON format
void ws_clear_dtcs() {
    
    // Clear and prepare json
    json_document.clear();
    memset(json_string, 0, JSON_STR_LEN);
    json_document["rsp"] = "dtc_clear";

    // Not implemented yet
    json_document["content"] = "For now, this function isn't implemented";

    // Send response
    if( serializeJson(json_document, json_string, JSON_STR_LEN) > 0 ) {
        dtc_ws.textAll(json_string);
    }

}

// Builds a response to a "list_pids" command from the client and sends it over the websocket
// Uses the OBD-II library to get the list of PIDs supported by the attached vehicle, and sends a list in the expected JSON format
void ws_send_pids() {
    /* 
    -> Expected JSON format:
        {
            "rsp": "list_pids",
            "pids": [string list]
        }
    */
}

// Builds a response to a "get_vals" command from the client and sends it over the websocket
// Uses the OBD-II library to get the PID values being currently monitored and sends a response in the expected JSON format
void ws_send_vals() {

    // Clear document and add response
    json_document.clear();
    memset(json_string, 0, JSON_STR_LEN);
    json_document["rsp"] = "pid_vals";

    // Get the pid vals eventually, for now send random vals
    json_document["pid_a"] = (uint8_t)(ESPTrueRandom.randomByte());
    json_document["pid_b"] = (uint8_t)(ESPTrueRandom.randomByte());
    json_document["pid_c"] = (uint8_t)(ESPTrueRandom.randomByte());

    // Send response
    if( serializeJson(json_document, json_string, JSON_STR_LEN) > 0 ) {
        obd_ws.textAll(json_string);
    }

}

// Changes the locally held variables determining what PID values to monitor
void ws_change_mon() {

    // Extract char arrays
    const char* temp_a = json_document["pid_a"];
    const char* temp_b = json_document["pid_b"];
    const char* temp_c = json_document["pid_c"];

    // Make string objects so we can parse
    String STR_A(temp_a, strlen(temp_a));
    String STR_B(temp_a, strlen(temp_b));
    String STR_C(temp_a, strlen(temp_c));

    // Print parsed values
    Serial.printf("[WEB] Received PIDs %02X %02X %02X\n", STR_A.toInt(), STR_B.toInt(), STR_C.toInt());

}

// Handles parsing inbound WebSocket data
void websocket_funct(void *arg, uint8_t *data, size_t len) {
	
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
  	
    if(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    	
        // Insert a null terminator to ensure interop with string libraries
        data[len] = 0;
    	char* message = (char*)data;

        // Clear any residual data
        json_document.clear();

        // Attempt to parse message as JSON
        DeserializationError parse_error = deserializeJson(json_document, message);

        // If there was a parsing error, print it and return
        if(parse_error) {
            Serial.printf("[WEB] Failed to parse json message %s\n", message);
            Serial.printf("[WEB] Parse error %s", parse_error.c_str());
            return;
        }

        // Look for the key "cmd" which is required for communication from client to server
        if(!json_document.containsKey("cmd")) {
            Serial.printf("[WEB] JSON Document does not contain a command\n");
            Serial.printf("[WEB] Provided JSON: %s\n", message);
            return;
        }

        // If there is a "cmd" key, it should be a string so extract it
        const char* command = json_document["cmd"];

        /* Below: Do stuff according to extracted command */

        if( strcmp(command, "dtc_list") == 0 ) {
            Serial.printf("[WEB] Received request for a list of DTCs\n");
            ws_send_dtcs();
            return;
        }

        if( strcmp(command, "dtc_clear") == 0 ) {
            Serial.printf("[WEB] Received request to clear DTCs\n");
            ws_clear_dtcs();
            return;
        }

        if( strcmp(command, "list_pids") == 0 ) {
            Serial.printf("[WEB] Received request to list supported PIDs\n");
            ws_send_pids();
            return;
        }

        if( strcmp(command, "change_mon") == 0 ) {
            Serial.printf("[WEB] Received request to change the monitored PIDs\n");
            ws_change_mon();
            return;
        }

        /* At this point, no matching command was found */

        Serial.printf("[WEB] Unknown command \"%s\" received\n", command);

  	}
}

// Event callback for WebSockets
void on_ws_event(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
	switch (type) {
    	
        // Client connected
        case WS_EVT_CONNECT:
      		Serial.printf("[WEB] WebSocket client connected\n");
      		break;
        
        // Client disconnected
    	case WS_EVT_DISCONNECT:
      		Serial.printf("[WEB] WebSocket client disconnected\n");
      		break;

        // Client sent data -> Parse the sent data
    	case WS_EVT_DATA:
      		websocket_funct(arg, data, len);
      		break;

        // Ignore these two?
    	case WS_EVT_PONG:
    	case WS_EVT_ERROR:
      		break;
  	}
}

// Initializes the HTTP webserver with a bunch of predefined responses
// Also initializes the websocket objects
void init_webserver() {

    // Register the same on-event function
    // We only care about the request data for this part
    dtc_ws.onEvent(on_ws_event);
    obd_ws.onEvent(on_ws_event);

    // Windows 11 captive portal workaround
    web_server.on("/connecttest.txt", [](AsyncWebServerRequest *request) { request->redirect("http://logout.net"); });
    web_server.on("/wpad.dat", [](AsyncWebServerRequest *request) { request->send(404); });	

    // Background responses: Probably not all are Required, but some are. Others might speed things up?
	// A Tier (commonly used by modern systems)
    // This was from the example code provided by the captive portal stuff. Looks like it just sends redirects
    // to the UI IP Address on specific requests
	web_server.on("/generate_204", [](AsyncWebServerRequest *request) { request->redirect(redirect_addr); });		    // android captive portal redirect
	web_server.on("/redirect", [](AsyncWebServerRequest *request) { request->redirect(redirect_addr); });			    // microsoft redirect
	web_server.on("/hotspot-detect.html", [](AsyncWebServerRequest *request) { request->redirect(redirect_addr); });    // apple call home
	web_server.on("/canonical.html", [](AsyncWebServerRequest *request) { request->redirect(redirect_addr); });	        // firefox captive portal call home
	web_server.on("/success.txt", [](AsyncWebServerRequest *request) { request->send(200); });					        // firefox captive portal call home
	web_server.on("/ncsi.txt", [](AsyncWebServerRequest *request) { request->redirect(redirect_addr); });			    // windows call home

    // Don't return an icon
    web_server.on("/favicon.ico", [](AsyncWebServerRequest *request) { request->send(404); });

	// Enable the DTC Websocket in the server
	web_server.addHandler(&dtc_ws);

    // Enable the OBD data websocket in the server
    web_server.addHandler(&obd_ws);

    // Initial connection, serve index.html
	web_server.on("/", HTTP_ANY, [](AsyncWebServerRequest *request) {
		request->send(SPIFFS, "/ui/index.html", "text/html");
		Serial.println("[WEB] Served Index");
	});

    // Explicit request for the index page
    web_server.on("/index.html", HTTP_ANY, [](AsyncWebServerRequest *request) {
		request->send(SPIFFS, "/ui/index.html", "text/html");
		Serial.println("[WEB] Served Index");
	});

    // Explicit request for the DTC page
    web_server.on("/dtc.html", HTTP_ANY, [](AsyncWebServerRequest *request) {
		request->send(SPIFFS, "/ui/dtc.html", "text/html");
		Serial.println("[WEB] Served DTC page");
	});

    // 404
	web_server.onNotFound([](AsyncWebServerRequest *request) {
		request->send(404);
        Serial.printf( "[WEB] Served 404: %s\n", request->url().c_str() );
	});

}

void ui_setup() {

    // CAN bus related Libs should be initialized in main before this task is created

    init_softAP();
    vTaskDelay(pdMS_TO_TICKS(100));

    init_dns();
    vTaskDelay(pdMS_TO_TICKS(100));

    init_webserver();
    vTaskDelay(pdMS_TO_TICKS(100));

    web_server.begin();

}

void web_main() {
    
    ui_setup();

    // Counts how many times the main loop has run
    uint8_t task_ticks = 0;

    while(1) {

        // 9 ticks * 30ms a tick = every 270ms (~4Hz)
        // Don't forget to count the 0th tick!
        if(task_ticks >= 8) {

            // Send OBD2 live data, give the client 10ms to read it
            ws_send_vals();
            vTaskDelay(pdMS_TO_TICKS(10));
            obd_ws._cleanBuffers();

            // Reset count
            task_ticks = 0;
        }

        // Always process DNS requests
        dns_server.processNextRequest();

        // Delay for a bit and increment ticks
        vTaskDelay(pdMS_TO_TICKS(DNS_INTERVAL_MS));
        task_ticks++;
    }
    
}