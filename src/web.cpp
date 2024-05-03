#include <Arduino.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "driver/twai.h"

#include "iso_tp.hpp"
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

void remove_front(uint8_t* data, uint32_t* len) {

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

void init_dns() {
    dns_server.setErrorReplyCode(DNSReplyCode::NoError);
	dns_server.setTTL(300);
    dns_server.start(53, "*", WiFi.softAPIP());
}

void init_softAP() {

	// Generate SSID from the chip's MAC address
	uint8_t mac_addr[6] = {0};
	esp_read_mac(mac_addr, ESP_MAC_WIFI_SOFTAP);
	sprintf(ssid, "APM_%02X%02X%02X", mac_addr[3], mac_addr[4], mac_addr[5]);
	Serial.printf("[WEB] Generated SSID %s\n", ssid);

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

void ws_send_dtcs() {

    json_document.clear();
    memset(json_string, 0, JSON_STR_LEN);

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

void websocket_funct(void *arg, uint8_t *data, size_t len) {
	
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
  	
    if(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    	data[len] = 0;
    	char* message = (char*)data;

        if(strcmp(message, "getDTC") == 0) {
            Serial.printf("[WEB] Got a websocket request for DTCs\n");
            ws_send_dtcs();
        } else {
            Serial.printf("[WEB] Unknown websocket request %s\n", message);
        }

  	}
}

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

        // Client sent data
    	case WS_EVT_DATA:
      		websocket_funct(arg, data, len);
      		break;

        // Ignore these two
    	case WS_EVT_PONG:
    	case WS_EVT_ERROR:
      		break;
  	}
}

void init_webserver() {

    dtc_ws.onEvent(on_ws_event);
    obd_ws.onEvent(on_ws_event);

    // Windows 11 workaround
    web_server.on("/connecttest.txt", [](AsyncWebServerRequest *request) { request->redirect("http://logout.net"); });
    web_server.on("/wpad.dat", [](AsyncWebServerRequest *request) { request->send(404); });	

    // Background responses: Probably not all are Required, but some are. Others might speed things up?
	// A Tier (commonly used by modern systems)
	web_server.on("/generate_204", [](AsyncWebServerRequest *request) { request->redirect(redirect_addr); });		    // android captive portal redirect
	web_server.on("/redirect", [](AsyncWebServerRequest *request) { request->redirect(redirect_addr); });			    // microsoft redirect
	web_server.on("/hotspot-detect.html", [](AsyncWebServerRequest *request) { request->redirect(redirect_addr); });    // apple call home
	web_server.on("/canonical.html", [](AsyncWebServerRequest *request) { request->redirect(redirect_addr); });	        // firefox captive portal call home
	web_server.on("/success.txt", [](AsyncWebServerRequest *request) { request->send(200); });					    // firefox captive portal call home
	web_server.on("/ncsi.txt", [](AsyncWebServerRequest *request) { request->redirect(redirect_addr); });			    // windows call home

    // Don't return an icon
    web_server.on("/favicon.ico", [](AsyncWebServerRequest *request) { request->send(404); });

	// Enable the DTC Websocket in the server
	web_server.addHandler(&dtc_ws);

    // Enable the OBD data websocket in the server
    web_server.addHandler(&obd_ws);

    // Eventually add an index page with three buttons:
    //      Read DTC
    //      Clear DTC
    //      Live Data
    // Reuse main APM live data page?

    // Initial connection, serve index.html
	web_server.on("/", HTTP_ANY, [](AsyncWebServerRequest *request) {
		request->send(SPIFFS, "/ui/index.html", "text/html");
		Serial.println("[WEB] Served Index");
	});

    web_server.on("/index.html", HTTP_ANY, [](AsyncWebServerRequest *request) {
		request->send(SPIFFS, "/ui/index.html", "text/html");
		Serial.println("[WEB] Served Index");
	});

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

    void tp_lib_init();
    vTaskDelay(pdMS_TO_TICKS(100));

    void mil_init();
    vTaskDelay(pdMS_TO_TICKS(100));

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

    uint8_t task_ticks = 0;

    while(1) {

        // Don't forget to count the 0th tick!
        if(task_ticks >= 239) {

            // Send OBD2 live data
            // I might try a "static" OBD-II library (not a task like main APM)

            // In the APM main version, to prevent overflowing a websocket buffer,
            // we just send a message, wait 10ms, and clean the buffers.

            task_ticks = 0;
        }

        // Always process DNS requests
        dns_server.processNextRequest();

        // Delay for a bit and increment ticks
        vTaskDelay(pdMS_TO_TICKS(DNS_INTERVAL_MS));
        task_ticks++;
    }
    
}