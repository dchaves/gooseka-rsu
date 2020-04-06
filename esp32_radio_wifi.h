#ifndef ESP32_RADIO_WIFI_H
#define ESP32_RADIO_WIFI_H

#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiAP.h>

#include "esp32_controller_defs.h"
#include "gooseka_structs.h"
#include "esp32_controller_helpers.h"

static const IPAddress LOCAL_IP(WIFI_GATEWAY_IP);
static const IPAddress NETMASK(WIFI_NETMASK);
static const IPAddress MULTICAST_IP(WIFI_MULTICAST_IP);
WiFiUDP udp;

void WiFiEvent(WiFiEvent_t event){
    switch(event) {
        case SYSTEM_EVENT_AP_START:
            break;
        case SYSTEM_EVENT_AP_STACONNECTED:
            DEBUG_PRINTLN("NEW CLIENT CONNECTED");
            break;
        case SYSTEM_EVENT_AP_STADISCONNECTED:
            DEBUG_PRINTLN("CLIENT DISCONNECTED");
            break;
        case SYSTEM_EVENT_STA_CONNECTED:
            DEBUG_PRINTLN("CONNECTED TO AP");
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            DEBUG_PRINTLN("DISCONNECTED FROM AP");
            break;
    }
}

void init_radio() {
    WiFi.disconnect(true);
    
    // Set event handler
    WiFi.onEvent(WiFiEvent);

    // Set WiFi AP
    WiFi.softAPConfig(LOCAL_IP, LOCAL_IP, NETMASK);
    WiFi.softAP(WIFI_BSSID, WIFI_PASSWORD);

    // Disable power saving
    WiFi.setSleep(false); 

    // Start multicast UDP interface
    udp.beginMulticast(MULTICAST_IP, WIFI_MULTICAST_PORT);
}

void send_via_radio(uint8_t* payload, size_t size) {
    udp.beginMulticastPacket();
    udp.write(payload, size);
    if(udp.endPacket() != 1) {
        DEBUG_PRINTLN("Error sending packet");
    } else {
        DEBUG_PRINT("Sent ");
        DEBUG_PRINT(size);
        DEBUG_PRINT(" bytes to ");
        DEBUG_PRINTLN(udp.remoteIP());
    }
}

int receive_radio_packet(uint8_t* buffer, int size) {
    int packetSize = 0;
    packetSize = udp.parsePacket();
    if(packetSize == size) {
        udp.read(buffer, size);
        DEBUG_PRINTLN("INCOMING WIFI");
    }
    return packetSize;
}

int16_t radio_rssi() {
    return WiFi.RSSI();
}

#endif //ESP32_RADIO_WIFI_H