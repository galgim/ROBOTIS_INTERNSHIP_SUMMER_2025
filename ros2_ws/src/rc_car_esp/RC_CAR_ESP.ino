#include <esp_now.h>
#include <WiFi.h>
// This esp is 3C:8A:1F:A0:DF:C0
uint8_t ControllerMacAddress[] = {0x3C, 0x8A, 0x1F, 0xA1, 0x61, 0xB8};
uint8_t data[32];

void sendDataJetson(String tag, const void* data, size_t size) {
    Serial.println(tag);
    Serial.write((const uint8_t*)data, size);
    Serial.println();
  }

// Called every time ESP sends data via ESP-NOW
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Last Packet Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Called when data is received
void onDataReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
    // Serial.println("Data received");
    if (len != 32) {
        Serial.println("Invalid data length");
        return;
    }
    int32_t movementArray[8];
    memcpy(movementArray, incomingData, sizeof(movementArray));

//    Serial.print("From MAC: ");
//    for (int i = 0; i < 6; i++) {
//        Serial.printf("%02X", info->src_addr[i]);
//        if (i < 5) Serial.print(":");
//    }
//    Serial.println();

//    Serial.print("[");
//    for (int i = 0; i < 8; i++) {
//        Serial.print(movementArray[i]);
//        if (i < 7) Serial.print(", ");
//    }
//    Serial.println("]");

    sendDataJetson("movementArray", movementArray, sizeof(movementArray));
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
    } else {
        Serial.println("ESP-NOW initialized successfully");
    }

    esp_now_register_recv_cb(onDataReceive);
    esp_now_register_send_cb(onDataSent);

    // Add peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, ControllerMacAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add master peer");
    }
}

void loop() {
//    uint8_t mac[6];
//    WiFi.macAddress(mac);
//    Serial.print("This ESP32 MAC Address: ");
//    for (int i = 0; i < 6; i++) {
//        Serial.printf("%02X", mac[i]);
//        if (i < 5) Serial.print(":");
//    }
//    Serial.println();



    delay(1000);
}
