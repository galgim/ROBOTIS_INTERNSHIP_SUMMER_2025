#include <esp_now.h>
#include <WiFi.h>
// This esp is 3C:8A:1F:A1:61:B8
uint8_t CarMacAddress[] = {0x3C, 0x8A, 0x1F, 0xA0, 0xDF, 0xC0};
uint8_t data[32];
#define Y_Pin 32
#define X_Pin 34

// Rotary encoder struct
struct RotaryEncoderData {
    const uint8_t clkPin;
    const uint8_t dtPin;
    const long minValue;
    const long maxValue;
    const bool reverse;
    const bool pullup;
    long currentValue;
    int lastClkState;

    RotaryEncoderData(uint8_t clk, uint8_t dt, int minV, int maxV, bool rev, bool pull, int initVal)
        : clkPin(clk), dtPin(dt), minValue(minV), maxValue(maxV),
          reverse(rev), pullup(pull), currentValue(initVal) {}

    void begin() {
        pinMode(clkPin, pullup ? INPUT_PULLUP : INPUT);
        pinMode(dtPin, pullup ? INPUT_PULLUP : INPUT);
        lastClkState = digitalRead(clkPin);
    }

    void update() {
        int clkState = digitalRead(clkPin);
        if (clkState != lastClkState) {
            if (digitalRead(dtPin) != clkState) {
                currentValue += reverse ? -1 : 1;
            } else {
                currentValue += reverse ? 1 : -1;
            }
            if (currentValue < minValue) currentValue = minValue;
            if (currentValue > maxValue) currentValue = maxValue;
        }
        lastClkState = clkState;
    }
};

// 5 encoders: (pins are CLK, DT)
RotaryEncoderData encoders[5] = {
    {4, 17, 0, 360, false, false, 180},
    {15, 22, 0, 180, false, false, 180},
    {0, 36, 0, 180, false, false, 0},
    {26, 14, 0, 180, false, false, 90},
    {5, 13, 0, 360, false, false, 180}
};

// Called every time ESP sends data via ESP-NOW
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Last Packet Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Called when data is received
void onDataReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
    Serial.println("Data received");
    if (len != 2) {
        Serial.println("Invalid data length");
        return;
    }
    int command;
    memcpy(&command, incomingData, sizeof(command));
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
    memcpy(peerInfo.peer_addr, CarMacAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add master peer");
    }

    // Initialize encoders
    for (int i = 0; i < 5; i++) {
        encoders[i].begin();
    }
}

unsigned long lastSend = 0;
void loop() {
    // Update encoders every loop
    for (int i = 0; i < 5; i++) {
        encoders[i].update();
    }

    // Only send data every 50 ms
    if (millis() - lastSend > 50) {
        int32_t movementArray[8] = {0};

        for (int i = 0; i < 5; i++) {
            movementArray[2 + i] = encoders[i].currentValue;
        }

        movementArray[0] = analogRead(X_Pin);
        movementArray[1] = analogRead(Y_Pin);
        movementArray[7] = 1;

        memcpy(data, movementArray, sizeof(movementArray));

        esp_err_t result = esp_now_send(CarMacAddress, data, sizeof(movementArray));
        if (result != ESP_OK) {
            Serial.println("Error sending the data");
        }

        lastSend = millis();
    }
}
