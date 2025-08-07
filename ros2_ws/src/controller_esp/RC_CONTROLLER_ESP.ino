#include <esp_now.h>
#include <WiFi.h>
#include <iostream>
// This esp is 3C:8A:1F:A1:61:B8
uint8_t CarMacAddress[] = {0x3C, 0x8A, 0x1F, 0xA0, 0xDF, 0xC0};
uint8_t data[32];
#define Y_Pin 32
#define X_Pin 34
#define Button *****

// Rotary encoder struct
struct RotaryEncoderData {
    const uint8_t clkPin; // pin for clock signal
    const uint8_t dtPin; // pin for data signal
    const long minValue; // minimum value for the encoder
    const long maxValue; // maximum value for the encoder
    const bool reverse; // reverse direction of the encoder
    const bool pullup; // use pull-up resistor
    long currentValue; // current value of the encoder
    int lastClkState; // last state of the clock pin

    RotaryEncoderData(uint8_t clk, uint8_t dt, int minV, int maxV, bool rev, bool pull, int initVal) // constructor to initialize the encoder
        : clkPin(clk), dtPin(dt), minValue(minV), maxValue(maxV), // initialize the encoder with given parameters
          reverse(rev), pullup(pull), currentValue(initVal) {}

    void begin() { // setup the pins and initial state
        pinMode(clkPin, pullup ? INPUT_PULLUP : INPUT); // set clock pin mode
        pinMode(dtPin, pullup ? INPUT_PULLUP : INPUT); // set data pin mode
        lastClkState = digitalRead(clkPin); // read the initial state of the clock pin
    }

    void update() { // update the encoder value based on the clock and data signals
        int clkState = digitalRead(clkPin); // read the current state of the clock pin
        if (clkState != lastClkState) { // if the clock state has changedq
            if (digitalRead(dtPin) != clkState) { 
                currentValue += reverse ? -1 : 1; // increment or decrement the value based on the direction
            } else {
                currentValue += reverse ? 1 : -1;
            }
            if (currentValue < minValue) currentValue = minValue; // ensure the value does not go below the minimum
            if (currentValue > maxValue) currentValue = maxValue; // ensure the value does not exceed the maximum
        }
        lastClkState = clkState; // update the last clock state
    }
};

// 5 encoders: (pins are CLK, DT)
RotaryEncoderData encoders[5] = { // initialize the encoders with their respective pins and parameters
    {4, 17, 0, 360, false, false, 180}, 
    {15, 22, 0, 180, false, false, 180},
    {0, 36, 0, 180, false, false, 0},
    {26, 14, 0, 180, false, false, 90},
    {5, 13, 0, 360, false, false, 180}
    // {CLK, DT, minValue, maxValue, reverse, pullup, initialValue}
};

// Called every time ESP sends data via ESP-NOW
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) { // callback function to handle the status of sent data
    Serial.print("Last Packet Send Status: "); // print the status of the last sent packet
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail"); // check if the data was sent successfully
}

// Called when data is received
void onDataReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) { // callback function to handle received data
    Serial.println("Data received"); // print a message indicating that data has been received
    if (len != 2) { // check if the length of the received data is valid
        Serial.println("Invalid data length"); // print an error message if the length is not valid
        return; // exit the function if the data length is invalid
    }
    int command; // variable to store the command received
    memcpy(&command, incomingData, sizeof(command)); // copy the received data into the command variable
}

void setup() {
    Serial.begin(115200); // initialize serial communication at 115200 baud rate
    WiFi.mode(WIFI_STA); // set WiFi mode to station (client) mode
    pinMode(Button, INPUT);
    digitalRead(Button)

    if (esp_now_init() != ESP_OK) { // initialize ESP-NOW
        Serial.println("Error initializing ESP-NOW"); // print an error message if initialization fails
    } else { // if initialization is successful
        Serial.println("ESP-NOW initialized successfully"); // print a success message
    }

    esp_now_register_recv_cb(onDataReceive); // register the callback function for receiving data
    esp_now_register_send_cb(onDataSent); // register the callback function for sending data

    // Add peer
    esp_now_peer_info_t peerInfo = {}; 
    memcpy(peerInfo.peer_addr, CarMacAddress, 6); // copy the car's MAC address to the peer info
    peerInfo.channel = 0; // use the default channel
    peerInfo.encrypt = false; // do not use encryption
    if (esp_now_add_peer(&peerInfo) != ESP_OK) { // add the peer to the ESP-NOW network
        Serial.println("Failed to add master peer"); // print an error message if adding the peer fails
    }

    // Initialize encoders
    for (int i = 0; i < 5; i++) { // loop through each encoder
        encoders[i].begin(); // call the begin function to set up the encoder pins and initial state
    }
}

unsigned long lastSend = 0; // variable to keep track of the last time data was sent
void loop() { 
    // Update encoders every loop
    for (int i = 0; i < 5; i++) { // loop through each encoder
        encoders[i].update(); // call the update function to read the current state of the encoder and update its value
    }

    // Only send data every 50 ms
    if (millis() - lastSend > 50) { // check if 50 ms have passed since the last data send
        int32_t movementArray[8] = {0}; // initialize an array to hold the movement data

        for (int i = 0; i < 5; i++) { // loop through each encoder
            movementArray[2 + i] = encoders[i].currentValue; // store the current value of each encoder in the movement array
        }

        movementArray[0] = analogRead(X_Pin); // read the X-axis value from the joystick
        movementArray[1] = analogRead(Y_Pin); // read the Y-axis value from the joystick
        movementArray[7] = 1;
        if (movementArray[7] )

        memcpy(data, movementArray, sizeof(movementArray)); // copy the movement data into the data array to be sent

        esp_err_t result = esp_now_send(CarMacAddress, data, sizeof(movementArray)); // send the data via ESP-NOW to the car's MAC address
        if (result != ESP_OK) { // check if the data was sent successfully
            Serial.println("Error sending the data"); // print an error message if sending fails
        }

        lastSend = millis(); // update the last send time to the current time
    }
}
