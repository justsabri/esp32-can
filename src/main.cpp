#include <Arduino.h>
#include "BLEModule.h"
#include "CANModule.h"
#include "ProtocolParser.h"

// 声明外部参数配置
extern ParamMapping ProtocolParams[];
extern const int PARAM_COUNT;

#define CAN_TX_PIN GPIO_NUM_15
#define CAN_RX_PIN GPIO_NUM_16

BLEModule bleModule;
CANModule canModule;
ProtocolParser protocolParser;

void onBLEDataReceived(uint8_t* data, size_t length) {
    Serial.printf("Received BLE data, length: %d\n", length);
    protocolParser.parseBLEData(data, length);
}

void onParsedCANFrame(uint32_t canId, uint8_t* data, size_t length) {
    Serial.printf("Protocol parsed: sending CAN frame - ID: 0x%lX\n", canId);
    canModule.sendMessage(canId, data, length);
}

void onCANDataReceived(uint32_t id, uint8_t* data, size_t length) {
    uint8_t bleBuffer[13];
    size_t bleLength;
    protocolParser.transferCan2Ble(id, data, length, bleBuffer, &bleLength);
    bleModule.sendData(bleBuffer, bleLength);
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("ESP32-S3 BLE-CAN Bridge Starting...");
    
    protocolParser.init();
    protocolParser.setCANFrameCallback(onParsedCANFrame);
    protocolParser.loadFromParams(ProtocolParams, PARAM_COUNT);
    
    bleModule.init("ESP32-CAN-Bridge");
    bleModule.setDataCallback(onBLEDataReceived);
    
    if (!canModule.init(CAN_TX_PIN, CAN_RX_PIN)) {
        Serial.println("Failed to initialize CAN module");
    }
    canModule.setDataCallback(onCANDataReceived);
}

void loop() {
    bleModule.loop();
    canModule.loop();
    // Serial.printf("111");
    // delay(1000);
}