#include "BLEModule.h"

ServerCallbacks::ServerCallbacks(BLEModule* module) {
    bleModule = module;
}

void ServerCallbacks::onConnect(BLEServer* pServer) {
    Serial.println("BLE device connected");
    bleModule->deviceConnected = true;
}

void ServerCallbacks::onDisconnect(BLEServer* pServer) {
    Serial.println("BLE device disconnected");
    bleModule->deviceConnected = false;
    delay(100);
    pServer->startAdvertising();
}

CharacteristicCallbacks::CharacteristicCallbacks(BLEModule* module) {
    bleModule = module;
}

void CharacteristicCallbacks::onWrite(BLECharacteristic* pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    if (value.length() > 0) {
        if (bleModule->dataCallback) {
            // Serial.printf("ble chara\n");
            // for (int i = 0; i < value.length(); i++) {
            //     Serial.printf("0x%x ", value.data()[i]);
            // }
            // Serial.println();
            bleModule->dataCallback((uint8_t*)value.data(), value.length());
        }
    }
}

BLEModule::BLEModule() {
    dataCallback = nullptr;
    connected = false;
    deviceConnected = false;
    oldDeviceConnected = false;
    pServer = nullptr;
    pService = nullptr;
    pTxCharacteristic = nullptr;
    pRxCharacteristic = nullptr;
}

void BLEModule::init(const char* deviceName) {
    BLEDevice::init(deviceName);
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks(this));

    pService = pServer->createService(SERVICE_UUID);

    pRxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_RX_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pRxCharacteristic->setCallbacks(new CharacteristicCallbacks(this));

    pTxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_TX_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pTxCharacteristic->addDescriptor(new BLE2902());

    pService->start();

    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    Serial.println("BLE server started, waiting for connection...");
}

void BLEModule::loop() {
    if (deviceConnected) {
        if (!connected) {
            connected = true;
        }
    } else {
        if (connected) {
            connected = false;
            pServer->startAdvertising();
            Serial.println("BLE advertising restarted");
        }
    }
    delay(200);
}

void BLEModule::sendData(const uint8_t* data, size_t length) {
    if (connected && pTxCharacteristic) {
        pTxCharacteristic->setValue(const_cast<uint8_t*>(data), length);
        pTxCharacteristic->notify();
        // Serial.printf("ble send %d\n", length);
    }
}

void BLEModule::setDataCallback(BLEDataCallback callback) {
    dataCallback = callback;
}

bool BLEModule::isConnected() {
    return connected;
}
