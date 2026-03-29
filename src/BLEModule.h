#ifndef BLE_MODULE_H
#define BLE_MODULE_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_RX_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_TX_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a9"

typedef void (*BLEDataCallback)(uint8_t* data, size_t length);

class BLEModule {
public:
    BLEModule();
    void init(const char* deviceName);
    void loop();
    void sendData(const uint8_t* data, size_t length);
    void setDataCallback(BLEDataCallback callback);
    bool isConnected();
    BLEDataCallback dataCallback;
    bool deviceConnected;

private:
    BLEServer* pServer;
    BLEService* pService;
    BLECharacteristic* pTxCharacteristic;
    BLECharacteristic* pRxCharacteristic;
    bool connected;
    bool oldDeviceConnected;
};

class ServerCallbacks: public BLEServerCallbacks {
public:
    ServerCallbacks(BLEModule* module);
    void onConnect(BLEServer* pServer) override;
    void onDisconnect(BLEServer* pServer) override;
private:
    BLEModule* bleModule;
};

class CharacteristicCallbacks: public BLECharacteristicCallbacks {
public:
    CharacteristicCallbacks(BLEModule* module);
    void onWrite(BLECharacteristic* pCharacteristic) override;
private:
    BLEModule* bleModule;
};

#endif
