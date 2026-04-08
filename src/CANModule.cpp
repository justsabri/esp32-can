#include "CANModule.h"

CANModule::CANModule() {
    dataCallback = nullptr;
    initialized = false;
}

bool CANModule::init(gpio_num_t txPin, gpio_num_t rxPin, twai_timing_config_t timingConfig) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(txPin, rxPin, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 20;
    g_config.tx_queue_len = 20;
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t result = twai_driver_install(&g_config, &timingConfig, &f_config);
    if (result != ESP_OK) {
        Serial.printf("Failed to install TWAI driver: %d\n", result);
        return false;
    }

    result = twai_start();
    if (result != ESP_OK) {
        Serial.printf("Failed to start TWAI driver: %d\n", result);
        twai_driver_uninstall();
        return false;
    }

    initialized = true;
    Serial.println("CAN module initialized successfully");
    return true;
}

void CANModule::loop() {
    if (!initialized) return;

    twai_message_t message;
    esp_err_t result = twai_receive(&message, pdMS_TO_TICKS(10));
    if (result == ESP_OK) {
        Serial.printf("Received CAN message - ID: 0x%lX, Data: ", message.identifier);
        for (int i = 0; i < message.data_length_code; i++) {
            Serial.printf("%02X ", message.data[i]);
        }
        Serial.println();

        if (dataCallback) {
            dataCallback(message.identifier, message.data, message.data_length_code);
        }
    }
}

bool CANModule::sendMessage(uint32_t id, const uint8_t* data, size_t length) {
    if (!initialized) return false;
    if (length > 8) return false;

    twai_message_t message;
    message.identifier = id;
    message.extd = 0;
    message.rtr = 0;
    message.ss = 0;
    message.self = 0;
    message.dlc_non_comp = 0;
    message.data_length_code = length;
    memcpy(message.data, data, length);

    esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(100));
    if (result == ESP_OK) {
        Serial.printf("Sent CAN message - ID: 0x%lX, Data: ", id);
        for (size_t i = 0; i < length; i++) {
            Serial.printf("%02X ", data[i]);
        }
        Serial.println();
        return true;
    } else {
        Serial.printf("Failed to send CAN message: %d\n", result);
        return false;
    }
}

void CANModule::setDataCallback(CANDataCallback callback) {
    dataCallback = callback;
}
