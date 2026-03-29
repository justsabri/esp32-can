#ifndef CAN_MODULE_H
#define CAN_MODULE_H

#include <Arduino.h>
#include <driver/twai.h>

typedef void (*CANDataCallback)(uint32_t id, uint8_t* data, size_t length);

class CANModule {
public:
    CANModule();
    bool init(gpio_num_t txPin, gpio_num_t rxPin, twai_timing_config_t timingConfig = TWAI_TIMING_CONFIG_1MBITS());
    void loop();
    bool sendMessage(uint32_t id, const uint8_t* data, size_t length);
    void setDataCallback(CANDataCallback callback);

private:
    CANDataCallback dataCallback;
    bool initialized;
};

#endif
