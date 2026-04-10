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
QueueHandle_t can_rx_queue;

typedef struct {
    uint32_t id;
    uint8_t data[8];
    uint8_t length;
} can_msg_t;

void onBLEDataReceived(uint8_t* data, size_t length) {
    // Serial.printf("Received BLE data, length: %d\n", length);
    protocolParser.parseBLEData(data, length);
}

void onParsedCANFrame(uint32_t canId, uint8_t* data, size_t length) {
    // Serial.printf("Protocol parsed: sending CAN frame - ID: 0x%lX len %d\n", canId, length);
    canModule.sendMessage(canId, data, length);
}

void onCANDataReceived(uint32_t id, uint8_t* data, size_t length) {
    can_msg_t my_msg;

    my_msg.id = id;
    my_msg.length = length;
    memcpy(my_msg.data, data, my_msg.length);

    xQueueSend(can_rx_queue, &my_msg, 0);

}

void can_process_task(void *arg)
{
    can_msg_t msg;

    while (1) {
        if (xQueueReceive(can_rx_queue, &msg, portMAX_DELAY) == pdTRUE) {

            // 👉 在这里随便处理（慢一点也没关系）
            uint8_t bleBuffer[13];
            size_t bleLength = 0;
            protocolParser.transferCan2Ble(msg.id, msg.data, msg.length, bleBuffer, &bleLength);
            // Serial.printf("send ble data: %d ", bleLength);
            // for (int i = 0; i < bleLength; i++) {
            //     Serial.printf("%02X ", bleBuffer[i]);
            // }
            // Serial.println();
            if (bleLength == 0) {
                Serial.print("ble transfer error");
                return;
            }
            bleModule.sendData(bleBuffer, bleLength);
        }
    }
}

void can_monitor_task(void *arg)
{
    twai_status_info_t status;

    while (1) {
        twai_get_status_info(&status);

        printf("missed=%d, overrun=%d, rx_queue=%d\n",
               status.rx_missed_count,
               status.rx_overrun_count,
               status.msgs_to_rx);

        vTaskDelay(pdMS_TO_TICKS(1000));  // 每1秒打印一次
    }
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

    can_rx_queue = xQueueCreate(50, sizeof(can_msg_t));
    xTaskCreate(can_process_task, "can_proc", 4096, NULL, 5, NULL);

    // xTaskCreate(can_monitor_task, "can_mon", 4096, NULL, 5, NULL);
}

void loop() {
    bleModule.loop();
    canModule.loop();
    // Serial.printf("111");
    // delay(1000);
}