#include "ProtocolParser.h"
#include <vector>

// 静态参数配置数组
ParamMapping ProtocolParams[] = {
    {"can_id", 0x01, 0x00, 0x2E, 1},
    {"position", 0x02, 0x08, 0x1E, 4},
    {"max_pos", 0x03, 0x1A, 0x26, 4},
    {"min_pos", 0x04, 0x1B, 0x27, 4},
    {"max_forward_speed", 0x05, 0x18, 0x24, 4},
    {"min_reverse_speed", 0x06, 0x19, 0x25, 4},
    {"encoder_voltage", 0x07, 0x78, 0x00, 4},
    {"current_can_id", 0x08, 0x0A, 0x00, 1}
};

// 参数数量
const int PARAM_COUNT = sizeof(ProtocolParams) / sizeof(ParamMapping);

ProtocolParser::ProtocolParser() {
    canFrameCallback = nullptr;
    currentContinuousTask = nullptr;
}

void ProtocolParser::init() {
    paramMap.clear();
}

void ProtocolParser::registerParam(uint8_t paramId, uint32_t get_can_cmd, uint32_t set_can_cmd, uint8_t dataLength) {
    ParamConfig config;
    config.get_can_cmd = get_can_cmd;
    config.set_can_cmd = set_can_cmd;
    config.dataLength = dataLength;
    paramMap[paramId] = config;
    Serial.printf("Registered param: 0x%02X -> CAN ID(get): 0x%lX, CAN ID(set): 0x%lX, length: %d\n", paramId, get_can_cmd, set_can_cmd, dataLength);
}

void ProtocolParser::loadFromParams(ParamMapping* params, int paramCount) {
    init();
    
    Serial.println("Loading params from static array...");
    for (int i = 0; i < paramCount; i++) {
        ParamMapping& param = params[i];
        Serial.printf("Loading param: %s (0x%02X) -> CAN ID(get): 0x%lX, CAN ID(set): 0x%lX, length: %d\n", 
            param.description, param.paramId, param.get_can_cmd, param.set_can_cmd, param.dataLength);
        registerParam(param.paramId, param.get_can_cmd, param.set_can_cmd, param.dataLength);
    }
}

bool ProtocolParser::hasParam(uint8_t paramId) {
    return paramMap.find(paramId) != paramMap.end();
}

ParamConfig ProtocolParser::getParamConfig(uint8_t paramId) {
    auto it = paramMap.find(paramId);
    if (it != paramMap.end()) {
        return it->second;
    }
    // 返回默认值
    ParamConfig config = {0, 0, 0};
    return config;
}

CANFrameCallback ProtocolParser::getCANFrameCallback() {
    return canFrameCallback;
}

void ProtocolParser::transferCan2Ble(uint32_t canId, uint8_t *src, size_t src_l, uint8_t *dst, size_t* dst_l) {

    for (int i = 0; i < src_l; i++) {
        Serial.printf("can receive: 0x%x\n", src[i]);
    }
    uint8_t can_id = (canId >> 0)  & 0xFF;  // 最低字节
    dst[1] = can_id;
    ParamConfig config;
    uint8_t paramId = 0;
    for (auto &param : paramMap) {
        if(param.second.get_can_cmd == src[1]) {
            config = param.second;
            paramId = param.first;
            dst[0] = CMD_GET;
            break;
        }
    }
    if (config.get_can_cmd != src[1] || paramId == 0) {
        Serial.printf("can not find can cmd 0x%x", src[1]);
        return;
    }

    dst[2] = paramId;
    switch(config.get_can_cmd) {
        case 0x0A:
            dst[3] = can_id;
            *dst_l = 4;
            break;
        case 0x08:
        case 0x1A:
        case 0x1B:
            {
                std::vector<uint8_t> data = std::vector<uint8_t>(src, src + src_l);
                float value = (parseInt32(data) / 65536.0 / 121) * 360.0;

                uint32_t temp;
                memcpy(&temp, &value, sizeof(float));

                dst[3] = (temp >> 0)  & 0xFF;
                dst[4] = (temp >> 8)  & 0xFF;
                dst[5] = (temp >> 16) & 0xFF;
                dst[6] = (temp >> 24) & 0xFF;
                *dst_l = 7;
                break;
            }
        case 0x18:
        case 0x19:
            {
                std::vector<uint8_t> data = std::vector<uint8_t>(src, src + src_l);
                float value = (parseInt32(data) / 100.0 / 121) * 360.0;

                uint32_t temp;
                memcpy(&temp, &value, sizeof(float));

                dst[3] = (temp >> 0)  & 0xFF;
                dst[4] = (temp >> 8)  & 0xFF;
                dst[5] = (temp >> 16) & 0xFF;
                dst[6] = (temp >> 24) & 0xFF;
                *dst_l = 7;
                break;
            }
        case 0x78:
            {
                std::vector<uint8_t> data = std::vector<uint8_t>(src, src + src_l);
                float value = parseInt32(data) * 0.01;

                uint32_t temp;
                memcpy(&temp, &value, sizeof(float));

                dst[3] = (temp >> 0)  & 0xFF;
                dst[4] = (temp >> 8)  & 0xFF;
                dst[5] = (temp >> 16) & 0xFF;
                dst[6] = (temp >> 24) & 0xFF;
                *dst_l = 7;
                break;
            }
        default:
            break;
    }

}

int32_t ProtocolParser::parseInt32(const std::vector<uint8_t> &data) {
    if (data.size() < 4)
        return 0;
    int32_t value =
        static_cast<int32_t>(data[1]) |
        (static_cast<int32_t>(data[2]) << 8) |
        (static_cast<int32_t>(data[3]) << 16) |
        (static_cast<int32_t>(data[4]) << 24);
    return value;
}

void ProtocolParser::stopContinuousTask() {
    if (currentContinuousTask) {
        Serial.println("Stopping existing continuous task...");
        currentContinuousTask->running = false;
        // 等待任务退出
        vTaskDelay(pdMS_TO_TICKS(100));
        delete currentContinuousTask;
        currentContinuousTask = nullptr;
        Serial.println("Existing continuous task stopped");
    }
}

void ProtocolParser::setCANFrameCallback(CANFrameCallback callback) {
    canFrameCallback = callback;
}

bool ProtocolParser::parseBLEData(const uint8_t* data, size_t length) {
    if (length < 1) {
        Serial.println("Invalid BLE data: too short");
        return false;
    }

    uint8_t cmd = data[0];

    bool res = false;
    switch(cmd) {
        case CMD_SET:
            res = handleSet(data, length);
            break;
        case CMD_GET:
            res = handleGet(data, length);
            break;
        case CMD_GET_CONTIUNOUS:
            res = handleGetContinuous(data, length);
            break;
        default:
            Serial.printf("Invalid CMD %x", cmd);
            break;
    }

    return res;
}

bool ProtocolParser::handleSet(const uint8_t* data, size_t length) {
    uint8_t can_id = data[1];
    Serial.println("Parsing SET command...");
    for (auto it : paramMap) {
        Serial.printf("map id: 0x%x\n", it.first);

    }
    size_t offset = 2;
    while (offset < length) {
        if (offset + 2 > length) {
            Serial.println("Invalid TLV block: incomplete header");
            return false;
        }

        uint8_t paramId = data[offset];
        uint8_t tlvLength = data[offset + 1];
        uint8_t can_data[5];

        if (offset + 2 + tlvLength > length) {
            Serial.printf("Invalid TLV block for param 0x%02X: insufficient data\n", paramId);
            return false;
        }

        // const uint8_t* tlvData = &data[offset + 2];
        Serial.printf("TLV block: paramId=0x%02X, length=%d\n", paramId, tlvLength);

        auto it = paramMap.find(paramId);
        if (it != paramMap.end()) {
            ParamConfig config = it->second;
            can_data[0] = config.set_can_cmd;
            memcpy(can_data + 1, &data[offset + 2], tlvLength);
            if (tlvLength == config.dataLength) {
                if (canFrameCallback) {
                    canFrameCallback(can_id, can_data, tlvLength+1);

                }
            } else {
                Serial.printf("Param 0x%02X length mismatch: expected %d, got %d\n", 
                    paramId, config.dataLength, tlvLength);
            }
        } else {
            Serial.printf("Unregistered paramId: 0x%02X\n", paramId);
        }

        offset += 2 + tlvLength;
    }

    return true;
}

bool ProtocolParser::handleGet(const uint8_t* data, size_t length) {
    if (length < 2) {
        Serial.println("Invalid GET command: too short");
        return false;
    }
    
    uint8_t can_id = data[1];
    Serial.println("Parsing GET command...");
    
    size_t offset = 2;
    while (offset < length) {
        uint8_t paramId = data[offset];
        uint8_t can_data[5] = {0};
        
        Serial.printf("GET param: 0x%02X\n", paramId);
        
        auto it = paramMap.find(paramId);
        if (it != paramMap.end()) {
            ParamConfig config = it->second;
            can_data[0] = config.get_can_cmd;
            // 其余字节为0
            if (canFrameCallback) {
                if (paramId == 0x08) {
                    canFrameCallback(1, can_data, 1);
                    canFrameCallback(2, can_data, 1);
                    canFrameCallback(3, can_data, 1);
                    canFrameCallback(4, can_data, 1);
                } else {
                    canFrameCallback(can_id, can_data, 1);
                }
            }
        } else {
            Serial.printf("Unregistered paramId: 0x%02X\n", paramId);
        }
        
        offset++;
    }
    
    return true;
}

// 连续发送任务
void continuousSendTask(void* pvParameters) {
    ContinuousTaskData* data = (ContinuousTaskData*)pvParameters;
    
    Serial.println("Continuous send task started");
    
    while (data->running) {
        for (uint8_t paramId : data->paramIds) {
            uint8_t can_data[5] = {0};
            
            if (data->parser->hasParam(paramId)) {
                ParamConfig config = data->parser->getParamConfig(paramId);
                can_data[0] = config.get_can_cmd;
                // 其余字节为0
                
                CANFrameCallback callback = data->parser->getCANFrameCallback();
                if (callback) {
                    callback(data->can_id, can_data, config.dataLength+1);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1秒延迟
    }
    
    Serial.println("Continuous send task stopped");
    vTaskDelete(NULL);
}

bool ProtocolParser::handleGetContinuous(const uint8_t* data, size_t length) {
    if (length < 2) {
        Serial.println("Invalid GET CONTINUOUS command: too short");
        return false;
    }
    
    uint8_t can_id = data[1];
    Serial.println("Parsing GET CONTINUOUS command...");
    
    // 停止现有的连续任务
    stopContinuousTask();
    
    // 收集所有paramId
    std::vector<uint8_t> paramIds;
    size_t offset = 2;
    while (offset < length) {
        uint8_t paramId = data[offset];
        paramIds.push_back(paramId);
        Serial.printf("GET CONTINUOUS param: 0x%02X\n", paramId);
        offset++;
    }
    
    // 创建任务数据
    ContinuousTaskData* taskData = new ContinuousTaskData();
    taskData->parser = this;
    taskData->can_id = can_id;
    taskData->paramIds = paramIds;
    taskData->running = true;
    
    // 保存当前任务
    currentContinuousTask = taskData;
    
    // 创建任务
    xTaskCreate(
        continuousSendTask,
        "ContinuousSendTask",
        2048,
        taskData,
        1,
        NULL
    );
    
    return true;
}
