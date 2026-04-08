#include "ProtocolParser.h"
#include <vector>

// 静态参数配置数组
ParamMapping ProtocolParams[] = {
    {"can_id", 0x01, 0x00, 0x2E, 4},
    {"position", 0x02, 0x08, 0x1E, 4},
    {"max_pos", 0x03, 0x1A, 0x26, 4},
    {"min_pos", 0x04, 0x1B, 0x27, 4},
    {"max_forward_speed", 0x05, 0x18, 0x24, 4},
    {"min_reverse_speed", 0x06, 0x19, 0x25, 4},
    {"encoder_voltage", 0x07, 0x78, 0x00, 4},
    {"current_can_id", 0x08, 0x0A, 0x00, 1},
    {"encoder_position", 0x09, 0x54, 0x53, 4}
};

// 参数数量
const int PARAM_COUNT = sizeof(ProtocolParams) / sizeof(ParamMapping);

ProtocolParser::ProtocolParser() {
    canFrameCallback = nullptr;
    currentContinuousTask = nullptr;
    continuousTaskHandle = nullptr;
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
    Serial.printf("can receive: id %d  ", canId);
    for (int i = 0; i < src_l; i++) {
        Serial.printf("0x%x ", src[i]);
    }
    Serial.println();
    uint8_t can_id = (canId >> 0)  & 0xFF;  // 最低字节
    dst[1] = can_id;
    ParamConfig config;
    uint8_t paramId = 0;
    uint8_t can_cmd = src[0];
    for (auto &param : paramMap) {
        if(param.second.get_can_cmd == can_cmd) {
            config = param.second;
            paramId = param.first;
            dst[0] = CMD_GET;
            break;
        }
    }
    if (config.get_can_cmd != can_cmd || paramId == 0) {
        Serial.printf("can not find can cmd 0x%x", can_cmd);
        return;
    }
    Serial.printf("get_can_cmd 0x%x\n", config.get_can_cmd);
    dst[2] = paramId;
    dst[3] = config.dataLength;
    int offset = 4;
    switch(config.get_can_cmd) {
        case 0x0A:
            dst[offset++] = can_id;
            *dst_l = offset;
            break;
        case 0x08:
        case 0x1A:
        case 0x1B:
            {
                std::vector<uint8_t> data = std::vector<uint8_t>(src, src + src_l);
                float value = (parseInt32(data) / 65536.0 / 121) * 360.0;

                uint32_t temp;
                memcpy(&temp, &value, sizeof(float));

                dst[offset++] = (temp >> 0)  & 0xFF;
                dst[offset++] = (temp >> 8)  & 0xFF;
                dst[offset++] = (temp >> 16) & 0xFF;
                dst[offset++] = (temp >> 24) & 0xFF;
                *dst_l = offset;
                break;
            }
        case 0x18:
        case 0x19:
            {
                std::vector<uint8_t> data = std::vector<uint8_t>(src, src + src_l);
                float value = (parseInt32(data) / 100.0 / 121) * 360.0;

                uint32_t temp;
                memcpy(&temp, &value, sizeof(float));

                dst[offset++] = (temp >> 0)  & 0xFF;
                dst[offset++] = (temp >> 8)  & 0xFF;
                dst[offset++] = (temp >> 16) & 0xFF;
                dst[offset++] = (temp >> 24) & 0xFF;
                *dst_l = offset;
                break;
            }
        case 0x78:
            {
                std::vector<uint8_t> data = std::vector<uint8_t>(src, src + src_l);
                float value = parseInt32(data) * 0.01;

                uint32_t temp;
                memcpy(&temp, &value, sizeof(float));

                dst[offset++] = (temp >> 0)  & 0xFF;
                dst[offset++] = (temp >> 8)  & 0xFF;
                dst[offset++] = (temp >> 16) & 0xFF;
                dst[offset++] = (temp >> 24) & 0xFF;
                *dst_l = offset;
                break;
            }
        case 0x54:
            {
                std::vector<uint8_t> data = std::vector<uint8_t>(src, src + src_l);
                int32_t value = parseInt32(data);

                dst[offset++] = (value >> 0)  & 0xFF;
                dst[offset++] = (value >> 8)  & 0xFF;
                dst[offset++] = (value >> 16) & 0xFF;
                dst[offset++] = (value >> 24) & 0xFF;
                *dst_l = offset;
                break;
            }
        default:
            break;
    }
    Serial.printf("transfer ble data: %d ", *dst_l);
    for (int i = 0; i < *dst_l; i++) {
        Serial.printf("%02X ", dst[i]);
    }
    Serial.println();

}

int32_t ProtocolParser::parseInt32(const std::vector<uint8_t> &data) {
    if (data.size() < 4)
        return 0;
    int32_t value =
        static_cast<int32_t>(data[1]) |
        (static_cast<int32_t>(data[2]) << 8) |
        (static_cast<int32_t>(data[3]) << 16) |
        (static_cast<int32_t>(data[4]) << 24);
    Serial.printf("parseint %d", value);
    return value;
}

void ProtocolParser::ble2CanConversion(uint8_t set_can_cmd, uint8_t* bleData, uint8_t* canData, uint8_t dataLength) {
    Serial.printf("BLE to CAN conversion for set_can_cmd: 0x%02X, dataLength: %d\n", set_can_cmd, dataLength);
    
    for (int i = 0; i < dataLength; i++) {
        Serial.printf("0x%x ", bleData[i]);
    }
    Serial.println();
    // 根据数据长度处理不同的转换逻辑
    if (dataLength == 1) {
        // 1字节参数，直接复制
        canData[1] = bleData[0];
        Serial.printf("1-byte data: 0x%02X\n", canData[1]);
    } else if (dataLength == 4) {
        // 4字节参数，需要转换
        // 从BLE数据中提取float值
        float value;
        uint32_t temp;
        memcpy(&temp, bleData, 4);
        memcpy(&value, &temp, sizeof(float));
        Serial.printf("Input float value: %f\n", value);
        
        int32_t convertedValue = 0;
        
        switch(set_can_cmd) {
            case 0x0E: // flush command, no conversion needed
                break;
            case 0x1E: // position
            case 0x26: // max_pos
            case 0x27: // min_pos
                // 转换公式: (value / 360.0) * 65536.0 * 121
                convertedValue = static_cast<int32_t>((value / 360.0) * 65536.0 * 121);
                break;
            case 0x24: // max_forward_speed
            case 0x25: // min_reverse_speed
                // 转换公式: (value / 360.0) * 100.0 * 121
                convertedValue = static_cast<int32_t>((value / 360.0) * 100.0 * 121);
                break;
            default:
                // 其他情况，直接复制数据
                memcpy(canData + 1, bleData, 4);
                return;
        }
        
        Serial.printf("Converted int32_t value: %d\n", convertedValue);
        
        // 将转换后的值写入CAN数据
        canData[1] = (convertedValue >> 0) & 0xFF;
        canData[2] = (convertedValue >> 8) & 0xFF;
        canData[3] = (convertedValue >> 16) & 0xFF;
        canData[4] = (convertedValue >> 24) & 0xFF;
        Serial.printf("Converted int32_t value1: %d\n", convertedValue);
        Serial.printf("CAN data: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", 
            canData[0], canData[1], canData[2], canData[3], canData[4]);
    } else {
        // 其他长度，直接复制数据
        memcpy(canData + 1, bleData, dataLength);
        Serial.printf("Direct copy for length %d\n", dataLength);
    }
}

void ProtocolParser::flushCanSetting(uint8_t can_id) {
    uint8_t data[5];
    data[0] = 0x0E;
    canFrameCallback(can_id, data, 1);
}

void ProtocolParser::stopContinuousTask() {
    if (currentContinuousTask) {
        Serial.println("Stopping existing continuous task...");
        currentContinuousTask->running = false;
        
        // 等待任务退出
        if (continuousTaskHandle) {
            // 等待最多2秒让任务退出
            for (int i = 0; i < 20; i++) {
                if (eTaskGetState(continuousTaskHandle) == eDeleted) {
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
        
        // 安全删除任务数据
        delete currentContinuousTask;
        currentContinuousTask = nullptr;
        continuousTaskHandle = nullptr;
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
    for (int i = 0; i < length; i++) {
        Serial.printf("0x%x ", data[i]);
    }
    Serial.println();
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
    // Serial.printf("Parsing SET len %d\n", length);
    // for (int i = 0; i < length; i++) {
    //     Serial.printf("0x%x ", data[i]);
    // }
    Serial.println();
    size_t offset = 2;
    bool is_need_flush = true;
    while (offset < length) {
        if (offset + 2 > length) {
            Serial.println("Invalid TLV block: incomplete header");
            return false;
        }

        uint8_t paramId = data[offset];
        uint8_t tlvLength = data[offset + 1];
        uint8_t can_data[5];

        if (offset + 2 + tlvLength > length) {
            Serial.printf("Invalid TLV block for param 0x%02X len %d: insufficient data\n", paramId, tlvLength);
            return false;
        }

        // const uint8_t* tlvData = &data[offset + 2];
        Serial.printf("TLV block: paramId=0x%02X, length=%d\n", paramId, tlvLength);

        auto it = paramMap.find(paramId);
        if (it != paramMap.end()) {
            ParamConfig config = it->second;
            can_data[0] = config.set_can_cmd;
            if (config.set_can_cmd == 0x1E) {
                is_need_flush = false;
            }
            // 使用ble2CanConversion方法进行数据转换
            ble2CanConversion(config.set_can_cmd, (uint8_t*)&data[offset + 2], can_data, config.dataLength);

            if (tlvLength == config.dataLength) {
                if (canFrameCallback) {
                    canFrameCallback(can_id, can_data, tlvLength+1);
                    if (is_need_flush) {
                        flushCanSetting(can_id);
                    }
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
                    // 停止现有的连续任务
                    stopContinuousTask();
                    canFrameCallback(0, can_data, 1);
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
                    callback(data->can_id, can_data, 1);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(500)); // 1秒延迟
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
        &continuousTaskHandle
    );
    
    return true;
}
