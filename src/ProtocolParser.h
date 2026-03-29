#ifndef PROTOCOL_PARSER_H
#define PROTOCOL_PARSER_H

#include <Arduino.h>
#include <map>
#include <vector>


#define CMD_SET 0x01
#define CMD_GET 0x02
#define CMD_GET_CONTIUNOUS 0x03

typedef struct {
    const char* description;
    uint8_t paramId;
    uint8_t get_can_cmd;
    uint8_t set_can_cmd;
    uint8_t dataLength;
} ParamMapping;

typedef struct {
    uint8_t get_can_cmd;
    uint8_t set_can_cmd;
    uint8_t dataLength;
} ParamConfig;

class ProtocolParser;
// 连续发送任务的数据结构
typedef struct {
    ProtocolParser* parser;
    uint8_t can_id;
    std::vector<uint8_t> paramIds;
    bool running;
} ContinuousTaskData;

typedef void (*CANFrameCallback)(uint32_t canId, uint8_t* data, size_t length);

extern const int PARAM_COUNT;
extern ParamMapping ProtocolParams[];

class ProtocolParser {
public:
    ProtocolParser();
    void init();
    bool parseBLEData(const uint8_t* data, size_t length);
    void setCANFrameCallback(CANFrameCallback callback);
    void registerParam(uint8_t paramId, uint32_t get_can_cmd, uint32_t set_can_cmd, uint8_t dataLength);
    void loadFromParams(ParamMapping* params, int paramCount);
    // 用于任务访问的公共方法
    bool hasParam(uint8_t paramId);
    ParamConfig getParamConfig(uint8_t paramId);
    CANFrameCallback getCANFrameCallback();
    void transferCan2Ble(uint32_t canId, uint8_t* src, size_t src_l, uint8_t* dst, size_t* dst_l);

private:
    std::map<uint8_t, ParamConfig> paramMap;
    CANFrameCallback canFrameCallback;
    ContinuousTaskData* currentContinuousTask;
    bool parseTLVBlock(const uint8_t* block, size_t blockLength);
    bool handleSet(const uint8_t* data, size_t length);
    bool handleGet(const uint8_t* data, size_t length);
    bool handleGetContinuous(const uint8_t* data, size_t length);
    void stopContinuousTask();
    int32_t parseInt32(const std::vector<uint8_t> &data);
};

#endif