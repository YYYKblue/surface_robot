#ifndef __PAVO2_PARSER_H__
#define __PAVO2_PARSER_H__

#include <memory>
#include <map>
#include <mutex>

#include "pavo2_net.h"
#include "pavo2_types.h"
#include "utils.h"
#include "pavo2_filter.h"
#include "LdrAlgorithm.h"

namespace pavo2 {

#define DEF_TIMEOUT 300

#define PAVO2_ANGLE_RANGE_MIN (0)
#define PAVO2_ANGLE_RANGE_MAX (36000)

#define PAVO2_CMD_SET_GET_INDEX (2)
#define PAVO2_CMD_TYPE_INDEX (3)
#define PAVO2_CMD_SET (0xA0)
#define PAVO2_CMD_GET (0xB0)
#define PAVO2_CMD_LEN (0x4)

enum PAVO2_CMD_INDEX {
    CMD_UNLOCK = 0, //0
    CMD_OS_VER,
    CMD_FW_VER,
    CMD_OS_EX,
    CMD_PN = 4,

    CMD_SN, //5
    CMD_ENABLE_DATA,
    CMD_ANGLE_RANGE,
    CMD_ANGLE_RESOLUTION,
    CMD_MOTOR_SPEED = 9,

    CMD_INTENSITY_MODE, //10 INTENSITY
    CMD_SLEEP_RESTART,
    CMD_RESET,
    CMD_ECHO_MODE,
    CMD_MAC = 14,

    CMD_DEVICE_IP, //15
    CMD_TCP_PORT,
    CMD_UDP_PORT,
    CMD_MASK,
    CMD_GATEWAY = 19,

    CMD_DEST_IP, //20
    CMD_DEST_PORT,
    CMD_PROTO_EN,
    CMD_PROTO_CURR,
    CMD_OUTDOOR = 24,

    CMD_ANGLE_RANGE_ENABLE, //25
    CMD_ANGLE_RESOLUTION_ENABLE,
    CMD_MOTOR_SPEED_ENABLE,
    CMD_MAX_DISTANCE,
    CMD_CAL_HOR_ANGLE = 29,

    CMD_CAL_WARN_ANGLE_RANGE, //30
    CMD_CAL_WARN_INTENSITY_DIS,
    CMD_CAL_WARN_DOT_NUMBER,
    CMD_ENABLE_PTP,
    CMD_PTP_DEVIATION_FREQUENCY = 34,

    CMD_ENABLE_NEAR_HR, //35
    CMD_TEMPERATURE,
    CMD_PTP_ABILITY,
    CMD_NEAR_HR_ABILITY,

    CMD_INDEX_LEN = 39
};

static const unsigned char PAVO2_CMD[][CMD_INDEX_LEN] = {
    { 0xEB, 0x90, 0x00, 0xA0 },  //CMD_UNLOCK //0
    { 0xEB, 0x90, 0x00, 0xA1 },  //CMD_OS_VER
    { 0xEB, 0x90, 0x00, 0xA2 },  //CMD_FW_VER
    { 0xEB, 0x90, 0x00, 0xA3 },  //CMD_OS_EX
    { 0xEB, 0x90, 0x00, 0xA4 },  //CMD_PN

    { 0xEB, 0x90, 0x00, 0xA5 },  //CMD_SN //5
    { 0xEB, 0x90, 0x00, 0xA6 },  //CMD_ENABLE_DATA
    { 0xEB, 0x90, 0x00, 0xA7 },  //CMD_ANGLE_RANGE
    { 0xEB, 0x90, 0x00, 0xA8 },  //CMD_ANGLE_RESOLUTION
    { 0xEB, 0x90, 0x00, 0xA9 },  //CMD_MOTOR_SPEED

    { 0xEB, 0x90, 0x00, 0xAA },  //CMD_INTENSITY_MODE 10
    { 0xEB, 0x90, 0x00, 0xAB },  //CMD_SLEEP_RESTART
    { 0xEB, 0x90, 0x00, 0xAC },  //CMD_RESET
    { 0xEB, 0x90, 0x00, 0xAE },  //CMD_ECHO_MODE
    { 0xEB, 0x90, 0x00, 0xE0 },  //CMD_MAC

    { 0xEB, 0x90, 0x00, 0xE1 },  //CMD_DEVICE_IP     //15
    { 0xEB, 0x90, 0x00, 0xE2 },  //CMD_TCP_PORT
    { 0xEB, 0x90, 0x00, 0xE3 },  //CMD_UDP_PORT
    { 0xEB, 0x90, 0x00, 0xE5 },  //CMD_MASK
    { 0xEB, 0x90, 0x00, 0xE6 },  //CMD_GATEWAY

    { 0xEB, 0x90, 0x00, 0xE7 },  //CMD_DEST_IP //20
    { 0xEB, 0x90, 0x00, 0xE8 },  //CMD_DEST_PORT
    { 0xEB, 0x90, 0x00, 0xE9 },  //CMD_PROTO_EN
    { 0xEB, 0x90, 0x00, 0xEA },  //CMD_PROTO_CURR
    { 0xEB, 0x90, 0x00, 0xB0 },  //CMD_OUTDOOR

    { 0xEB, 0x90, 0x00, 0xAF },  //CMD_ANGLE_RANGE_ENABLE //25
    { 0xEB, 0x90, 0x00, 0xB3 },  //CMD_ANGLE_RESOLUTION_ENABLE
    { 0xEB, 0x90, 0x00, 0xB2 },  //CMD_MOTOR_SPEED_ENABLE
    { 0xEB, 0x90, 0x00, 0xB1 },  //CMD_MAX_DISTANCE
    { 0xEB, 0x90, 0x00, 0xB4 },  //CMD_CAL_HOR_ANGLE

    { 0xEB, 0x90, 0x00, 0xB5 },  //CMD_CAL_WARN_ANGLE_RANGE //30
    { 0xEB, 0x90, 0x00, 0xB6 },  //CMD_CAL_WARN_INTENSITY_DIS
    { 0xEB, 0x90, 0x00, 0xB7 },  //CMD_CAL_WARN_DOT_NUMBER
    { 0xEB, 0x90, 0x00, 0xC0 },  //CMD_ENABLE_PTP
    { 0xEB, 0x90, 0x00, 0xC1 },  //CMD_PTP_DEVIATION_FREQUENCY

    { 0xEB, 0x90, 0x00, 0xC9 },  //CMD_ENABLE_NEAR_HR //35
    { 0xEB, 0x90, 0x00, 0xCA },  //CMD_TEMPERATURE
    { 0xEB, 0x90, 0x00, 0xC2 },  //CMD_PTP_ABILITY,
    { 0xEB, 0x90, 0x00, 0xC7 }   //CMD_NEAR_HR_ABILITY,

};

static const uint8_t PAVO2_CMD_ARG_LEN[CMD_INDEX_LEN] = {
    0x4, //CMD_UNLOCK //0
    0x4, //CMD_OS_VER
    0x4, //CMD_FW_VER
    0x4, //CMD_OS_EX
    0x4, //CMD_PN

    0x4, //CMD_SN //5
    0x4, //CMD_ENABLE_DATA
    0x4, //CMD_ANGLE_RANGE
    0x4, //CMD_ANGLE_RESOLUTION
    0x4, //CMD_MOTOR_SPEED

    0x4, //CMD_INTENSITY_MODE 10
    0x4, //CMD_SLEEP_RESTART
    0x4, //CMD_RESET
    0x4, //CMD_ECHO_MODE
    0x8, //CMD_MAC

    0x4, //CMD_DEVICE_IP     //15
    0x4, //CMD_TCP_PORT
    0x4, //CMD_UDP_PORT
    0x4, //CMD_MASK
    0x4, //CMD_GATEWAY

    0x4, //CMD_DEST_IP //20
    0x4, //CMD_DEST_PORT
    0x4, //CMD_PROTO_EN
    0x4, //CMD_PROTO_CURR
    0x4, //CMD_OUTDOOR

    0x4, //CMD_ANGLE_RANGE_ENABLE //25
    0x4, //CMD_ANGLE_RESOLUTION_ENABLE
    0x4, //CMD_MOTOR_SPEED_ENABLE
    0x4, //CMD_MAX_DISTANCE
    0x4, //CMD_CAL_HOR_ANGLE

    0x4, //CMD_CAL_WARN_ANGLE_RANGE //30
    0x4, //CMD_CAL_WARN_INTENSITY_DIS
    0x4, //CMD_CAL_WARN_DOT_NUMBER
    0x4, //CMD_ENABLE_PTP
    0x4, //CMD_PTP_DEVIATION_FREQUENCY

    0x4, //CMD_ENABLE_NEAR_HR //35
    0x4,  //CMD_TEMPERATURE
    0x4, //CMD_PTP_ABILITY,
    0x4  //CMD_NEAR_HR_ABILITY,
};


const std::size_t MAX_CMD_DATA_SIZE = 128;
struct Pavo2CmdDataDef
{

    Pavo2CmdDataDef() = delete;

    Pavo2CmdDataDef(char* data, std::size_t size)
    {
        cmdSize = size;
        memset(szCmd, 0, sizeof(szCmd));
        memcpy(szCmd, data, size);
    }

    uint16_t CmdIndex()
    {
        return (uint16_t)szCmd[PAVO2_CMD_TYPE_INDEX] & 0x00FF;
    }

    char szCmd[MAX_CMD_DATA_SIZE];
    std::size_t cmdSize;
};

enum class PAVO2_PROTO_TYPE : int
{
    DEFAULT = 0,
    TCP = 1,
    UDP = 2
};

enum class PAVO2_FRAME_DATA_TYPE : uint16_t
{
    INTENSITY_DISTANCE = 0x0000,
    CHECK_MODE = 0x0001,
    DISTANCE = 0x0002,
};

enum class PAVO2_MSG_TYPE : uint8_t
{
    POINT_CLOUD = 0x01,
    COMMAND = 0x11,
    PTP = 0x21
};

enum class PAVO2_DATA_TYPE : uint16_t
{
    INTENSITY_DISTANCE = 0x0000,
    //CHECK_MODE = 0x0001,
    DISTANCE = 0x0002,
};

class Pavo2Parser
{
private:
    static const std::size_t PTP_DEVATION_LEADING_LEN = 4;
    static const std::size_t PTP_DEVATION_LAST_SYNC_TIME_LEN = 8;
    static const std::size_t PTP_DEVATION_TIME_OFFSET_LEN = 8;
    static const std::size_t PTP_DEVATION_LEN = PTP_DEVATION_LEADING_LEN + PTP_DEVATION_LAST_SYNC_TIME_LEN + PTP_DEVATION_TIME_OFFSET_LEN;
    static const uint32_t PTP_DEVATION_LEADING_VALUE = 0xEB90E000;

public:
    static std::string& trimSpace(std::string& str);
    static bool ip2Uint32(const std::string& ip_str, uint32_t& ip_u32);
    static bool ip2Bytes(const std::string& ip_str, unsigned char* ip_bytes);
    static bool bytes2Ip(std::string& ip_str, const unsigned char* ip_bytes);
    static bool check_cmd(PAVO2_CMD_INDEX cmd, char* content, std::size_t size);
    static std::string packReadCommand(PAVO2_CMD_INDEX cmd_idx);
    static std::string packWriteCommand(PAVO2_CMD_INDEX cmd_idx, uint32_t value);

public:
    Pavo2Parser(std::shared_ptr<Pavo2Connector> connector);
    virtual ~Pavo2Parser();

    //Function for pavo2_net
    virtual void parseRecvData(std::unique_ptr<Pavo2RecvDataUnit>&& data_ptr) = 0;
    virtual std::string packCommand2Proto(const std::string& total_command) = 0;

public:
    //----------------functions for filters---------
    bool enable_tail_filter(uint16_t method = 0);
    bool enable_front_filter(uint16_t iLevel = 0);
    bool enable_hr_filter(uint16_t iLevel = 0);

    bool recvPtpDeviation(pavo_ptp_deviation_t& ptp_dev, uint16_t time_out = DEF_TIMEOUT);
    std::size_t getPtpDeviationsAvailable();

    bool recvData(pavo_point_cloud_scan_t& scan, uint16_t time_out = DEF_TIMEOUT);
    std::size_t getScansAvailable();

    bool recvData(pavo_response_scan_t* data_buffer, uint16_t& count, uint32_t& time_stamp, uint16_t time_out = DEF_TIMEOUT);
    bool recvData(std::vector<pavo_response_scan_t>& vec_buff, uint32_t& time_stamp, uint16_t time_out = DEF_TIMEOUT);
    bool recvData(pavo_packet_info_scan_t& packet_info_scan, uint16_t time_out = DEF_TIMEOUT);

    bool get_cmd_value(PAVO2_CMD_INDEX cmd_type, std::string& response, uint16_t time_out = DEF_TIMEOUT);
    bool get_cmd_value(PAVO2_CMD_INDEX cmd_type, const std::string& del, std::string& response, uint16_t time_out = DEF_TIMEOUT);
    bool get_cmd_value(PAVO2_CMD_INDEX cmd_type, uint32_t& val, uint16_t time_out = DEF_TIMEOUT);
    bool get_cmd_value(PAVO2_CMD_INDEX cmd_type, uint16_t& val, uint16_t time_out = DEF_TIMEOUT);
    bool get_cmd_value(PAVO2_CMD_INDEX cmd_type, uint8_t& val, uint16_t time_out = DEF_TIMEOUT);
    bool get_cmd_value(PAVO2_CMD_INDEX cmd_type, bool& en, uint16_t time_out = DEF_TIMEOUT);
    //left:bytes 0-1, right:bytes 2-3; big endian
    bool get_cmd_value(PAVO2_CMD_INDEX cmd_type, uint16_t& left, uint16_t& right, uint16_t time_out = DEF_TIMEOUT);
    bool get_cmd_ip_address(PAVO2_CMD_INDEX cmd_type, std::string& ip_addr, uint16_t time_out = DEF_TIMEOUT);

    bool set_cmd_value(PAVO2_CMD_INDEX cmd_type, uint32_t val);
    bool set_cmd_value(PAVO2_CMD_INDEX cmd_type, uint16_t val);
    bool set_cmd_value(PAVO2_CMD_INDEX cmd_type, uint8_t val);
    bool set_cmd_value(PAVO2_CMD_INDEX cmd_type, bool en);
    bool set_cmd_value(PAVO2_CMD_INDEX cmd_type, uint16_t left, uint16_t right);
    bool set_cmd_ip_address(PAVO2_CMD_INDEX cmd_type, const std::string& ip_addr);

protected:
    bool parsePtpDeviation(char* data_ptr, std::size_t data_size);

private:
    bool recvCMD(PAVO2_CMD_INDEX cmd_idx, std::string& response, uint16_t time_out = DEF_TIMEOUT);
    bool sendCMD(PAVO2_CMD_INDEX cmd_idx, uint32_t value); //for write register
    bool unlockWrite();

protected:
    std::shared_ptr<Pavo2Connector> m_connector;
    MADataFilter* m_pMADataFilter;
    LdrAlgorithm algorithm;
    int hr_level;

    std::mutex m_cmd_op_mutx;
    std::condition_variable m_cmd_op_cond;
    std::map<uint16_t, std::shared_ptr<Pavo2CmdDataDef>> m_map_cmd;

    SynchronizedQueue<pavo_point_cloud_scan_t> m_scans_que;
    SynchronizedQueue<pavo_ptp_deviation_t> m_ptp_que;
};

class Pavo2FrameParser : public Pavo2Parser
{
private:
    static const uint32_t PAVO2_FRAME_HEAD1_UINT32 = 0xFF534D49;
    static const uint32_t PAVO2_FRAME_HEAD2_UINT32 = 0x4E494353;
    const unsigned char PAVO2_FRAME_HEAD[sizeof(PAVO2_FRAME_HEAD1_UINT32) + sizeof(PAVO2_FRAME_HEAD2_UINT32)] = {0xFF, 0x53, 0x4D, 0x49, 0x4E, 0x49, 0x43, 0x53};
    static const uint64_t PAVO2_FRAME_HEAD_UINT64 = 0xFF534D494E494353;

    static const std::size_t PAVO2_FRAME_HEAD_LEN = 8;
    static const std::size_t PAVO2_FRAME_VERSION_RESERVER_LEN = 4;
    const unsigned char PAVO2_FRAME_VERSION_RESERVER[PAVO2_FRAME_VERSION_RESERVER_LEN] = { 0x01, 0x0, 0x0, 0x0 };

    static const std::size_t PAVO2_FRAME_DATA_LEN = 4;

    static const std::size_t PAVO2_FRAME_END_LEN = 4;
    const unsigned char PAVO2_FRAME_END[PAVO2_FRAME_END_LEN] = { 0xFE, 0xFE, 0xFE, 0xFE };
    static const uint32_t PAVO2_FRAME_END_UINT32 = 0xFEFEFEFE;

    static const std::size_t PAVO2_FRAME_LEAST_LEN = PAVO2_FRAME_HEAD_LEN + PAVO2_FRAME_VERSION_RESERVER_LEN + PAVO2_FRAME_DATA_LEN + PAVO2_FRAME_END_LEN;

    static const uint32_t  PAVO2_FRAME_DATA_TYPE_UINT32 = 0xEB90C000;
    static const uint32_t  PAVO2_FRAME_PTP_DEVIATION_TYPE_UINT32 = 0xEB90E000;
    static const uint32_t  PAVO2_FRAME_CMD_GET_TYPE_UINT32 = 0xEB90B000;
    static const uint32_t  PAVO2_FRAME_CMD_SET_TYPE_UINT32 = 0xEB90A000;

public:
    Pavo2FrameParser(std::shared_ptr<Pavo2Connector> connector);
    virtual ~Pavo2FrameParser();

    //Function for pavo2_net
    void parseRecvData(std::unique_ptr<Pavo2RecvDataUnit>&& data_ptr) override;
    std::string packCommand2Proto(const std::string& total_command) override;

private:
    bool parsePointCloud(char* data_ptr, std::size_t data_size);

private:
    std::unique_ptr<Pavo2RecvDataUnit> m_parse_buff;
};

class Pavo2MsgParser : public Pavo2Parser
{
    const uint32_t PAVO2_MSG_START_SIGN1 = 0x584D4744;
    const uint32_t PAVO2_MSG_START_SIGN2 = 0x31323031;
    const uint8_t PAVO2_MSG_VERSION = 0x01;
public:
    Pavo2MsgParser(std::shared_ptr<Pavo2Connector> connector);
    virtual ~Pavo2MsgParser();

    //Function for pavo2_net
    void parseRecvData(std::unique_ptr<Pavo2RecvDataUnit>&& data_ptr) override;
    std::string packCommand2Proto(const std::string& total_command) override;
private:
    msg_protocol_header_t buildMsgHeader(PAVO2_MSG_TYPE type);
    void resetReceiveState();

private:
    uint16_t m_scan_serial_now;
    uint16_t m_points_expected_now;
    uint16_t m_recved_points_num;

    pavo_point_cloud_scan_t m_current_scan;
    uint32_t m_curr_time_stamp;
};

} //namespace pavo2

#endif // __PAVO2_PARSER_H__
