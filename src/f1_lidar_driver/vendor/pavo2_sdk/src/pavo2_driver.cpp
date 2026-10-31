#include <iostream>
#include <string>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <iostream>
#include <set>

#include "../include/pavo2_driver.h"

namespace pavo2
{

pavo2_driver::pavo2_driver()
    : m_connector(new Pavo2TcpConnector)
    , m_parser(new Pavo2FrameParser(m_connector))
    , m_protocol(PAVO2_PROTO_TYPE::DEFAULT)
    , m_local_ip("")
    , m_local_port(0)
{

}

pavo2_driver::~pavo2_driver()
{
    m_parser.reset();
    m_connector.reset();
}

void pavo2_driver::set_local_address(const std::string& local_ip, uint16_t local_port)
{
    m_local_ip = local_ip;
    m_local_port = local_port;
    m_connector->SetLocalAddress(local_ip, local_port);
}

void pavo2_driver::set_conn_protocol(PAVO2_PROTO_TYPE proto_type)
{
    m_protocol = proto_type;
}

bool pavo2_driver::pavo2_open(const std::string& lidar_ip, uint16_t lidar_port, uint32_t timeout)
{
    if((PAVO2_PROTO_TYPE::DEFAULT == m_protocol) || (PAVO2_PROTO_TYPE::TCP == m_protocol))
    {
        m_connector.reset(new Pavo2TcpConnector);
        m_parser.reset(new Pavo2FrameParser(m_connector));
    }
    else if(PAVO2_PROTO_TYPE::UDP == m_protocol)
    {
        m_connector.reset(new Pavo2UdpConnector);
        m_parser.reset(new Pavo2MsgParser(m_connector));
    }
    else
    {
        return false;
    }
    m_connector->SetLocalAddress(m_local_ip, m_local_port);
    return m_connector->Connect(lidar_ip, lidar_port, timeout);
}

bool pavo2_driver::pavo2_is_open()
{
    return m_connector->IsConnected();
}

bool pavo2_driver::pavo2_close()
{
    m_connector->Disconnect();
    return true;
}

std::size_t pavo2_driver::get_scans_available() const
{
    return m_parser->getScansAvailable();
}

std::size_t pavo2_driver::get_ptp_deviations_available() const
{
    return m_parser->getPtpDeviationsAvailable();
}

bool pavo2_driver::get_scanned_data(pavo_response_scan_t* data_buffer, uint16_t& count, uint16_t timeout)
{
    uint32_t time_stamp;
    return m_parser->recvData(data_buffer, count, time_stamp, timeout);
}

bool pavo2_driver::get_scanned_data(std::vector<pavo_response_scan_t>& vec_buff, uint16_t timeout)
{
    uint32_t time_stamp;
    return  m_parser->recvData(vec_buff, time_stamp, timeout);
}

bool pavo2_driver::get_scanned_data(pavo_response_pcd_t* data_buffer, uint16_t& count, uint16_t timeout)
{
    uint32_t time_stamp;
    uint32_t cnt;
    std::vector<pavo_response_scan_t> cn;
    if(!m_parser->recvData(cn, time_stamp, timeout))
        return false;

    for(cnt = 0; (cnt < cn.size()) && (cnt < count); cnt++)
    {
        data_buffer[cnt].intensity = cn[cnt].intensity;
        data_buffer[cnt].x = cn[cnt].distance * cos(Degree2Radians(cn[cnt].angle / 100.0f));
        data_buffer[cnt].y = cn[cnt].distance * sin(Degree2Radians(cn[cnt].angle / 100.0f));
        data_buffer[cnt].z = 0;
    }
    count = cnt;

    return true;
}

bool pavo2_driver::get_scanned_data(std::vector<pavo_response_pcd_t>& vec_buff, uint16_t timeout)
{
    uint32_t time_stamp;
    uint32_t cnt;
    std::vector<pavo_response_scan_t> cn;
    pavo_response_pcd_t pcd;
    if(!m_parser->recvData(cn, time_stamp, timeout))
        return false;
    vec_buff.resize(cn.size());
    for(cnt = 0; cnt < cn.size(); cnt++)
    {
        pcd.intensity = cn[cnt].intensity;
        pcd.x = cn[cnt].distance * cos(Degree2Radians(cn[cnt].angle/100.0f));
        pcd.y = cn[cnt].distance * sin(Degree2Radians(cn[cnt].angle/100.0f));
        pcd.z = 0;
        vec_buff[cnt] = pcd;
    }

    return true;
}

bool pavo2_driver::get_scanned_data(pavo_response_scan_t* data_buffer, uint16_t& count, uint32_t& timestamp, uint16_t timeout)
{
    return m_parser->recvData(data_buffer, count, timestamp, timeout);
}

bool pavo2_driver::get_scanned_data(std::vector<pavo_response_scan_t>& vec_buff, uint32_t& timestamp, uint16_t timeout)
{
    return  m_parser->recvData(vec_buff, timestamp, timeout);
}

bool pavo2_driver::get_scanned_data(pavo_response_pcd_t* data_buffer, uint16_t& count, uint32_t& timestamp, uint16_t timeout)
{
    uint32_t cnt;
    std::vector<pavo_response_scan_t> cn;
    if(!m_parser->recvData(cn, timestamp, timeout))
        return false;

    for(cnt = 0; (cnt < cn.size()) && (cnt < count); cnt++)
    {
        data_buffer[cnt].intensity = cn[cnt].intensity;
        data_buffer[cnt].x = cn[cnt].distance * cos(Degree2Radians(cn[cnt].angle / 100.0f));
        data_buffer[cnt].y = cn[cnt].distance * sin(Degree2Radians(cn[cnt].angle / 100.0f));
        data_buffer[cnt].z = 0;
    }
    count = cnt;

    return true;
}

bool pavo2_driver::get_scanned_data(std::vector<pavo_response_pcd_t>& vec_buff, uint32_t& timestamp, uint16_t timeout)
{
    uint32_t cnt;
    std::vector<pavo_response_scan_t> cn;
    pavo_response_pcd_t pcd;
    if(!m_parser->recvData(cn, timestamp, timeout))
        return false;
    vec_buff.resize(cn.size());
    for(cnt = 0; cnt < cn.size(); cnt++)
    {
        pcd.intensity = cn[cnt].intensity;
        pcd.x = cn[cnt].distance * cos(Degree2Radians(cn[cnt].angle / 100.0f));
        pcd.y = cn[cnt].distance * sin(Degree2Radians(cn[cnt].angle / 100.0f));
        pcd.z = 0;
        vec_buff[cnt] = pcd;
    }

    return true;
}

bool pavo2_driver::get_scanned_data(pavo_packet_info_scan_t& packet_info_scan, uint16_t timeout/* = 0*/)
{
    return m_parser->recvData(packet_info_scan, timeout);
}

bool pavo2_driver::get_scanned_data(pavo_packet_info_pcd_t& packet_info_pcd, uint16_t timeout/* = 0*/)
{
    pavo_packet_info_scan_t packet_info_scan;
    if(!m_parser->recvData(packet_info_scan, timeout))
        return false;
    
    pavo_response_pcd_t pcd;
    for(uint32_t cnt = 0; cnt < packet_info_scan.vec_buff.size(); cnt++)
    {
        pcd.intensity = packet_info_scan.vec_buff[cnt].intensity;
        pcd.x = packet_info_scan.vec_buff[cnt].distance * cos(Degree2Radians(packet_info_scan.vec_buff[cnt].angle / 100.0f));
        pcd.y = packet_info_scan.vec_buff[cnt].distance * sin(Degree2Radians(packet_info_scan.vec_buff[cnt].angle / 100.0f));
        pcd.z = 0;
        packet_info_pcd.vec_buff.push_back(pcd);
    }
    packet_info_pcd.time_stamp = packet_info_scan.time_stamp;
    packet_info_pcd.time_delay = packet_info_scan.time_delay;
    
    return true;
}

bool pavo2_driver::recv_point_cloud(pavo_point_cloud_scan_t& scan, uint16_t timeout)
{
    return m_parser->recvData(scan, timeout);
}

bool pavo2_driver::recv_ptp_deviation_data(pavo_ptp_deviation_t& ptp_deviation, uint16_t timeout)
{
    return m_parser->recvPtpDeviation(ptp_deviation, timeout);
}

bool pavo2_driver::get_device_sn(std::string& sn)
{
    return m_parser->get_cmd_value(CMD_SN, sn);
}

bool pavo2_driver::get_device_pn(std::string& pn)
{
    return m_parser->get_cmd_value(CMD_PN, pn);
}

bool pavo2_driver::get_sdk_ver(std::string& ver)
{
    ver = std::string(SDK_VER);
    return true;
}

bool pavo2_driver::get_os_ver(std::string& ver)
{
    return m_parser->get_cmd_value(CMD_OS_VER, ".", ver);
}

bool pavo2_driver::get_firmware_ver(std::string& ver)
{
    return m_parser->get_cmd_value(CMD_FW_VER, ".", ver);
}

bool pavo2_driver::get_os_error(uint32_t& errCode, uint8_t iIndex/* = 0*/)
{
    uint32_t response_u32;
    if(!m_parser->get_cmd_value(CMD_OS_EX, response_u32))
        return false;
    if(iIndex != 0)
    {
        errCode = (0xFF >> iIndex) & response_u32;    
    }
    else
    {
        errCode = response_u32;
    }
    return true;
}

int  pavo2_driver::get_error_code()
{
    return 0;
}

bool pavo2_driver::enable_data(bool en)
{
    return m_parser->set_cmd_value(CMD_ENABLE_DATA, en);
}

bool pavo2_driver::set_intensity_mode(bool en)
{
    return m_parser->set_cmd_value(CMD_INTENSITY_MODE, en);
}

bool pavo2_driver::get_intensity_mode(bool& en)
{
    return m_parser->get_cmd_value(CMD_INTENSITY_MODE, en);
}

bool pavo2_driver::set_angle_range(uint16_t start, uint16_t end)
{
    //ANGLE_RANGE_MIN <= start < end <= ANGLE_RANGE_MAX
    if((end > PAVO2_ANGLE_RANGE_MAX) || (start >= end))
        return false;

    return m_parser->set_cmd_value(CMD_ANGLE_RANGE, start, end);
}

bool pavo2_driver::get_angle_range(uint16_t& start, uint16_t& end)
{
    return m_parser->get_cmd_value(CMD_ANGLE_RANGE, start, end, DEF_TIMEOUT);
}

bool pavo2_driver::get_max_angle_range(uint32_t& val)
{
    return m_parser->get_cmd_value(CMD_ANGLE_RANGE_ENABLE, val);
}

bool pavo2_driver::set_motor_speed(uint8_t val)
{
    return m_parser->set_cmd_value(CMD_MOTOR_SPEED, val);
}

bool pavo2_driver::get_motor_speed(uint8_t& val)
{
    return m_parser->get_cmd_value(CMD_MOTOR_SPEED, val);
}

bool pavo2_driver::get_motor_speed_enable(uint32_t& val)
{
    return m_parser->get_cmd_value(CMD_MOTOR_SPEED_ENABLE, val);
}

std::set<uint16_t> LEGAL_ANGLE_RESOLUTIONS{1, 2, 4, 8, 16, 32};
bool pavo2_driver::set_angle_resolution(uint16_t val)
{
    if(LEGAL_ANGLE_RESOLUTIONS.find(val) == LEGAL_ANGLE_RESOLUTIONS.end())
        return false;
    return m_parser->set_cmd_value(CMD_ANGLE_RESOLUTION, val);
}

bool pavo2_driver::get_angle_resolution(uint16_t& val)
{
    return m_parser->get_cmd_value(CMD_ANGLE_RESOLUTION, val);
}

bool pavo2_driver::get_angle_resolution_enable(uint32_t& val)
{
    return m_parser->get_cmd_value(CMD_ANGLE_RESOLUTION_ENABLE, val);
}

bool pavo2_driver::set_device_ip(const std::string& ip)
{
    return m_parser->set_cmd_ip_address(CMD_DEVICE_IP, ip);
}

bool pavo2_driver::get_device_ip(std::string& ip)
{
    return m_parser->get_cmd_ip_address(CMD_DEVICE_IP, ip);
}

bool pavo2_driver::set_tcp_port(uint16_t val)
{
    return m_parser->set_cmd_value(CMD_TCP_PORT, val);
}

bool pavo2_driver::get_tcp_port(uint16_t& val)
{
    return m_parser->get_cmd_value(CMD_TCP_PORT, val);
}

bool pavo2_driver::set_udp_port(uint16_t val)
{
    return m_parser->set_cmd_value(CMD_UDP_PORT, val);
}

bool pavo2_driver::get_udp_port(uint16_t& val)
{
    return m_parser->get_cmd_value(CMD_UDP_PORT, val);
}

bool pavo2_driver::set_dest_ip(const std::string& ip)
{
    return m_parser->set_cmd_ip_address(CMD_DEST_IP, ip);
}

bool pavo2_driver::get_dest_ip(std::string& ip)
{
    return m_parser->get_cmd_ip_address(CMD_DEST_IP, ip);
}

bool pavo2_driver::set_dest_port(uint16_t val)
{
    return m_parser->set_cmd_value(CMD_DEST_PORT, val);
}

bool pavo2_driver::get_dest_port(uint16_t& val)
{
    return m_parser->get_cmd_value(CMD_DEST_PORT, val);
}

bool pavo2_driver::set_device_mask(const std::string& mask)
{
    return m_parser->set_cmd_ip_address(CMD_MASK, mask);
}

bool pavo2_driver::get_device_mask(std::string& mask)
{
    return m_parser->get_cmd_ip_address(CMD_MASK, mask);
}

bool pavo2_driver::set_device_gateway(const std::string& gateway)
{
    return m_parser->set_cmd_ip_address(CMD_GATEWAY, gateway);
}

bool pavo2_driver::get_device_gateway(std::string& gateway)
{
    return m_parser->get_cmd_ip_address(CMD_GATEWAY, gateway);
}

//not used now
bool pavo2_driver::set_os_restart()
{
    return false;
}

bool pavo2_driver::enable_tail_filter(uint16_t method)
{
    return m_parser->enable_tail_filter(method);
}

bool pavo2_driver::enable_front_filter(uint16_t iLevel)
{
    return m_parser->enable_front_filter(iLevel);
}

bool pavo2_driver::enable_far_hr_filter(uint16_t hrLevel)
{
    return m_parser->enable_hr_filter(hrLevel);
}

bool pavo2_driver::enable_near_hr(bool en)
{
    return m_parser->set_cmd_value(CMD_ENABLE_NEAR_HR, en);
}

bool pavo2_driver::is_near_hr_enabled(bool& en)
{
    return m_parser->get_cmd_value(CMD_ENABLE_NEAR_HR, en);
}

/*
 * ret: whether call success
 * en: whether near_hr supported
 */
bool pavo2_driver::is_near_hr_supported(bool& en)
{
    //Jude by "CMD_NEAR_HR_ABILITY"
    if(m_parser->get_cmd_value(CMD_NEAR_HR_ABILITY, en))
    {
        return true;
    }

    //Judge by "CMD_ENABLE_NEAR_HR"
    bool ptp_en = false;
    if(m_parser->get_cmd_value(CMD_ENABLE_NEAR_HR, ptp_en))
    {
        en = true;
    }
    else
    {
        en = false;
    }

    return true;
}

bool pavo2_driver::get_device_mac(std::string& mac) //FIX bug
{
    std::string response;
    if(!m_parser->get_cmd_value(CMD_MAC, response))
        return false;

    std::stringstream ss;
    for(uint8_t i = PAVO2_CMD_LEN + 2; i < response.size(); i++)
    {
        uint16_t value = (uint16_t)response[i] & 0xFF;
        ss <<std::hex<<value;
        if(i == (response.size() - 1))
            continue;
        ss << ":";
    }
    mac = ss.str();
    return true;
}

bool pavo2_driver::set_curr_proto(PAVO2_PROTO_TYPE proto_type)
{
    uint32_t proto_u32 = static_cast<uint32_t>(proto_type);
    return m_parser->set_cmd_value(CMD_PROTO_CURR, proto_u32);
}

bool pavo2_driver::get_curr_proto(PAVO2_PROTO_TYPE& proto_type)
{
    uint8_t proto_u32;
    if(!m_parser->get_cmd_value(CMD_PROTO_CURR, proto_u32))
        return false;

    if(proto_u32 == 0)
    {
        proto_type = PAVO2_PROTO_TYPE::DEFAULT;
        return true;
    }
    else if((proto_u32 & 0x01) != 0)
    {
        proto_type = PAVO2_PROTO_TYPE::TCP;
        return true;
    }
    else if((proto_u32 & 0x02) != 0)
    {
        proto_type = PAVO2_PROTO_TYPE::UDP;
        return true;
    }

    return false;
}

bool pavo2_driver::get_supported_protos(std::vector<PAVO2_PROTO_TYPE>& protos)
{
    uint8_t proto_en_u32;
    if(!m_parser->get_cmd_value(CMD_PROTO_EN, proto_en_u32))
        return false;

    if(proto_en_u32 == 0)
    {
        protos.push_back(PAVO2_PROTO_TYPE::DEFAULT);
        return true;
    }

    if((proto_en_u32 & 0x01) != 0)
    {
        protos.push_back(PAVO2_PROTO_TYPE::TCP);
    }

    if((proto_en_u32 & 0x02) != 0)
    {
        protos.push_back(PAVO2_PROTO_TYPE::UDP);
    }

    return true;
}

bool pavo2_driver::set_echo_mode(bool en)
{
    return m_parser->set_cmd_value(CMD_ECHO_MODE, en);
}

bool pavo2_driver::get_echo_mode(bool& en)
{
    return m_parser->get_cmd_value(CMD_ECHO_MODE, en);
}

bool pavo2_driver::set_outdoor_mode(bool en)
{
    return m_parser->set_cmd_value(CMD_OUTDOOR, en);
}

bool pavo2_driver::get_outdoor_mode(bool& en)
{
    return m_parser->get_cmd_value(CMD_OUTDOOR, en);
}

bool pavo2_driver::set_calibrate_horizontal_angle(uint32_t angle)
{
    //0 < angle < 36000
    if(angle >= 36000)
        return false;

    return m_parser->set_cmd_value(CMD_CAL_HOR_ANGLE, angle);
}

bool pavo2_driver::get_calibrate_horizontal_angle(uint32_t& angle)
{
    return m_parser->get_cmd_value(CMD_CAL_HOR_ANGLE, angle);
}

bool pavo2_driver::get_max_angle_distance(uint32_t& val)
{
    return m_parser->get_cmd_value(CMD_MAX_DISTANCE, val);
}

bool pavo2_driver::set_warn_angle_range(uint16_t start, uint16_t end)
{
    if((end > PAVO2_ANGLE_RANGE_MAX) || (start >= end))
        return false;

    return m_parser->set_cmd_value(CMD_CAL_WARN_ANGLE_RANGE, end, start);
}

bool pavo2_driver::get_warn_angle_range(uint16_t& start, uint16_t& end)
{
    return m_parser->get_cmd_value(CMD_CAL_WARN_ANGLE_RANGE, start, end, 0);
}

bool pavo2_driver::set_warn_intensity_distance(uint16_t distance, uint16_t intensity)
{
    return m_parser->set_cmd_value(CMD_CAL_WARN_INTENSITY_DIS, intensity, distance);
}

bool pavo2_driver::get_warn_intensity_distance(uint16_t& distance, uint16_t& intensity)
{
    return m_parser->get_cmd_value(CMD_CAL_WARN_INTENSITY_DIS, intensity, distance, 0);
}

bool pavo2_driver::set_warn_point_number(uint16_t number)
{
    return m_parser->set_cmd_value(CMD_CAL_WARN_DOT_NUMBER, number);
}

bool pavo2_driver::get_warn_point_number(uint16_t& number)
{
    return m_parser->get_cmd_value(CMD_CAL_WARN_DOT_NUMBER, number);
}

bool pavo2_driver::enable_ptp(bool en)
{
    return m_parser->set_cmd_value(CMD_ENABLE_PTP, en);
}

bool pavo2_driver::is_ptp_enabled(bool& en)
{
    return m_parser->get_cmd_value(CMD_ENABLE_PTP, en);
}

bool pavo2_driver::is_ptp_supported(bool& en)
{
    if(m_parser->get_cmd_value(CMD_PTP_ABILITY, en))
    {
        return true;
    }

    bool ptp_en = false;
    if(m_parser->get_cmd_value(CMD_ENABLE_PTP, ptp_en))
    {
        en = true;
    }
    else
    {
        en = false;
    }

    return true;
}

//precision is one digit
bool pavo2_driver::get_temperature(float& temperature)
{
    uint32_t temperature_u32;
    if(!m_parser->get_cmd_value(CMD_TEMPERATURE, temperature_u32))
        return false;
    int sign = (temperature_u32 & 0x80000000) == 0 ? 1 : -1;
    temperature = float((temperature_u32 & 0x7FFFFFF) / 10) / 10 * sign;
    return true;
}

bool pavo2_driver::get_heartbeat()
{
    std::string ver;
    return m_parser->get_cmd_value(CMD_OS_VER, ".", ver);
}

bool pavo2_driver::get_cmd_raw(PAVO2_CMD_INDEX cmd_type, uint32_t& val)
{
    return m_parser->get_cmd_value(cmd_type, val);
}

}//namespace pavo2;
