#include <sstream>
#include <iostream>
#include <iomanip>
#include "../include/pavo2_parser.h"
#include "../include/utils.h"

namespace pavo2 {

//------------------public functions as tools---------------
std::string& Pavo2Parser::trimSpace(std::string &str)
{
    if(str.empty())
    {
        return str;
    }

    str.erase(0, str.find_first_not_of(" "));  //trim white space
    str.erase(str.find_last_not_of(" ") + 1);

    str.erase(0, str.find_first_not_of("\t"));  //trim tab
    str.erase(str.find_last_not_of("\t") + 1);

    return str;
}

bool Pavo2Parser::ip2Uint32(const std::string& ip_str, uint32_t& ip_u32)
{
    unsigned char ip_bytes[4];
    if(!ip2Bytes(ip_str, ip_bytes))
        return false;

    ip_u32 = (((uint32_t)ip_bytes[0] << 24) & 0xFF000000) |  (((uint32_t)ip_bytes[1] << 16) & 0x00FF0000)
            | (((uint32_t)ip_bytes[0] << 8) & 0x0000FF00) | ((uint32_t)ip_bytes[0] & 0x000000FF);

    return true;
}

bool Pavo2Parser::ip2Bytes(const std::string& ip_str, unsigned char* ip_bytes)
{
    if(ip_str.empty())  //empty string
        return false;

    if(count(ip_str.begin(), ip_str.end(), '.') != 3)
        return false;

    std::string ip_str_copy(ip_str);
    trimSpace(ip_str_copy); //remove leading and tail space

    std::string str_tmp;
    std::size_t prev_pos = 0;

    int dot_num = 0;
    int byte_single = 0;
    char tmp_bytes[4];

    for(std::size_t i = 0; i < ip_str_copy.length(); i++)
    {
        if(ip_str_copy[i] == '.')
        {
            if(dot_num++ > 3)
                return false;

            str_tmp = ip_str_copy.substr(prev_pos, i - prev_pos);
            prev_pos = i + 1;

            byte_single = atoi(str_tmp.c_str());  //invalid value
            if(byte_single > 255)
                return false;
            tmp_bytes[dot_num - 1] = static_cast<char>(byte_single);

            continue;
        }
        if((ip_str_copy[i] < '0') || (ip_str_copy[i] > '9')) //invalid character
            return false;
    }

    str_tmp = ip_str_copy.substr(prev_pos);
    byte_single = atoi(str_tmp.c_str());
    tmp_bytes[dot_num] = static_cast<char>(byte_single);

    memcpy(ip_bytes, tmp_bytes, 4);

    return true;
}

bool Pavo2Parser::bytes2Ip(std::string& ip_str, const unsigned char* ip_bytes)
{
    if(ip_bytes == 0)
        return false;

    ip_str = std::to_string(static_cast<unsigned char>(ip_bytes[0])) + "." +
        std::to_string(static_cast<unsigned char>(ip_bytes[1])) + "." +
        std::to_string(static_cast<unsigned char>(ip_bytes[2])) + "." +
        std::to_string(static_cast<unsigned char>(ip_bytes[3]));
    return true;
}

bool Pavo2Parser::check_cmd(PAVO2_CMD_INDEX cmd, char* content, std::size_t size)
{
    if(cmd == PAVO2_CMD_INDEX::CMD_ENABLE_NEAR_HR)
	return true;
    content[PAVO2_CMD_SET_GET_INDEX] = 0x00;
    if(size < PAVO2_CMD_LEN)
    {
        return false;
    }
    if(memcmp(content, PAVO2_CMD[cmd], PAVO2_CMD_LEN))
    {
        return false;
    }
    uint8_t arg_len;
    arg_len = size - PAVO2_CMD_LEN;
    if(arg_len != PAVO2_CMD_ARG_LEN[cmd])
    {
        return false;
    }
    return true;
}

//-------------------------constructor------------------------------------
Pavo2Parser::Pavo2Parser(std::shared_ptr<Pavo2Connector> connector)
{
    m_connector = connector;
    ReadCallback_t call_back = std::bind(&Pavo2Parser::parseRecvData, this, std::placeholders::_1);

    m_connector->SetReadCallback(call_back);
    m_pMADataFilter = new MADataFilter();
    hr_level = 0;
}

//-------------------------destructor-------------------------------------
Pavo2Parser::~Pavo2Parser()
{
    if(m_pMADataFilter)
    {
        delete m_pMADataFilter;
        m_pMADataFilter = nullptr;
    }
}

//------------------------functions for driver-----------------------------
bool Pavo2Parser::enable_tail_filter(uint16_t method)
{
    if(m_pMADataFilter)
    {
        m_pMADataFilter->EnableTailFilter(method);
        return true;
    }
    return false;
}

bool Pavo2Parser::enable_front_filter(uint16_t iLevel)
{
    if(m_pMADataFilter)
    {
        m_pMADataFilter->EnableFrontFilter(iLevel);
        return true;
    }
    return false;
}

bool Pavo2Parser::enable_hr_filter(uint16_t iLevel)
{
    hr_level = iLevel;
    return true;
}

bool Pavo2Parser::recvPtpDeviation(pavo_ptp_deviation_t& ptp_dev, uint16_t time_out)
{
    return m_ptp_que.dequeue(ptp_dev, time_out);
}

std::size_t Pavo2Parser::getPtpDeviationsAvailable()
{
    return m_ptp_que.size();
}

bool Pavo2Parser::recvData(pavo_point_cloud_scan_t& scan, uint16_t time_out)
{
    return m_scans_que.dequeue(scan, time_out);
}

std::size_t Pavo2Parser::getScansAvailable()
{
    return m_scans_que.size();
}

bool Pavo2Parser::recvData(pavo_response_scan_t* data_buffer, uint16_t& count, uint32_t &time_stamp, uint16_t time_out)
{
    pavo_point_cloud_scan_t pcl_scan;
    if(!recvData(pcl_scan, time_out))
        return false;
    count = uint16_t(pcl_scan.data.size());
    time_stamp = pcl_scan.time_stamp;
    memcpy(data_buffer, &pcl_scan.data[0], sizeof(pavo_response_scan_t) * count);
    return true;
}

bool Pavo2Parser::recvData(std::vector<pavo_response_scan_t>& vec_buff, uint32_t &time_stamp, uint16_t time_out)
{
    pavo_point_cloud_scan_t pcl_scan;
    if(!recvData(pcl_scan, time_out))
        return false;
    time_stamp = pcl_scan.time_stamp;

    vec_buff.swap(pcl_scan.data);
    return true;
}

bool Pavo2Parser::recvData(pavo_packet_info_scan_t& packet_info_scan, uint16_t time_out)
{
    pavo_point_cloud_scan_t pcl_scan;
    if(!recvData(pcl_scan, time_out))
        return false;

    packet_info_scan.time_stamp = pcl_scan.time_stamp;
    packet_info_scan.time_delay = pcl_scan.process_delay;
    packet_info_scan.vec_buff.swap(pcl_scan.data);
    return true;
}

std::string Pavo2Parser::packReadCommand(PAVO2_CMD_INDEX cmd_idx)
{
    //deal with command format
    char send_cmd[PAVO2_CMD_LEN];
    uint16_t total_cmd_len = sizeof(send_cmd) / sizeof(unsigned char);
    memset(send_cmd, 0, total_cmd_len);
    //set the content
    memcpy(send_cmd, PAVO2_CMD[cmd_idx], PAVO2_CMD_LEN);
    send_cmd[PAVO2_CMD_SET_GET_INDEX] = PAVO2_CMD_GET;

    return std::string(send_cmd, PAVO2_CMD_LEN);
}

std::string Pavo2Parser::packWriteCommand(PAVO2_CMD_INDEX cmd_idx, uint32_t value)
{
    char send_cmd[32];
    uint16_t total_cmd_len = sizeof(send_cmd) / sizeof(unsigned char);
    memset(send_cmd, 0, total_cmd_len);

    //set the content
    memcpy(send_cmd, PAVO2_CMD[cmd_idx], PAVO2_CMD_LEN);
    send_cmd[PAVO2_CMD_SET_GET_INDEX] = PAVO2_CMD_SET;

    send_cmd[PAVO2_CMD_LEN] = 0xFF & (value >> 24);
    send_cmd[PAVO2_CMD_LEN + 1] = 0xFF & (value >> 16);
    send_cmd[PAVO2_CMD_LEN + 2] = 0xFF & (value >> 8);
    send_cmd[PAVO2_CMD_LEN + 3] = 0xFF & (value);

    //set the send len
    total_cmd_len = PAVO2_CMD_LEN + PAVO2_CMD_ARG_LEN[cmd_idx];
    return std::string(send_cmd, total_cmd_len);
}

bool Pavo2Parser::get_cmd_value(PAVO2_CMD_INDEX cmd_type, std::string& response, uint16_t time_out)
{
    std::string tmp_str;
    if(!recvCMD(cmd_type, tmp_str, time_out))
        return false;

    std::stringstream ss;
    for(uint8_t i = PAVO2_CMD_LEN; i < tmp_str.size(); i++)
    {
        uint16_t value = ((uint16_t)tmp_str[i]) & 0x00FF;
        ss << std::hex << std::setw(2) << std::setfill('0') << std::setiosflags(std::ios::uppercase) << value;
    }
    response = ss.str();
    return true;
}

bool Pavo2Parser::get_cmd_value(PAVO2_CMD_INDEX cmd_type, const std::string& del, std::string& response, uint16_t time_out)
{
    std::string tmp_str;
    if(!recvCMD(cmd_type, tmp_str, time_out))
        return false;

    std::stringstream ss;
    for(uint8_t i = PAVO2_CMD_LEN; i < tmp_str.size(); i++)
    {
        uint16_t value = ((uint16_t)tmp_str[i]) & 0x00FF;
        ss << value;
        if(i == (tmp_str.size() - 1))
            break;
        ss << del;
    }
    response = ss.str();
    return true;
}

bool Pavo2Parser::get_cmd_value(PAVO2_CMD_INDEX cmd_type, uint32_t& val, uint16_t time_out)
{
    std::string response;
    if(!recvCMD(cmd_type, response, time_out))
        return false;

    val = ((uint32_t)response[PAVO2_CMD_LEN + 3] & 0xFF) | (((uint32_t)response[PAVO2_CMD_LEN + 2] & 0xFF) << 8)
            | (((uint32_t)response[PAVO2_CMD_LEN + 1] & 0xFF) << 16) | (((uint32_t)response[PAVO2_CMD_LEN] & 0xFF) << 24);
    return true;
}

bool Pavo2Parser::get_cmd_value(PAVO2_CMD_INDEX cmd_type, uint16_t& val, uint16_t time_out)
{
    uint32_t tmp_val;
    if(!get_cmd_value(cmd_type, tmp_val, time_out))
        return false;
    val = tmp_val & 0xFFFF;
    return true;
}

bool Pavo2Parser::get_cmd_value(PAVO2_CMD_INDEX cmd_type, uint8_t& val, uint16_t time_out)
{
    uint32_t tmp_val;
    if(!get_cmd_value(cmd_type, tmp_val, time_out))
        return false;
    val = tmp_val & 0xFF;
    return true;
}

bool Pavo2Parser::get_cmd_value(PAVO2_CMD_INDEX cmd_type, bool& en, uint16_t time_out)
{
    std::string tmp_str;
    if(!recvCMD(cmd_type, tmp_str, time_out))
        return false;

    //check the value
    if(tmp_str.back())
    {
        en = true;
    }
    else
    {
        en = false;
    }
    return true;
}

//left:bytes 0-1, right:bytes 2-3; big endian
bool Pavo2Parser::get_cmd_value(PAVO2_CMD_INDEX cmd_type, uint16_t& left, uint16_t& right, uint16_t time_out)
{
    uint32_t tmp_val;
    if(!get_cmd_value(cmd_type, tmp_val, time_out))
        return false;

    left = (uint16_t)((tmp_val & 0xFFFF0000) >> 16);
    right = (uint16_t)(tmp_val & 0x0000FFFF);
    return true;
}

bool Pavo2Parser::get_cmd_ip_address(PAVO2_CMD_INDEX cmd_type, std::string& ip_addr, uint16_t time_out)
{
    std::string response;
    if(!recvCMD(cmd_type, response, time_out))
        return false;
    if(!bytes2Ip(ip_addr, (const unsigned char*)&response[PAVO2_CMD_LEN]))
        return false;
    return true;
}

bool Pavo2Parser::set_cmd_value(PAVO2_CMD_INDEX cmd_type, uint32_t val)
{
    return sendCMD(cmd_type, val);
}

bool Pavo2Parser::set_cmd_value(PAVO2_CMD_INDEX cmd_type, uint16_t val)
{
    uint32_t val_u32 = val & 0x0000FFFF;
    return sendCMD(cmd_type, val_u32);
}

bool Pavo2Parser::set_cmd_value(PAVO2_CMD_INDEX cmd_type, uint8_t val)
{
    uint32_t val_u32 = val & 0x000000FF;
    return sendCMD(cmd_type, val_u32);
}

bool Pavo2Parser::set_cmd_value(PAVO2_CMD_INDEX cmd_type, bool en)
{
    uint32_t val_u32 = en ? 0x01 : 0x00;
    return sendCMD(cmd_type, val_u32);
}

bool Pavo2Parser::set_cmd_value(PAVO2_CMD_INDEX cmd_type, uint16_t left, uint16_t right)
{
    uint32_t left_u32 = (uint32_t(left) << 16) & 0xFFFF0000 ;
    uint32_t right_u32 = (uint32_t(right) & 0x0000FFFF);
    uint32_t tmp_val = left_u32 | right_u32;
    return sendCMD(cmd_type, tmp_val);
}

bool Pavo2Parser::set_cmd_ip_address(PAVO2_CMD_INDEX cmd_type, const std::string& ip_addr)
{
    uint32_t ip_u32;
    if(!ip2Uint32(ip_addr, ip_u32))
        return false;
    return sendCMD(cmd_type, ip_u32);
}

bool Pavo2Parser::recvCMD(PAVO2_CMD_INDEX cmd_idx, std::string& response, uint16_t time_out)
{
    std::string read_command = packReadCommand(cmd_idx);
    std::string read_frame = packCommand2Proto(read_command);

    //send the data
    if(!m_connector->Write(read_frame.c_str(), read_frame.length()))
    {
        return false;
    }

    std::unique_lock<std::mutex> guard(this->m_cmd_op_mutx);

	
    if(time_out == 0)
    {
        while((m_map_cmd.find(PAVO2_CMD[cmd_idx][PAVO2_CMD_TYPE_INDEX]) == m_map_cmd.end()) && m_connector->IsConnected())
        {
            this->m_cmd_op_cond.wait(guard);
        }
    }
    else //timeout > 0
    {
        while((m_map_cmd.find(PAVO2_CMD[cmd_idx][PAVO2_CMD_TYPE_INDEX]) == m_map_cmd.end()) && m_connector->IsConnected())
        {
            if(m_cmd_op_cond.wait_for(guard, std::chrono::milliseconds(time_out)) == std::cv_status::timeout)
	    {
                return false;
	    }
        }
    }

    auto it = m_map_cmd.find(uint16_t(PAVO2_CMD[cmd_idx][PAVO2_CMD_TYPE_INDEX]));
    if(it == m_map_cmd.end())
    {
        return false;
    }

    auto cmdTemp = std::move(it->second);
    m_map_cmd.erase(it);

    response = std::string((char*)cmdTemp->szCmd, cmdTemp->cmdSize);

    return check_cmd(cmd_idx, (char*)cmdTemp->szCmd, cmdTemp->cmdSize);
}

bool Pavo2Parser::sendCMD(PAVO2_CMD_INDEX cmd_idx, uint32_t value) //for write register
{
    if(!unlockWrite())
        return false;

    std::string write_command = packWriteCommand(cmd_idx, value);
    std::string write_frame = packCommand2Proto(write_command);

    //send the data
    return m_connector->Write(write_frame.c_str(), write_frame.length());
}

bool Pavo2Parser::unlockWrite()
{
    std::string write_command = packWriteCommand(CMD_UNLOCK, 0x01);
    std::string write_frame = packCommand2Proto(write_command);

    //send the data
    return m_connector->Write(write_frame.c_str(), write_frame.length());
}

bool Pavo2Parser::parsePtpDeviation(char* data_ptr, std::size_t data_size)
{
    if(data_size != PTP_DEVATION_LEN)
        return false;

    if(UInt32Val2HostOrder(data_ptr) != PTP_DEVATION_LEADING_VALUE)
        return false;

    pavo_ptp_deviation_t ptp_dev;
    pavo_ptp_deviation_t* deviation_ptr = reinterpret_cast<pavo_ptp_deviation_t*>(data_ptr + PTP_DEVATION_LEADING_LEN);
    ptp_dev.last_sync_time = Int64Val2HostOrder(&deviation_ptr->last_sync_time);
    ptp_dev.time_offset = Int64Val2HostOrder(&deviation_ptr->time_offset);

    m_ptp_que.enqueue(ptp_dev);

    return true;
}

//------------------------------------Pavo2FrameParser----------------------
Pavo2FrameParser::Pavo2FrameParser(std::shared_ptr<Pavo2Connector> connector)
    : Pavo2Parser(connector)
{

}

Pavo2FrameParser::~Pavo2FrameParser()
{

}

//Function for pavo2_net
void Pavo2FrameParser::parseRecvData(std::unique_ptr<Pavo2RecvDataUnit>&& data_ptr)
{
    //buffer empty, or recv data > remain buffer size
    if(!m_parse_buff || (data_ptr->data_size > (PAVO2_PARSE_BUFFER_SIZE - m_parse_buff->data_size)))
    {
        m_parse_buff.swap(data_ptr);
    }
    else
    {
        memcpy(m_parse_buff->data_buff + m_parse_buff->data_size, data_ptr->data_buff, data_ptr->data_size); //copy to tail
        m_parse_buff->data_size += data_ptr->data_size;
    }

    while(m_parse_buff && (m_parse_buff->data_size > PAVO2_FRAME_LEAST_LEN))//one packet, one buffer
    {
        frame_protocol_header_t* protocol_ptr = (frame_protocol_header_t*)m_parse_buff->data_buff;

        uint32_t protol_head1_uint32 = UInt32Val2HostOrder(&protocol_ptr->start_tag_1);
        uint32_t protol_head2_uint32 = UInt32Val2HostOrder(&protocol_ptr->start_tag_2);

        //if beginning is not header, clear buffer and retart anothe parse cycle
        if((PAVO2_FRAME_HEAD1_UINT32 != protol_head1_uint32) || (PAVO2_FRAME_HEAD2_UINT32 != protol_head2_uint32))
        {
            m_parse_buff.reset();
            break;
        }
        uint32_t protocol_data_len = UInt32Val2HostOrder(&protocol_ptr->data_len);
        //whole pcl frame not received, the receive more data to parse
        if(protocol_data_len > (m_parse_buff->data_size - PAVO2_FRAME_LEAST_LEN))
            break;

        uint32_t protol_end_uint32 = ntohl(*((unsigned int*)(m_parse_buff->data_buff + sizeof(frame_protocol_header_t) + protocol_data_len)));
        //wrong pcl frame: protocol end missing
        if(PAVO2_FRAME_END_UINT32 != protol_end_uint32)
        {
            m_parse_buff.reset();
            break;
        }

        std::unique_ptr<Pavo2RecvDataUnit> trans_buffer;
        //if(size of data in buffer > a protocol frame), copy data of another frame to trans_buffer
        if(m_parse_buff->data_size > (protocol_data_len + PAVO2_FRAME_LEAST_LEN))
        {
            std::size_t remain_size = m_parse_buff->data_size - (protocol_data_len + PAVO2_FRAME_LEAST_LEN);

            trans_buffer.reset(new Pavo2RecvDataUnit);
            memcpy(trans_buffer->data_buff, m_parse_buff->data_buff + protocol_data_len + PAVO2_FRAME_LEAST_LEN, remain_size);
            trans_buffer->data_size = remain_size;
        }

        point_cloud_packet* packet_ptr = (point_cloud_packet*)(m_parse_buff->data_buff + sizeof(frame_protocol_header_t));

        uint32_t data_type_uint32 = UInt32Val2HostOrder(&packet_ptr->header.data_tag);
        uint32_t data_type_uint32_cmd = data_type_uint32 & 0xFFFFFF00;
        if(PAVO2_FRAME_DATA_TYPE_UINT32 == data_type_uint32)
        {
            parsePointCloud(m_parse_buff->data_buff + sizeof(frame_protocol_header_t), protocol_data_len);
        }
        else if(PAVO2_FRAME_PTP_DEVIATION_TYPE_UINT32 == data_type_uint32)//combine the 2
        {
            parsePtpDeviation(m_parse_buff->data_buff + sizeof(frame_protocol_header_t), protocol_data_len);
        }
        else if((PAVO2_FRAME_CMD_GET_TYPE_UINT32 == data_type_uint32_cmd) || (PAVO2_FRAME_CMD_SET_TYPE_UINT32 == data_type_uint32_cmd)) //command
        {
            auto cmd_ptr = std::shared_ptr<Pavo2CmdDataDef>(new Pavo2CmdDataDef(m_parse_buff->data_buff + sizeof(frame_protocol_header_t), protocol_data_len));
            std::unique_lock<std::mutex> guard(this->m_cmd_op_mutx);
            m_map_cmd[cmd_ptr->CmdIndex()] = std::move(cmd_ptr);
            this->m_cmd_op_cond.notify_one();
        }

        m_parse_buff.swap(trans_buffer);
    }
}

bool Pavo2FrameParser::parsePointCloud(char* data_ptr, std::size_t data_size)
{
    std::size_t totalSizeParsed = sizeof(point_cloud_packet_t);
    if(totalSizeParsed > data_size)
        return false;

    point_cloud_packet_t* packet_ptr = reinterpret_cast<point_cloud_packet_t*>(data_ptr);

    bool ptp_en = ((packet_ptr->header.ptp_en & 0x1) != 0) ? true : false;
    uint16_t start_angle = UInt16Val2HostOrder(&packet_ptr->header.start_angle);
    uint16_t end_angle = UInt16Val2HostOrder(&packet_ptr->header.end_angle);
    uint16_t angle_resolution = UInt16Val2HostOrder(&packet_ptr->header.angle_resolution);

    if(start_angle >= end_angle)
        return false;
    uint32_t point_number = (end_angle - start_angle) / angle_resolution;

    auto data_type = static_cast<PAVO2_FRAME_DATA_TYPE>(UInt16Val2HostOrder(&packet_ptr->header.data_type));

    pavo_point_cloud_scan_t scan;

    scan.packet_id = UInt32Val2HostOrder(&packet_ptr->header.packet_id);
    scan.ptp_en = ptp_en;
    scan.process_delay = UInt16Val2HostOrder(&packet_ptr->header.process_delay);
    scan.angle_resolution = angle_resolution;
    scan.start_angle = start_angle;
    scan.end_angle = end_angle;

    uint32_t timestamp_h, timestamp_l;
    if(ptp_en)
    {
        timestamp_h = ntohl(*(unsigned int*)(data_ptr + data_size - 12)); //high part: unit second
        timestamp_l = ntohl(*(unsigned int*)(data_ptr + data_size - 8)); //low part: nano second
        scan.time_stamp = ((uint64_t(timestamp_h) << 32) & 0xFFFFFFFF00000000)
                    | (uint64_t(timestamp_l) & 0xFFFFFFFF);
    }
    else
    {
        timestamp_h = 0;
        timestamp_l = ntohl(*(unsigned int*)(data_ptr + data_size - 8)); //unit: us
        scan.time_stamp = timestamp_l;
    }

    if(PAVO2_FRAME_DATA_TYPE::INTENSITY_DISTANCE == data_type)
    {
        totalSizeParsed += sizeof(point_cloud_pointA_t) * point_number;
        if(totalSizeParsed > data_size)
            return false;

        point_cloud_pointA_t* point_ptr = (point_cloud_pointA_t*)(&packet_ptr->data_payload);
        scan.data.resize(point_number);
        for(uint32_t cnt = 0; cnt < point_number; ++cnt)
        {
            pavo_response_scan_t point;
            point.angle = start_angle + cnt * angle_resolution;
            point.intensity = UInt16Val2HostOrder(&point_ptr[cnt].intensity);
            point.distance = UInt16Val2HostOrder(&point_ptr[cnt].distance);

            scan.data[cnt] = point;
        }
    }
    else if(PAVO2_FRAME_DATA_TYPE::CHECK_MODE == data_type)
    {
        totalSizeParsed += sizeof(point_cloud_pointA_t) * point_number * 2;
        if(totalSizeParsed > data_size)
            return false;

        point_cloud_pointA_t* point_ptr = (point_cloud_pointA_t*)(&packet_ptr->data_payload);
        scan.data.resize(point_number * 2);
        for(uint32_t cnt = 0; cnt < point_number; ++cnt)
        {
            pavo_response_scan_t point1, point2;

            //first point
            point1.angle = start_angle + cnt * angle_resolution;
            point1.intensity = UInt16Val2HostOrder(&point_ptr[2 * cnt].intensity);
            point1.distance = UInt16Val2HostOrder(&point_ptr[2 * cnt].distance);
            scan.data[2 * cnt] = point1;

            //second point
            point2.angle = start_angle + cnt * angle_resolution;
            point2.intensity = UInt16Val2HostOrder(&point_ptr[2 * cnt + 1].intensity);
            point2.distance = UInt16Val2HostOrder(&point_ptr[2 * cnt + 1].distance);
            scan.data[2 * cnt + 1] = point2;

        }
    }
    else if(PAVO2_FRAME_DATA_TYPE::DISTANCE == data_type)
    {
        totalSizeParsed += sizeof(point_cloud_pointB_t) * point_number;
        if(totalSizeParsed > data_size)
            return false;

        point_cloud_pointB_t* point_ptr = (point_cloud_pointB_t*)(&packet_ptr->data_payload);
        scan.data.resize(point_number);
        for(uint32_t cnt = 0; cnt < point_number; ++cnt)
        {
            pavo_response_scan_t point;
            point.angle = start_angle + cnt * angle_resolution;
            point.intensity = 0;
            point.distance = UInt16Val2HostOrder(&point_ptr[cnt].distance);
            scan.data[cnt] = point;
        }
    }

    if(hr_level == 1)
        algorithm.FarHighRelectivityPreProcess(&(scan.data[0]), (int)scan.data.size());
    else if(hr_level == 2)
        algorithm.FarHighRelectivityFilter(&(scan.data[0]), (int)scan.data.size());

    if(m_pMADataFilter)
    {
        m_pMADataFilter->Filter(&(scan.data[0]), (int)scan.data.size());
    }

    m_scans_que.enqueue(scan);

    return true;
}


std::string Pavo2FrameParser::packCommand2Proto(const std::string& total_command)
{
    char send_vec[PAVO2_RECV_SEND_BUFFER_SIZE];

    memset(send_vec, 0, PAVO2_RECV_SEND_BUFFER_SIZE);
    //add the protocol fixed head
    uint16_t send_len = 0;
    memcpy(send_vec + send_len, PAVO2_FRAME_HEAD, PAVO2_FRAME_HEAD_LEN);
    send_len = send_len + PAVO2_FRAME_HEAD_LEN;
    //add the version and reserver
    memcpy(send_vec + send_len, PAVO2_FRAME_VERSION_RESERVER, PAVO2_FRAME_VERSION_RESERVER_LEN);
    send_len = send_len + PAVO2_FRAME_VERSION_RESERVER_LEN;

    uint16_t total_cmd_len = total_command.size();
    //add the len
    send_vec[send_len++] = 0x00;
    send_vec[send_len++] = 0x00;
    send_vec[send_len++] = (0xFF & (total_cmd_len >> 8));
    send_vec[send_len++] = (0xFF & (total_cmd_len));
    //add the content
    memcpy(send_vec + send_len, total_command.c_str(), total_cmd_len);
    send_len = send_len + total_cmd_len;
    //add the protocol fixed tail
    memcpy(send_vec + send_len, PAVO2_FRAME_END, PAVO2_FRAME_END_LEN);
    send_len = send_len + PAVO2_FRAME_END_LEN;

    return std::string(send_vec, send_len);
}


//---------------For Packet Parser---------------------
Pavo2MsgParser::Pavo2MsgParser(std::shared_ptr<Pavo2Connector> connector)
    : Pavo2Parser(connector)
{
    resetReceiveState();
}

Pavo2MsgParser::~Pavo2MsgParser()
{

}

//----------------------Function for pavo2_net
void Pavo2MsgParser::parseRecvData(std::unique_ptr<Pavo2RecvDataUnit>&& data_ptr)
{
    if(!data_ptr)
    {
        return;
    }

    msg_protocol_header_t* msg_protocol_ptr = (msg_protocol_header_t*)data_ptr->data_buff;
    msg_protocol_ptr->ToHostOrder();
    if((msg_protocol_ptr->start_sign_1 != PAVO2_MSG_START_SIGN1)
            || (msg_protocol_ptr->start_sign_2 != PAVO2_MSG_START_SIGN2))
    {
        return;
    }

    auto packet_type = static_cast<PAVO2_MSG_TYPE>(msg_protocol_ptr->packet_type);
    if(PAVO2_MSG_TYPE::POINT_CLOUD == packet_type) // point cloud
    {
        msg_segment_header_t* pPacketHead = (msg_segment_header_t*)(data_ptr->data_buff + sizeof(msg_protocol_header_t));
        pPacketHead->ToHostOrder();
/*
 * may receive data
 * 1. packet lost (lost first, middle, or last)
 * 2. receive packet from the middle of the scan
 * 3. receive packet from the first packet of the scan(packet 0)
 */
        if(pPacketHead->packet_serial == 0) //new scan
        {
            //push previous scan. filter the initial state
            if((m_points_expected_now > 0)
                    && (m_current_scan.data.size() == m_points_expected_now))
            {
                m_current_scan.time_stamp = m_curr_time_stamp;
                m_scans_que.enqueue(std::move(m_current_scan));
            }
            resetReceiveState();

            m_scan_serial_now = pPacketHead->scan_serial;
            m_points_expected_now = pPacketHead->scan_points_number;
            m_current_scan.packet_id = pPacketHead->scan_serial; //Qiang, FIXME packet_id
            m_current_scan.ptp_en = false;
            m_current_scan.process_delay = 0;
            m_current_scan.angle_resolution = pPacketHead->angular_increment;
            m_current_scan.start_angle = pPacketHead->first_angle;
            m_current_scan.end_angle = m_current_scan.start_angle + pPacketHead->scan_points_number * m_current_scan.angle_resolution;
            m_current_scan.time_stamp = pPacketHead->time_stamp;
            m_current_scan.data.resize(pPacketHead->scan_points_number);
        }
        else if(m_scan_serial_now != pPacketHead->scan_serial)//continued scan: filter receive halfway
        {
            resetReceiveState();
            return;
        }

        m_curr_time_stamp = pPacketHead->time_stamp; //update time stamp for

        m_recved_points_num += pPacketHead->packet_points_number;
        auto pcl_ptr = data_ptr->data_buff + sizeof(msg_protocol_header_t) + sizeof(msg_segment_header_t);
        auto first_angle = pPacketHead->first_angle;
        auto agular_increment = pPacketHead->angular_increment;

        for(uint16_t i = 0; i < pPacketHead->packet_points_number; i++)
        {
            std::size_t point_idx = i + pPacketHead->first_index;
            if(point_idx >= m_current_scan.data.size())
                return;
            pavo_response_scan_t& point = m_current_scan.data[point_idx];
            point.angle = first_angle + agular_increment * i;
            point.intensity = UInt16Val2HostOrder(pcl_ptr);
            point.distance = UInt16Val2HostOrder(pcl_ptr + 2);
            pcl_ptr += 4;
        }
    }
    else if(PAVO2_MSG_TYPE::COMMAND == packet_type) //command
    {
        std::unique_lock<std::mutex> guard(this->m_cmd_op_mutx);

        auto cmd_ptr = std::shared_ptr<Pavo2CmdDataDef>(new Pavo2CmdDataDef(data_ptr->data_buff + 16, data_ptr->data_size - 20));
        m_map_cmd[cmd_ptr->CmdIndex()] = cmd_ptr;

        this->m_cmd_op_cond.notify_one();
    }
    else if(PAVO2_MSG_TYPE::PTP == packet_type) //no ptp for udp
    {

    }
}

msg_protocol_header_t Pavo2MsgParser::buildMsgHeader(PAVO2_MSG_TYPE type)
{
    msg_protocol_header_t header;
    header.start_sign_1 = PAVO2_MSG_START_SIGN1;
    header.start_sign_2 = PAVO2_MSG_START_SIGN2;
    header.version = PAVO2_MSG_VERSION;
    header.packet_type = static_cast<uint8_t>(type);
    header.reserved_h0 = 0;
    header.packet_size = 0; // need to refill
    header.header_size = sizeof(msg_protocol_header_t);
    return header;
}

void Pavo2MsgParser::resetReceiveState()
{
    m_points_expected_now = 0;
    m_recved_points_num = 0;

    m_current_scan.time_stamp = 0;
    m_current_scan.data.clear();
}

std::string Pavo2MsgParser::packCommand2Proto(const std::string& total_command)
{
    char send_vec[PAVO2_RECV_SEND_BUFFER_SIZE];

    memset(send_vec, 0, PAVO2_RECV_SEND_BUFFER_SIZE);

    std::size_t send_len = 0;
    msg_protocol_header_t header = buildMsgHeader(PAVO2_MSG_TYPE::COMMAND); //0x11: command
    header.packet_size = sizeof(msg_protocol_header_t) + total_command.size() + 4; //4:reserved and checksum
    header.ToNetworkOrder();
    memcpy(send_vec + send_len, &header, sizeof(msg_protocol_header_t));
    send_len += sizeof(msg_protocol_header_t);

    memcpy(send_vec + send_len, total_command.c_str(), total_command.size());
    send_len += total_command.size();

    send_len += 4; //reserved and checksum
    return std::string(send_vec, send_len);
}

} //namespace pavo2
