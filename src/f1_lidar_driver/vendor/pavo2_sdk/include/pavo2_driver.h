#ifndef __PAVO2_DRIVER_H__
#define __PAVO2_DRIVER_H__
#include <memory>

#include "pavo2_types.h"
#include "pavo2_net.h"
#include "pavo2_parser.h"

#ifdef _WIN32
#include <winsock2.h>
#include <WS2tcpip.h>
#include <iostream>
#pragma comment(lib,"ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdlib>
#include <string>
#include <cstring>
#endif //_WIN32

#define SDK_VER "2.1.2"


namespace pavo2
{
    class pavo2_driver
    {
    public:
        pavo2_driver();
        ~pavo2_driver();
        void set_local_address(const std::string& local_ip, uint16_t local_port);
        void set_conn_protocol(PAVO2_PROTO_TYPE proto_type);

        bool pavo2_open(const std::string& lidar_ip, uint16_t lidar_port, uint32_t timeout = 0);

        bool pavo2_is_open();

        bool pavo2_close();
        std::size_t get_scans_available() const;
        std::size_t get_ptp_deviations_available() const;

        bool get_scanned_data(pavo_response_scan_t* data_buffer, uint16_t& count, uint16_t timeout = 0);
        bool get_scanned_data(std::vector<pavo_response_scan_t>& vec_buff, uint16_t timeout = 0);

        bool get_scanned_data(pavo_response_pcd_t* data_buffer, uint16_t& count, uint16_t timeout = 0);
        bool get_scanned_data(std::vector<pavo_response_pcd_t>& vec_buff, uint16_t timeout = 0);

        bool get_scanned_data(pavo_response_scan_t* data_buffer, uint16_t& count, uint32_t& timestamp, uint16_t timeout = 0);
        bool get_scanned_data(std::vector<pavo_response_scan_t>& vec_buff, uint32_t& timestamp, uint16_t timeout = 0);

        bool get_scanned_data(pavo_response_pcd_t* data_buffer, uint16_t& count, uint32_t& timestamp, uint16_t timeout = 0);
        bool get_scanned_data(std::vector<pavo_response_pcd_t>& vec_buff, uint32_t& timestamp, uint16_t timeout = 0);

        bool get_scanned_data(pavo_packet_info_scan_t& packet_info_scan, uint16_t timeout = 0);
        bool get_scanned_data(pavo_packet_info_pcd_t& packet_info_pcd, uint16_t timeout = 0);

        bool recv_point_cloud(pavo_point_cloud_scan_t& scan, uint16_t timeout = 0);

        bool recv_ptp_deviation_data(pavo_ptp_deviation_t& ptp_deviation, uint16_t timeout = 0);

        bool get_device_sn(std::string& sn);
        bool get_device_pn(std::string& pn);
        bool get_sdk_ver(std::string& ver);
        bool get_os_ver(std::string& ver);
        bool get_firmware_ver(std::string& ver);
        bool get_os_error(uint32_t& errCode, uint8_t iIndex = 0);
        int  get_error_code();
        bool enable_data(bool en);
        bool set_intensity_mode(bool en);
        bool get_intensity_mode(bool& en);
        bool set_angle_range(uint16_t start, uint16_t end);
        bool get_angle_range(uint16_t& start, uint16_t& end);
        bool get_max_angle_range(uint32_t& val);
        bool set_motor_speed(uint8_t val);
        bool get_motor_speed(uint8_t& val);
        bool get_motor_speed_enable(uint32_t& val);
        bool set_angle_resolution(uint16_t val);
        bool get_angle_resolution(uint16_t& val);
        bool get_angle_resolution_enable(uint32_t& val);

        bool set_device_ip(const std::string& ip);
        bool get_device_ip(std::string& ip);
        bool set_tcp_port(uint16_t val);
        bool get_tcp_port(uint16_t& val);
        bool set_udp_port(uint16_t val);
        bool get_udp_port(uint16_t& val);
        bool set_dest_ip(const std::string& ip);
        bool get_dest_ip(std::string& ip);
        bool set_dest_port(uint16_t val);
        bool get_dest_port(uint16_t& val);
        bool set_device_mask(const std::string& mask);
        bool get_device_mask(std::string& mask);
        bool set_device_gateway(const std::string& gateway);
        bool get_device_gateway(std::string& gateway);
        bool get_device_mac(std::string& mac);

        bool set_curr_proto(PAVO2_PROTO_TYPE proto_type);
        bool get_curr_proto(PAVO2_PROTO_TYPE& proto_type);
        bool get_supported_protos(std::vector<PAVO2_PROTO_TYPE>& protos);

        bool set_os_restart();
        bool set_echo_mode(bool en);
        bool get_echo_mode(bool& en);
        bool set_outdoor_mode(bool en);
        bool get_outdoor_mode(bool& en);

        bool set_calibrate_horizontal_angle(uint32_t angle);
        bool get_calibrate_horizontal_angle(uint32_t& angle);
        bool get_max_angle_distance(uint32_t& val);
        bool enable_tail_filter(uint16_t method);
        bool enable_front_filter(uint16_t iLevel);
        bool enable_far_hr_filter(uint16_t hrLevel);

        bool set_warn_angle_range(uint16_t start, uint16_t end);
        bool get_warn_angle_range(uint16_t& start, uint16_t& end);
        bool set_warn_intensity_distance(uint16_t distance, uint16_t intensity);
        bool get_warn_intensity_distance(uint16_t& distance, uint16_t& intensity);
        bool set_warn_point_number(uint16_t number);
        bool get_warn_point_number(uint16_t& number);

        bool enable_ptp(bool en);
        bool is_ptp_enabled(bool& en);
        bool is_ptp_supported(bool& en);
        bool enable_near_hr(bool en);
        bool is_near_hr_enabled(bool& en);
        bool is_near_hr_supported(bool& en);

        bool get_temperature(float& temperature);
        bool get_heartbeat();

        bool get_cmd_raw(PAVO2_CMD_INDEX cmd_type, uint32_t& val);

    protected:
        std::shared_ptr<Pavo2Connector> m_connector;
        std::shared_ptr<Pavo2Parser> m_parser;
        PAVO2_PROTO_TYPE m_protocol;
        std::string m_local_ip;
        uint16_t m_local_port;
    };
} //namespace  pavo2 {

#endif //__PAVO2_DRIVER_H__
