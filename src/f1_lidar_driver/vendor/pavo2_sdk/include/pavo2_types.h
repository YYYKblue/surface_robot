#ifndef __PAVO2_TYPES_H__
#define __PAVO2_TYPES_H__

#include <cstdint>
#include <vector>
#include "utils.h"

#if defined(_WIN32)
#pragma pack(1)
#define __DATA_ALIGN__
#else
#define __DATA_ALIGN__ __attribute__((packed))
#endif


namespace pavo2 {

//format for output
typedef struct pavo_response_scan
{
    uint16_t angle;
    uint16_t distance;
    uint16_t intensity;
} pavo_response_scan_t;

typedef struct pavo_point_cloud_scan
{
    uint32_t packet_id;
    bool ptp_en;
    uint16_t process_delay;
    uint16_t angle_resolution;
    uint16_t start_angle;
    uint16_t end_angle;
    std::vector<pavo_response_scan_t> data;
    uint64_t time_stamp;
} pavo_point_cloud_scan_t;

typedef struct pavo_response_pcd
{
    double x;
    double y;
    double z;  //fixed as 0 for single-laser device
    uint16_t  intensity;
} pavo_response_pcd_t;

typedef struct pavo_packet_info_pcd
{
    std::vector<pavo_response_pcd_t> vec_buff;
    uint32_t time_stamp;
    uint32_t time_delay;
} pavo_packet_info_pcd_t;

typedef struct pavo_packet_info_scan
{
    std::vector<pavo_response_scan_t> vec_buff;
    uint32_t time_stamp;
    uint32_t time_delay;
} pavo_packet_info_scan_t;

typedef struct pavo_ptp_deviation
{
    int64_t last_sync_time;
    int64_t time_offset;
} pavo_ptp_deviation_t;

//------------format for parsing
typedef struct frame_protocol_header{
    uint32_t start_tag_1;
    uint32_t start_tag_2;
    uint8_t  version;
    uint8_t  reserved[3];
    uint32_t data_len;
} __DATA_ALIGN__ frame_protocol_header_t;

typedef struct point_cloud_header{
    uint32_t data_tag;
    uint32_t packet_id;
    uint8_t  reserver;
    uint8_t  ptp_en;
    uint16_t process_delay;
    uint16_t data_type; //check, distance_intensity, distance
    uint16_t angle_resolution;
    uint16_t start_angle;
    uint16_t end_angle;
} __DATA_ALIGN__ point_cloud_header_t;

typedef struct msg_protocol_header{
    uint32_t start_sign_1;
    uint32_t start_sign_2;
    uint8_t  version;
    uint8_t  packet_type;
    uint16_t reserved_h0;
    uint16_t packet_size;
    uint16_t header_size;

    void ToNetworkOrder()
    {
        UInt32Val2BigOrder(start_sign_1, &start_sign_1);
        UInt32Val2BigOrder(start_sign_2, &start_sign_2);
        UInt16Val2BigOrder(reserved_h0, &reserved_h0);
        UInt16Val2BigOrder(packet_size, &packet_size);
        UInt16Val2BigOrder(header_size, &header_size);
    }

    void ToHostOrder()
    {
        start_sign_1 = UInt32Val2HostOrder(&start_sign_1);
        start_sign_2 = UInt32Val2HostOrder(&start_sign_2);
        reserved_h0 = UInt16Val2HostOrder(&reserved_h0);
        packet_size = UInt16Val2HostOrder(&packet_size);
        header_size = UInt16Val2HostOrder(&header_size);
    }
} __DATA_ALIGN__ msg_protocol_header_t;

typedef struct msg_segment_header
{
    uint16_t scan_serial;
    uint16_t packet_serial;
    uint16_t totoal_packet_number;
    uint8_t  reserved_d0;
    uint8_t  data_type;
    uint32_t time_stamp;
    uint32_t status_flag;
    uint32_t scan_frequency;
    uint16_t scan_points_number;
    uint16_t packet_points_number;
    uint16_t first_index;
    uint16_t reserved_d1;
    uint16_t first_angle;
    uint16_t angular_increment;

    void ToNetworkOrder()
    {
        UInt16Val2BigOrder(scan_serial, &scan_serial);
        UInt16Val2BigOrder(packet_serial, &packet_serial);
        UInt16Val2BigOrder(totoal_packet_number, &totoal_packet_number);
        //reserved_d0
        //data_type
        UInt32Val2BigOrder(time_stamp, &time_stamp);
        UInt32Val2BigOrder(status_flag, &status_flag);
        UInt32Val2BigOrder(scan_frequency, &scan_frequency);
        UInt16Val2BigOrder(scan_points_number, &scan_points_number);
        UInt16Val2BigOrder(packet_points_number, &packet_points_number);
        UInt16Val2BigOrder(first_index, &first_index);
        UInt16Val2BigOrder(reserved_d1, &reserved_d1);
        UInt16Val2BigOrder(first_angle, &first_angle);
        UInt16Val2BigOrder(angular_increment, &angular_increment);
    }

    void ToHostOrder()
    {
        scan_serial = UInt16Val2HostOrder(&scan_serial);
        packet_serial = UInt16Val2HostOrder(&packet_serial);
        totoal_packet_number = UInt16Val2HostOrder(&totoal_packet_number);
        //reserved_d0
        //data_type
        time_stamp = UInt32Val2HostOrder(&time_stamp);
        status_flag = UInt32Val2HostOrder(&status_flag);
        scan_frequency = UInt32Val2HostOrder(&scan_frequency);
        scan_points_number = UInt16Val2HostOrder(&scan_points_number);
        packet_points_number = UInt16Val2HostOrder(&packet_points_number);
        first_index = UInt16Val2HostOrder(&first_index);
        reserved_d1 = UInt16Val2HostOrder(&reserved_d1);
        first_angle = UInt16Val2HostOrder(&first_angle);
        angular_increment = UInt16Val2HostOrder(&angular_increment);
    }
} __DATA_ALIGN__ msg_segment_header_t;

//intensity, distance
typedef struct point_cloud_pointA{
    uint16_t intensity;
    uint16_t distance;
} __DATA_ALIGN__ point_cloud_pointA_t;

typedef struct point_cloud_pointB{
    uint16_t distance;
} __DATA_ALIGN__ point_cloud_pointB_t;

typedef struct point_cloud_packet{
    point_cloud_header_t header;
    uint32_t data_payload; //just a place holder
} __DATA_ALIGN__ point_cloud_packet_t;


//when the distance is 0, set the output value is 0 or 50000(100m) that is out of the valid range
#define PAVO2_DISTANCE_ZERO 0

#if PAVO2_DISTANCE_ZERO
const int PAVO2_DISTANCE_MAX = 50000; //100m
#else
const int PAVO2_DISTANCE_MAX = 0; //0m
#endif

#if defined(_WIN32)
#pragma pack()
#endif

}//namespace pavo2

#endif //__PAVO2_TYPES_H__
