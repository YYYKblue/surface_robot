#ifndef __UTILS_H__
#define __UTILS_H__

#include <queue>
#include <algorithm>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <utility>
#include <sstream>
#include <array>
#include <cstring>
#include <cmath>

const double PI = std::acos(-1);

#define Degree2Radians(x) ((x) * PI / 180.0)

//16 bit
inline uint16_t UInt16Val2HostOrder(const void* ptr)
{
    auto uint8_ptr = reinterpret_cast<const uint8_t*>(ptr);
    return (((uint16_t(*uint8_ptr) << 8) & 0xFF00)
    | ((uint16_t(*(uint8_ptr + 1))) & 0x00FF));
}

inline int16_t Int16Val2HostOrder(const void* ptr)
{
    return int16_t(UInt16Val2HostOrder(ptr));
}

inline void UInt16Val2BigOrder(uint16_t val, void* ptr)
{
    auto uint8_ptr = reinterpret_cast<uint8_t*>(ptr);
    uint8_ptr[0] = char((val & 0xFF00) >> 8);
    uint8_ptr[1] = char(val & 0x00FF);
}

inline void Int16Val2BigOrder(int16_t val, void* ptr)
{
    UInt16Val2BigOrder(uint16_t(val), ptr);
}

// 32 bit
inline uint32_t UInt32Val2HostOrder(const void* ptr)
{
    auto uint8_ptr = reinterpret_cast<const uint8_t*>(ptr);
    return (((uint32_t(*uint8_ptr) << 24)   & 0xFF000000)
    | ((uint32_t(*(uint8_ptr + 1)) << 16) & 0x00FF0000)
    | ((uint32_t(*(uint8_ptr + 2)) << 8) & 0x0000FF00)
    | (uint32_t(*(uint8_ptr + 3)) & 0x000000FF));
}

inline int32_t Int32Val2HostOrder(const void* ptr)
{
    return int32_t(UInt32Val2HostOrder(ptr));
}

inline void UInt32Val2BigOrder(uint32_t val, void* ptr)
{
    auto uint8_ptr = reinterpret_cast<uint8_t*>(ptr);
    uint8_ptr[0] = uint8_t((val & 0xFF000000) >> 24);
    uint8_ptr[1] = uint8_t((val & 0x00FF0000) >> 16);
    uint8_ptr[2] = uint8_t((val & 0x0000FF00) >> 8);
    uint8_ptr[3] = uint8_t(val & 0x000000FF);
}

inline void Int32Val2BigOrder(int32_t val, void* ptr)
{
    UInt32Val2BigOrder(uint16_t(val), ptr);
}

// 64 bit
inline uint64_t UInt64Val2HostOrder(const void* ptr)
{
    auto uint8_ptr = reinterpret_cast<const uint8_t*>(ptr);
    return (((uint64_t(*uint8_ptr) << 56) & 0xFF00000000000000)
    | ((uint64_t(*(uint8_ptr + 1)) << 48) & 0x00FF000000000000)
    | ((uint64_t(*(uint8_ptr + 2)) << 40) & 0x0000FF0000000000)
    | ((uint64_t(*(uint8_ptr + 3)) << 32) & 0x000000FF00000000)
    | ((uint64_t(*(uint8_ptr + 4)) << 24) & 0x00000000FF000000)
    | ((uint64_t(*(uint8_ptr + 5)) << 16) & 0x0000000000FF0000)
    | ((uint64_t(*(uint8_ptr + 6)) << 8) & 0x000000000000FF00)
    | ((uint64_t(*(uint8_ptr + 7))) & 0x00000000000000FF));
}

inline int64_t Int64Val2HostOrder(const void* ptr)
{
    return int64_t(UInt64Val2HostOrder(ptr));
}

inline void UInt64Val2BigOrder(uint64_t val, void* ptr)
{
    auto uint8_ptr = reinterpret_cast<uint8_t*>(ptr);
    uint8_ptr[0] = uint8_t((val & 0xFF00000000000000) >> 56);
    uint8_ptr[1] = uint8_t((val & 0x00FF000000000000) >> 48);
    uint8_ptr[2] = uint8_t((val & 0x0000FF0000000000) >> 40);
    uint8_ptr[3] = uint8_t((val & 0x000000FF00000000) >> 32);
    uint8_ptr[4] = uint8_t((val & 0x00000000FF000000) >> 24);
    uint8_ptr[5] = uint8_t((val & 0x0000000000FF0000) >> 16);
    uint8_ptr[6] = uint8_t((val & 0x000000000000FF00) >> 8);
    uint8_ptr[7] = uint8_t(val & 0x00000000000000FF);
}

inline void Int64Val2BigOrder(int64_t val, void* ptr)
{
    UInt64Val2BigOrder(uint64_t(val), ptr);
}

template<typename int_type>
inline std::string Int2HexString(int_type val)
{
    std::ostringstream ss;
    ss << std::hex << val;
    return ss.str();
}

inline std::string IpUInt32ToString(uint32_t ip_int)
{
    uint32_t byte_hh = (ip_int & 0xFF000000) >> 24;
    uint32_t byte_hl = (ip_int & 0x00FF0000) >> 16;
    uint32_t byte_lh = (ip_int & 0x0000FF00) >> 8;
    uint32_t byte_ll = ip_int & 0x000000FF;
    std::string ip_addr = std::to_string(byte_hh) + "." + std::to_string(byte_hl) + "."
            + std::to_string(byte_lh) + "." + std::to_string(byte_ll);
    return ip_addr;
}

inline uint32_t IpStringToUInt32(const std::string& ip_str)//FIXME: parse error
{
    std::array<uint8_t, 4> ipAddress;
    std::istringstream iss(ip_str);
    std::string token;
    int i = 0;
    while(std::getline(iss, token, '.'))
    {
        if(i >= 4)
        {
            return 0;
        }
        try{
            ipAddress[i] = static_cast<uint8_t>(std::stoi(token, nullptr, 10));
        }catch(const std::invalid_argument& e){
            (void)e;
            return 0;
        }catch(const std::out_of_range& e){
            (void)e;
            return 0;
        }
        ++i;
    }
    if(i < 4)
    {
        return 0;
    }
    uint32_t ip_uint32 (((uint32_t(ipAddress[0]) << 24) & 0xFF000000)
    | ((uint32_t(ipAddress[1]) << 16) & 0x00FF0000)
    | ((uint32_t(ipAddress[2]) << 8) & 0x0000FF00)
    | (uint32_t(ipAddress[3]) & 0x000000FF));

    return ip_uint32;
}

template<typename T>
class SynchronizedQueue
{
public:
    SynchronizedQueue(std::size_t max_size = 0)
        : m_max_size(max_size)
        , m_queue()
        , m_mutex()
        , m_cond()
        , m_request_to_end(false)
        , m_enqueue_data(true)
    {
    }

    void enqueue(const T& data)
    {
        std::unique_lock<std::mutex> lock(m_mutex);

        if(m_enqueue_data)
        {
            if((m_max_size != 0) && (m_queue.size() >= m_max_size)) //exceed size limit
            {
                m_queue.pop();
            }
            m_queue.push(data);
            m_cond.notify_one();
        }
    }

    void enqueue(T&& data)
    {
        std::unique_lock<std::mutex> lock(m_mutex);

        if(m_enqueue_data)
        {
            if((m_max_size != 0) && (m_queue.size() >= m_max_size)) //exceed size limit
            {
                m_queue.pop();
            }
            m_queue.push(std::forward<T>(data));
            m_cond.notify_one();
        }
    }

    template <typename... _Args>
    void emplace(_Args&&... __args)
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        if(m_enqueue_data)
        {
            m_queue.emplace(std::forward<_Args>(__args)...);
            m_cond.notify_one();
        }
    }

    bool dequeue(T& result, uint16_t time_out = 0)
    {
        std::unique_lock<std::mutex> lock(m_mutex);

        if(time_out == 0)
        {
            while(m_queue.empty())
            {
                m_cond.wait(lock);
            }
        }
        else
        {
            while(m_queue.empty())
            {
                if(m_cond.wait_for(lock, std::chrono::milliseconds(time_out)) == std::cv_status::timeout)
                    return false;
            }
        }

        if(m_request_to_end)
        {
            doEndActions();
            return false;
        }
        result = std::move(m_queue.front());
        m_queue.pop();
        return true;
    }

    bool tryDequeue(T& result)
    {
        std::unique_lock<std::mutex> lock(m_mutex);

        if(m_queue.empty())
            return false;

        if(m_request_to_end)
        {
            doEndActions();
            return false;
        }

        result = std::move(m_queue.front());
        m_queue.pop();

        return true;
    }

    void stopQueue()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_request_to_end = true;
        m_cond.notify_one();
    }

    std::size_t size() const
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        return m_queue.size();
    }

    bool isEmpty() const
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        return (m_queue.empty());
    }

    void clear()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        while(!m_queue.empty())
        {
            m_queue.pop();
        }
    }

private:
    void doEndActions()
    {
        m_enqueue_data = false;

        while(!m_queue.empty())
        {
            m_queue.pop();
        }
    }

    std::size_t m_max_size;
    std::queue<T> m_queue; // Use STL queue to store data
    mutable std::mutex m_mutex; // The mutex to synchronise on
    std::condition_variable m_cond; // The condition to wait for
    bool m_request_to_end;
    bool m_enqueue_data;
};

class NetCoverter
{
public:
    static bool String2Bytes(const std::string& ip_str, char* ip_bytes)
    {
        if(ip_str.empty())  //empty string
        {
            return false;
        }

        if(count(ip_str.begin(), ip_str.end(), '.') != 3)
        {
            return false;
        }

        std::string ip_str_copy(ip_str);
        TrimSpace(ip_str_copy); //remove leading and tail space

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
                {
                    return false;
                }

                str_tmp = ip_str_copy.substr(prev_pos, i - prev_pos);
                prev_pos = i + 1;

                byte_single = atoi(str_tmp.c_str());  //invalid value
                if(byte_single > 255)
                    return false;
                tmp_bytes[dot_num-1] = static_cast<char>(byte_single);

                continue;
            }
            if((ip_str_copy[i]<'0') || (ip_str_copy[i] > '9')) //invalid character
            {
                return false;
            }
        }

        str_tmp = ip_str_copy.substr(prev_pos);
        byte_single = atoi(str_tmp.c_str());
        tmp_bytes[dot_num] = static_cast<char>(byte_single);

        memcpy(ip_bytes, tmp_bytes, 4);

        return true;
    }

    static bool Bytes2String(std::string& ip_str, const char* ip_bytes)
    {
        if(ip_bytes == 0)
        {
            return false;
        }

        ip_str = std::to_string(static_cast<unsigned>(ip_bytes[0])) + "."
                 + std::to_string(static_cast<unsigned>(ip_bytes[1])) + "."
                 + std::to_string(static_cast<unsigned>(ip_bytes[2])) + "."
                 + std::to_string(static_cast<unsigned>(ip_bytes[3]));

        return true;
    }

    static bool Port2Bytes(const uint16_t& port_int, char* port_bytes)  //from uint16_t to char[4]
    {
        if(port_bytes == nullptr)
        {
            return false;
        }

        memset(port_bytes, 0, 4);
        port_bytes[2] = (port_int & 0xFF00) >> 8;
        port_bytes[3] = (port_int & 0x00FF);

        return true;
    }

    static bool Bytes2Port(uint16_t& port_int, const char* port_bytes)
    {
        if(port_bytes == nullptr)
        {
            return false;
        }

        port_int = (static_cast<unsigned char>(port_bytes[2]) << 8)
                   + static_cast<unsigned char>(port_bytes[3]);

        return true;
    }

    static std::string& TrimSpace(std::string& str)
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
}; //class NetCoverter

#endif //__UTILS_H__
