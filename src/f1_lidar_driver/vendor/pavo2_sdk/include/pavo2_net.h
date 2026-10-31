#ifndef __PAVO2_NET_H__
#define __PAVO2_NET_H__

#include <string>
#include <functional>
#include <thread>
#include <mutex>
#include <memory>

#ifdef _WIN32
#include <winsock2.h>
#include <WS2tcpip.h>
#include <iostream>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <cstdlib>
#include <string>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#endif

namespace pavo2
{

#define PAVO2_RECV_SEND_BUFFER_SIZE (0x8000)   //8K
//16k
#define PAVO2_PARSE_BUFFER_SIZE (PAVO2_RECV_SEND_BUFFER_SIZE * 2)

struct Pavo2RecvDataUnit
{
    Pavo2RecvDataUnit();
    Pavo2RecvDataUnit(char* data, std::size_t size);
    std::size_t data_size;
    char data_buff[PAVO2_PARSE_BUFFER_SIZE];
};

typedef std::function<void(std::unique_ptr<Pavo2RecvDataUnit>)> ReadCallback_t;


class Pavo2Connector
{
public:
    virtual ~Pavo2Connector()
    {

    }

    virtual void SetLocalAddress(const std::string& local_ip, uint16_t local_port) = 0;

    virtual bool Connect(const std::string& lidar_ip, uint16_t lidar_port, uint32_t timeout) = 0;
    virtual bool IsConnected() = 0;
    virtual void Disconnect() = 0;

    virtual bool Write(const char* content, std::size_t length) = 0;
    void SetReadCallback(ReadCallback_t read_callback);

protected:
    ReadCallback_t f_ReadCallback;
};

class Pavo2TcpConnector : public Pavo2Connector
{
public:
    Pavo2TcpConnector();
    virtual ~Pavo2TcpConnector();

    void SetLocalAddress(const std::string& local_ip, uint16_t local_port) override;

    bool Connect(const std::string& lidar_ip, uint16_t lidar_port, uint32_t timeout) override;
    bool IsConnected() override;
    void Disconnect() override;

    bool Write(const char* content, std::size_t length) override;

private:
    bool ConnecTimeout(const std::string& lidar_ip, uint16_t lidar_port, uint32_t timeout);

    static void ReadThread(Pavo2TcpConnector* conn_ptr);

private:
    bool m_should_recv;
    bool m_is_recving;

#ifdef _WIN32
    SOCKET m_iSocket;
#else
    int m_iSocket;
#endif

    std::mutex m_socket_mutx;

    std::string m_local_ip;
    uint16_t m_local_port;

    std::string m_lidar_ip;
    uint16_t m_lidar_port;

    std::thread m_recv_thread;
};


class Pavo2UdpConnector : public Pavo2Connector
{
public:
    Pavo2UdpConnector();
    virtual ~Pavo2UdpConnector();

    void SetLocalAddress(const std::string& local_ip, uint16_t local_port) override;

    bool Connect(const std::string& lidar_ip, uint16_t lidar_port, uint32_t timeout) override;
    bool IsConnected() override;
    void Disconnect() override;

    bool Write(const char* content, std::size_t length) override;

private:
    static void ReadThread(Pavo2UdpConnector* conn_ptr);

private:
#ifdef _WIN32
    SOCKET m_iSocket;
    SOCKADDR_IN sendAddr;
#else
    int m_iSocket;
    struct sockaddr_in sendAddr;
#endif
    bool m_should_recv;
    bool m_is_recving;

    std::mutex m_socket_mutx;
    std::thread m_recv_thread;

    std::string m_local_ip;
    uint16_t m_local_port;

    std::string m_strLidarIP;
};

} //namespace pavo2
#endif // __PAVO2_NET_H__
