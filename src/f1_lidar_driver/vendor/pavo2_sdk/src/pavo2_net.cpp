#include <chrono>
#include <iostream>
#include "../include/pavo2_net.h"

namespace pavo2 {

Pavo2RecvDataUnit::Pavo2RecvDataUnit(char* data, std::size_t size)
    :data_size(size)
{
    memcpy(data_buff, data, size);
}


Pavo2RecvDataUnit::Pavo2RecvDataUnit()
    :data_size(0)
{
    memset(data_buff, 0, PAVO2_PARSE_BUFFER_SIZE);
}


void Pavo2Connector::SetReadCallback(ReadCallback_t read_callback)
{
    f_ReadCallback = read_callback;
}

//-------------------TCP Class------------------------------
Pavo2TcpConnector::Pavo2TcpConnector()
    : m_should_recv(false)
    , m_is_recving(false)
#ifdef _WIN32
    , m_iSocket(INVALID_SOCKET)
#else
    , m_iSocket(-1)
#endif
    , m_local_ip("")
    , m_local_port(0)
    , m_lidar_ip("")
    , m_lidar_port(0)
{

}

Pavo2TcpConnector::~Pavo2TcpConnector()
{
    this->Disconnect();
}

void Pavo2TcpConnector::SetLocalAddress(const std::string& local_ip, uint16_t local_port)
{
    m_local_ip = local_ip;
    m_local_port = local_port;
}

bool Pavo2TcpConnector::Connect(const std::string& lidar_ip, uint16_t lidar_port, uint32_t timeout)
{
    std::unique_lock<std::mutex> guard(this->m_socket_mutx);
    int no_delay_flag = 1;

#ifdef _WIN32
    m_iSocket = socket(AF_INET, SOCK_STREAM, 0);
    if(m_iSocket == INVALID_SOCKET)
    {
        closesocket(m_iSocket);
        return false;
    }

    if(setsockopt(m_iSocket, IPPROTO_TCP, TCP_NODELAY, (char *)&no_delay_flag, sizeof(int)) < 0)
    {
        return false;
    }

   //bind local ip address
    SOCKADDR_IN local_addr;
    local_addr.sin_family = AF_INET;
    if(m_local_ip.length() != 0)
    {
        if(inet_pton(AF_INET, m_local_ip.c_str(), &local_addr.sin_addr) != 1)
            return false;
    }
    else
    {
        local_addr.sin_addr.s_addr = INADDR_ANY;
    }

    //m_local_port: 0 or specified
    local_addr.sin_port = htons(m_local_port);

    if(bind(m_iSocket, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0)
    {
        closesocket(m_iSocket);
        return false;
    }

    if(!ConnecTimeout(lidar_ip, lidar_port, timeout))
    {
        return false;
    }
    int nNetTimeout = 1000;
    setsockopt(m_iSocket, SOL_SOCKET, SO_SNDTIMEO, (char *)&nNetTimeout, sizeof(int));
    setsockopt(m_iSocket, SOL_SOCKET, SO_RCVTIMEO, (char *)&nNetTimeout, sizeof(int));

#else //linux
    m_iSocket = socket(AF_INET, SOCK_STREAM, 0);
    if(m_iSocket < 0)
    {
        return false;
    }


    if(setsockopt(m_iSocket, IPPROTO_TCP, TCP_NODELAY, (char *)&no_delay_flag, sizeof(int)) < 0)
    {
        return false;
    }

    struct sockaddr_in local_addr;
    local_addr.sin_family = AF_INET;
    if(m_local_ip.length() != 0)
    {
        local_addr.sin_addr.s_addr = inet_addr(m_local_ip.c_str());
        if(local_addr.sin_addr.s_addr == INADDR_NONE)
            return false;
    }
    else
    {
        local_addr.sin_addr.s_addr = INADDR_ANY;
    }

    local_addr.sin_port = htons(m_local_port);

    if(bind(m_iSocket, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0)
    {
        close(m_iSocket);
        return false;
    }

    if(!ConnecTimeout(lidar_ip, lidar_port, timeout))
    {
        return false;
    }
    struct timeval timeOut = {3, 0};
    //设置发送超时
    setsockopt(m_iSocket, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeOut, sizeof(struct timeval));
    //设置接收超时
    setsockopt(m_iSocket, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeOut, sizeof(struct timeval));

#endif
    m_should_recv = true;

    m_recv_thread = std::thread(Pavo2TcpConnector::ReadThread, this);

    //wait for 500ms to confirm whether thread is started.
    auto expire = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while(true)
    {
        if(m_is_recving)
            return true;

        if(std::chrono::steady_clock::now() > expire)
        {
            m_should_recv = false;
            if(m_recv_thread.joinable())
                m_recv_thread.join();
            return false;
        }
    }

    return true;
}


bool Pavo2TcpConnector::ConnecTimeout(const std::string& lidar_ip, uint16_t lidar_port, uint32_t timeout)
{
    bool bRt = false;
    int ret = 0;

    std::unique_ptr<struct timeval> tm_ptr(nullptr);
    if(timeout != 0)
    {
        tm_ptr.reset(new struct timeval);
        tm_ptr->tv_sec = timeout / 1000;
        tm_ptr->tv_usec = (timeout % 1000) * 1000;
    }

#ifdef _WIN32
    fd_set rset;
    fd_set wset;
    unsigned long mode = 1;

    SOCKADDR_IN targetAddr;
    targetAddr.sin_family = AF_INET;
    targetAddr.sin_port = htons(lidar_port);
    if(inet_pton(AF_INET, lidar_ip.c_str(), &targetAddr.sin_addr) != 1)
        return false;

    ret = ioctlsocket(m_iSocket, FIONBIO, &mode);
    if(ret)
    {
        closesocket(m_iSocket);
        return false;
    }
    ret = connect(m_iSocket, (SOCKADDR*)&targetAddr, sizeof(SOCKADDR));
    if(ret)
    {
        ret = WSAGetLastError();
        if(WSAEWOULDBLOCK == ret)
        {
            FD_ZERO(&rset);
            FD_ZERO(&wset);
            FD_SET(m_iSocket, &rset);
            FD_SET(m_iSocket, &wset);
            ret = select(m_iSocket + 1, &rset, &wset, NULL, tm_ptr.get());
            if(ret < 0)
            {
                closesocket(m_iSocket);
                return false;
            }
            else if(ret == 0)
            {
                closesocket(m_iSocket);
                return false;
            }
            else
            {
                if(FD_ISSET(m_iSocket, &wset))
                {
                    mode = 0;
                    ret = ioctlsocket(m_iSocket, FIONBIO, &mode);
                    if(ret)
                    {
                        closesocket(m_iSocket);
                        return false;
                    }
                    return true;
                }
                else
                {
                    closesocket(m_iSocket);
                    return false;
                }
            }
        }
        else
        {
            closesocket(m_iSocket);
            return false;
        }
    }
    else
    {
        return true;
    }
#else
    int flag = fcntl(m_iSocket, F_GETFL);
    flag |= O_NONBLOCK;
    fcntl(m_iSocket, F_SETFL, flag);

    struct sockaddr_in ser;
    ser.sin_family = AF_INET;
    ser.sin_port = htons(lidar_port);
    ser.sin_addr.s_addr = inet_addr(lidar_ip.c_str());
    socklen_t len = sizeof(struct sockaddr_in);

    ret = connect(m_iSocket, (struct sockaddr*)&ser, len);
    if((ret < 0) && (errno == EINPROGRESS))
    {
        fd_set connect_fdset;
        FD_ZERO(&connect_fdset);
        FD_SET(m_iSocket, &connect_fdset);
        do
        {
            ret = select(m_iSocket + 1, NULL, &connect_fdset, NULL, tm_ptr.get());
        }while((ret < 0) && (errno == EINTR));

        if(ret == 1)
        {
            int err;
            socklen_t sockLen = sizeof(err);
            int sockoptret = getsockopt(m_iSocket, SOL_SOCKET, SO_ERROR, &err, &sockLen);

            if(sockoptret == -1)
            {
                bRt = false;
            }
            if(err == 0)
            {
                bRt = true;
            }
        }
    }
    int flags = fcntl(m_iSocket, F_GETFL);
    flags &= ~O_NONBLOCK;
    ret = fcntl(m_iSocket, F_SETFL, flags);

#endif

    return bRt;
}

bool Pavo2TcpConnector::IsConnected()
{
    return  this->m_is_recving;
}

void Pavo2TcpConnector::Disconnect()
{
    std::unique_lock<std::mutex> guard(this->m_socket_mutx);
    m_should_recv = false;

#ifdef _WIN32
    shutdown(m_iSocket, SD_BOTH);
    closesocket(m_iSocket);
    m_iSocket = INVALID_SOCKET;
#else //_WIN32
    shutdown(m_iSocket, SHUT_RDWR);
    close(m_iSocket);
    m_iSocket = -1;
#endif

    if(m_recv_thread.joinable())
        m_recv_thread.join();

    m_local_ip = "";
    m_local_port = 0;

    m_lidar_ip = "";
    m_lidar_port = 0;

    return;
}

bool Pavo2TcpConnector::Write(const char* content, std::size_t length)
{
#ifdef _WIN32
    if(m_iSocket == INVALID_SOCKET)
#else
    if(m_iSocket < 0)
#endif
    {
        return false;
    }

    int ret = send(m_iSocket, content, (int)length, 0);
    if(ret <= 0)
    {
        return false;
    }
    return true;
}

void Pavo2TcpConnector::ReadThread(Pavo2TcpConnector* pTcpConnect)
{
    pTcpConnect->m_is_recving = true;
    while(pTcpConnect->m_should_recv)
    {
        auto recv_buff = std::unique_ptr<Pavo2RecvDataUnit>(new Pavo2RecvDataUnit);
        int iRt = recv(pTcpConnect->m_iSocket, (char*)recv_buff->data_buff, PAVO2_RECV_SEND_BUFFER_SIZE, 0);

        if(iRt == 0)
        {
            break;
        }
        else if(iRt < 0)
        {
#ifdef _WIN32
            int err = WSAGetLastError();
            if((err == WSAEWOULDBLOCK) || (err == WSAEINVAL) || (err == WSAETIMEDOUT))
            {
                continue;
            }
#else
            if((errno == EAGAIN) || (errno == EWOULDBLOCK))
            {
                continue;
            }
#endif
            break; // other errors
        }

        recv_buff->data_size = iRt;
        if(pTcpConnect->f_ReadCallback)
            pTcpConnect->f_ReadCallback(std::move(recv_buff));
        else
            recv_buff.reset();
    }
    pTcpConnect->m_is_recving = false;
}

//--------------------------UDP Class-----------------------
Pavo2UdpConnector::Pavo2UdpConnector()
{
    m_is_recving = false;
    m_should_recv = false;
#ifdef _WIN32
    m_iSocket = INVALID_SOCKET;
#else
    m_iSocket = -1;
#endif
}

Pavo2UdpConnector::~Pavo2UdpConnector()
{
    this->Disconnect();
}

void Pavo2UdpConnector::SetLocalAddress(const std::string& local_ip, uint16_t local_port)
{
    m_local_ip = local_ip;
    m_local_port = local_port;
}

bool Pavo2UdpConnector::Connect(const std::string& lidar_ip, uint16_t lidar_port, uint32_t timeout)
{
    (void) timeout;
    if(m_local_port == 0)
        return false;

    std::unique_lock<std::mutex> guard(this->m_socket_mutx);

#ifdef _WIN32
    SOCKADDR_IN receiveAddr;
    receiveAddr.sin_family = AF_INET;
    receiveAddr.sin_port = htons(m_local_port);
    //receiveAddr.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
    if(m_local_ip.length() != 0)
    {
        if(inet_pton(AF_INET, m_local_ip.c_str(), &receiveAddr.sin_addr) != 1)
            return false;
    }
    else
    {
        receiveAddr.sin_addr.s_addr = INADDR_ANY;
    }

    m_iSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if(m_iSocket == INVALID_SOCKET)
    {
        closesocket(m_iSocket);
        return false;
    }
    int retVal = ::bind((SOCKET)m_iSocket, (sockaddr*)(&receiveAddr), sizeof(SOCKADDR));
    if(retVal == SOCKET_ERROR)
    {
        closesocket(m_iSocket);
        return false;
    }
    memset(&sendAddr, 0, sizeof(SOCKADDR_IN));
    sendAddr.sin_family = AF_INET;
    sendAddr.sin_port = htons(lidar_port);
    sendAddr.sin_addr.S_un.S_addr = inet_addr(lidar_ip.c_str());
#else
    struct sockaddr_in receiveAddr;
    memset(&receiveAddr, 0, sizeof(receiveAddr));
    receiveAddr.sin_family = AF_INET;
    receiveAddr.sin_port = htons(m_local_port);
    if(m_local_ip.length() != 0)
    {
        receiveAddr.sin_addr.s_addr = inet_addr(m_local_ip.c_str());
        if(receiveAddr.sin_addr.s_addr == INADDR_NONE)
            return false;
    }
    else
    {
        receiveAddr.sin_addr.s_addr = INADDR_ANY;
    }
    m_iSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(m_iSocket < 0)
    {
        return false;
    }

    if(bind(m_iSocket, (struct sockaddr*)&receiveAddr, sizeof(receiveAddr)) == -1)
    {
        return false;
    }

    memset(&sendAddr, 0, sizeof(sendAddr));
    sendAddr.sin_family = AF_INET;
    sendAddr.sin_port = htons(lidar_port);
    sendAddr.sin_addr.s_addr = inet_addr(lidar_ip.c_str());
#endif
    m_should_recv = true;
    m_strLidarIP = lidar_ip;

    m_recv_thread = std::thread(&Pavo2UdpConnector::ReadThread, this);
    return true;
}

bool Pavo2UdpConnector::IsConnected()
{
    return this->m_is_recving;
}

void Pavo2UdpConnector::Disconnect()
{
    std::unique_lock<std::mutex> guard(this->m_socket_mutx);
#ifdef _WIN32
    closesocket(m_iSocket);
    m_iSocket = INVALID_SOCKET;
#else
    shutdown(m_iSocket, SHUT_RDWR);
    close(m_iSocket);
#endif //_WIN32

    m_should_recv = false;
    if(m_recv_thread.joinable())
        m_recv_thread.join();
}

bool Pavo2UdpConnector::Write(const char* content, std::size_t length)
{
    //if(m_iSocket == 0)
    //    return false;

    int iRt = 0;

#ifdef _WIN32
    if(m_iSocket == INVALID_SOCKET)
	    return false;
    iRt = sendto(m_iSocket, (char*)content, (int)length, 0, (sockaddr*)&sendAddr, sizeof(sendAddr));
#else
    if(m_iSocket < 0)
        return false;
    iRt = sendto(m_iSocket, content, length, 0, (const struct sockaddr *)&sendAddr, sizeof(sendAddr));
#endif
    return (iRt > 0);
}

void Pavo2UdpConnector::ReadThread(Pavo2UdpConnector* conn_ptr)
{
    conn_ptr->m_is_recving = true;
#ifdef _WIN32
    SOCKADDR_IN peer;
    int len = sizeof(peer);
    while(conn_ptr->m_should_recv)
    {
        auto recv_buff = std::unique_ptr<Pavo2RecvDataUnit>(new Pavo2RecvDataUnit);
        int iRt = recvfrom(conn_ptr->m_iSocket, (char*)recv_buff->data_buff, PAVO2_RECV_SEND_BUFFER_SIZE - 1, 0, (struct sockaddr*)&peer, &len);
        if(iRt == SOCKET_ERROR)
            continue;
        recv_buff->data_size = iRt;
        std::string from_ip = inet_ntoa(peer.sin_addr);
        if(from_ip.compare(conn_ptr->m_strLidarIP) == 0)
        {
            //conn_ptr->parsePacket(recv_buffer, iRt);
            recv_buff->data_size = iRt;
            if(conn_ptr->f_ReadCallback)
                conn_ptr->f_ReadCallback(std::move(recv_buff));
            else
                recv_buff.reset();
        }
    }
#else
    struct sockaddr_in peer;
    socklen_t len = sizeof(peer);
    while(conn_ptr->m_should_recv)
    {
        auto recv_buff = std::unique_ptr<Pavo2RecvDataUnit>(new Pavo2RecvDataUnit);
        ssize_t s = recvfrom(conn_ptr->m_iSocket, recv_buff->data_buff, PAVO2_RECV_SEND_BUFFER_SIZE - 1, 0, (struct sockaddr*)&peer, &len);
        if(s > 0)
        {
            std::string from_ip = inet_ntoa(peer.sin_addr);
            if(from_ip.compare(conn_ptr->m_strLidarIP) == 0)
            {
                recv_buff->data_size = s;
                if(conn_ptr->f_ReadCallback)
                {
                    conn_ptr->f_ReadCallback(std::move(recv_buff));
                }
                else
                    recv_buff.reset();
            }
        }
        else //0 
        {
            continue;
        }
    }
#endif
    conn_ptr->m_is_recving = false;
}

} //namespace pavo2;
