#include "clientsocket.hpp"
#include <stdexcept>

#ifndef WIN32
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define SD_SEND 1
#endif

ClientSocket::ClientSocket (const char *addr, int port)
{
    int code = 0;
#ifdef WIN32
    code = WSAStartup (MAKEWORD (1, 1), &mWsaData);
#endif

    m_SD = EstablishConnection (inet_addr (addr), htons ( (unsigned short) port));
    if ( (code != 0) || m_SD == INVALID_SOCKET)
    {
        throw (std::runtime_error ("Unable to connect"));
    }
}

ClientSocket::~ClientSocket()
{
    if (m_SD != INVALID_SOCKET)
    {
        ShutdownConnection (m_SD);
    }
#ifdef WIN32
    WSACleanup();
#endif
}

int ClientSocket::receiveFrame (int byte_size, void *buffer)
{
    int total_bytes = 0;

    while (total_bytes < byte_size)
    {
        int new_bytes = ::recv (m_SD, (char *) buffer + total_bytes, byte_size - total_bytes, 0);

        if (new_bytes == SOCKET_ERROR)
        {
            throw (std::runtime_error ("receive error"));
        }

        if (new_bytes == 0)
        {
            throw (std::runtime_error ("Connection closed"));
        }

        total_bytes += new_bytes;
    }

    return total_bytes;
}

void ClientSocket::sendFrame (int byte_size, const void *buffer)
{
    if (::send (m_SD, (char *) buffer, byte_size, 0) == SOCKET_ERROR)
    {
        throw (std::runtime_error ("send error"));
    }
}

SOCKET ClientSocket::EstablishConnection (u_long nRemoteAddr, u_short nPort)
{
    // Create a stream socket
    SOCKET sd = socket (AF_INET, SOCK_STREAM, 0);
    if (sd != INVALID_SOCKET)
    {
        sockaddr_in sinRemote;
        sinRemote.sin_family = AF_INET;
        sinRemote.sin_addr.s_addr = nRemoteAddr;
        sinRemote.sin_port = nPort;
        int r = connect (sd, (sockaddr *) &sinRemote, sizeof (sockaddr_in));
        if (r == SOCKET_ERROR)
        {
            sd = INVALID_SOCKET;
        }
    }
    return sd;
}

bool ClientSocket::ShutdownConnection (SOCKET sd)
{
    if (shutdown (sd, SD_SEND) == SOCKET_ERROR)
    {
        return false;
    }
#ifdef WIN32
    if (closesocket (sd) == SOCKET_ERROR)
    {
        return false;
    }
#else
    close (sd);
#endif
    return true;

}
