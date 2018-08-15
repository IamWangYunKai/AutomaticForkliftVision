#ifndef CLIENTSOCKET_HEADER_SEEN
#define CLIENTSOCKET_HEADER_SEEN

#ifdef WIN32
#include <winsock2.h>
#else // assume unix environment
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>
#define SOCKET int
#endif

// simple wrapper class for communication with PIL server
class ClientSocket
{
public:
    ClientSocket(const char* addr, int port);
    ~ClientSocket();
    int receiveFrame(int byte_size,void* buffer);
    void sendFrame(int byte_size,const void* buffer);

private:
#ifdef WIN32
    WSAData mWsaData;
#endif
       SOCKET m_SD;
    SOCKET EstablishConnection(u_long nRemoteAddr, u_short nPort);
    bool ShutdownConnection(SOCKET sd);
};

#endif // CLIENTSOCKET_HEADER_SEEN