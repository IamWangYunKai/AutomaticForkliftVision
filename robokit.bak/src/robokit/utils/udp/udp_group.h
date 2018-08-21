#ifndef _RBK_UTILS_UDP_GROUP_H_
#define _RBK_UTILS_UDP_GROUP_H_

#include <robokit/config.h>
#include <boost/asio/ip/udp.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>

namespace rbk {
    namespace utils {
        namespace udp {
            class RBK_API UDPGroupSender {
            public:
                UDPGroupSender();

                UDPGroupSender(const char* multicastAddress, const uint16_t multicastPort);

                ~UDPGroupSender();

                void bind(const char* multicastAddress, const uint16_t multicastPort);

                void bind(const char* multicastAddress, const uint16_t multicastPort, boost::system::error_code& ec);

                inline bool isOpen() const { return _socket.is_open(); }

                void close();
                void close(boost::system::error_code& ec);

                size_t write(const char *data, size_t size);
                size_t write(const char *data, size_t size, boost::system::error_code& ec);

                size_t write(const std::vector<char> &data);
                size_t write(const std::vector<char> &data, boost::system::error_code& ec);

                size_t write(const std::string &s);
                size_t write(const std::string &s, boost::system::error_code& ec);

                inline boost::asio::ip::udp::socket& socket() { return _socket; }

            private:
                // noncopyable
                UDPGroupSender(const UDPGroupSender&);
                UDPGroupSender& operator=(const UDPGroupSender&);

                boost::asio::io_service _io; ///< Io service object
                boost::asio::ip::udp::socket _socket; ///< Socket object
                boost::asio::ip::udp::endpoint _endpoint;
            };

            class RBK_API UDPGroupReceiver {
            public:

                UDPGroupReceiver();

                UDPGroupReceiver(const char* multicastAddress, const uint16_t multicastPort);

                ~UDPGroupReceiver();

                void listen(const char* multicastAddress, const uint16_t multicastPort, const bool reuseAddr = false);

                void listen(const char* multicastAddress, const uint16_t multicastPort, const bool reuseAddr, boost::system::error_code& ec);

                inline bool isOpen() const { return _socket.is_open(); }

                void close();
                void close(boost::system::error_code& ec);

                size_t read(char *data, size_t size);
                size_t read(char *data, size_t size, boost::system::error_code& ec);

                inline boost::asio::ip::udp::socket& socket() { return _socket; }

            private:
                // noncopyable
                UDPGroupReceiver(const UDPGroupReceiver&);
                UDPGroupReceiver& operator=(const UDPGroupReceiver&);

                boost::asio::io_service _io;
                boost::asio::ip::udp::socket _socket;
                boost::asio::ip::udp::endpoint _endpoint;
            };
        } // namespace udp
    } // namespace utils
} // namespace rbk

#endif // ~_RBK_UTILS_UDP_GROUP_H_
