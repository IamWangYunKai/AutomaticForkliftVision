#ifndef _RBK_UTILS_TCP_SERVER_H_
#define _RBK_UTILS_TCP_SERVER_H_

#include <robokit/config.h>
#include <robokit/core/mutex.h>

#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/shared_array.hpp>

namespace rbk {
    namespace utils {
        namespace tcp {
            /*
             * TCPSession, multithread unsafe, noncopyable, asynchronously
             */
            class RBK_API TCPSession {
            public:
                /**
                 * Constructor. Creates a socket session.
                 * \param io_service boost::asio::io_service
                 */
                TCPSession(boost::asio::io_service& io_service) : _socket(io_service) {}

                /**
                 * \return internal socket object
                 */
                inline boost::asio::ip::tcp::socket& socket() { return _socket; }

                std::vector<char> readBuffer;

                std::vector<char> writeQueue; ///< Data are queued here before they go in writeBuffer

                boost::shared_array<char> writeBuffer; ///< Data being written

                size_t writeBufferSize; ///< Size of writeBuffer

                rbk::mutex writeQueueMutex; ///< Mutex for access to writeQueue

            private:
                // noncopyable
                TCPSession(const TCPSession&);
                TCPSession& operator=(const TCPSession&);
                TCPSession();

                boost::asio::ip::tcp::socket _socket; ///< Socket object
            };

            class RBK_API TCPServer {
            public:
                TCPServer();

                ~TCPServer();

                void listen(const uint16_t port, const bool reuseAddr, boost::system::error_code &ec);
                void listen(const uint16_t port, const bool reuseAddr = false);

                inline bool isOpen() const { return open; }

                void close(boost::system::error_code &ec);
                void close();

                void closeSession(boost::shared_ptr<TCPSession> session, boost::system::error_code &ec);
                void closeSession(boost::shared_ptr<TCPSession> session);

                void setKeepAliveParam(boost::shared_ptr<TCPSession> session,
                    const bool on, const uint32_t idle, const uint32_t interval, const uint32_t count,
                    boost::system::error_code &ec);
                void setKeepAliveParam(boost::shared_ptr<TCPSession> session,
                    const bool on, const uint32_t idle, const uint32_t interval, const uint32_t count);

                /**
                 * Write data asynchronously. Returns immediately.
                 * \param data array of char to be sent through the socket
                 * \param size array size
                 */
                void write(boost::shared_ptr<TCPSession> session, const char *data, const size_t size);

                /**
                 * Write data asynchronously. Returns immediately.
                 * \param data to be sent through the socket
                 */
                void write(boost::shared_ptr<TCPSession> session, const std::vector<char> &data);

                /**
                 * Write data asynchronously. Returns immediately.
                 * Can be used to send ASCII data to the socket.
                 * \param s string to send
                 */
                void write(boost::shared_ptr<TCPSession> session, const std::string &s);

                /**
                 * Read data asynchronously until data is full or error occured.
                 * \param data array of char to store the data
                 * \param size array size
                 */
                void read(boost::shared_ptr<TCPSession> session, const size_t size,
                    const boost::function<void(boost::shared_ptr<TCPSession>,
                        const std::vector<char>&,
                        size_t,
                        const boost::system::error_code&)> &cb);

                /**
                 * Read data asynchronously, one or more buffer into data.
                 * readSome may not read all of the requested number of bytes. Consider using read function
                 * if you need to ensure that the requested amount of data is read before the blocking operation completes.
                 * \param data array of char to store the data
                 * \param size array size
                 */
                void readSome(boost::shared_ptr<TCPSession> session, const size_t size,
                    const boost::function<void(boost::shared_ptr<TCPSession>,
                        const std::vector<char>&,
                        size_t,
                        const boost::system::error_code&)> &cb);

                void setAcceptCallback(const boost::function<void(boost::shared_ptr<TCPSession>)> &cb);

                void clearAcceptCallback();

                inline boost::asio::ip::tcp::acceptor& acceptor() { return _acceptor; }
            private:
                // noncopyable
                TCPServer(const TCPServer&);
                TCPServer& operator=(const TCPServer&);

                void doAccept();

                void acceptEnd(boost::shared_ptr<TCPSession> session, const boost::system::error_code &ec);

                /**
                 * Callback called to start an asynchronous write operation.
                 * If it is already in progress, does nothing.
                 * This callback is called by the io_service in the spawned thread.
                 */
                void doWrite(boost::shared_ptr<TCPSession> session);

                /**
                 * Callback called at the end of an asynchronous write operation,
                 * if there is more data to write, restarts a new write operation.
                 * This callback is called by the io_service in the spawned thread.
                 */
                void writeEnd(boost::shared_ptr<TCPSession> session, const boost::system::error_code &ec);

                boost::asio::io_service _io;
                boost::asio::ip::tcp::acceptor _acceptor;

                volatile bool open; ///< True if port open

                boost::function<void(boost::shared_ptr<TCPSession>)> _callback;
            };
        } // namespace tcp
    } // namespace utils
} // namespace rbk

#endif // ~_RBK_UTILS_TCP_SERVER_H_
