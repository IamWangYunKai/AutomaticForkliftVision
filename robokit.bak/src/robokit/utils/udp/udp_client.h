#ifndef _RBK_UTILS_UDP_CLIENT_H_
#define _RBK_UTILS_UDP_CLIENT_H_

#include <robokit/config.h>
#include <robokit/core/mutex.h>

#include <boost/asio/steady_timer.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>
#include <boost/function.hpp>
#include <boost/shared_array.hpp>
#include <boost/thread/thread.hpp>

namespace rbk {
    namespace utils {
        namespace udp {
            class RBK_API AsyncUDPClient {
            public:
                AsyncUDPClient();

                AsyncUDPClient(const char* ip, const uint16_t port);

                ~AsyncUDPClient();

                void bind(const uint16_t port, const bool reuseAddr = false);

                void connect(const char* ip, const uint16_t port);

                /**
                 * \return true if serial device is open
                 */
                inline bool isOpen() const { return _socket.is_open(); }

                /**
                 * \return true if error were found
                 */
                bool errorStatus() const;

                /**
                 * Close the socket
                 * \throws boost::system::system_error if any error
                 */
                void close();

                /**
                 * Write data asynchronously. Returns immediately.
                 * \param data array of char to be sent through the socket
                 * \param size array size
                 */
                void write(const char *data, size_t size);

                // 需要自己保证 data 指向内容的不会在数据发送完前被销毁
                void writeTo(const char *data, size_t size, const char* ip, const uint16_t port);

                /**
                 * Write data asynchronously. Returns immediately.
                 * \param data to be sent through the socket
                 */
                void write(const std::vector<char> &data);

                void writeTo(const std::vector<char> &data, const char* ip, const uint16_t port);

                /**
                 * Write a string asynchronously. Returns immediately.
                 * Can be used to send ASCII data to the socket.
                 * \param s string to send
                 */
                void write(const std::string &s);

                void writeTo(const std::string &s, const char* ip, const uint16_t port);

                /**
                 * \return internal socket object
                 */
                inline boost::asio::ip::udp::socket& socket() { return _socket; }

                /**
                 * Set the read callback, the callback will be called from the background thread
                 * when data arrives from the socket
                 * \param callback the receive callback
                 */
                void setReadCallback(const boost::function<void(const char *, size_t)> &cb);

                /**
                 * Removes the callback. Any data received after this function call will
                 * be lost.
                 */
                void clearReadCallback();

            protected:
                /**
                 * To allow derived classes to report errors, if there are any error
                 * You should use connect to establish a new socket connection
                 * \param e error status
                 */
                void setErrorStatus(bool e);

            private:
                AsyncUDPClient(const AsyncUDPClient&);
                AsyncUDPClient& operator=(const AsyncUDPClient&);

                /**
                 * Callback called to start an asynchronous read operation.
                 * This callback is called by the io_service in the spawned thread.
                 */
                void doRead();

                /**
                 * Callback called at the end of the asynchronous operation.
                 * This callback is called by the io_service in the spawned thread.
                 */
                void readEnd(const boost::system::error_code &ec, size_t bytes_transferred);

                /**
                 * Callback called to start an asynchronous write operation.
                 * If it is already in progress, does nothing.
                 * This callback is called by the io_service in the spawned thread.
                 */
                void doWrite();

                /**
                 * Callback called at the end of an asynchronous write operation,
                 * if there is more data to write, restarts a new write operation.
                 * This callback is called by the io_service in the spawned thread.
                 */
                void writeEnd(const boost::system::error_code &ec);

                void doClose();

                /**
                 * Read buffer maximum size
                 */
                static const int readBufferSize = 102400;

                std::vector<char> writeQueue; ///< Data are queued here before they go in writeBuffer

                boost::shared_array<char> writeBuffer; ///< Data being written

                size_t writeBufferSize; ///< Size of writeBuffer

                rbk::mutex writeQueueMutex; ///< Mutex for access to writeQueue

                char readBuffer[readBufferSize]; ///< data being read

                boost::asio::io_service _io; ///< Io service object

                boost::asio::ip::udp::socket _socket; ///< Socket object

                boost::thread _backgroundThread; ///< Thread that runs read/write operations

                std::atomic<bool> error; ///< Error flag

                /// Read complete callback
                boost::function<void(const char*, size_t)> _callback;
            };

            typedef boost::asio::basic_waitable_timer<std::chrono::steady_clock> steadyTimer;

            class RBK_API SyncUDPClient {
            public:
                /**
                * Default Constructor.
                */
                SyncUDPClient();

                SyncUDPClient(const uint16_t port);

                SyncUDPClient(const char* ip, const uint16_t port);

                ~SyncUDPClient();

                void bind(const uint16_t port, const bool reuseAddr = false);
                void bind(const uint16_t port, const bool reuseAddr, boost::system::error_code& ec);

                /**
                 * Opens a socket, if isOpen() == true, it will close the previous socket first.
                 * \param ip remote udp server ip, example "192.168.1.1"
                 * \param port udp server port
                 * \throws boost::system::system_error if cannot connect the server
                 */
                void connect(const char *ip, const uint16_t port);
                void connect(const char *ip, const uint16_t port, boost::system::error_code& ec);

                /**
                 * \return true if serial device is open
                 */
                inline bool isOpen() const { return _socket.is_open(); }

                /**
                 * Close the socket
                 * \throws boost::system::system_error if any error
                 */
                void close();
                void close(boost::system::error_code& ec);

                /**
                 * Set the timeout on read operations.
                 */
                void setTimeout(const steadyTimer::duration& t);

                /**
                 * Clear the timeout on read operations.
                 */
                void clearTimeout();

                /**
                 * Write data synchronously. Blocking IO.
                 * \param data array of char to be sent through the socket
                 * \param size array size
                 * \return the number of bytes written. Return 0 if an error occurred
                 */
                size_t write(const char *data, size_t size);
                size_t write(const char *data, size_t size, boost::system::error_code& ec);

                size_t writeTo(const char *data, size_t size, const char* ip, const uint16_t port);
                size_t writeTo(const char *data, size_t size, const char* ip, const uint16_t port, boost::system::error_code& ec);

                /**
                 * Write data synchronously. Blocking IO.
                 * \param data to be sent through the socket
                 * \return the number of bytes written. Return 0 if an error occurred
                 */
                size_t write(const std::vector<char> &data);
                size_t write(const std::vector<char> &data, boost::system::error_code& ec);

                size_t writeTo(const std::vector<char> &data, const char* ip, const uint16_t port);
                size_t writeTo(const std::vector<char> &data, const char* ip, const uint16_t port, boost::system::error_code& ec);

                /**
                 * Write data synchronously. Blocking IO.
                 * Can be used to send ASCII data to the socket.
                 * \param s string to send
                 * \return the number of bytes written. Return 0 if an error occurred
                 */
                size_t write(const std::string &s);
                size_t write(const std::string &s, boost::system::error_code& ec);

                size_t writeTo(const std::string &s, const char* ip, const uint16_t port);
                size_t writeTo(const std::string &s, const char* ip, const uint16_t port, boost::system::error_code& ec);

                /**
                 * Read data synchronously until data is full. Blocking IO.
                 * \param data array of char to store the data
                 * \param size array size
                 * \return the number of bytes read. Return 0 if an error occurred
                 */
                size_t read(char *data, size_t size);

                size_t read(char *data, size_t size, boost::system::error_code& ec);

                size_t readFrom(char *data, size_t size, std::string& ip, uint16_t& port);

                size_t readFrom(char *data, size_t size, std::string& ip, uint16_t& port, boost::system::error_code& ec);

                /**
                 * \return internal socket object
                 */
                inline boost::asio::ip::udp::socket& socket() { return _socket; }

            private:
                // noncopyable
                SyncUDPClient(const SyncUDPClient&);
                SyncUDPClient& operator=(const SyncUDPClient&);

                void performRead(char *data, size_t size);

                void performReadFrom(boost::asio::ip::udp::endpoint& endPoint, char *data, size_t size);

                /**
                 * Callback called either when the read timeout is expired or canceled.
                 * If called because timeout expired, sets result to resultTimeoutExpired
                 */
                void timeoutExpired(const boost::system::error_code& error);

                /**
                 * Callback called either if a read complete or read error occurs
                 * If called because of read complete, sets result to resultSuccess
                 * If called because read error, sets result to resultError
                 */
                void readCompleted(const boost::system::error_code& error,
                    const size_t bytesTransferred);

                /**
                 * Possible outcome of a read. Set by callbacks, read from main code
                 */
                enum ReadResult {
                    resultInProgress,
                    resultSuccess,
                    resultError,
                    resultTimeoutExpired
                };

                boost::asio::io_service _io; ///< Io service object
                boost::asio::ip::udp::socket _socket; ///< Socket object

                steadyTimer _timer; ///< Timer for timeout
                steadyTimer::duration _timeout; ///< Read timeout
                enum ReadResult _result;  ///< Used by read with timeout
                size_t _bytesTransferred; ///< Used by async read callback

                boost::system::error_code _ec;
            };
        }
    }
}

#endif //~_RBK_UTILS_UDP_CLIENT_H_
