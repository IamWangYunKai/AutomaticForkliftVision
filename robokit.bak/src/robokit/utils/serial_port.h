#ifndef _RBK_UTILS_SERIAL_PORT_H_
#define _RBK_UTILS_SERIAL_PORT_H_

#include <robokit/config.h>
#include <robokit/core/mutex.h>

#include <boost/asio/steady_timer.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/write.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/function.hpp>
#include <boost/shared_array.hpp>
#include <boost/thread/thread.hpp>

#include <mutex>

namespace rbk {
    namespace utils {
        namespace serialport {
            class RBK_API AsyncSerial {
            public:
                AsyncSerial();

                /**
                 * Constructor. Creates and opens a serial device.
                 * \param devname serial device name, example "/dev/ttyS0"(POSIX) or "COM1"(Windows)
                 * \param baud_rate serial baud rate
                 * \param opt_parity serial parity, default none
                 * \param opt_csize serial character size, default 8bit
                 * \param opt_flow serial flow control, default none
                 * \param opt_stop serial stop bits, default 1
                 * \throws boost::system::system_error if cannot open the
                 * serial device
                 */
                AsyncSerial(const std::string &devname, unsigned int baud_rate,
                    boost::asio::serial_port_base::parity opt_parity =
                    boost::asio::serial_port_base::parity(
                        boost::asio::serial_port_base::parity::none),
                    boost::asio::serial_port_base::character_size opt_csize =
                    boost::asio::serial_port_base::character_size(8),
                    boost::asio::serial_port_base::flow_control opt_flow =
                    boost::asio::serial_port_base::flow_control(
                        boost::asio::serial_port_base::flow_control::none),
                    boost::asio::serial_port_base::stop_bits opt_stop =
                    boost::asio::serial_port_base::stop_bits(
                        boost::asio::serial_port_base::stop_bits::one));

                ~AsyncSerial();

                /**
                * Opens a serial device, if isOpen() == true, it will close the device first.
                * \param devname serial device name, example "/dev/ttyS0"(POSIX) or "COM1"(Windows)
                * \param baud_rate serial baud rate
                * \param opt_parity serial parity, default none
                * \param opt_csize serial character size, default 8bit
                * \param opt_flow serial flow control, default none
                * \param opt_stop serial stop bits, default 1
                * \throws boost::system::system_error if cannot open the
                * serial device
                */
                void open(const std::string &devname, unsigned int baud_rate,
                    boost::asio::serial_port_base::parity opt_parity =
                    boost::asio::serial_port_base::parity(
                        boost::asio::serial_port_base::parity::none),
                    boost::asio::serial_port_base::character_size opt_csize =
                    boost::asio::serial_port_base::character_size(8),
                    boost::asio::serial_port_base::flow_control opt_flow =
                    boost::asio::serial_port_base::flow_control(
                        boost::asio::serial_port_base::flow_control::none),
                    boost::asio::serial_port_base::stop_bits opt_stop =
                    boost::asio::serial_port_base::stop_bits(
                        boost::asio::serial_port_base::stop_bits::one));

                void setBaudRate(unsigned int baud_rate);

                /**
                 * \return true if serial device is open
                 */
                inline bool isOpen() const { return _port.is_open(); }

                /**
                 * \return true if error were found
                 */
                bool errorStatus() const;

                void flush(boost::system::error_code& ec);

                /**
                 * Close the serial device
                 * \throws boost::system::system_error if any error
                 */
                void close();

                /**
                 * Write data asynchronously. Returns immediately.
                 * \param data array of char to be sent through the serial device
                 * \param size array size
                 */
                void write(const char *data, size_t size);

                /**
                * Write data asynchronously. Returns immediately.
                * \param data to be sent through the serial device
                */
                void write(const std::vector<char> &data);

                /**
                * Write a string asynchronously. Returns immediately.
                * Can be used to send ASCII data to the serial device.
                * To send binary data, use write()
                * \param s string to send
                */
                void write(const std::string &s);

                /**
                 * \return internal port object
                 */
                inline boost::asio::serial_port& port() { return _port; }

                /**
                 * To allow derived classes to set a read callback
                 */
                void setReadCallback(const boost::function<void(const char *, size_t)> &cb);

                /**
                 * To unregister the read callback in the derived class destructor so it
                 * does not get called after the derived class destructor but before the
                 * base class destructor
                 */
                void clearReadCallback();

            protected:
                /**
                 * To allow derived classes to report errors
                 * \param e error status
                 */
                void setErrorStatus(bool e);

            private:
                AsyncSerial(const AsyncSerial&);
                AsyncSerial& operator=(const AsyncSerial&);

                /**
                 * Callback called to start an asynchronous read operation.
                 * This callback is called by the io_service in the spawned thread.
                 */
                void doRead();

                /**
                 * Callback called at the end of the asynchronous operation.
                 * This callback is called by the io_service in the spawned thread.
                 */
                void readEnd(const boost::system::error_code &ec,
                    size_t bytes_transferred);

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

                /**
                 * Callback to close serial port
                 */
                void doClose();

                /**
                 * Read buffer maximum size
                 */
                static const int readBufferSize = RBK_SERIAL_PORT_READ_BUFFER_SIZE;

                boost::asio::io_service _io;     ///< Io service object
                boost::asio::serial_port _port;  ///< Serial port object
                boost::thread _backgroundThread; ///< Thread that runs read/write operations
                std::atomic<bool> error; ///< Error flag

                /// Data are queued here before they go in writeBuffer
                std::vector<char> writeQueue;
                boost::shared_array<char> writeBuffer; ///< Data being written
                size_t writeBufferSize; ///< Size of writeBuffer
                rbk::mutex writeQueueMutex; ///< Mutex for access to writeQueue
                char readBuffer[AsyncSerial::readBufferSize]; ///< data being read

                /// Read complete callback
                boost::function<void(const char *, size_t)> _callback;
            };

            typedef boost::asio::basic_waitable_timer<std::chrono::steady_clock> steadyTimer;

            /**
             * Serial port class, with timeout on read operations.
             */

            class RBK_API SyncSerial {
            public:
                SyncSerial();

                /**
                 * Opens a serial device. By default timeout is disabled.
                 * \param devname serial device name, example "/dev/ttyS0" or "COM1"
                 * \param baud_rate serial baud rate
                 * \param opt_parity serial parity, default none
                 * \param opt_csize serial character size, default 8bit
                 * \param opt_flow serial flow control, default none
                 * \param opt_stop serial stop bits, default 1
                 * \throws boost::system::system_error if cannot open the
                 * serial device
                 */
                SyncSerial(const std::string& devname, unsigned int baud_rate,
                    boost::asio::serial_port_base::parity opt_parity =
                    boost::asio::serial_port_base::parity(
                        boost::asio::serial_port_base::parity::none),
                    boost::asio::serial_port_base::character_size opt_csize =
                    boost::asio::serial_port_base::character_size(8),
                    boost::asio::serial_port_base::flow_control opt_flow =
                    boost::asio::serial_port_base::flow_control(
                        boost::asio::serial_port_base::flow_control::none),
                    boost::asio::serial_port_base::stop_bits opt_stop =
                    boost::asio::serial_port_base::stop_bits(
                        boost::asio::serial_port_base::stop_bits::one));

                ~SyncSerial();

                /**
                 * Opens a serial device.
                 * \param devname serial device name, example "/dev/ttyS0" or "COM1"
                 * \param baud_rate serial baud rate
                 * \param opt_parity serial parity, default none
                 * \param opt_csize serial character size, default 8bit
                 * \param opt_flow serial flow control, default none
                 * \param opt_stop serial stop bits, default 1
                 * \throws boost::system::system_error if cannot open the
                 * serial device
                 */
                void open(const std::string& devname, unsigned int baud_rate,
                    boost::asio::serial_port_base::parity opt_parity =
                    boost::asio::serial_port_base::parity(
                        boost::asio::serial_port_base::parity::none),
                    boost::asio::serial_port_base::character_size opt_csize =
                    boost::asio::serial_port_base::character_size(8),
                    boost::asio::serial_port_base::flow_control opt_flow =
                    boost::asio::serial_port_base::flow_control(
                        boost::asio::serial_port_base::flow_control::none),
                    boost::asio::serial_port_base::stop_bits opt_stop =
                    boost::asio::serial_port_base::stop_bits(
                        boost::asio::serial_port_base::stop_bits::one));

                void open(const std::string& devname, unsigned int baud_rate,
                    boost::asio::serial_port_base::parity opt_parity,
                    boost::asio::serial_port_base::character_size opt_csize,
                    boost::asio::serial_port_base::flow_control opt_flow,
                    boost::asio::serial_port_base::stop_bits opt_stop,
                    boost::system::error_code& ec);

                /**
                 * \return true if serial device is open
                 */
                inline bool isOpen() const { return _port.is_open(); }

                /**
                 * Close the serial device
                 * \throws boost::system::system_error if any error
                 */
                void close();
                void close(boost::system::error_code& ec);

                void setBaudRate(unsigned int baud_rate);
                void setBaudRate(unsigned int baud_rate, boost::system::error_code& ec);

                /**
                 * Set the timeout on read operations.
                 */
                void setTimeout(const steadyTimer::duration& t);

                /**
                 * Clear the timeout on read operations.
                 */
                void clearTimeout();

                bool flush();
                bool flush(boost::system::error_code& ec);

                /**
                 * Write data
                 * \param data array of char to be sent through the serial device
                 * \param size array size
                 * \throws boost::system::system_error if any error
                 */
                size_t write(const char *data, size_t size);
                size_t write(const char *data, size_t size, boost::system::error_code& ec);

                /**
                * Write data
                * \param data to be sent through the serial device
                * \throws boost::system::system_error if any error
                */
                size_t write(const std::vector<char>& data);
                size_t write(const std::vector<char>& data, boost::system::error_code& ec);

                /**
                * Write a string. Can be used to send ASCII data to the serial device.
                * To send binary data, use write()
                * \param s string to send
                * \throws boost::system::system_error if any error
                */
                size_t write(const std::string& s);
                size_t write(const std::string& s, boost::system::error_code& ec);

                /**
                 * Read some data, blocking
                 * \param data array of char to be read through the serial device
                 * \param size array size
                 * \return number of character actually read 0<=return<=size
                 * \throws boost::system::system_error if any error
                 * \throws timeout_exception in case of timeout
                 */
                void read(char *data, size_t size);

                void read(char *data, size_t size, boost::system::error_code& ec);

                /**
                 * Read some data, blocking
                 * \param size how much data to read
                 * \return the receive buffer. It empty if no data is available
                 * \throws boost::system::system_error if any error
                 * \throws timeout_exception in case of timeout
                 */
                std::vector<char> read(size_t size);

                std::vector<char> read(size_t size, boost::system::error_code& ec);

                /**
                 * Read a string, blocking
                 * Can only be used if the user is sure that the serial device will not
                 * send binary data. For binary data read, use read()
                 * The returned string is empty if no data has arrived
                 * \param size how much data to read
                 * \return a string with the received data.
                 * \throws boost::system::system_error if any error
                 * \throws timeout_exception in case of timeout
                 */
                std::string readString(size_t size);

                std::string readString(size_t size, boost::system::error_code& ec);

                /**
                 * Read a line, blocking
                 * Can only be used if the user is sure that the serial device will not
                 * send binary data. For binary data read, use read()
                 * The returned string is empty if the line delimiter has not yet arrived.
                 * \param delimiter line delimiter, default="\n"
                 * \return a string with the received data. The delimiter is removed from
                 * the string.
                 * \throws boost::system::system_error if any error
                 * \throws timeout_exception in case of timeout
                 */
                std::string readStringUntil(const std::string& delim = "\n");

                std::string readStringUntil(const std::string& delim, boost::system::error_code& ec);

                /**
                 * \return internal serial port object
                 */
                inline boost::asio::serial_port& port() { return _port; }

            private:
                SyncSerial(const SyncSerial&);
                SyncSerial& operator=(const SyncSerial&);

                /**
                 * Parameters of performReadSetup.
                 * Just wrapper class, no encapsulation provided
                 */
                class ReadSetupParameters {
                public:
                    ReadSetupParameters() : fixedSize(false), delim(""), data(0), size(0) {}

                    explicit ReadSetupParameters(const std::string& delim) :
                        fixedSize(false), delim(delim), data(0), size(0) { }

                    ReadSetupParameters(char *data, size_t size) : fixedSize(true),
                        delim(""), data(data), size(size) { }

                    //Using default copy constructor, operator=

                    bool fixedSize; ///< True if need to read a fixed number of parameters
                    std::string delim; ///< String end delimiter (valid if fixedSize=false)
                    char *data; ///< Pointer to data array (valid if fixedSize=true)
                    size_t size; ///< Array size (valid if fixedSize=true)
                };

                /**
                 * This member function sets up a read operation, both reading a specified
                 * number of characters and reading until a delimiter string.
                 */
                void performReadSetup(const ReadSetupParameters& param);

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
                boost::asio::serial_port _port; ///< Serial port object

                steadyTimer _timer; ///< Timer for timeout
                //boost::posix_time::time_duration _timeout; ///< Read/write timeout
                steadyTimer::duration _timeout;
                boost::asio::streambuf _readData; ///< Holds eventual read but not consumed
                enum ReadResult _result;  ///< Used by read with timeout
                size_t _bytesTransferred; ///< Used by async read callback
                ReadSetupParameters _setupParameters; ///< Global because used in the OSX fix

                boost::system::error_code _ec;
            };
        } // namespace serialport
    } // namespace utils
} // namespace rbk

#endif // _RBK_UTILS_SERIAL_PORT_H_
