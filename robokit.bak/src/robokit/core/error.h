#ifndef _RBK_CORE_ERROR_H_
#define _RBK_CORE_ERROR_H_

#include <iostream>
#include <cstdint>
#include <string>
#include <map>

#include <robokit/config.h>
#include <robokit/core/mutex.h>
#include <robokit/core/logger.h>
#include <robokit/utils/json.h>

#include <chrono>
#include <atomic>

namespace rbk {
    class ErrorCodes;

    class RBK_API Code {
    public:
        uint16_t value() const {
            return _code;
        }

        virtual std::string getString() const = 0;

        virtual std::string message() const = 0;

        virtual void setMessage(const std::string&) = 0;

        std::chrono::time_point<std::chrono::system_clock> getTimePoint() const {
            return _timePoint;
        }

        uint64_t count() const {
            return _count;
        }

        virtual bool assign(const uint16_t code) {
            _code = code;
            return true;
        }

        virtual void clear() {
            _code = 0;
        }

        typedef void(*unspecified_bool_type)();
        static void unspecified_bool_true() {}

        operator unspecified_bool_type() const { // true if error
            return _code == 0 ? 0 : unspecified_bool_true;
        }

        bool operator!() const { // true if no error
            return _code == 0;
        }

        friend std::ostream& operator<< (std::ostream& os, const Code& c) {
            os << c.getString();
            return os;
        }

        uint16_t operator() () const {
            return _code;
        }

        // relationals:
        inline friend bool operator==(const Code& lhs,
            const Code& rhs)
            //  the more symmetrical non-member syntax allows enum
            //  conversions work for both rhs and lhs.
        {
            return lhs._code == rhs._code;
        }

        inline friend bool operator<(const Code& lhs,
            const Code& rhs)
            //  the more symmetrical non-member syntax allows enum
            //  conversions work for both rhs and lhs.
        {
            return lhs._code < rhs._code;
        }

    protected:
        Code();
        Code(uint16_t code);
        virtual ~Code();

        uint16_t _code;
        uint64_t _count = 0;
        std::chrono::time_point<std::chrono::system_clock> _timePoint;
        std::string _desc = "";
        friend ErrorCodes;
    private:
        Code& operator++();
        void updateTimePoint();
    };

    class RBK_API Fatal : public Code {
    public:
        Fatal();
        Fatal(uint16_t code);
        Fatal(const Fatal&);
        Fatal& operator=(const Fatal&);
        virtual ~Fatal();
        virtual bool assign(const uint16_t code);
        virtual std::string getString() const;
        virtual std::string message() const;
        virtual void setMessage(const std::string&);
    };

    class RBK_API Error : public Code {
    public:
        Error();
        Error(uint16_t code);
        Error(const Error&);
        Error& operator=(const Error&);
        virtual ~Error();
        virtual bool assign(const uint16_t code);
        virtual std::string getString() const;
        virtual std::string message() const;
        virtual void setMessage(const std::string&);
    };

    class RBK_API Warning : public Code {
    public:
        Warning();
        Warning(uint16_t code);
        Warning(const Warning&);
        Warning& operator=(const Warning&);
        virtual ~Warning();
        virtual bool assign(const uint16_t code);
        virtual std::string getString() const;
        virtual std::string message() const;
        virtual void setMessage(const std::string&);
    };

    class RBK_API Notice : public Code {
    public:
        Notice();
        Notice(uint16_t code);
        Notice(const Notice&);
        Notice& operator=(const Notice&);
        virtual ~Notice();
        virtual bool assign(const uint16_t code);
        virtual std::string getString() const;
        virtual std::string message() const;
        virtual void setMessage(const std::string&);
    };

    class RBK_API ErrorCodes {
    private:
        struct object_creator {
            // This constructor does nothing more than ensure that Instance()
            // is called before main() begins, thus creating the static
            // SingletonClass object before multithreading race issues can come up.
            object_creator() { ErrorCodes::Instance(); }
            inline void do_nothing() const { }
        };

        static object_creator create_object;

        ErrorCodes();

        ~ErrorCodes();

        ErrorCodes(const ErrorCodes&);

        ErrorCodes& operator=(const ErrorCodes&);

    public:
        static ErrorCodes* Instance();

        bool init(const rbk::utils::json& json, std::string& errMsg);

        std::string getFatalMessage(const uint16_t code) const;
        std::string getErrorMessage(const uint16_t code) const;
        std::string getWarningMessage(const uint16_t code) const;
        std::string getNoticeMessage(const uint16_t code) const;

        bool setFatal(const Fatal& fatal, const std::string& msg = "");

        bool setFatal(const uint16_t fatalCode, const std::string& msg = "");

        bool clearFatal(const Fatal& fatal);

        bool clearFatal(const uint16_t fatalCode);

        void clearAllFatals();

        Fatal getFatal(const uint16_t fatalCode);
        
        bool fatalExists(const uint16_t fatalCode);

        size_t fatalNum();

        std::map<uint16_t, Fatal> fatals();

        bool setError(const Error& error, const std::string& msg = "");

        bool setError(const uint16_t errorCode, const std::string& msg = "");

        bool clearError(const Error& error);

        bool clearError(const uint16_t errorCode);

        Error getError(const uint16_t errorCode);

        bool errorExists(const uint16_t errorCode);

        size_t errorNum();

        std::map<uint16_t, Error> errors();

        bool setWarning(const Warning& warning, const std::string& msg = "");

        bool setWarning(const uint16_t warningCode, const std::string& msg = "");

        bool clearWarning(const Warning& warning);

        bool clearWarning(const uint16_t warningCode);

        Warning getWarning(const uint16_t warningCode);

        bool warningExists(const uint16_t warningCode);

        size_t warningNum();

        std::map<uint16_t, Warning> warnings();

        bool setNotice(const Notice& notice, const std::string& msg = "");

        bool setNotice(const uint16_t noticeCode, const std::string& msg = "");

        Notice getNotice(const uint16_t noticeCode);

        bool noticeExists(const uint16_t noticeCode);

        size_t noticeNum();

        std::map<uint16_t, Notice> notices();

    private:
        rbk::utils::json _errorDoc;
        std::atomic<bool> _init;

        std::map<uint16_t, Fatal> _fatals;
        rbk::rwMutex _fatalsMutex;

        std::map<uint16_t, Error> _errors;
        rbk::rwMutex _errorsMutex;

        std::map<uint16_t, Warning> _warnings;
        rbk::rwMutex _warningsMutex;

        std::map<uint16_t, Notice> _notices;
        rbk::rwMutex _noticesMutex;
    };
} // namespace rbk

#endif // ~_RBK_CORE_ERROR_H_
