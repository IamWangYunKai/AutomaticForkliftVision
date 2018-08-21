#ifndef _RBK_LUA_WRAPPER_H_
#define _RBK_LUA_WRAPPER_H_

#include <vector>
#include <string>
#include <cstdint>

#include <robokit/config.h>

#include <boost/mpl/or.hpp>
#include <boost/type_traits/is_integral.hpp>
#include <boost/type_traits/is_floating_point.hpp>
#include <boost/utility/enable_if.hpp>

#include <lua.hpp>

namespace rbk {
    namespace lua {
        struct nilType {
            nilType() {}
        };

        static const rbk::lua::nilType nil;

        template<typename T>
        class Nil {
        public:
            Nil(typename boost::enable_if<boost::mpl::or_<boost::is_integral<T>, boost::is_floating_point<T>>>::type* dummy = 0) : _nil(true) {}
            Nil(const T& value, typename boost::enable_if<boost::mpl::or_<boost::is_integral<T>, boost::is_floating_point<T>>>::type* dummy = 0) : _value(value), _nil(false) {}
            inline bool isNil() const { return _nil; }
            inline const T& value() const { return _value; }
            inline T& mutableValue() { return _value; }
            inline void setNil() { _nil = true; }
            const T& operator() () const {
                return _value;
            }
            void operator() (const T& value) {
                _value = value;
                _nil = false;
            }
            Nil<T>& operator= (const T& value) {
                _value = value;
                _nil = false;
                return *this;
            }
            operator const T&() const {
                return _value;
            }
            friend std::ostream& operator<< (std::ostream& os, const Nil<T>& n) {
                if (n.isNil()) {
                    os << std::string("nil");
                }
                else {
                    os << n.value();
                }
                return os;
            }
        private:
            T _value;
            bool _nil;
        };

        // specialization for std::vector<T>
        template<typename T>
        class Nil<std::vector<T>> {
        public:
            Nil(typename boost::enable_if<boost::mpl::or_<boost::is_integral<T>, boost::is_floating_point<T>>>::type* dummy = 0) : _nil(true) {}
            Nil(const std::vector<T>& value, typename boost::enable_if<boost::mpl::or_<boost::is_integral<T>, boost::is_floating_point<T>>>::type* dummy = 0) : _value(value), _nil(false) {}
            inline bool isNil() const { return _nil; }
            inline const std::vector<T>& value() const { return _value; }
            inline std::vector<T>& mutableValue() { return _value; }
            inline void setNil() { _nil = true; }
            const std::vector<T>& operator() () const {
                return _value;
            }
            void operator() (const std::vector<T>& value) {
                _value = value;
                _nil = false;
            }
            Nil<std::vector<T>>& operator= (const std::vector<T>& value) {
                _value = value;
                _nil = false;
                return *this;
            }
            friend std::ostream& operator<< (std::ostream& os, const Nil<std::vector<T>>& n) {
                if (n.isNil()) {
                    os << std::string("nil");
                }
                else {
                    os << std::string("(");
                    auto v = n.value();
                    for (auto i = v.begin(); i != v.end(); ++i) {
                        if (i != (v.end() - 1)) {
                            os << *i << std::string(",");
                        }
                        else {
                            os << *i;
                        }
                    }
                    os << std::string(")");
                }
                return os;
            }
            operator const std::vector<T>&() const {
                return _value;
            }
            inline size_t size() const {
                return _value.size();
            }
            inline size_t capacity() const {
                return _value.capacity();
            }
        private:
            std::vector<T> _value;
            bool _nil;
        };

        // specialization for std::vector<std::string>
        template<>
        class Nil<std::vector<std::string>> {
        public:
            Nil() : _nil(true) {}
            Nil(const std::vector<std::string>& value) : _value(value), _nil(false) {}
            inline bool isNil() const { return _nil; }
            inline const std::vector<std::string>& value() const { return _value; }
            inline std::vector<std::string>& mutableValue() { return _value; }
            inline void setNil() { _nil = true; }
            const std::vector<std::string>& operator() () const {
                return _value;
            }
            void operator() (const std::vector<std::string>& value) {
                _value = value;
                _nil = false;
            }
            Nil<std::vector<std::string>>& operator= (const std::vector<std::string>& value) {
                _value = value;
                _nil = false;
                return *this;
            }
            friend std::ostream& operator<< (std::ostream& os, const Nil<std::vector<std::string>>& n) {
                if (n.isNil()) {
                    os << std::string("nil");
                }
                else {
                    os << std::string("(");
                    auto v = n.value();
                    for (auto i = v.begin(); i != v.end(); ++i) {
                        if (i != (v.end() - 1)) {
                            os << *i << std::string(",");
                        }
                        else {
                            os << *i;
                        }
                    }
                    os << std::string(")");
                }
                return os;
            }
            operator const std::vector<std::string>&() const {
                return _value;
            }
            inline size_t size() const {
                return _value.size();
            }
            inline size_t capacity() const {
                return _value.capacity();
            }
        private:
            std::vector<std::string> _value;
            bool _nil;
        };

        // specialization for std::string
        template<>
        class Nil<std::string> {
        public:
            Nil() : _value(""), _nil(true) {}
            Nil(const std::string& value) : _value(value), _nil(false) {}
            Nil(const char* value) : _value(value), _nil(false) {}
            inline bool isNil() const { return _nil; }
            inline const std::string& value() const { return _value; }
            inline std::string& mutableValue() { return _value; }
            inline void setNil() { _nil = true; }
            const std::string& operator() () const {
                return _value;
            }
            void operator() (const std::string& value) {
                _value = value;
                _nil = false;
            }
            void operator() (const char* value) {
                _value = value;
                _nil = false;
            }
            Nil& operator= (const std::string& value) {
                _value = value;
                _nil = false;
                return *this;
            }
            Nil& operator= (const char* value) {
                _value = value;
                _nil = false;
                return *this;
            }
            operator const std::string&() const {
                return _value;
            }
            inline const char* c_str() const {
                return _value.c_str();
            }
            friend std::ostream& operator<< (std::ostream& os, const Nil<std::string>& n) {
                if (n.isNil()) {
                    os << std::string("nil");
                }
                else {
                    os << n.value();
                }
                return os;
            }
            inline size_t length() const {
                return _value.length();
            }
            inline size_t size() const {
                return _value.size();
            }
        private:
            std::string _value;
            bool _nil;
        };

        // specialization for const char*
        template<>
        class Nil<const char*> {
        public:
            Nil() : _value(""), _nil(true) {}
            Nil(const char* value) : _value(value), _nil(false) {}
            inline bool isNil() const { return _nil; }
            inline const char* value() const { return _value; }
            inline void setNil() { _nil = true; }
            const char* operator() () const {
                return _value;
            }
            void operator() (const char* value) {
                _value = value;
                _nil = false;
            }
            Nil& operator= (const char* value) {
                _value = value;
                _nil = false;
                return *this;
            }
            friend std::ostream& operator<< (std::ostream& os, const Nil<const char*>& n) {
                if (n.isNil()) {
                    os << std::string("nil");
                }
                else {
                    os << std::string(n.value());
                }
                return os;
            }
            operator const char*() const {
                return _value;
            }
        private:
            const char* _value;
            bool _nil;
        };

        RBK_API void reverseLuaState(lua_State *L);

        //-------------------------pushLuaStackTraits---------------------------//

        // TODO std::vector<rbk::lua::Nil<T>>

        template<typename TIN>
        void pushLuaStackTraits(lua_State* L, const TIN& t) {
            t.ToLuaTable(L);
        }

        template<typename TIN>
        void pushLuaStackTraits(lua_State* L, const rbk::lua::Nil<std::vector<TIN>>& t) {
            if (t.isNil()) {
                lua_pushnil(L);
            }
            else {
                pushLuaStackTraits(L, t.value());
            }
        }

        template<> RBK_API void pushLuaStackTraits(lua_State* L, const double& t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const std::vector<double>& t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const rbk::lua::Nil<double>& nil_t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const float& t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const std::vector<float>& t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const rbk::lua::Nil<float>& nil_t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const int& t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const std::vector<int>& t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const rbk::lua::Nil<int>& nil_t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const unsigned int& t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const std::vector<unsigned int>& t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const rbk::lua::Nil<unsigned int>& nil_t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const int16_t& t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const std::vector<int16_t>& t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const rbk::lua::Nil<int16_t>& nil_t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const uint16_t& t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const std::vector<uint16_t>& t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const rbk::lua::Nil<uint16_t>& nil_t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const int64_t& t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const std::vector<int64_t>& t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const rbk::lua::Nil<int64_t>& nil_t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const uint64_t& t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const std::vector<uint64_t>& t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const rbk::lua::Nil<uint64_t>& nil_t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const char& t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const std::vector<char>& t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const rbk::lua::Nil<char>& nil_t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const unsigned char& t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const std::vector<unsigned char>& t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const rbk::lua::Nil<unsigned char>& nil_t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const bool& t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const std::vector<bool>& t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const rbk::lua::Nil<bool>& nil_t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const std::string& t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const std::vector<std::string>& t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const rbk::lua::Nil<std::string>& nil_t);
        template<> RBK_API void pushLuaStackTraits(lua_State* L, const rbk::lua::nilType& nil);

        RBK_API void pushLuaStackTraits(lua_State* L, const char* t);
        RBK_API void pushLuaStackTraits(lua_State* L, char* t);

        //-------------------------popLuaStackTraits---------------------------//
        template<typename TOUT>
        void popLuaStackTraits(lua_State* L, TOUT& t) {
            t.FromLuaTable(L);
        }

        template<typename TOUT>
        void popLuaStackTraits(lua_State* L, rbk::lua::Nil<std::vector<TOUT>>& t) {
            if (lua_isnoneornil(L, -1)) {
                t.setNil();
            }
            else {
                std::vector<TOUT> r;
                popLuaStackTraits(L, r);
                t = r;
            }
        }

        template<> RBK_API void popLuaStackTraits(lua_State* L, double& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, std::vector<double>& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, rbk::lua::Nil<double>& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, float& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, std::vector<float>& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, rbk::lua::Nil<float>& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, int& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, std::vector<int>& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, rbk::lua::Nil<int>& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, unsigned int& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, std::vector<unsigned int>& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, rbk::lua::Nil<unsigned int>& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, int16_t& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, std::vector<int16_t>& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, rbk::lua::Nil<int16_t>& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, uint16_t& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, std::vector<uint16_t>& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, rbk::lua::Nil<uint16_t>& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, int64_t& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, std::vector<int64_t>& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, rbk::lua::Nil<int64_t>& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, uint64_t& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, std::vector<uint64_t>& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, rbk::lua::Nil<uint64_t>& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, char& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, std::vector<char>& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, rbk::lua::Nil<char>& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, unsigned char& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, std::vector<unsigned char>& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, rbk::lua::Nil<unsigned char>& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, bool& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, std::vector<bool>& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, rbk::lua::Nil<bool>& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, std::string& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, std::vector<std::string>& t);
        template<> RBK_API void popLuaStackTraits(lua_State* L, rbk::lua::Nil<std::string>& t);
        RBK_API void popLuaStackTraits(lua_State* L, char* t);

        //-------------------------getLuaStackTraits---------------------------//
        template<typename TI>
        void getLuaStackTraits(lua_State* L, TI& t, const int& index) {
            t.FromLuaTable(L);
        }

        template<typename TI>
        void getLuaStackTraits(lua_State* L, rbk::lua::Nil<std::vector<TI>>& t, const int& index) {
            if (lua_isnoneornil(L, index)) {
                t.setNil();
            }
            else {
                std::vector<TI> r;
                getLuaStackTraits(L, r, index);
                t = r;
            }
        }

        template<> RBK_API void getLuaStackTraits(lua_State* L, double& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, std::vector<double>& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, rbk::lua::Nil<double>& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, float& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, std::vector<float>& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, rbk::lua::Nil<float>& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, int& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, std::vector<int>& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, rbk::lua::Nil<int>& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, unsigned int& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, std::vector<unsigned int>& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, rbk::lua::Nil<unsigned int>& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, int16_t& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, std::vector<int16_t>& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, rbk::lua::Nil<int16_t>& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, uint16_t& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, std::vector<uint16_t>& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, rbk::lua::Nil<uint16_t>& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, int64_t& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, std::vector<int64_t>& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, rbk::lua::Nil<int64_t>& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, uint64_t& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, std::vector<uint64_t>& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, rbk::lua::Nil<uint64_t>& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, char& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, std::vector<char>& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, rbk::lua::Nil<char>& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, unsigned char& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, std::vector<unsigned char>& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, rbk::lua::Nil<unsigned char>& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, bool& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, std::vector<bool>& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, rbk::lua::Nil<bool>& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, std::string& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, std::vector<std::string>& t, const int& index);
        template<> RBK_API void getLuaStackTraits(lua_State* L, rbk::lua::Nil<std::string>& t, const int& index);
        RBK_API void getLuaStackTraits(lua_State* L, char* t, const int& index);

        //-------------------------popFirstLuaStackTraits---------------------------//
        template<typename TOUT>
        void popFirstLuaStackTraits(lua_State* L, TOUT& t) {
            t.FromLuaTable(L);
        }

        template<typename TOUT>
        void popFirstLuaStackTraits(lua_State* L, rbk::lua::Nil<std::vector<TOUT>>& t) {
            if (lua_isnoneornil(L, 1)) {
                t.setNil();
                lua_remove(L, 1);
            }
            else {
                std::vector<TOUT> r;
                popFirstLuaStackTraits(L, r);
                t = r;
            }
        }

        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, double& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, std::vector<double>& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, rbk::lua::Nil<double>& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, float& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, std::vector<float>& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, rbk::lua::Nil<float>& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, int& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, std::vector<int>& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, rbk::lua::Nil<int>& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, unsigned int& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, std::vector<unsigned int>& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, rbk::lua::Nil<unsigned int>& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, int16_t& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, std::vector<int16_t>& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, rbk::lua::Nil<int16_t>& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, uint16_t& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, std::vector<uint16_t>& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, rbk::lua::Nil<uint16_t>& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, int64_t& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, std::vector<int64_t>& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, rbk::lua::Nil<int64_t>& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, uint64_t& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, std::vector<uint64_t>& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, rbk::lua::Nil<uint64_t>& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, char& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, std::vector<char>& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, rbk::lua::Nil<char>& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, unsigned char& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, std::vector<unsigned char>& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, rbk::lua::Nil<unsigned char>& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, bool& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, std::vector<bool>& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, rbk::lua::Nil<bool>& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, std::string& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, std::vector<std::string>& t);
        template<> RBK_API void popFirstLuaStackTraits(lua_State* L, rbk::lua::Nil<std::string>& t);
        RBK_API void popFirstLuaStackTraits(lua_State* L, char* t);

        //-------------------------executeLuaFunc---------------------------//

        RBK_API void executeLuaFunc(lua_State* L, const char* func_name);

        template<typename TOUT1>
        void executeLuaFunc(
            lua_State* L, const char* func_name,
            TOUT1& o1) {
            lua_getglobal(L, func_name);
            lua_pcall(L, 0, LUA_MULTRET, 1);
            popLuaStackTraits(L, o1);
        }

        template<typename TOUT1, typename TOUT2>
        void executeLuaFunc(
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2) {
            lua_getglobal(L, func_name);
            lua_pcall(L, 0, LUA_MULTRET, 2);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
        }

        template<typename TOUT1, typename TOUT2, typename TOUT3>
        void executeLuaFunc(
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3) {
            lua_getglobal(L, func_name);
            lua_pcall(L, 0, LUA_MULTRET, 3);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
        }

        template<typename TOUT1, typename TOUT2, typename TOUT3, typename TOUT4>
        void executeLuaFunc(
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3, TOUT4& o4) {
            lua_getglobal(L, func_name);
            lua_pcall(L, 0, LUA_MULTRET, 4);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
            popLuaStackTraits(L, o4);
        }

        template<typename TOUT1, typename TOUT2, typename TOUT3, typename TOUT4, typename TOUT5>
        void executeLuaFunc(
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3, TOUT4& o4, TOUT5& o5) {
            lua_getglobal(L, func_name);
            lua_pcall(L, 0, LUA_MULTRET, 5);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
            popLuaStackTraits(L, o4);
            popLuaStackTraits(L, o5);
        }

        template<typename TIN1>
        void executeLuaFunc(const TIN1& i1,
            lua_State* L, const char* func_name) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            lua_pcall(L, 1, LUA_MULTRET, 0);
        }

        template<typename TIN1, typename TOUT1>
        void executeLuaFunc(const TIN1& i1,
            lua_State* L, const char* func_name,
            TOUT1& o1) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            lua_pcall(L, 1, LUA_MULTRET, 1);
            popLuaStackTraits(L, o1);
        }

        template<typename TIN1, typename TOUT1, typename TOUT2>
        void executeLuaFunc(const TIN1& i1,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            lua_pcall(L, 1, LUA_MULTRET, 2);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
        }

        template<typename TIN1, typename TOUT1, typename TOUT2, typename TOUT3>
        void executeLuaFunc(const TIN1& i1,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            lua_pcall(L, 1, LUA_MULTRET, 3);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
        }

        template<typename TIN1, typename TOUT1, typename TOUT2, typename TOUT3, typename TOUT4>
        void executeLuaFunc(const TIN1& i1,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3, TOUT4& o4) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            lua_pcall(L, 1, LUA_MULTRET, 4);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
            popLuaStackTraits(L, o4);
        }

        template<typename TIN1, typename TOUT1, typename TOUT2, typename TOUT3, typename TOUT4, typename TOUT5>
        void executeLuaFunc(const TIN1& i1,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3, TOUT4& o4, TOUT5& o5) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            lua_pcall(L, 1, LUA_MULTRET, 5);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
            popLuaStackTraits(L, o4);
            popLuaStackTraits(L, o5);
        }

        template<typename TIN1, typename TIN2>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2,
            lua_State* L, const char* func_name) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            lua_pcall(L, 2, LUA_MULTRET, 0);
        }

        template<typename TIN1, typename TIN2, typename TOUT1>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2,
            lua_State* L, const char* func_name,
            TOUT1& o1) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            lua_pcall(L, 2, LUA_MULTRET, 1);
            popLuaStackTraits(L, o1);
        }

        template<typename TIN1, typename TIN2, typename TOUT1, typename TOUT2>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            lua_pcall(L, 2, LUA_MULTRET, 2);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
        }

        template<typename TIN1, typename TIN2, typename TOUT1, typename TOUT2, typename TOUT3>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            lua_pcall(L, 2, LUA_MULTRET, 3);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
        }

        template<typename TIN1, typename TIN2, typename TOUT1, typename TOUT2, typename TOUT3, typename TOUT4>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3, TOUT4& o4) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            lua_pcall(L, 2, LUA_MULTRET, 4);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
            popLuaStackTraits(L, o4);
        }

        template<typename TIN1, typename TIN2, typename TOUT1, typename TOUT2, typename TOUT3, typename TOUT4, typename TOUT5>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3, TOUT4& o4, TOUT5& o5) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            lua_pcall(L, 2, LUA_MULTRET, 5);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
            popLuaStackTraits(L, o4);
            popLuaStackTraits(L, o5);
        }

        template<typename TIN1, typename TIN2, typename TIN3>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3,
            lua_State* L, const char* func_name) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            lua_pcall(L, 3, LUA_MULTRET, 0);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TOUT1>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3,
            lua_State* L, const char* func_name,
            TOUT1& o1) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            lua_pcall(L, 3, LUA_MULTRET, 1);
            popLuaStackTraits(L, o1);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TOUT1, typename TOUT2>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            lua_pcall(L, 3, LUA_MULTRET, 2);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TOUT1, typename TOUT2, typename TOUT3>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            lua_pcall(L, 3, LUA_MULTRET, 3);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TOUT1, typename TOUT2, typename TOUT3, typename TOUT4>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3, TOUT4& o4) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            lua_pcall(L, 3, LUA_MULTRET, 4);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
            popLuaStackTraits(L, o4);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TOUT1, typename TOUT2, typename TOUT3, typename TOUT4, typename TOUT5>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3, TOUT4& o4, TOUT5& o5) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            lua_pcall(L, 3, LUA_MULTRET, 5);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
            popLuaStackTraits(L, o4);
            popLuaStackTraits(L, o5);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4,
            lua_State* L, const char* func_name) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            lua_pcall(L, 4, LUA_MULTRET, 0);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TOUT1>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4,
            lua_State* L, const char* func_name,
            TOUT1& o1)
        {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            lua_pcall(L, 4, LUA_MULTRET, 1);
            popLuaStackTraits(L, o1);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TOUT1, typename TOUT2>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            lua_pcall(L, 4, LUA_MULTRET, 2);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TOUT1, typename TOUT2, typename TOUT3>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            lua_pcall(L, 4, LUA_MULTRET, 3);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TOUT1, typename TOUT2, typename TOUT3, typename TOUT4>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3, TOUT4& o4) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            lua_pcall(L, 4, LUA_MULTRET, 4);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
            popLuaStackTraits(L, o4);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TOUT1, typename TOUT2, typename TOUT3, typename TOUT4, typename TOUT5>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3, TOUT4& o4, TOUT5& o5) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            lua_pcall(L, 4, LUA_MULTRET, 5);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
            popLuaStackTraits(L, o4);
            popLuaStackTraits(L, o5);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5,
            lua_State* L, const char* func_name) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            lua_pcall(L, 5, LUA_MULTRET, 0);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TOUT1>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5,
            lua_State* L, const char* func_name,
            TOUT1& o1) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            lua_pcall(L, 5, LUA_MULTRET, 1);
            popLuaStackTraits(L, o1);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TOUT1, typename TOUT2>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            lua_pcall(L, 5, LUA_MULTRET, 2);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TOUT1, typename TOUT2, typename TOUT3>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            lua_pcall(L, 5, LUA_MULTRET, 3);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TOUT1, typename TOUT2, typename TOUT3, typename TOUT4>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3, TOUT4& o4) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            lua_pcall(L, 5, LUA_MULTRET, 4);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
            popLuaStackTraits(L, o4);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TOUT1, typename TOUT2, typename TOUT3, typename TOUT4, typename TOUT5>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3, TOUT4& o4, TOUT5& o5) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            lua_pcall(L, 5, LUA_MULTRET, 5);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
            popLuaStackTraits(L, o4);
            popLuaStackTraits(L, o5);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6,
            lua_State* L, const char* func_name) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            lua_pcall(L, 6, LUA_MULTRET, 0);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TOUT1>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6,
            lua_State* L, const char* func_name,
            TOUT1& o1) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            lua_pcall(L, 6, LUA_MULTRET, 1);
            popLuaStackTraits(L, o1);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TOUT1, typename TOUT2>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            lua_pcall(L, 6, LUA_MULTRET, 2);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TOUT1, typename TOUT2, typename TOUT3>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            lua_pcall(L, 6, LUA_MULTRET, 3);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TOUT1, typename TOUT2, typename TOUT3, typename TOUT4>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3, TOUT4& o4) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            lua_pcall(L, 6, LUA_MULTRET, 4);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
            popLuaStackTraits(L, o4);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TOUT1, typename TOUT2, typename TOUT3, typename TOUT4, typename TOUT5>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3, TOUT4& o4, TOUT5& o5) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            lua_pcall(L, 6, LUA_MULTRET, 5);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
            popLuaStackTraits(L, o4);
            popLuaStackTraits(L, o5);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TIN7>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6, const TIN7& i7,
            lua_State* L, const char* func_name) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            pushLuaStackTraits(L, i7);
            lua_pcall(L, 7, LUA_MULTRET, 0);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TIN7, typename TOUT1>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6, const TIN7& i7,
            lua_State* L, const char* func_name,
            TOUT1& o1) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            pushLuaStackTraits(L, i7);
            lua_pcall(L, 7, LUA_MULTRET, 1);
            popLuaStackTraits(L, o1);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TIN7, typename TOUT1, typename TOUT2>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6, const TIN7& i7,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            pushLuaStackTraits(L, i7);
            lua_pcall(L, 7, LUA_MULTRET, 2);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TIN7, typename TOUT1, typename TOUT2, typename TOUT3>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6, const TIN7& i7,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            pushLuaStackTraits(L, i7);
            lua_pcall(L, 7, LUA_MULTRET, 3);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TIN7, typename TOUT1, typename TOUT2, typename TOUT3, typename TOUT4>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6, const TIN7& i7,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3, TOUT4& o4) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            pushLuaStackTraits(L, i7);
            lua_pcall(L, 7, LUA_MULTRET, 4);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
            popLuaStackTraits(L, o4);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TIN7, typename TOUT1, typename TOUT2, typename TOUT3, typename TOUT4, typename TOUT5>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6, const TIN7& i7,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3, TOUT4& o4, TOUT5& o5) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            pushLuaStackTraits(L, i7);
            lua_pcall(L, 7, LUA_MULTRET, 5);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
            popLuaStackTraits(L, o4);
            popLuaStackTraits(L, o5);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TIN7, typename TIN8>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6, const TIN7& i7, const TIN8& i8,
            lua_State* L, const char* func_name) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            pushLuaStackTraits(L, i7);
            pushLuaStackTraits(L, i8);
            lua_pcall(L, 8, LUA_MULTRET, 0);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TIN7, typename TIN8, typename TOUT1>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6, const TIN7& i7, const TIN8& i8,
            lua_State* L, const char* func_name,
            TOUT1& o1) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            pushLuaStackTraits(L, i7);
            pushLuaStackTraits(L, i8);
            lua_pcall(L, 8, LUA_MULTRET, 1);
            popLuaStackTraits(L, o1);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TIN7, typename TIN8, typename TOUT1, typename TOUT2>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6, const TIN7& i7, const TIN8& i8,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            pushLuaStackTraits(L, i7);
            pushLuaStackTraits(L, i8);
            lua_pcall(L, 8, LUA_MULTRET, 2);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TIN7, typename TIN8, typename TOUT1, typename TOUT2, typename TOUT3>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6, const TIN7& i7, const TIN8& i8,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            pushLuaStackTraits(L, i7);
            pushLuaStackTraits(L, i8);
            lua_pcall(L, 8, LUA_MULTRET, 3);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TIN7, typename TIN8, typename TOUT1, typename TOUT2, typename TOUT3, typename TOUT4>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6, const TIN7& i7, const TIN8& i8,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3, TOUT4& o4) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            pushLuaStackTraits(L, i7);
            pushLuaStackTraits(L, i8);
            lua_pcall(L, 8, LUA_MULTRET, 4);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
            popLuaStackTraits(L, o4);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TIN7, typename TIN8, typename TOUT1, typename TOUT2, typename TOUT3, typename TOUT4, typename TOUT5>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6, const TIN7& i7, const TIN8& i8,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3, TOUT4& o4, TOUT5& o5) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            pushLuaStackTraits(L, i7);
            pushLuaStackTraits(L, i8);
            lua_pcall(L, 8, LUA_MULTRET, 5);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
            popLuaStackTraits(L, o4);
            popLuaStackTraits(L, o5);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TIN7, typename TIN8, typename TIN9>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6, const TIN7& i7, const TIN8& i8, const TIN9& i9,
            lua_State* L, const char* func_name) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            pushLuaStackTraits(L, i7);
            pushLuaStackTraits(L, i8);
            pushLuaStackTraits(L, i9);
            lua_pcall(L, 9, LUA_MULTRET, 0);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TIN7, typename TIN8, typename TIN9, typename TOUT1>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6, const TIN7& i7, const TIN8& i8, const TIN9& i9,
            lua_State* L, const char* func_name,
            TOUT1& o1) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            pushLuaStackTraits(L, i7);
            pushLuaStackTraits(L, i8);
            pushLuaStackTraits(L, i9);
            lua_pcall(L, 9, LUA_MULTRET, 1);
            popLuaStackTraits(L, o1);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TIN7, typename TIN8, typename TIN9, typename TOUT1, typename TOUT2>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6, const TIN7& i7, const TIN8& i8, const TIN9& i9,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            pushLuaStackTraits(L, i7);
            pushLuaStackTraits(L, i8);
            pushLuaStackTraits(L, i9);
            lua_pcall(L, 9, LUA_MULTRET, 2);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TIN7, typename TIN8, typename TIN9, typename TOUT1, typename TOUT2, typename TOUT3>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6, const TIN7& i7, const TIN8& i8, const TIN9& i9,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            pushLuaStackTraits(L, i7);
            pushLuaStackTraits(L, i8);
            pushLuaStackTraits(L, i9);
            lua_pcall(L, 9, LUA_MULTRET, 3);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TIN7, typename TIN8, typename TIN9, typename TOUT1, typename TOUT2, typename TOUT3, typename TOUT4>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6, const TIN7& i7, const TIN8& i8, const TIN9& i9,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3, TOUT4& o4) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            pushLuaStackTraits(L, i7);
            pushLuaStackTraits(L, i8);
            pushLuaStackTraits(L, i9);
            lua_pcall(L, 9, LUA_MULTRET, 4);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
            popLuaStackTraits(L, o4);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TIN7, typename TIN8, typename TIN9, typename TOUT1, typename TOUT2, typename TOUT3, typename TOUT4, typename TOUT5>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6, const TIN7& i7, const TIN8& i8, const TIN9& i9,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3, TOUT4& o4, TOUT5& o5) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            pushLuaStackTraits(L, i7);
            pushLuaStackTraits(L, i8);
            pushLuaStackTraits(L, i9);
            lua_pcall(L, 9, LUA_MULTRET, 5);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
            popLuaStackTraits(L, o4);
            popLuaStackTraits(L, o5);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TIN7, typename TIN8, typename TIN9, typename TIN10>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6, const TIN7& i7, const TIN8& i8, const TIN9& i9, const TIN10& i10,
            lua_State* L, const char* func_name) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            pushLuaStackTraits(L, i7);
            pushLuaStackTraits(L, i8);
            pushLuaStackTraits(L, i9);
            pushLuaStackTraits(L, i10);
            lua_pcall(L, 10, LUA_MULTRET, 0);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TIN7, typename TIN8, typename TIN9, typename TIN10, typename TOUT1>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6, const TIN7& i7, const TIN8& i8, const TIN9& i9, const TIN10& i10,
            lua_State* L, const char* func_name,
            TOUT1& o1) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            pushLuaStackTraits(L, i7);
            pushLuaStackTraits(L, i8);
            pushLuaStackTraits(L, i9);
            pushLuaStackTraits(L, i10);
            lua_pcall(L, 10, LUA_MULTRET, 1);
            popLuaStackTraits(L, o1);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TIN7, typename TIN8, typename TIN9, typename TIN10, typename TOUT1, typename TOUT2>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6, const TIN7& i7, const TIN8& i8, const TIN9& i9, const TIN10& i10,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            pushLuaStackTraits(L, i7);
            pushLuaStackTraits(L, i8);
            pushLuaStackTraits(L, i9);
            pushLuaStackTraits(L, i10);
            lua_pcall(L, 10, LUA_MULTRET, 2);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TIN7, typename TIN8, typename TIN9, typename TIN10, typename TOUT1, typename TOUT2, typename TOUT3>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6, const TIN7& i7, const TIN8& i8, const TIN9& i9, const TIN10& i10,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            pushLuaStackTraits(L, i7);
            pushLuaStackTraits(L, i8);
            pushLuaStackTraits(L, i9);
            pushLuaStackTraits(L, i10);
            lua_pcall(L, 10, LUA_MULTRET, 3);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TIN7, typename TIN8, typename TIN9, typename TIN10, typename TOUT1, typename TOUT2, typename TOUT3, typename TOUT4>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6, const TIN7& i7, const TIN8& i8, const TIN9& i9, const TIN10& i10,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3, TOUT4& o4) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            pushLuaStackTraits(L, i7);
            pushLuaStackTraits(L, i8);
            pushLuaStackTraits(L, i9);
            pushLuaStackTraits(L, i10);
            lua_pcall(L, 10, LUA_MULTRET, 4);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
            popLuaStackTraits(L, o4);
        }

        template<typename TIN1, typename TIN2, typename TIN3, typename TIN4, typename TIN5, typename TIN6, typename TIN7, typename TIN8, typename TIN9, typename TIN10, typename TOUT1, typename TOUT2, typename TOUT3, typename TOUT4, typename TOUT5>
        void executeLuaFunc(const TIN1& i1, const TIN2& i2, const TIN3& i3, const TIN4& i4, const TIN5& i5, const TIN6& i6, const TIN7& i7, const TIN8& i8, const TIN9& i9, const TIN10& i10,
            lua_State* L, const char* func_name,
            TOUT1& o1, TOUT2& o2, TOUT3& o3, TOUT4& o4, TOUT5& o5) {
            lua_getglobal(L, func_name);
            pushLuaStackTraits(L, i1);
            pushLuaStackTraits(L, i2);
            pushLuaStackTraits(L, i3);
            pushLuaStackTraits(L, i4);
            pushLuaStackTraits(L, i5);
            pushLuaStackTraits(L, i6);
            pushLuaStackTraits(L, i7);
            pushLuaStackTraits(L, i8);
            pushLuaStackTraits(L, i9);
            pushLuaStackTraits(L, i10);
            lua_pcall(L, 10, LUA_MULTRET, 5);
            popLuaStackTraits(L, o1);
            popLuaStackTraits(L, o2);
            popLuaStackTraits(L, o3);
            popLuaStackTraits(L, o4);
            popLuaStackTraits(L, o5);
        }
    } // namespace lua
} // namespace rbk

#endif // ~_RBK_LUA_WRAPPER_H_
