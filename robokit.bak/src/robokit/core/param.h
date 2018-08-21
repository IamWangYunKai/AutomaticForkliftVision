#ifndef _RBK_PARAM_H_
#define _RBK_PARAM_H_

#include <robokit/config.h>
#include <robokit/core/mutex.h>
#include <robokit/core/filesystem.h>
#include <robokit/core/logger.h>
#include <robokit/core/thread_pool.h>
#include <robokit/core/error.h>
#include <robokit/utils/json.h>

#include <sqlite3pp/sqlite3pp.h>

#include <boost/mpl/or.hpp>
#include <boost/type_traits/is_integral.hpp>
#include <boost/type_traits/is_floating_point.hpp>
#include <boost/utility/enable_if.hpp>

#include <climits>

namespace rbk {

    namespace ParamGroup {
        const std::string Ungrouped = "Ungrouped";        // 未分组
        const std::string Chassis = "Chassis";            // 底盘
        const std::string Simulation = "Simulation";      // 仿真器
        const std::string DebugGUI = "DebugGUI";          // 调试GUI
        const std::string Laser = "Laser";                // 激光
        const std::string SensorFuser = "SensorFuser";    // 传感器融合
        const std::string IFMO3d303 = "IFMO3d303";
        const std::string Map = "Map";                    // 地图
        const std::string Navigation = "Navigation";      // 导航
        const std::string Localization = "Localization";  // 定位
        const std::string Differential = "Differential";  // 差动
        const std::string Omni = "Omni";                  // 全向
        const std::string Steer = "Steer";                // 舵轮
        const std::string Network = "Network";            // 网络
        const std::string Joystick = "Joystick";          // 手柄
        const std::string Other = "Other";                // 其他
        // ......
    };

    class BaseParam;

    template<typename T>
    class Param;

    template<typename T>
    class MutableParam;

    class Params;

    class RBK_API ParamsServer {
    public:
        bool init(const std::string& path, const std::string& dbName);

        bool closeDB();

        sqlite3pp::database* DB() const;

        Params* find(const std::string& pluginName);

        bool insert(Params* pluginParams);

        bool insert(const std::string& pluginName, Params* pluginParams);

        bool remove(const std::string& pluginName);

        bool remove(Params* pluginParams);

        bool update(const std::string& pluginName, Params* pluginParams);

        template<typename T>
        bool set(const std::string& pluginName, const std::string& paramKey, T value, std::string& errMsg = std::string(""));

        template<typename T>
        bool save(const std::string& pluginName, const std::string& paramKey, T value, std::string& errMsg = std::string(""));

        bool save(const std::string& pluginName, std::string& errMsg = std::string(""));

        bool reload(const std::string& pluginName, const std::string& paramKey, std::string& errMsg = std::string(""));

        bool reload(const std::string& pluginName, std::string& errMsg = std::string(""));

        bool reload(std::string& errMsg = std::string(""));

        void empty();

        size_t size();

        bool getJson(const std::string& pluginName, const std::string& key, rbk::utils::json& d, std::string& errMsg = std::string());

        static ParamsServer* Instance();

    private:
        struct object_creator {
            // This constructor does nothing more than ensure that Instance()
            // is called before main() begins, thus creating the static
            // SingletonClass object before multithreading race issues can come up.
            object_creator() { ParamsServer::Instance(); }
            inline void do_nothing() const { }
        };

        static object_creator create_object;

        ParamsServer();

        ~ParamsServer();

        ParamsServer(const ParamsServer&);

        ParamsServer& operator= (const ParamsServer&);

        std::map<std::string, Params*> _paramsServer;

        rbk::rwMutex _mutex;

        sqlite3pp::database* _db;

        std::string _dbName;

        bool _opened;
    };

    class RBK_API Params {
    public:
        Params();
        Params(const std::string& pluginName);
        ~Params();
        bool init(const std::string& pluginName);

        template<typename T>
        MutableParam<T>* insert(const std::string& key, T value);

        template<typename T>
        MutableParam<T>* insert(MutableParam<T>* newParam);

        bool remove(const std::string& key);

        template<typename T>
        bool remove(MutableParam<T>* param);

        BaseParam* find(const std::string& key);

        template<typename T>
        bool findParam(const std::string& key, T& value, T defaultValue, bool isMutable) {
            if (!_init.load()) {
                return false;
            }

            rbk::writeLock wlock(_mutex);

            auto db = rbk::ParamsServer::Instance()->DB();
            std::string qryString("SELECT Value, Mutable FROM " + _pluginName + " WHERE Key = '" + key + "' LIMIT 1");
            sqlite3pp::query qry(*db, qryString.c_str());
            sqlite3pp::query::iterator i = qry.begin();

            bool found = false;

            try {
                if (i != qry.end()) {
                    if (qry.column_count() == 2 && (*i).get<bool>(1) == isMutable) { // if mutability is different, ignore
                        value = (*i).get<T>(0);
                        found = true;
                    }
                }
                else // not found in database, insert one
                {
                    sqlite3pp::command cmd(*db, std::string("INSERT INTO " + _pluginName + "(Key, Type, Value, Mutable) VALUES (?, ?, ?, ?)").c_str());
                    cmd.binder() << key << typeid(T).name() << defaultValue << isMutable;
                    if (cmd.execute() != SQLITE_OK) {
                        LogError(formatLogText("findParam: " << _pluginName << ":" << key << " failed"));
                        return false;
                    }
                }
            }
            catch (const std::exception& e) {
                LogError(formatLogText("findParam: " << _pluginName << ":" << key << " error: " << e.what()));
                rbk::ErrorCodes::Instance()->setNotice(57001);   // 参数数据库相关操作出错
                return false;
            }

            if (found) {
                if (isMutable) {
                    _mutableJsonDoc[key] = value;
                    _jsonDoc[key] = defaultValue;  // set to the origin value for reload
                }
                else {
                    _jsonDoc[key] = value;
                }
            }
            else {
                if (isMutable) {
                    _mutableJsonDoc[key] = defaultValue;
                    _jsonDoc[key] = defaultValue;
                }
                else {
                    _jsonDoc[key] = defaultValue;
                }
            }

            return found;
        }

        template<typename T>
        void setMutableJson(const std::string& key, T value) {
            if (!_init.load()) { return; }

            rbk::writeLock wlock(_mutex);
            _mutableJsonDoc[key] = value;
        }

        void empty();
        size_t size();
        bool isChanged();
        bool reload(std::string& errMsg = std::string());
        bool reload(const std::string& key, std::string& errMsg = std::string());

        bool save(std::string& errMsg = std::string(""));

        template<typename T>
        bool save(const std::string& key, T value, bool reload = false, std::string& errMsg = std::string("")) {
            if (!_init.load()) { 
                errMsg = "params hasn't been inited already";
                return false;
            }

            // if reload, don't set it again
            if (reload || this->set(key, value)) {
                auto db = rbk::ParamsServer::Instance()->DB();
                try {
                    std::string cmdString("UPDATE " + _pluginName + " SET Value = ? WHERE Key = ?");
                    sqlite3pp::command cmd(*db, cmdString.c_str());
                    cmd.binder() << value << key;
                    if (cmd.execute() == SQLITE_OK) {
                        return true;
                    }
                    else {
                        errMsg = std::string("Save param ").append(_pluginName).append(": ").append(key).append(" failed");
                        return false;
                    }
                }
                catch (const std::exception& e) {
                    errMsg = std::string("Save param ").append(_pluginName).append(": ").append(key).append(" error: ").append(e.what());
                    rbk::ErrorCodes::Instance()->setNotice(57001);   // 参数数据库相关操作出错
                    return false;
                }
            }
            else {
                errMsg = std::string("Save param ").append(_pluginName).append(": ").append(key).append(" aborted");
                return false;
            }
        }

        template<typename T>
        bool get(const std::string& key, T& value);

        template<typename T>
        bool set(const std::string& key, T value, std::string& errMsg = std::string(""));

        std::string pluginName() const;

        bool getJson(const std::string& key, rbk::utils::json& d, std::string& errMsg = std::string());

    private:
        Params(const Params&);
        Params& operator= (const Params&);
        std::map<std::string, BaseParam*> _params;
        std::string _pluginName;
        rbk::rwMutex _mutex;
        std::string _jsonFilePath;
        std::string _mutableJsonFilePath;
        rbk::utils::json _mutableJsonDoc;
        rbk::utils::json _jsonDoc;
        std::atomic<bool> _init;
    };

    class RBK_API BaseParam {
    public:
        virtual ~BaseParam();
        virtual size_t type() const;
        virtual bool isChanged() const;
    protected:
        BaseParam(size_t typeHashCode);
        size_t _type;
        std::atomic<bool> _changed;
    };

    template<typename T>
    class Param : public BaseParam {
    public:
        // SFINAE(匹配失败不是错误)
        Param(typename boost::enable_if<boost::mpl::or_<boost::is_integral<T>, boost::is_floating_point<T>>>::type* dummy = 0) : BaseParam(typeid(T).hash_code()) { }

        ~Param() { }

        bool init(const std::string& key, T value, T defaultValue, T minValue = std::numeric_limits<T>::lowest(), T maxValue = std::numeric_limits<T>::max(), const std::string& group = rbk::ParamGroup::Ungrouped, const std::string& desc = "", bool advanced = false, std::string& errMsg = std::string("")) {
            if (_init.test_and_set()) {
                errMsg = "this param has been inited already";
                return false;
            }
            _key = key;
            if (minValue > maxValue) {
                errMsg = std::string("param ").append(key).append("'s minValue is larger than maxValue");
                return false;
            }
            else {
                _minValue = minValue;
                _maxValue = maxValue;
            }
            if (defaultValue > maxValue) {
                errMsg = std::string("param ").append(key).append("'s defaultValue is larger than maxValue");
                return false;
            }
            else if (defaultValue < minValue) {
                errMsg = std::string("param ").append(key).append("'s defaultValue is smaller than minValue");
                return false;
            }
            else {
                _defaultValue = defaultValue;
            }
            if (value > maxValue) {
                errMsg = std::string("param ").append(key).append("'s value is larger than maxValue");
                return false;
            }
            else if (value < minValue) {
                errMsg = std::string("param ").append(key).append("'s value is smaller than minValue");
                return false;
            }
            else {
                _value = value;
            }
            _group = group;
            _desc = desc;
            _advanced = advanced;
            return true;
        }

        inline std::string key() const { return _key; }

        inline std::string desc() const { return _desc; }

        inline std::string group() const { return _group; }

        inline T minValue() const { return _minValue; }

        inline T maxValue() const { return _maxValue; }

        inline T defaultValue() const { return _defaultValue; }

        inline bool advanced() const { return _advanced; }

        inline T get() const {
            return _value;
        }

        friend std::ostream& operator<< (std::ostream& os, const Param<T>& p) {
            os << p.get();
            return os;
        }

        T operator() () const {
            return _value;
        }

        operator T() const {
            return _value;
        }

    private:
        Param<T>(const Param<T>&);

        Param<T>& operator= (const Param<T>&);

        virtual bool isChanged() const { return false; }

        std::string _key;
        T _value;
        std::string _desc;
        std::string _group;
        T _minValue;
        T _maxValue;
        T _defaultValue;
        bool _advanced;
        std::atomic_flag _init = ATOMIC_FLAG_INIT;
    };

    // specialization for bool
    template<>
    class Param<bool> : public BaseParam {
    public:
        // SFINAE(匹配失败不是错误)
        Param() : BaseParam(typeid(bool).hash_code()) { }

        ~Param() { }

        bool init(const std::string& key, bool value, bool defaultValue, const std::string& group = rbk::ParamGroup::Ungrouped, const std::string& desc = "", bool advanced = false, std::string& errMsg = std::string("")) {
            if (_init.test_and_set()) {
                errMsg = "this param has been inited already";
                return false;
            }
            _key = key;
            _value = value;
            _defaultValue = defaultValue;
            _group = group;
            _desc = desc;
            _advanced = advanced;
            return true;
        }

        inline std::string key() const { return _key; }

        inline std::string desc() const { return _desc; }

        inline std::string group() const { return _group; }

        inline bool defaultValue() const { return _defaultValue; }

        inline bool advanced() const { return _advanced; }

        inline bool get() const {
            return _value;
        }

        friend std::ostream& operator<< (std::ostream& os, const Param<bool>& p) {
            os << p.get();
            return os;
        }

        bool operator() () const {
            return _value;
        }

        operator bool() const {
            return _value;
        }

    private:
        Param(const Param<bool>&);

        Param<bool>& operator= (const Param<bool>&);

        virtual bool isChanged() const { return false; }

        std::string _key;
        bool _value;
        std::string _desc;
        std::string _group;
        bool _defaultValue;
        bool _advanced;
        std::atomic_flag _init = ATOMIC_FLAG_INIT;
    };

    // specialization for std::string
    template<>
    class Param<std::string> : public BaseParam {
    public:
        Param() : BaseParam(typeid(std::string).hash_code()) {
            _init.clear();
        }

        ~Param() {}

        bool init(const std::string& key, const std::string& value, const std::string& defaultValue, const std::string& group = rbk::ParamGroup::Ungrouped, const std::string& desc = "", bool advanced = false, std::string& errMsg = std::string("")) {
            if (_init.test_and_set()) {
                errMsg = "this param has been inited already";
                return false;
            }
            _key = key;
            _value = value;
            _defaultValue = defaultValue;
            _group = group;
            _desc = desc;
            _advanced = advanced;
            return true;
        }

        inline std::string key() const { return _key; }

        inline std::string desc() const { return _desc; }

        inline std::string group() const { return _group; }

        inline std::string defaultValue() const { return _defaultValue; }

        inline bool advanced() const { return _advanced; }

        inline std::string get() const {
            return _value;
        }

        friend std::ostream& operator<< (std::ostream& os, const Param<std::string>& p) {
            os << p.get();
            return os;
        }

        std::string operator() () const {
            return _value;
        }

        operator std::string() const {
            return _value;
        }

        inline const char* c_str() const {
            return _value.c_str();
        }

        inline size_t length() const {
            return _value.length();
        }

        inline size_t size() const {
            return _value.size();
        }

    private:
        Param(const Param<std::string>&);
        Param& operator= (const Param<std::string>&);
        virtual bool isChanged() const { return false; }
        std::string _key;
        std::string _value;
        std::string _desc;
        std::string _group;
        std::string _defaultValue;
        bool _advanced;
        std::atomic_flag _init = ATOMIC_FLAG_INIT;
    };

    template<typename T>
    class MutableParam : public BaseParam {
    public:
        // SFINAE(匹配失败不是错误)
        MutableParam(typename boost::enable_if<boost::mpl::or_<boost::is_integral<T>, boost::is_floating_point<T>>>::type* dummy = 0) : BaseParam(typeid(T).hash_code()), 
            _params(nullptr) { }

        ~MutableParam() {
            if (_params != nullptr) {
                _params->remove(this);
            }
        }

        bool init(const std::string& key, T value, Params* params, T defaultValue, T minValue = std::numeric_limits<T>::lowest(), T maxValue = std::numeric_limits<T>::max(), const std::string& group = rbk::ParamGroup::Ungrouped, const std::string& desc = "", bool advanced = false, std::string& errMsg = std::string("")) {
            if (_init.test_and_set()) {
                errMsg = "this param has been inited already";
                return false;
            }
            _key = key;
            _params = params;
            if (minValue > maxValue) {
                errMsg = std::string("param ").append(key).append("'s minValue is larger than maxValue");
                return false;
            }
            else {
                _minValue = minValue;
                _maxValue = maxValue;
            }
            if (defaultValue > maxValue) {
                errMsg = std::string("param ").append(key).append("'s defaultValue is larger than maxValue");
                return false;
            }
            else if (defaultValue < minValue) {
                errMsg = std::string("param ").append(key).append("'s defaultValue is smaller than minValue");
                return false;
            }
            else {
                _defaultValue = defaultValue;
            }
            if (value > maxValue) {
                errMsg = std::string("param ").append(key).append("'s value is larger than maxValue");
                return false;
            }
            else if (value < minValue) {
                errMsg = std::string("param ").append(key).append("'s value is smaller than minValue");
                return false;
            }
            else {
                _value = value;
            }
            _group = group;
            _desc = desc;
            _advanced = advanced;

            if (_params == nullptr) {
                errMsg = std::string("father parmas is NULL");
                return false;
            }
            else {
                if (_params->insert(this) == nullptr) {
                    errMsg = std::string("redefinition detected");
                    rbk::ErrorCodes::Instance()->setNotice(57001);   // 参数数据库相关操作出错
                    return false;
                }
                else {
                    return true;
                }
            }
        }

        inline std::string key() const { return _key; }

        inline std::string desc() const { return _desc; }

        inline std::string group() const { return _group; }

        inline T minValue() const { return _minValue; }

        inline T maxValue() const { return _maxValue; }

        inline T defaultValue() const { return _defaultValue; }

        inline bool advanced() const { return _advanced; }

        inline bool isChanged() const {
            return _changed.load();
        }

        inline T get() {
            T v = _value.load();
            _changed.store(false);
            return v;
        }

        // for param server to generate json api, won't set the _changed to false
        inline T query() {
            T v = _value.load();
            return v;
        }

        bool set(const T& value, bool fromParams = false, std::string& errMsg = std::string("")) {
            if (value > _maxValue) {
                errMsg = std::string("value ").append(std::to_string(value)).append(" is larger than max value: ").append(std::to_string(_maxValue));
                return false;
            }
            else if (value < _minValue) {
                errMsg = std::string("value ").append(std::to_string(value)).append(" is smaller than min value: ").append(std::to_string(_minValue));
                return false;
            }
            else {
                if (!fromParams && _params != nullptr) {
                    _params->setMutableJson(_key, value);
                }
                _value.store(value);
                _changed.store(true);
                return true;
            }
        }

        bool save(T value, std::string& errMsg = std::string("")) {
            if (value > _maxValue) {
                errMsg = std::string("value ").append(std::to_string(value)).append(" is larger than max value: ").append(std::to_string(_maxValue));
                return false;
            }
            else if (value < _minValue) {
                errMsg = std::string("value ").append(std::to_string(value)).append(" is smaller than min value: ").append(std::to_string(_minValue));
                return false;
            }
            else {
                if (_params != nullptr) {
                    return _params->save(_key, value, false, errMsg);
                }
                return true;
            }
        }

        bool save(std::string& errMsg = std::string("")) {
            if (_params != nullptr) {
                return _params->save(_key, _value.load(), false, errMsg);
            }
            return true;
        }

        bool reload(std::string& errMsg = std::string("")) {
            if (_params != nullptr) {
                return _params->reload(_key, errMsg);
            }
            return true;
        }

        friend std::ostream& operator<< (std::ostream& os, MutableParam<T>& p) {
            os << p.get();
            return os;
        }

        friend std::istream& operator >> (std::istream& is, MutableParam<T>& p) {
            T value;
            is >> value;
            p.set(value);
            return is;
        }

        T operator() () {
            return this->get();
        }

        void operator() (T value) {
            this->set(value);
        }

        MutableParam<T>& operator= (MutableParam<T>& p) {
            if (this == &p) {
                return *this;
            }
            this->set(p);
            return *this;
        }

        MutableParam<T>(MutableParam<T>& p) : BaseParam(typeid(T).hash_code()) {
            this->set(p);
        }

        MutableParam<T>& operator= (const Param<T>& p) {
            this->set(p);
            return *this;
        }

        MutableParam<T>& operator= (const T& value) {
            this->set(value);
            return *this;
        }

        operator T() {
            return this->get();
        }

    private:
        std::string _key;
        std::string _desc;
        std::atomic<T> _value;
        std::string _group;
        T _minValue;
        T _maxValue;
        T _defaultValue;
        bool _advanced;
        Params* _params;
        std::atomic_flag _init = ATOMIC_FLAG_INIT;
    };

    // specialization for bool
    template<>
    class MutableParam<bool> : public BaseParam {
    public:
        // SFINAE(匹配失败不是错误)
        MutableParam() : BaseParam(typeid(bool).hash_code()),
            _params(nullptr) { }

        ~MutableParam() {
            if (_params != nullptr) {
                _params->remove(this);
            }
        }

        bool init(const std::string& key, bool value, Params* params, bool defaultValue, const std::string& group = "", const std::string& desc = "", bool advanced = false, std::string& errMsg = std::string("")) {
            if (_init.test_and_set()) {
                errMsg = "this param has been inited already";
                return false;
            }
            _key = key;
            _params = params;
            _value = value;
            _defaultValue = defaultValue;
            _desc = desc;
            _group = group;
            _advanced = advanced;

            if (_params == nullptr) {
                errMsg = std::string("father parmas is NULL");
                return false;
            }
            else {
                if (_params->insert(this) == nullptr) {
                    errMsg = std::string("redefinition detected");
                    rbk::ErrorCodes::Instance()->setNotice(57001);   // 参数数据库相关操作出错
                    return false;
                }
                else {
                    return true;
                }
            }
        }

        inline std::string key() const { return _key; }

        inline std::string desc() const { return _desc; }

        inline std::string group() const { return _group; }

        inline bool defaultValue() const { return _defaultValue; }

        inline bool advanced() const { return _advanced; }

        inline bool isChanged() const {
            return _changed.load();
        }

        inline bool get() {
            bool v = _value.load();
            _changed.store(false);
            return v;
        }

        // for param server to generate json api, won't set the _changed to false
        inline bool query() {
            bool v = _value.load();
            return v;
        }

        bool set(bool value, bool fromParams = false, std::string& errMsg = std::string("")) {
            if (!fromParams && _params != nullptr) {
                _params->setMutableJson(_key, value);
            }
            _value.store(value);
            _changed.store(true);
            return true;
        }

        bool save(bool value, std::string& errMsg = std::string("")) {
            if (_params != nullptr) {
                return _params->save(_key, value, false, errMsg);
            }
            return true;
        }

        bool save(std::string& errMsg = std::string("")) {
            if (_params != nullptr) {
                return _params->save(_key, _value.load(), false, errMsg);
            }
            return true;
        }

        bool reload(std::string& errMsg = std::string("")) {
            if (_params != nullptr) {
                return _params->reload(_key, errMsg);
            }
            return true;
        }

        friend std::ostream& operator<< (std::ostream& os, MutableParam<bool>& p) {
            os << p.get();
            return os;
        }

        friend std::istream& operator >> (std::istream& is, MutableParam<bool>& p) {
            bool value;
            is >> value;
            p.set(value);
            return is;
        }

        bool operator() () {
            return this->get();
        }

        void operator() (bool value) {
            this->set(value);
        }

        MutableParam<bool>& operator= (MutableParam<bool>& p) {
            if (this == &p) {
                return *this;
            }
            this->set(p);
            return *this;
        }

        MutableParam(MutableParam<bool>& p) : BaseParam(typeid(bool).hash_code()) {
            this->set(p);
        }

        MutableParam<bool>& operator= (const Param<bool>& p) {
            this->set(p);
            return *this;
        }

        MutableParam<bool>& operator= (const bool& value) {
            this->set(value);
            return *this;
        }

        operator bool() {
            return this->get();
        }

    private:
        std::string _key;
        std::string _desc;
        std::atomic<bool> _value;
        std::string _group;
        bool _defaultValue;
        bool _advanced;
        Params* _params;
        std::atomic_flag _init = ATOMIC_FLAG_INIT;
    };

    // specialization for std::string
    template<>
    class MutableParam<std::string> : public BaseParam {
    public:
        MutableParam() : BaseParam(typeid(std::string).hash_code()), _params(nullptr) { }

        ~MutableParam() {
            if (_params != nullptr) {
                _params->remove(this);
            }
        }

        bool init(std::string key, std::string value, Params* params, const std::string& defaultValue, const std::string& group = "", const std::string desc = "", bool advanced = false, std::string& errMsg = std::string("")) {
            if (_init.test_and_set()) {
                errMsg = "this param has been inited already";
                return false;
            }
            _key = key;
            _params = params;
            _value = value;
            _defaultValue = defaultValue;
            _desc = desc;
            _group = group;
            _advanced = advanced;

            if (_params == nullptr) {
                errMsg = std::string("father parmas is NULL");
                return false;
            }
            else {
                if (_params->insert(this) == nullptr) {
                    errMsg = std::string("redefinition detected");
                    rbk::ErrorCodes::Instance()->setNotice(57001);   // 参数数据库相关操作出错
                    return false;
                }
                else {
                    return true;
                }
            }
        }

        inline std::string key() const { return _key; }

        inline std::string desc() const { return _desc; }

        inline std::string group() const { return _group; }

        inline std::string defaultValue() const { return _defaultValue; }

        inline bool advanced() const { return _advanced; }

        inline bool isChanged() const {
            return _changed.load();
        }

        std::string get() {
            rbk::readLock rlock(_mutex);
            _changed.store(false);
            return _value;
        }

        // for param server to generate json api, won't set the _changed to false
        std::string query() {
            rbk::readLock rlock(_mutex);
            return _value;
        }

        bool set(const std::string& value, bool fromParams = false, std::string& errMsg = std::string("")) {
            if (!fromParams && _params != nullptr) {
                _params->setMutableJson(_key, value);
            }
            // 这把锁一定要放下面，否则可能死锁
            rbk::writeLock wlock(_mutex);
            _value = value;
            _changed.store(true);
            return true;
        }

        bool save(const std::string& value, std::string& errMsg = std::string("")) {
            if (_params != nullptr) {
                return _params->save(_key, value, false, errMsg);
            }
            return true;
        }

        bool save(std::string& errMsg = std::string("")) {
            if (_params != nullptr) {
                return _params->save(_key, _value, false, errMsg);
            }
            return true;
        }

        bool reload(std::string& errMsg = std::string("")) {
            if (_params != nullptr) {
                return _params->reload(_key, errMsg);
            }
            return true;
        }

        friend std::ostream& operator<< (std::ostream& os, MutableParam<std::string>& p) {
            os << p.get();
            return os;
        }

        friend std::istream& operator >> (std::istream& is, MutableParam<std::string>& p) {
            std::string value;
            is >> value;
            p.set(value);
            return is;
        }

        std::string operator() () {
            return this->get();
        }

        void operator() (const std::string& value) {
            this->set(value);
        }

        MutableParam& operator= (MutableParam<std::string>& p) {
            if (this == &p) {
                return *this;
            }
            this->set(p);
            return *this;
        }

        MutableParam(MutableParam<std::string>& p) : BaseParam(typeid(std::string).hash_code()) {
            this->set(p);
        }

        MutableParam& operator= (const Param<std::string>& p) {
            this->set(p);
            return *this;
        }

        MutableParam& operator= (const std::string& value) {
            this->set(value);
            return *this;
        }

        operator std::string() {
            return this->get();
        }

        const char* c_str() {
            // 这里如果直接 return this->get().c_str() 的话， get() 返回的临时变量会被销毁掉
            rbk::readLock rlock(_mutex);
            _changed.store(false);
            return _value.c_str();
        }

        size_t length() {
            return this->get().length();
        }

        size_t size() {
            return this->get().size();
        }

    private:
        std::string _key;
        std::string _desc;
        std::string _value;
        std::string _group;
        std::string _defaultValue;
        bool _advanced;
        rbk::rwMutex _mutex;
        Params* _params;
        std::atomic_flag _init = ATOMIC_FLAG_INIT;
    };

    typedef std::map<std::string, BaseParam*>::iterator PARAM_ITER;

    template<typename T>
    MutableParam<T>* Params::insert(const std::string& key, T value) {
        if (!_init.load()) { return; }
        rbk::writeLock wlock(_mutex);
        MutableParam<T>* newParam = new MutableParam<T>(key, value, this);
        std::pair<PARAM_ITER, bool> ret = _params.insert(std::make_pair(key, newParam));
        if (ret.second) {
            rbk::json::setInDoc(_mutableJsonDoc, key, value);
            return newParam;
        }
        else {
            return nullptr;
        }
    }

    template<typename T>
    MutableParam<T>* Params::insert(MutableParam<T>* newParam) {
        if (!_init.load()) { return nullptr; }
        rbk::writeLock wlock(_mutex);
        std::pair<PARAM_ITER, bool> ret = _params.insert(std::make_pair(newParam->key(), newParam));
        if (ret.second) {
            _mutableJsonDoc[newParam->key()] = newParam->get();
            return newParam;
        }
        else {
            return nullptr;
        }
    }

    template<typename T>
    bool Params::remove(MutableParam<T>* param) {
        if (!_init.load()) { return false; }
        rbk::writeLock wlock(_mutex);
        return (_params.erase(param->key()) != 0);
    }

    template<typename T>
    bool Params::get(const std::string& key, T& value) {
        if (!_init.load()) { return false; }
        rbk::readLock rlock(_mutex);
        PARAM_ITER it = _params.find(key);
        if (it != _params.end()) {
            auto p = dynamic_cast<MutableParam<T>*>(it->second);
            if (p != nullptr) {
                value = p->get();
                return true;
            }
        }
        return false;
    }

    template<typename T>
    bool Params::set(const std::string& key, T value, std::string& errMsg) {
        if (!_init.load()) {
            errMsg = "uninitialized";
            return false;
        }
        rbk::writeLock wlock(_mutex);
        PARAM_ITER it = _params.find(key);
        if (it != _params.end()) {
            _mutableJsonDoc[key] = value;
            auto p = dynamic_cast<MutableParam<T>*>(it->second);
            if (p != nullptr) {
                return p->set(value, true, errMsg);
            }
            else {
                errMsg = "set this param failed, internal error";
                return false;
            }
        }
        else {
            errMsg = "not found this param";
            return false;
        }
    }

    typedef std::map<std::string, Params*>::iterator PARAMSERVER_ITER;

    template<typename T>
    bool ParamsServer::set(const std::string& pluginName, const std::string& paramKey, T value, std::string& errMsg) {
        rbk::writeLock wlock(_mutex);
        rbk::Params* pluginParams = this->find(pluginName);
        if (pluginParams != nullptr) {
            rbk::BaseParam* param = pluginParams->find(paramKey);
            if (param != nullptr) {
                return pluginParams->set(paramKey, value, errMsg);
            }
            else {
                errMsg = "set this param failed, internal error";
                return false;
            }
        }
        else {
            errMsg = "not found this param";
            return false;
        }
    }

    template<typename T>
    bool ParamsServer::save(const std::string& pluginName, const std::string& paramKey, T value, std::string& errMsg) {
        rbk::writeLock wlock(_mutex);
        rbk::Params* pluginParams = this->find(pluginName);
        if (pluginParams != nullptr) {
            rbk::BaseParam* param = pluginParams->find(paramKey);
            if (param != nullptr) {
                return pluginParams->save(paramKey, value, errMsg);
            }
            else {
                errMsg = "save this param failed, internal error";
                return false;
            }
        }
        else {
            errMsg = "not found this param";
            return false;
        }
    }
} // namespace rbk

#endif // ~_RBK_PARAM_H_
