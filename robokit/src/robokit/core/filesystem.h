#ifndef _RBK_FILESYSTEM_H_
#define _RBK_FILESYSTEM_H_

#include <vector>

#include <robokit/config.h>

#include <boost/system/error_code.hpp>

namespace rbk {
    RBK_API void listFilesInDirectory(std::vector<std::string>& list,
        const std::string& dir,
        const std::string& extension,
        const bool& fullPath,
        const bool& withExtension,
        boost::system::error_code &ec);

    RBK_API std::string lastWriteFileInDirectory(const std::string& dir,
        const std::string& extension,
        const bool& fullPath,
        const bool& withExtension,
        boost::system::error_code &ec);

    RBK_API std::string oldestWriteFileInDirectory(const std::string& dir,
        const std::string& extension,
        const bool& fullPath,
        const bool& withExtension,
        boost::system::error_code &ec);

    RBK_API bool deleteLastWriteFileInDirectory(const std::string& dir,
        const std::string& extension,
        boost::system::error_code &ec);

    RBK_API bool deleteOldestWriteFileInDirectory(const std::string& dir,
        const std::string& extension,
        boost::system::error_code &ec);


    RBK_API std::string constructPath(const std::string& dir, const std::string& filename, const std::string& extenstion = "");

    RBK_API std::string getNativePath(const std::string& path);

    RBK_API bool pathExists(const std::string& path);

    RBK_API bool pathExists(const std::string& path, boost::system::error_code& ec);

    RBK_API std::string currentPath();

    RBK_API std::string currentPath(boost::system::error_code& ec);

    RBK_API bool isFile(const std::string& path);

    RBK_API bool isFile(const std::string& path, boost::system::error_code& ec);

    RBK_API bool isDirectory(const std::string& path);

    RBK_API bool isDirectory(const std::string& path, boost::system::error_code& ec);

    RBK_API bool isEmpty(const std::string& path);

    RBK_API bool isEmpty(const std::string& path, boost::system::error_code& ec);

    RBK_API bool deleteFile(const std::string& path);

    RBK_API bool deleteFile(const std::string& path, boost::system::error_code& ec);

    RBK_API uintmax_t deleteDirectory(const std::string& path);

    RBK_API uintmax_t deleteDirectory(const std::string& path, boost::system::error_code& ec);

    RBK_API bool createDirectory(const std::string& path);

    RBK_API bool createDirectory(const std::string& path, boost::system::error_code& ec);

    RBK_API bool createDirectories(const std::string& path);

    RBK_API bool createDirectories(const std::string& path, boost::system::error_code& ec);

    RBK_API bool directorySize(const std::string& path, unsigned long long& size);

    RBK_API bool directorySize(const std::string& path, unsigned long long& size, boost::system::error_code& ec);

    RBK_API void copy(const std::string& from, const std::string& to);

    RBK_API void copy(const std::string& from, const std::string& to, boost::system::error_code& ec);

    RBK_API void copyFile(const std::string& from, const std::string& to, bool overwrite = false);

    RBK_API void copyFile(const std::string& from, const std::string& to, bool overwrite, boost::system::error_code& ec);

    RBK_API void copyDirectory(const std::string& from, const std::string& to);

    RBK_API void copyDirectory(const std::string& from, const std::string& to, boost::system::error_code& ec);

    RBK_API uintmax_t fileSize(const std::string& path);

    RBK_API uintmax_t fileSize(const std::string& path, boost::system::error_code& ec);
} // namespace rbk

#endif // ~_RBK_FILESYSTEM_H_
