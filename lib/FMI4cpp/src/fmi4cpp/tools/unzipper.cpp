
#include "fmi4cpp/tools/unzipper.hpp"

#include "fmi4cpp/mlog.hpp"

#include <tinyzip.hpp>

#include <exception>

bool fmi4cpp::unzip(const std::filesystem::path& zip_file, const std::filesystem::path& tmp_path)
{
    try {
        tinyzip::extract_all(absolute(zip_file), tmp_path);
        return true;
    } catch (const std::exception& e) {
        MLOG_ERROR("Failed to unzip '" << zip_file.string() << "': " << e.what());
        return false;
    }
}
