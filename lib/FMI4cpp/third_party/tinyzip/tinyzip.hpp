// tinyzip - minimal, dependency-free ZIP archive reader (C++17).
//
// Bundled with FMI4cpp so that FMUs can be unpacked without libzip/zlib on the
// host. Supports what FMU packages actually use: "stored" (method 0) and
// "deflate" (method 8) entries, with CRC-32 verification of every file.
//
// Not supported (a std::runtime_error is thrown): encryption, ZIP64 archives,
// multi-disk archives and compression methods other than stored/deflate.

#ifndef TINYZIP_HPP
#define TINYZIP_HPP

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace tinyzip
{

struct entry
{
    std::string name;          // path inside the archive, '/' separated
    std::uint32_t method{0};   // 0 = stored, 8 = deflate
    std::uint32_t crc32{0};
    std::uint64_t compressed_size{0};
    std::uint64_t uncompressed_size{0};
    std::uint64_t local_header_offset{0};

    bool is_directory() const
    {
        return !name.empty() && name.back() == '/';
    }
};

class archive
{
public:
    // Reads the whole archive into memory and parses the central directory.
    explicit archive(const std::filesystem::path& path);

    const std::vector<entry>& entries() const
    {
        return entries_;
    }

    // Decompresses a single entry and verifies its CRC-32.
    std::vector<std::uint8_t> read(const entry& e) const;

private:
    std::vector<std::uint8_t> data_;
    std::vector<entry> entries_;
};

// Extracts every entry of `zip_file` below `destination`, creating directories
// as needed. Entries whose path would escape `destination` are rejected.
void extract_all(const std::filesystem::path& zip_file, const std::filesystem::path& destination);

// RFC 1951 "deflate" decompression of a raw (header-less) stream.
// `expected_size` is a hint used to reserve the output buffer; it may be 0.
std::vector<std::uint8_t> inflate(const std::uint8_t* data, std::size_t size, std::size_t expected_size = 0);

std::uint32_t crc32(const std::uint8_t* data, std::size_t size, std::uint32_t crc = 0);

} // namespace tinyzip

#endif // TINYZIP_HPP
