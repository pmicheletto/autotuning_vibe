#include "tinyzip.hpp"

#include <array>
#include <fstream>
#include <stdexcept>

namespace tinyzip
{

// =============================================================================
// CRC-32 (IEEE 802.3, as used by ZIP)
// =============================================================================

namespace
{

std::array<std::uint32_t, 256> make_crc_table()
{
    std::array<std::uint32_t, 256> table{};
    for (std::uint32_t n = 0; n < 256; ++n) {
        std::uint32_t c = n;
        for (int k = 0; k < 8; ++k) {
            c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        }
        table[n] = c;
    }
    return table;
}

} // namespace

std::uint32_t crc32(const std::uint8_t* data, std::size_t size, std::uint32_t crc)
{
    static const std::array<std::uint32_t, 256> table = make_crc_table();
    crc = ~crc;
    for (std::size_t i = 0; i < size; ++i) {
        crc = table[(crc ^ data[i]) & 0xFFu] ^ (crc >> 8);
    }
    return ~crc;
}

// =============================================================================
// Inflate (RFC 1951). Structure follows Mark Adler's reference "puff".
// =============================================================================

namespace
{

constexpr int kMaxBits = 15;      // max bits in a Huffman code
constexpr int kMaxLitLenCodes = 288;
constexpr int kMaxDistCodes = 30;
constexpr int kMaxCodes = kMaxLitLenCodes + kMaxDistCodes;
constexpr int kFixedLitLenCodes = 288;

class bit_reader
{
public:
    bit_reader(const std::uint8_t* data, std::size_t size) : data_(data), size_(size) {}

    // Reads `n` (0..32) bits, least-significant bit first.
    std::uint32_t bits(int n)
    {
        while (bit_count_ < n) {
            if (pos_ >= size_) {
                throw std::runtime_error("tinyzip: unexpected end of deflate stream");
            }
            bit_buf_ |= static_cast<std::uint64_t>(data_[pos_++]) << bit_count_;
            bit_count_ += 8;
        }
        const std::uint32_t value = static_cast<std::uint32_t>(bit_buf_ & ((1ull << n) - 1));
        bit_buf_ >>= n;
        bit_count_ -= n;
        return value;
    }

    // Discards bits up to the next byte boundary.
    void align_to_byte()
    {
        bit_buf_ = 0;
        bit_count_ = 0;
    }

    std::size_t position() const
    {
        return pos_;
    }

    void skip(std::size_t n)
    {
        pos_ += n;
    }

    const std::uint8_t* current() const
    {
        return data_ + pos_;
    }

    std::size_t remaining() const
    {
        return size_ - pos_;
    }

private:
    const std::uint8_t* data_;
    std::size_t size_;
    std::size_t pos_{0};
    std::uint64_t bit_buf_{0};
    int bit_count_{0};
};

// Canonical Huffman decoding table: number of codes per length + symbols
// ordered by code.
struct huffman
{
    std::array<std::uint16_t, kMaxBits + 1> count{};
    std::array<std::uint16_t, kMaxCodes> symbol{};
};

// Builds a decoding table from code lengths. Returns 0 for a complete code,
// a negative value for an over-subscribed (invalid) set of lengths and a
// positive value for an incomplete code.
int build_huffman(huffman& h, const std::uint16_t* lengths, int n)
{
    h.count.fill(0);
    for (int sym = 0; sym < n; ++sym) {
        ++h.count[lengths[sym]];
    }
    if (h.count[0] == n) {
        return 0; // no codes at all; decoding will fail if attempted
    }

    int left = 1;
    for (int len = 1; len <= kMaxBits; ++len) {
        left <<= 1;
        left -= h.count[len];
        if (left < 0) {
            return left;
        }
    }

    std::array<std::uint16_t, kMaxBits + 1> offs{};
    for (int len = 1; len < kMaxBits; ++len) {
        offs[len + 1] = static_cast<std::uint16_t>(offs[len] + h.count[len]);
    }
    for (int sym = 0; sym < n; ++sym) {
        if (lengths[sym] != 0) {
            h.symbol[offs[lengths[sym]]++] = static_cast<std::uint16_t>(sym);
        }
    }
    return left;
}

int decode_symbol(bit_reader& in, const huffman& h)
{
    int code = 0;  // code bits so far
    int first = 0; // first code of the current length
    int index = 0; // index of first symbol of the current length
    for (int len = 1; len <= kMaxBits; ++len) {
        code |= static_cast<int>(in.bits(1));
        const int count = h.count[len];
        if (code - count < first) {
            return h.symbol[index + (code - first)];
        }
        index += count;
        first += count;
        first <<= 1;
        code <<= 1;
    }
    throw std::runtime_error("tinyzip: invalid Huffman code in deflate stream");
}

constexpr std::uint16_t kLengthBase[29] = {
    3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31,
    35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258};
constexpr std::uint16_t kLengthExtra[29] = {
    0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2,
    3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0};
constexpr std::uint16_t kDistBase[30] = {
    1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193,
    257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577};
constexpr std::uint16_t kDistExtra[30] = {
    0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6,
    7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13};

void inflate_codes(bit_reader& in, std::vector<std::uint8_t>& out, const huffman& lencode, const huffman& distcode)
{
    for (;;) {
        int symbol = decode_symbol(in, lencode);
        if (symbol < 256) {
            out.push_back(static_cast<std::uint8_t>(symbol));
        } else if (symbol == 256) {
            return; // end of block
        } else {
            symbol -= 257;
            if (symbol >= 29) {
                throw std::runtime_error("tinyzip: invalid length symbol in deflate stream");
            }
            const std::size_t len = kLengthBase[symbol] + in.bits(kLengthExtra[symbol]);

            const int dsym = decode_symbol(in, distcode);
            if (dsym >= 30) {
                throw std::runtime_error("tinyzip: invalid distance symbol in deflate stream");
            }
            const std::size_t dist = kDistBase[dsym] + in.bits(kDistExtra[dsym]);
            if (dist > out.size()) {
                throw std::runtime_error("tinyzip: deflate distance too far back");
            }

            // Byte-by-byte copy: source and destination may overlap.
            std::size_t from = out.size() - dist;
            for (std::size_t i = 0; i < len; ++i) {
                out.push_back(out[from++]);
            }
        }
    }
}

void inflate_stored(bit_reader& in, std::vector<std::uint8_t>& out)
{
    in.align_to_byte();
    if (in.remaining() < 4) {
        throw std::runtime_error("tinyzip: truncated stored block");
    }
    const std::uint8_t* p = in.current();
    const unsigned len = p[0] | (p[1] << 8);
    const unsigned nlen = p[2] | (p[3] << 8);
    if (len != (~nlen & 0xFFFFu)) {
        throw std::runtime_error("tinyzip: stored block length check failed");
    }
    in.skip(4);
    if (in.remaining() < len) {
        throw std::runtime_error("tinyzip: truncated stored block");
    }
    out.insert(out.end(), in.current(), in.current() + len);
    in.skip(len);
}

void inflate_fixed(bit_reader& in, std::vector<std::uint8_t>& out)
{
    static const auto tables = [] {
        std::pair<huffman, huffman> t;
        std::uint16_t lengths[kFixedLitLenCodes];
        int sym = 0;
        for (; sym < 144; ++sym) lengths[sym] = 8;
        for (; sym < 256; ++sym) lengths[sym] = 9;
        for (; sym < 280; ++sym) lengths[sym] = 7;
        for (; sym < kFixedLitLenCodes; ++sym) lengths[sym] = 8;
        build_huffman(t.first, lengths, kFixedLitLenCodes);

        for (sym = 0; sym < kMaxDistCodes; ++sym) lengths[sym] = 5;
        build_huffman(t.second, lengths, kMaxDistCodes);
        return t;
    }();
    inflate_codes(in, out, tables.first, tables.second);
}

void inflate_dynamic(bit_reader& in, std::vector<std::uint8_t>& out)
{
    static constexpr std::uint8_t kOrder[19] = {16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15};

    const int nlen = static_cast<int>(in.bits(5)) + 257;
    const int ndist = static_cast<int>(in.bits(5)) + 1;
    const int ncode = static_cast<int>(in.bits(4)) + 4;
    if (nlen > kMaxLitLenCodes - 2 || ndist > kMaxDistCodes) {
        throw std::runtime_error("tinyzip: bad dynamic block counts");
    }

    std::uint16_t lengths[kMaxCodes] = {};
    for (int i = 0; i < ncode; ++i) {
        lengths[kOrder[i]] = static_cast<std::uint16_t>(in.bits(3));
    }
    huffman lencode;
    if (build_huffman(lencode, lengths, 19) != 0) {
        throw std::runtime_error("tinyzip: invalid code-length code");
    }

    int index = 0;
    while (index < nlen + ndist) {
        int symbol = decode_symbol(in, lencode);
        if (symbol < 16) {
            lengths[index++] = static_cast<std::uint16_t>(symbol);
        } else {
            std::uint16_t len = 0;
            int repeat;
            if (symbol == 16) {
                if (index == 0) {
                    throw std::runtime_error("tinyzip: repeat with no previous length");
                }
                len = lengths[index - 1];
                repeat = 3 + static_cast<int>(in.bits(2));
            } else if (symbol == 17) {
                repeat = 3 + static_cast<int>(in.bits(3));
            } else {
                repeat = 11 + static_cast<int>(in.bits(7));
            }
            if (index + repeat > nlen + ndist) {
                throw std::runtime_error("tinyzip: too many code lengths");
            }
            while (repeat--) {
                lengths[index++] = len;
            }
        }
    }
    if (lengths[256] == 0) {
        throw std::runtime_error("tinyzip: missing end-of-block code");
    }

    int err = build_huffman(lencode, lengths, nlen);
    if (err < 0 || (err > 0 && nlen - lencode.count[0] != 1)) {
        throw std::runtime_error("tinyzip: invalid literal/length code");
    }
    huffman distcode;
    err = build_huffman(distcode, lengths + nlen, ndist);
    if (err < 0 || (err > 0 && ndist - distcode.count[0] != 1)) {
        throw std::runtime_error("tinyzip: invalid distance code");
    }

    inflate_codes(in, out, lencode, distcode);
}

} // namespace

std::vector<std::uint8_t> inflate(const std::uint8_t* data, std::size_t size, std::size_t expected_size)
{
    std::vector<std::uint8_t> out;
    out.reserve(expected_size);
    bit_reader in(data, size);

    bool last = false;
    while (!last) {
        last = in.bits(1) != 0;
        const std::uint32_t type = in.bits(2);
        switch (type) {
            case 0: inflate_stored(in, out); break;
            case 1: inflate_fixed(in, out); break;
            case 2: inflate_dynamic(in, out); break;
            default: throw std::runtime_error("tinyzip: invalid deflate block type");
        }
    }
    return out;
}

// =============================================================================
// ZIP container
// =============================================================================

namespace
{

constexpr std::uint32_t kLocalHeaderSig = 0x04034b50;
constexpr std::uint32_t kCentralDirSig = 0x02014b50;
constexpr std::uint32_t kEndOfCentralDirSig = 0x06054b50;
constexpr std::size_t kEndOfCentralDirSize = 22;
constexpr std::size_t kMaxCommentSize = 0xFFFF;

constexpr std::uint32_t kMethodStored = 0;
constexpr std::uint32_t kMethodDeflate = 8;
constexpr std::uint16_t kFlagEncrypted = 1u << 0;

std::uint16_t read_u16(const std::uint8_t* p)
{
    return static_cast<std::uint16_t>(p[0] | (p[1] << 8));
}

std::uint32_t read_u32(const std::uint8_t* p)
{
    return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
        (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}

std::vector<std::uint8_t> read_file(const std::filesystem::path& path)
{
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        throw std::runtime_error("tinyzip: cannot open '" + path.string() + "'");
    }
    return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
}

// Rejects absolute paths and any ".." component so that extraction cannot
// write outside the destination directory.
bool is_safe_relative_path(const std::string& name)
{
    if (name.empty() || name.front() == '/' || name.front() == '\\') {
        return false;
    }
    if (name.size() > 1 && name[1] == ':') {
        return false; // Windows drive letter
    }
    std::size_t start = 0;
    while (start <= name.size()) {
        const std::size_t end = name.find_first_of("/\\", start);
        const std::string component = name.substr(start, end == std::string::npos ? std::string::npos : end - start);
        if (component == "..") {
            return false;
        }
        if (end == std::string::npos) {
            break;
        }
        start = end + 1;
    }
    return true;
}

} // namespace

archive::archive(const std::filesystem::path& path) : data_(read_file(path))
{
    // Locate the end-of-central-directory record (scan backwards past an
    // optional archive comment).
    if (data_.size() < kEndOfCentralDirSize) {
        throw std::runtime_error("tinyzip: '" + path.string() + "' is not a zip archive");
    }
    const std::size_t scan_start = data_.size() - kEndOfCentralDirSize;
    const std::size_t scan_stop = scan_start > kMaxCommentSize ? scan_start - kMaxCommentSize : 0;
    std::size_t eocd = std::string::npos;
    for (std::size_t i = scan_start + 1; i-- > scan_stop;) {
        if (read_u32(&data_[i]) == kEndOfCentralDirSig) {
            eocd = i;
            break;
        }
    }
    if (eocd == std::string::npos) {
        throw std::runtime_error("tinyzip: '" + path.string() + "' is not a zip archive (no end of central directory)");
    }

    const std::uint8_t* e = &data_[eocd];
    const std::uint16_t disk_number = read_u16(e + 4);
    const std::uint16_t disk_with_cd = read_u16(e + 6);
    const std::uint16_t total_entries = read_u16(e + 10);
    const std::uint32_t cd_size = read_u32(e + 12);
    const std::uint32_t cd_offset = read_u32(e + 16);

    if (disk_number != 0 || disk_with_cd != 0) {
        throw std::runtime_error("tinyzip: multi-disk archives are not supported");
    }
    if (total_entries == 0xFFFF || cd_size == 0xFFFFFFFFu || cd_offset == 0xFFFFFFFFu) {
        throw std::runtime_error("tinyzip: ZIP64 archives are not supported");
    }
    if (static_cast<std::uint64_t>(cd_offset) + cd_size > data_.size()) {
        throw std::runtime_error("tinyzip: central directory is out of bounds");
    }

    // Walk the central directory.
    std::size_t pos = cd_offset;
    const std::size_t cd_end = cd_offset + cd_size;
    entries_.reserve(total_entries);
    for (std::uint16_t i = 0; i < total_entries; ++i) {
        if (pos + 46 > cd_end || read_u32(&data_[pos]) != kCentralDirSig) {
            throw std::runtime_error("tinyzip: corrupt central directory");
        }
        const std::uint8_t* h = &data_[pos];
        const std::uint16_t flags = read_u16(h + 8);
        const std::uint16_t name_len = read_u16(h + 28);
        const std::uint16_t extra_len = read_u16(h + 30);
        const std::uint16_t comment_len = read_u16(h + 32);
        if (pos + 46 + name_len + extra_len + comment_len > cd_end) {
            throw std::runtime_error("tinyzip: corrupt central directory");
        }

        entry ent;
        ent.name.assign(reinterpret_cast<const char*>(h + 46), name_len);
        ent.method = read_u16(h + 10);
        ent.crc32 = read_u32(h + 16);
        ent.compressed_size = read_u32(h + 20);
        ent.uncompressed_size = read_u32(h + 24);
        ent.local_header_offset = read_u32(h + 42);

        if (flags & kFlagEncrypted) {
            throw std::runtime_error("tinyzip: encrypted entry '" + ent.name + "' is not supported");
        }
        if (ent.compressed_size == 0xFFFFFFFFu || ent.uncompressed_size == 0xFFFFFFFFu ||
            ent.local_header_offset == 0xFFFFFFFFu) {
            throw std::runtime_error("tinyzip: ZIP64 entry '" + ent.name + "' is not supported");
        }

        entries_.push_back(std::move(ent));
        pos += 46 + name_len + extra_len + comment_len;
    }
}

std::vector<std::uint8_t> archive::read(const entry& e) const
{
    if (e.local_header_offset + 30 > data_.size() || read_u32(&data_[e.local_header_offset]) != kLocalHeaderSig) {
        throw std::runtime_error("tinyzip: corrupt local header for '" + e.name + "'");
    }
    const std::uint8_t* h = &data_[e.local_header_offset];
    // Name/extra lengths in the local header may differ from the central
    // directory, so they must be read from here. Sizes and CRC come from the
    // central directory (local values may be zero when a data descriptor is
    // used).
    const std::uint16_t name_len = read_u16(h + 26);
    const std::uint16_t extra_len = read_u16(h + 28);
    const std::uint64_t data_start = e.local_header_offset + 30 + name_len + extra_len;
    if (data_start + e.compressed_size > data_.size()) {
        throw std::runtime_error("tinyzip: entry '" + e.name + "' is out of bounds");
    }
    const std::uint8_t* src = &data_[data_start];

    std::vector<std::uint8_t> out;
    if (e.method == kMethodStored) {
        out.assign(src, src + e.compressed_size);
    } else if (e.method == kMethodDeflate) {
        out = inflate(src, e.compressed_size, e.uncompressed_size);
    } else {
        throw std::runtime_error("tinyzip: unsupported compression method " + std::to_string(e.method) +
            " for '" + e.name + "'");
    }

    if (out.size() != e.uncompressed_size) {
        throw std::runtime_error("tinyzip: size mismatch for '" + e.name + "'");
    }
    if (crc32(out.data(), out.size()) != e.crc32) {
        throw std::runtime_error("tinyzip: CRC mismatch for '" + e.name + "'");
    }
    return out;
}

void extract_all(const std::filesystem::path& zip_file, const std::filesystem::path& destination)
{
    const archive zip(zip_file);
    for (const entry& e : zip.entries()) {
        if (!is_safe_relative_path(e.name)) {
            throw std::runtime_error("tinyzip: refusing to extract unsafe path '" + e.name + "'");
        }
        const std::filesystem::path target = destination / e.name;
        if (e.is_directory()) {
            std::filesystem::create_directories(target);
            continue;
        }
        std::filesystem::create_directories(target.parent_path());

        const std::vector<std::uint8_t> content = zip.read(e);
        std::ofstream file(target, std::ios::binary | std::ios::trunc);
        if (!file) {
            throw std::runtime_error("tinyzip: cannot create '" + target.string() + "'");
        }
        file.write(reinterpret_cast<const char*>(content.data()), static_cast<std::streamsize>(content.size()));
        if (!file) {
            throw std::runtime_error("tinyzip: failed writing '" + target.string() + "'");
        }
    }
}

} // namespace tinyzip
