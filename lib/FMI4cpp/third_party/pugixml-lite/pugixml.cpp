#include "pugixml.hpp"

#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <string_view>

namespace pugi
{

// =============================================================================
// Parser
// =============================================================================

namespace
{

struct parse_error
{
    xml_parse_status status;
    std::size_t offset;
};

bool is_space(char c)
{
    return c == ' ' || c == '\t' || c == '\n' || c == '\r';
}

bool is_name_start(unsigned char c)
{
    return std::isalpha(c) || c == '_' || c == ':' || c >= 0x80;
}

bool is_name_char(unsigned char c)
{
    return is_name_start(c) || std::isdigit(c) || c == '-' || c == '.';
}

void append_utf8(std::string& out, unsigned long cp)
{
    if (cp < 0x80) {
        out += static_cast<char>(cp);
    } else if (cp < 0x800) {
        out += static_cast<char>(0xC0 | (cp >> 6));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    } else if (cp < 0x10000) {
        out += static_cast<char>(0xE0 | (cp >> 12));
        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    } else {
        out += static_cast<char>(0xF0 | (cp >> 18));
        out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    }
}

class parser
{
public:
    parser(const char* text, std::size_t size) : s_(text), n_(size) {}

    void parse(detail::node_data& root)
    {
        // Skip a UTF-8 byte order mark.
        if (n_ >= 3 && static_cast<unsigned char>(s_[0]) == 0xEF && static_cast<unsigned char>(s_[1]) == 0xBB &&
            static_cast<unsigned char>(s_[2]) == 0xBF) {
            pos_ = 3;
        }
        parse_content(root, /*is_document=*/true);
        if (pos_ != n_) {
            throw parse_error{status_unrecognized_tag, pos_};
        }
        bool has_element = false;
        for (const auto& child : root.children) {
            if (!child->name.empty()) {
                has_element = true;
            }
        }
        if (!has_element) {
            throw parse_error{status_no_document_element, pos_};
        }
    }

private:
    bool starts_with(const char* lit) const
    {
        const std::size_t len = std::strlen(lit);
        return n_ - pos_ >= len && std::memcmp(s_ + pos_, lit, len) == 0;
    }

    void skip_space()
    {
        while (pos_ < n_ && is_space(s_[pos_])) {
            ++pos_;
        }
    }

    // Advances past `terminator`, or throws `status`.
    void skip_until(const char* terminator, xml_parse_status status)
    {
        const std::size_t len = std::strlen(terminator);
        for (; pos_ + len <= n_; ++pos_) {
            if (std::memcmp(s_ + pos_, terminator, len) == 0) {
                pos_ += len;
                return;
            }
        }
        throw parse_error{status, pos_};
    }

    std::string parse_name(xml_parse_status status)
    {
        const std::size_t start = pos_;
        if (pos_ >= n_ || !is_name_start(static_cast<unsigned char>(s_[pos_]))) {
            throw parse_error{status, pos_};
        }
        while (pos_ < n_ && is_name_char(static_cast<unsigned char>(s_[pos_]))) {
            ++pos_;
        }
        return std::string(s_ + start, pos_ - start);
    }

    // Decodes "&...;" starting at pos_ (which points at '&').
    void parse_reference(std::string& out, xml_parse_status status)
    {
        const std::size_t semi = std::string_view(s_ + pos_, n_ - pos_).find(';');
        if (semi == std::string_view::npos || semi > 16) {
            throw parse_error{status, pos_};
        }
        const std::string_view ref(s_ + pos_ + 1, semi - 1);
        if (ref == "lt") {
            out += '<';
        } else if (ref == "gt") {
            out += '>';
        } else if (ref == "amp") {
            out += '&';
        } else if (ref == "apos") {
            out += '\'';
        } else if (ref == "quot") {
            out += '"';
        } else if (!ref.empty() && ref[0] == '#') {
            const std::string digits(ref.substr(ref.size() > 1 && (ref[1] == 'x' || ref[1] == 'X') ? 2 : 1));
            const int base = (ref.size() > 1 && (ref[1] == 'x' || ref[1] == 'X')) ? 16 : 10;
            char* end = nullptr;
            const unsigned long cp = std::strtoul(digits.c_str(), &end, base);
            if (digits.empty() || *end != '\0' || cp > 0x10FFFF) {
                throw parse_error{status, pos_};
            }
            append_utf8(out, cp);
        } else {
            // Unknown entity (would need a DTD): keep it verbatim, like a
            // non-validating parser without entity expansion would.
            out.append(s_ + pos_, semi + 1);
        }
        pos_ += semi + 1;
    }

    // Parses attribute value text up to `quote`, normalising line endings and
    // decoding references.
    std::string parse_attribute_value(char quote)
    {
        std::string value;
        for (;;) {
            if (pos_ >= n_) {
                throw parse_error{status_bad_attribute, pos_};
            }
            const char c = s_[pos_];
            if (c == quote) {
                ++pos_;
                return value;
            }
            if (c == '<') {
                throw parse_error{status_bad_attribute, pos_};
            }
            if (c == '&') {
                parse_reference(value, status_bad_attribute);
            } else if (c == '\r') {
                value += ' ';
                ++pos_;
                if (pos_ < n_ && s_[pos_] == '\n') {
                    ++pos_;
                }
            } else if (c == '\n' || c == '\t') {
                value += ' ';
                ++pos_;
            } else {
                value += c;
                ++pos_;
            }
        }
    }

    void parse_element(detail::node_data& parent)
    {
        // pos_ is just past '<'.
        auto node = std::make_unique<detail::node_data>();
        node->name = parse_name(status_bad_start_element);

        for (;;) {
            skip_space();
            if (pos_ >= n_) {
                throw parse_error{status_bad_start_element, pos_};
            }
            const char c = s_[pos_];
            if (c == '>') {
                ++pos_;
                break;
            }
            if (c == '/') {
                ++pos_;
                if (pos_ >= n_ || s_[pos_] != '>') {
                    throw parse_error{status_bad_start_element, pos_};
                }
                ++pos_;
                parent.children.push_back(std::move(node));
                return;
            }

            detail::attribute_data attr;
            attr.name = parse_name(status_bad_attribute);
            skip_space();
            if (pos_ >= n_ || s_[pos_] != '=') {
                throw parse_error{status_bad_attribute, pos_};
            }
            ++pos_;
            skip_space();
            if (pos_ >= n_ || (s_[pos_] != '"' && s_[pos_] != '\'')) {
                throw parse_error{status_bad_attribute, pos_};
            }
            const char quote = s_[pos_++];
            attr.value = parse_attribute_value(quote);
            node->attributes.push_back(std::move(attr));
        }

        parse_content(*node, /*is_document=*/false);

        // pos_ is just past "</".
        const std::size_t name_pos = pos_;
        const std::string end_name = parse_name(status_bad_end_element);
        if (end_name != node->name) {
            throw parse_error{status_end_element_mismatch, name_pos};
        }
        skip_space();
        if (pos_ >= n_ || s_[pos_] != '>') {
            throw parse_error{status_bad_end_element, pos_};
        }
        ++pos_;
        parent.children.push_back(std::move(node));
    }

    void flush_text(detail::node_data& parent, std::string& text)
    {
        bool only_space = true;
        for (char c : text) {
            if (!is_space(c)) {
                only_space = false;
                break;
            }
        }
        if (!only_space) {
            auto node = std::make_unique<detail::node_data>();
            node->value = std::move(text);
            parent.children.push_back(std::move(node));
        }
        text.clear();
    }

    // Parses children of `parent` until the matching end tag (or end of input
    // for the document).
    void parse_content(detail::node_data& parent, bool is_document)
    {
        std::string text;
        while (pos_ < n_) {
            const char c = s_[pos_];
            if (c != '<') {
                if (c == '&') {
                    parse_reference(text, status_bad_pcdata);
                } else if (c == '\r') {
                    text += '\n';
                    ++pos_;
                    if (pos_ < n_ && s_[pos_] == '\n') {
                        ++pos_;
                    }
                } else {
                    text += c;
                    ++pos_;
                }
                continue;
            }

            if (starts_with("</")) {
                if (is_document) {
                    throw parse_error{status_bad_end_element, pos_};
                }
                flush_text(parent, text);
                pos_ += 2;
                return;
            }
            if (starts_with("<!--")) {
                pos_ += 4;
                skip_until("-->", status_bad_comment);
            } else if (starts_with("<![CDATA[")) {
                pos_ += 9;
                const std::size_t start = pos_;
                skip_until("]]>", status_bad_cdata);
                text.append(s_ + start, pos_ - 3 - start);
            } else if (starts_with("<!DOCTYPE")) {
                parse_doctype();
            } else if (starts_with("<?")) {
                pos_ += 2;
                skip_until("?>", status_bad_pi);
            } else if (starts_with("<!")) {
                throw parse_error{status_unrecognized_tag, pos_};
            } else {
                flush_text(parent, text);
                ++pos_;
                parse_element(parent);
            }
        }
        if (!is_document) {
            throw parse_error{status_end_element_mismatch, pos_};
        }
        flush_text(parent, text);
    }

    void parse_doctype()
    {
        // Skip "<!DOCTYPE ... >" including an optional internal subset "[...]".
        int depth = 0;
        for (; pos_ < n_; ++pos_) {
            const char c = s_[pos_];
            if (starts_with("<!--")) {
                pos_ += 4;
                skip_until("-->", status_bad_doctype);
                --pos_; // compensate for the loop increment
            } else if (starts_with("<?")) {
                pos_ += 2;
                skip_until("?>", status_bad_doctype);
                --pos_;
            } else if (c == '[') {
                ++depth;
            } else if (c == ']') {
                --depth;
            } else if (c == '>' && depth <= 0) {
                ++pos_;
                return;
            } else if (c == '"' || c == '\'') {
                const char quote = c;
                ++pos_;
                while (pos_ < n_ && s_[pos_] != quote) {
                    ++pos_;
                }
            }
        }
        throw parse_error{status_bad_doctype, pos_};
    }

    const char* s_;
    std::size_t n_;
    std::size_t pos_{0};
};

} // namespace

// =============================================================================
// xml_parse_result
// =============================================================================

const char* xml_parse_result::description() const
{
    switch (status) {
        case status_ok: return "No error";
        case status_file_not_found: return "File was not found";
        case status_io_error: return "Error reading from file";
        case status_bad_start_element: return "Error parsing start element tag";
        case status_bad_end_element: return "Error parsing end element tag";
        case status_end_element_mismatch: return "Start-end tags mismatch";
        case status_bad_attribute: return "Error parsing element attribute";
        case status_unrecognized_tag: return "Could not determine tag type";
        case status_bad_pi: return "Error parsing processing instruction";
        case status_bad_comment: return "Error parsing comment";
        case status_bad_cdata: return "Error parsing CDATA section";
        case status_bad_doctype: return "Error parsing document type declaration";
        case status_bad_pcdata: return "Error parsing PCDATA section";
        case status_no_document_element: return "No document element found";
    }
    return "Unknown error";
}

// =============================================================================
// xml_attribute
// =============================================================================

namespace
{

template<typename T>
T clamp_ll(long long v)
{
    if (v < static_cast<long long>(std::numeric_limits<T>::min())) {
        return std::numeric_limits<T>::min();
    }
    if (v > static_cast<long long>(std::numeric_limits<T>::max())) {
        return std::numeric_limits<T>::max();
    }
    return static_cast<T>(v);
}

} // namespace

const char* xml_attribute::name() const
{
    return p_ ? p_->name.c_str() : "";
}

const char* xml_attribute::value() const
{
    return p_ ? p_->value.c_str() : "";
}

const char* xml_attribute::as_string(const char* def) const
{
    return p_ ? p_->value.c_str() : def;
}

long long xml_attribute::as_llong(long long def) const
{
    if (!p_) {
        return def;
    }
    errno = 0;
    char* end = nullptr;
    const long long v = std::strtoll(p_->value.c_str(), &end, 10);
    if (end == p_->value.c_str()) {
        return def;
    }
    return v;
}

unsigned long long xml_attribute::as_ullong(unsigned long long def) const
{
    if (!p_) {
        return def;
    }
    const char* text = p_->value.c_str();
    while (is_space(*text)) {
        ++text;
    }
    if (*text == '-') {
        return 0; // negative values clamp to 0, as in pugixml
    }
    char* end = nullptr;
    const unsigned long long v = std::strtoull(text, &end, 10);
    if (end == text) {
        return def;
    }
    return v;
}

int xml_attribute::as_int(int def) const
{
    return p_ ? clamp_ll<int>(as_llong(def)) : def;
}

unsigned int xml_attribute::as_uint(unsigned int def) const
{
    if (!p_) {
        return def;
    }
    const unsigned long long v = as_ullong(def);
    return v > std::numeric_limits<unsigned int>::max() ? std::numeric_limits<unsigned int>::max()
                                                        : static_cast<unsigned int>(v);
}

double xml_attribute::as_double(double def) const
{
    if (!p_) {
        return def;
    }
    char* end = nullptr;
    const double v = std::strtod(p_->value.c_str(), &end);
    if (end == p_->value.c_str()) {
        return def;
    }
    return v;
}

float xml_attribute::as_float(float def) const
{
    return p_ ? static_cast<float>(as_double(def)) : def;
}

bool xml_attribute::as_bool(bool def) const
{
    if (!p_) {
        return def;
    }
    // pugixml semantics: true iff the first character is one of "1tTyY".
    const char c = p_->value.empty() ? '\0' : p_->value[0];
    return c == '1' || c == 't' || c == 'T' || c == 'y' || c == 'Y';
}

// =============================================================================
// xml_node
// =============================================================================

const char* xml_node::name() const
{
    return p_ ? p_->name.c_str() : "";
}

const char* xml_node::value() const
{
    return p_ ? p_->value.c_str() : "";
}

const char* xml_node::child_value() const
{
    if (p_) {
        for (const auto& child : p_->children) {
            if (child->name.empty()) {
                return child->value.c_str();
            }
        }
    }
    return "";
}

xml_attribute xml_node::attribute(const char* name) const
{
    if (p_ && name) {
        for (const auto& attr : p_->attributes) {
            if (attr.name == name) {
                return xml_attribute(&attr);
            }
        }
    }
    return xml_attribute();
}

xml_attribute xml_node::first_attribute() const
{
    return (p_ && !p_->attributes.empty()) ? xml_attribute(&p_->attributes.front()) : xml_attribute();
}

std::vector<xml_attribute> xml_node::attributes() const
{
    std::vector<xml_attribute> result;
    if (p_) {
        result.reserve(p_->attributes.size());
        for (const auto& attr : p_->attributes) {
            result.emplace_back(&attr);
        }
    }
    return result;
}

xml_node xml_node::child(const char* name) const
{
    if (p_ && name) {
        for (const auto& child : p_->children) {
            if (child->name == name) {
                return xml_node(child.get());
            }
        }
    }
    return xml_node();
}

xml_node xml_node::first_child() const
{
    return (p_ && !p_->children.empty()) ? xml_node(p_->children.front().get()) : xml_node();
}

// =============================================================================
// xml_document
// =============================================================================

void xml_document::reset()
{
    root_.reset();
}

xml_parse_result xml_document::load_buffer(const void* contents, std::size_t size)
{
    reset();
    auto root = std::make_unique<detail::node_data>();
    xml_parse_result result;
    try {
        parser(static_cast<const char*>(contents), size).parse(*root);
        root_ = std::move(root);
    } catch (const parse_error& e) {
        result.status = e.status;
        result.offset = static_cast<std::ptrdiff_t>(e.offset);
    }
    return result;
}

xml_parse_result xml_document::load_string(const char* contents)
{
    return load_buffer(contents, contents ? std::strlen(contents) : 0);
}

xml_parse_result xml_document::load_file(const char* path)
{
    reset();
    xml_parse_result result;
    std::ifstream file(path ? path : "", std::ios::binary);
    if (!file) {
        result.status = status_file_not_found;
        return result;
    }
    const std::string contents((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    if (file.bad()) {
        result.status = status_io_error;
        return result;
    }
    return load_buffer(contents.data(), contents.size());
}

} // namespace pugi
