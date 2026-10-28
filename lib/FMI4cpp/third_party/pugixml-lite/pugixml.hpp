// pugixml-lite - a small, dependency-free XML DOM exposing the subset of the
// pugixml API that FMI4cpp uses to read modelDescription.xml.
//
// This is NOT the real pugixml (https://pugixml.org). It is bundled with
// FMI4cpp so the library builds offline without host packages. It keeps the
// `pugi` namespace and method names so FMI4cpp's parser compiles unchanged.
//
// Supported: UTF-8 documents, elements, attributes ('"' or '\''), text and
// CDATA content, comments, processing instructions, DOCTYPE (skipped), the
// predefined entities (&lt; &gt; &amp; &apos; &quot;) and numeric character
// references. Like pugixml's default options, whitespace-only text nodes are
// dropped.
//
// Not supported: UTF-16/32 input, DTD entity expansion, namespaces-aware
// queries, XPath, document mutation, saving.

#ifndef PUGIXML_LITE_HPP
#define PUGIXML_LITE_HPP

#include <cstddef>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

namespace pugi
{

namespace detail
{

struct attribute_data
{
    std::string name;
    std::string value;
};

struct node_data
{
    std::string name;  // empty for text (pcdata) nodes
    std::string value; // text content for pcdata nodes
    std::vector<attribute_data> attributes;
    std::vector<std::unique_ptr<node_data>> children;
};

} // namespace detail

enum xml_parse_status
{
    status_ok = 0,
    status_file_not_found,
    status_io_error,
    status_bad_start_element,
    status_bad_end_element,
    status_end_element_mismatch,
    status_bad_attribute,
    status_unrecognized_tag,
    status_bad_pi,
    status_bad_comment,
    status_bad_cdata,
    status_bad_doctype,
    status_bad_pcdata,
    status_no_document_element
};

struct xml_parse_result
{
    xml_parse_status status{status_ok};
    std::ptrdiff_t offset{0};

    explicit operator bool() const
    {
        return status == status_ok;
    }

    const char* description() const;
};

// Lightweight handle to an attribute owned by an xml_document.
class xml_attribute
{
public:
    xml_attribute() = default;
    explicit xml_attribute(const detail::attribute_data* p) : p_(p) {}

    bool empty() const
    {
        return p_ == nullptr;
    }
    explicit operator bool() const
    {
        return p_ != nullptr;
    }

    const char* name() const;
    const char* value() const;

    const char* as_string(const char* def = "") const;
    int as_int(int def = 0) const;
    unsigned int as_uint(unsigned int def = 0) const;
    long long as_llong(long long def = 0) const;
    unsigned long long as_ullong(unsigned long long def = 0) const;
    double as_double(double def = 0) const;
    float as_float(float def = 0) const;
    bool as_bool(bool def = false) const;

private:
    const detail::attribute_data* p_{nullptr};
};

class xml_node_iterator;

// Lightweight handle to a node owned by an xml_document.
class xml_node
{
public:
    using iterator = xml_node_iterator;

    xml_node() = default;
    explicit xml_node(const detail::node_data* p) : p_(p) {}

    bool empty() const
    {
        return p_ == nullptr;
    }
    explicit operator bool() const
    {
        return p_ != nullptr;
    }

    // Element name, or "" for text nodes and null handles.
    const char* name() const;
    // Text content for text nodes, "" otherwise.
    const char* value() const;
    // Text of the first text child, "" if none.
    const char* child_value() const;

    xml_attribute attribute(const char* name) const;
    xml_attribute first_attribute() const;
    std::vector<xml_attribute> attributes() const;

    xml_node child(const char* name) const;
    xml_node first_child() const;

    iterator begin() const;
    iterator end() const;

    bool operator==(const xml_node& o) const
    {
        return p_ == o.p_;
    }
    bool operator!=(const xml_node& o) const
    {
        return p_ != o.p_;
    }

private:
    friend class xml_node_iterator;
    const detail::node_data* p_{nullptr};
};

// Forward iterator over the children of a node (elements and text nodes).
class xml_node_iterator
{
public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = xml_node;
    using difference_type = std::ptrdiff_t;
    using pointer = const xml_node*;
    using reference = const xml_node&;

    xml_node_iterator() = default;
    xml_node_iterator(const detail::node_data* parent, std::size_t index) : parent_(parent), index_(index)
    {
        update();
    }

    const xml_node& operator*() const
    {
        return current_;
    }
    const xml_node* operator->() const
    {
        return &current_;
    }
    xml_node_iterator& operator++()
    {
        ++index_;
        update();
        return *this;
    }
    xml_node_iterator operator++(int)
    {
        xml_node_iterator tmp = *this;
        ++*this;
        return tmp;
    }
    bool operator==(const xml_node_iterator& o) const
    {
        return parent_ == o.parent_ && index_ == o.index_;
    }
    bool operator!=(const xml_node_iterator& o) const
    {
        return !(*this == o);
    }

private:
    void update()
    {
        current_ = (parent_ && index_ < parent_->children.size()) ? xml_node(parent_->children[index_].get()) : xml_node();
    }

    const detail::node_data* parent_{nullptr};
    std::size_t index_{0};
    xml_node current_;
};

inline xml_node::iterator xml_node::begin() const
{
    return iterator(p_, 0);
}

inline xml_node::iterator xml_node::end() const
{
    return iterator(p_, p_ ? p_->children.size() : 0);
}

class xml_document
{
public:
    xml_document() = default;
    xml_document(const xml_document&) = delete;
    xml_document& operator=(const xml_document&) = delete;

    xml_parse_result load_file(const char* path);
    xml_parse_result load_string(const char* contents);
    xml_parse_result load_buffer(const void* contents, std::size_t size);

    void reset();

    // Document-level node accessors (the document acts as the root node).
    xml_node root() const
    {
        return xml_node(root_.get());
    }
    xml_node child(const char* name) const
    {
        return root().child(name);
    }
    xml_node first_child() const
    {
        return root().first_child();
    }
    xml_node document_element() const
    {
        return root().first_child();
    }
    xml_node::iterator begin() const
    {
        return root().begin();
    }
    xml_node::iterator end() const
    {
        return root().end();
    }

private:
    std::unique_ptr<detail::node_data> root_;
};

} // namespace pugi

#endif // PUGIXML_LITE_HPP
