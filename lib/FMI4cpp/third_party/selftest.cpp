// Self-test for the bundled tinyzip and pugixml-lite implementations.
// Usage: fmi4cpp_third_party_selftest <file.fmu>
//
// Extracts the FMU to a temporary directory (tinyzip verifies the CRC-32 of
// every entry) and parses its modelDescription.xml, checking the fields that
// FMI4cpp relies on.

#include <pugixml.hpp>
#include <tinyzip.hpp>

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

namespace fs = std::filesystem;

namespace
{

int fail(const std::string& what)
{
    std::cerr << "FAIL: " << what << '\n';
    return EXIT_FAILURE;
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "usage: " << argv[0] << " <file.fmu>\n";
        return EXIT_FAILURE;
    }
    const fs::path fmu = argv[1];
    const fs::path tmp = fs::temp_directory_path() / ("fmi4cpp_selftest_" + std::to_string(std::rand()));

    try {
        tinyzip::extract_all(fmu, tmp);

        const tinyzip::archive archive(fmu);
        std::size_t files = 0;
        for (const tinyzip::entry& e : archive.entries()) {
            if (!e.is_directory()) {
                ++files;
                if (!fs::exists(tmp / e.name) || fs::file_size(tmp / e.name) != e.uncompressed_size) {
                    return fail("extracted size mismatch for " + e.name);
                }
            }
        }
        if (!fs::exists(tmp / "modelDescription.xml")) {
            return fail("modelDescription.xml missing after extraction");
        }

        pugi::xml_document doc;
        const pugi::xml_parse_result result = doc.load_file((tmp / "modelDescription.xml").string().c_str());
        if (!result) {
            return fail(std::string("modelDescription.xml: ") + result.description());
        }
        const pugi::xml_node root = doc.child("fmiModelDescription");
        if (!root) {
            return fail("no fmiModelDescription element");
        }
        if (std::string(root.attribute("fmiVersion").as_string()) != "2.0") {
            return fail("unexpected fmiVersion");
        }
        if (root.attribute("guid").empty() || root.attribute("modelName").empty()) {
            return fail("guid/modelName missing");
        }
        if (!root.attribute("doesNotExist").empty() || root.attribute("doesNotExist").as_string("dflt") != std::string("dflt")) {
            return fail("missing attribute handling");
        }

        std::size_t variables = 0;
        for (const pugi::xml_node& v : root.child("ModelVariables")) {
            if (std::string("ScalarVariable") == v.name()) {
                ++variables;
                if (v.attribute("name").empty() || v.attribute("valueReference").empty()) {
                    return fail("ScalarVariable without name/valueReference");
                }
            }
        }
        if (variables == 0) {
            return fail("no ScalarVariable parsed");
        }

        std::cout << "OK: " << files << " files extracted, model '" << root.attribute("modelName").as_string()
                  << "' with " << variables << " variables\n";
    } catch (const std::exception& e) {
        fs::remove_all(tmp);
        return fail(e.what());
    }
    fs::remove_all(tmp);
    return EXIT_SUCCESS;
}
