# Bundled dependencies

FMI4cpp upstream depends on [libzip](https://libzip.org) (to unpack `.fmu`
archives) and [pugixml](https://pugixml.org) (to read `modelDescription.xml`).
To allow building offline with nothing but a C++17 compiler, this copy replaces
both with small, dependency-free implementations:

| Directory       | Replaces | What it is |
|-----------------|----------|------------|
| `tinyzip/`      | libzip   | ZIP reader with its own RFC 1951 inflate; stored + deflate entries, CRC-32 verified. No ZIP64/encryption. |
| `pugixml-lite/` | pugixml  | Minimal XML DOM exposing the subset of the `pugi::` API used by `model_description_parser.cpp` (same names, so that file is unchanged). UTF-8 only, no XPath/DTD entities. |

Changes relative to upstream FMI4cpp:

* `CMakeLists.txt`: no `find_package(LIBZIP)` / `find_package(PugiXML)`.
* `src/CMakeLists.txt`: the two implementations are compiled into `fmi4cpp`.
* `src/fmi4cpp/tools/unzipper.cpp`: uses `tinyzip::extract_all`.
* `cmake/fmi4cpp-config.cmake.in`: no `find_dependency` calls.

`selftest.cpp` (target `fmi4cpp_third_party_selftest`, registered with CTest)
extracts an FMU and parses its model description as a smoke test.
