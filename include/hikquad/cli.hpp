#pragma once

#include <string>
#include <vector>

namespace hikquad {

class CliParser {
public:
    CliParser(int argc, char** argv);

    bool has(const std::string& name) const;
    std::string get(const std::string& name, const std::string& fallback = "") const;
    int getInt(const std::string& name, int fallback) const;
    double getDouble(const std::string& name, double fallback) const;
    std::vector<std::string> getAll(const std::string& name) const;

    const std::vector<std::string>& positional() const {
        return positional_;
    }
    bool validate(const std::vector<std::string>& allowed, std::string* error) const;
    static void printUsage(const char* program, const std::string& text);

private:
    std::vector<std::string> tokens_;
    std::vector<std::string> positional_;
};

}  // namespace hikquad
