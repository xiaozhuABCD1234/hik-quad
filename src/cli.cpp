#include "hikquad/cli.hpp"

#include <cstdlib>
#include <iostream>
#include <sstream>

namespace hikquad {

CliParser::CliParser(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        tokens_.emplace_back(argv[i]);
    }
}

bool CliParser::has(const std::string& name) const {
    for (const auto& token : tokens_) {
        if (token == name || token.rfind(name + "=", 0) == 0) {
            return true;
        }
    }
    return false;
}

std::string CliParser::get(const std::string& name, const std::string& fallback) const {
    for (size_t i = 0; i < tokens_.size(); ++i) {
        const std::string& token = tokens_[i];
        if (token == name) {
            if (i + 1 < tokens_.size()) {
                return tokens_[i + 1];
            }
            return fallback;
        }
        if (token.rfind(name + "=", 0) == 0) {
            return token.substr(name.size() + 1);
        }
    }
    return fallback;
}

std::vector<std::string> CliParser::getAll(const std::string& name) const {
    std::vector<std::string> values;
    for (size_t i = 0; i < tokens_.size(); ++i) {
        if (tokens_[i] == name && i + 1 < tokens_.size()) {
            values.push_back(tokens_[i + 1]);
        } else if (tokens_[i].rfind(name + "=", 0) == 0) {
            values.push_back(tokens_[i].substr(name.size() + 1));
        }
    }
    return values;
}

int CliParser::getInt(const std::string& name, int fallback) const {
    const std::string text = get(name);
    if (text.empty()) {
        return fallback;
    }
    try {
        return std::stoi(text);
    } catch (...) {
        return fallback;
    }
}

double CliParser::getDouble(const std::string& name, double fallback) const {
    const std::string text = get(name);
    if (text.empty()) {
        return fallback;
    }
    try {
        return std::stod(text);
    } catch (...) {
        return fallback;
    }
}

bool CliParser::validate(const std::vector<std::string>& allowed, std::string* error) const {
    for (const auto& token : tokens_) {
        if (token.rfind("--", 0) != 0) {
            continue;
        }
        const std::string name = token.substr(0, token.find('='));
        bool ok = false;
        for (const auto& candidate : allowed) {
            if (name == candidate) {
                ok = true;
                break;
            }
        }
        if (!ok) {
            if (error != nullptr) {
                *error = "unknown option: " + name;
            }
            return false;
        }
    }
    return true;
}

void CliParser::printUsage(const char* program, const std::string& text) {
    std::cout << "Usage: " << program << "\n\n" << text << "\n";
}

}  // namespace hikquad
