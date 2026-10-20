#pragma once

#include <cmath>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

namespace hikquad_test {

struct Case {
    std::string name;
    std::function<void()> function;
};

inline std::vector<Case>& registry() {
    static std::vector<Case> cases;
    return cases;
}

struct Registrar {
    Registrar(const std::string& name, std::function<void()> function) {
        registry().push_back({name, std::move(function)});
    }
};

class Failure : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

inline void check(bool condition, const std::string& expression, const char* file, int line) {
    if (!condition) {
        std::ostringstream out;
        out << file << ":" << line << ": check failed: " << expression;
        throw Failure(out.str());
    }
}

template <typename T>
std::string valueText(const T& value) {
    if constexpr (std::is_enum_v<T>) {
        return std::to_string(static_cast<long long>(value));
    } else {
        std::ostringstream out;
        out << value;
        return out.str();
    }
}

template <typename A, typename B>
void checkEqual(const A& left, const B& right, const std::string& expression, const char* file,
                int line) {
    if (!(left == right)) {
        std::ostringstream out;
        out << file << ":" << line << ": check failed: " << expression << " (" << valueText(left)
            << " != " << valueText(right) << ")";
        throw Failure(out.str());
    }
}

inline void checkNear(double left, double right, double tolerance, const std::string& expression,
                      const char* file, int line) {
    if (std::fabs(left - right) > tolerance) {
        std::ostringstream out;
        out << file << ":" << line << ": check failed: " << expression << " (" << left << " vs "
            << right << ", tolerance " << tolerance << ")";
        throw Failure(out.str());
    }
}

}  // namespace hikquad_test

#define HIK_TEST(name)                                              \
    static void name();                                             \
    static ::hikquad_test::Registrar registrar_##name(#name, name); \
    static void name()

#define CHECK_TRUE(expression) ::hikquad_test::check((expression), #expression, __FILE__, __LINE__)
#define CHECK_FALSE(expression) \
    ::hikquad_test::check(!(expression), "!(" #expression ")", __FILE__, __LINE__)
#define CHECK_EQ(left, right) \
    ::hikquad_test::checkEqual((left), (right), #left " == " #right, __FILE__, __LINE__)
#define CHECK_NEAR(left, right, tolerance) \
    ::hikquad_test::checkNear((left), (right), (tolerance), #left " ~= " #right, __FILE__, __LINE__)
