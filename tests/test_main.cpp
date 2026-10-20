#include <exception>
#include <iostream>

#include "test_framework.hpp"

int main() {
    int failed = 0;
    for (const auto& test : hikquad_test::registry()) {
        try {
            test.function();
            std::cout << "[ PASS ] " << test.name << "\n";
        } catch (const std::exception& error) {
            std::cerr << "[ FAIL ] " << test.name << ": " << error.what() << "\n";
            ++failed;
        }
    }
    const size_t total = hikquad_test::registry().size();
    std::cout << total - static_cast<size_t>(failed) << "/" << total << " tests passed\n";
    return failed == 0 ? 0 : 1;
}
