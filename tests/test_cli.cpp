#include "hikquad/cli.hpp"
#include "test_framework.hpp"

HIK_TEST(cli_parses_values_and_flags) {
    const char* argv[] = {"tool", "--frames", "3", "--out=/tmp/x", "--raw", "positional"};
    hikquad::CliParser cli(6, const_cast<char**>(argv));
    CHECK_EQ(cli.getInt("--frames", 0), 3);
    CHECK_EQ(cli.get("--out"), std::string("/tmp/x"));
    CHECK_TRUE(cli.has("--raw"));
    CHECK_EQ(cli.positional().size(), size_t(1));
    CHECK_EQ(cli.positional()[0], std::string("positional"));
}
