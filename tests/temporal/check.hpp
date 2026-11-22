#pragma once
// A test harness small enough to read: no framework, so the core keeps no
// dependency beyond nlohmann_json. Each TEST is a function; run_all() reports
// every failed check and returns non-zero if any failed.

#include <cstdlib>
#include <exception>
#include <functional>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace {

int g_failures = 0;
std::string g_current;

#define CHECK(cond)                                                                    \
    do {                                                                               \
        if (!(cond)) {                                                                 \
            ++g_failures;                                                              \
            std::cerr << "  FAILED " << g_current << " (" << __FILE__ << ":" << __LINE__ \
                      << "): " #cond "\n";                                             \
        }                                                                              \
    } while (0)

#define CHECK_MSG(cond, msg)                                                           \
    do {                                                                               \
        if (!(cond)) {                                                                 \
            ++g_failures;                                                              \
            std::cerr << "  FAILED " << g_current << " (" << __FILE__ << ":" << __LINE__ \
                      << "): " #cond " -- " << (msg) << "\n";                          \
        }                                                                              \
    } while (0)

std::vector<std::pair<std::string, std::function<void()>>>& registry()
{
    static std::vector<std::pair<std::string, std::function<void()>>> tests;
    return tests;
}

struct Register {
    Register(const char* name, std::function<void()> fn) { registry().emplace_back(name, fn); }
};

#define TEST(name)                                       \
    void name();                                         \
    const Register register_##name{#name, name};         \
    void name()

inline int run_all()
{
    int run = 0;
    for (const auto& [name, fn] : registry()) {
        g_current = name;
        const int before = g_failures;
        try {
            fn();
        } catch (const std::exception& e) {
            ++g_failures;
            std::cerr << "  FAILED " << name << ": threw " << e.what() << "\n";
        }
        ++run;
        std::cout << (g_failures == before ? "[ OK ] " : "[FAIL] ") << name << "\n";
    }
    std::cout << run << " test(s), " << g_failures << " failed check(s)\n";
    return g_failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}

} // namespace
