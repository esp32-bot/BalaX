#pragma once

// A minimal test runner for the laptop tests: TEST(name) { CHECK(...); }, run by tests.cpp.

#include <cstdio>

namespace check {

using TestFunction = void (*)();

struct Test {
    const char* name;
    TestFunction run;
    Test* next;
};

inline Test* first = nullptr;
inline int failures = 0;

struct Register {
    Register(Test& test) {
        test.next = first;
        first = &test;
    }
};

inline void fail(const char* file, int line, const char* expression) {
    printf("%s:%d: CHECK(%s) failed\n", file, line, expression);
    ++failures;
}

}  // namespace check

#define TEST(name)                                                   \
    static void name();                                              \
    static check::Test name##_test = {#name, name, nullptr};         \
    static check::Register name##_register(name##_test);             \
    static void name()

#define CHECK(expression)                                  \
    do {                                                   \
        if (!(expression)) {                               \
            check::fail(__FILE__, __LINE__, #expression);  \
        }                                                  \
    } while (0)
