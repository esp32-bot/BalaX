// Runs the laptop tests of the firmware's plain C++ code: tools/bx test.

#include "check.h"

int main() {
    int count = 0;
    for (const check::Test* test = check::first; test; test = test->next) {
        test->run();
        ++count;
    }
    printf("%d tests, %d failed checks\n", count, check::failures);
    return check::failures == 0 ? 0 : 1;
}
