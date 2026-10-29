#include "manual_clean_manager.h"
#include <cassert>
#include <cstdio>
uint32_t testMillis = 100;
int main() {
    NeatoSerial serial;
    ManualCleanManager manual(serial);
    bool enabled = false;
    assert(manual.enable(true, [&](bool ok) { enabled = ok; }));
    assert(manual.enable(false, nullptr)); // cancellation must work before TestMode reply
    serial.drain();
    assert(!enabled && !manual.isActive());
    assert(serial.modes.size() == 2 && !serial.modes.back());

    assert(manual.enable(true, nullptr));
    serial.drain();
    bool result = true;
    serial.stopOk = false;
    assert(manual.enable(false, [&](bool ok) { result = ok; }));
    serial.drain();
    assert(!result); // a failed wheel stop must not look like successful teardown
    std::puts("manual navigation: startup cancellation and teardown failures passed");
}
