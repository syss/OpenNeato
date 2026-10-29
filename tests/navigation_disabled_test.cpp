#include "navigation_manager.h"
#include "manual_clean_manager.h"
#include <cassert>
#include <cstdio>
uint32_t testMillis = 100;
int main() {
    NeatoSerial serial;
    ManualCleanManager manual(serial);
    DataLogger logger;
    NavigationManager nav(serial, manual, logger);
    assert(nav.start("[{\"x\":0,\"y\":0,\"t\":0}]") == 501);
    nav.loop();
    nav.stop();
    nav.loop();
    assert(serial.pending.empty() && serial.wheels.empty() && !nav.isBusy());
    assert(nav.getStatusJson().find("\"enabled\":false") != String::npos);
    std::puts("ordinary build: navigation disabled, no robot commands");
}
