#include "navigation_manager.h"
#include "manual_clean_manager.h"
#include <cassert>
#include <cstdio>
#include <limits>
#include <cmath>

uint32_t testMillis = 100;
constexpr const char *ROUTE = "[{\"x\":0,\"y\":0,\"t\":0},{\"x\":0.1,\"y\":0,\"t\":0}]";
struct Fixture {
    NeatoSerial serial;
    ManualCleanManager manual{serial};
    DataLogger logger;
    NavigationManager nav{serial, manual, logger};
    void pump() {
        manual.loop();
        serial.drain();
        nav.loop();
        serial.drain();
    }
    void start() {
        assert(nav.start(ROUTE) == 202);
        assert(serial.modes.empty()); // HTTP submission must not perform I/O
        pump();
        pump();
        pump();
    }
    bool status(const char *value) { return nav.getStatusJson().find(value) != String::npos; }
    int movements() {
        int count = 0;
        for (const auto& w: serial.wheels)
            if (w.left || w.right)
                ++count;
        return count;
    }
};

int main() {
    for (int completedCallbacks: {1, 2, 3}) {
        Fixture f;
        assert(f.nav.start(ROUTE) == 202);
        f.nav.loop();
        for (int i = 0; i < completedCallbacks; ++i) {
            auto callback = std::move(f.serial.pending.front());
            f.serial.pending.pop_front();
            callback();
        }
        f.nav.stop();
        f.nav.loop();
        f.serial.drain();
        f.pump();
        assert(f.status("cancelled") && !f.nav.isBusy() && !f.manual.isActive());
        assert(f.movements() == 0);
    }
    {
        Fixture f;
        assert(f.nav.start("[]") == 400);
        assert(!f.nav.isBusy() && f.serial.modes.empty());
        f.start();
        assert(f.nav.start(ROUTE) == 409);
        assert(f.movements() == 1);
        testMillis += 3000;
        f.serial.pose.raw = "X=0.1 Y=0 Theta=0 Time=2";
        f.pump();
        f.pump();
        assert(f.status("\"state\":\"done\"") && !f.nav.isBusy());
        assert(!f.manual.isActive() && !f.serial.navigationActive);
        assert(!f.serial.modes.back());
    }
    {
        Fixture f;
        assert(f.nav.start(ROUTE) == 202);
        f.nav.loop(); // state read pending
        f.nav.stop();
        f.nav.loop();
        f.serial.drain();
        f.pump();
        assert(f.movements() == 0 && f.serial.modes.empty());
        assert(f.status("cancelled") && !f.nav.isBusy());
    }
    {
        Fixture f;
        f.serial.state.idle = false;
        f.start();
        assert(f.status("robot_not_idle") && f.movements() == 0 && f.serial.modes.empty());
    }
    {
        Fixture f;
        f.serial.charger.extPwrPresent = true;
        f.start();
        assert(f.status("robot_on_dock") && f.movements() == 0);
    }
    {
        Fixture f;
        f.serial.pose.raw = "X=nan Y=0 Theta=0 Time=1";
        f.start();
        f.pump();
        assert(f.status("invalid_pose") && f.movements() == 0 && !f.nav.isBusy());
    }
    {
        Fixture f;
        f.serial.pose.raw = "X=1 Y=0 Theta=0 Time=1";
        f.start();
        f.pump();
        assert(f.status("start_too_far") && f.movements() == 0);
    }
    {
        Fixture f;
        f.start();
        testMillis += 3000;
        f.pump();
        f.pump();
        assert(f.status("stale_pose") && f.movements() == 1 && !f.nav.isBusy());
    }
    {
        Fixture f;
        f.start();
        f.serial.sensors.lFrontBit = true;
        testMillis += 1000;
        f.pump();
        f.pump();
        assert(f.status("obstacle") && f.movements() == 1);
    }
    {
        Fixture f;
        f.start();
        f.serial.safetyOk = false;
        testMillis += 1000;
        f.pump();
        f.pump();
        assert(f.status("safety_feedback_lost") && f.movements() == 1);
    }
    {
        Fixture f;
        f.serial.moveOk = false;
        f.start();
        f.pump();
        assert(f.status("move_failed") && !f.nav.isBusy());
    }
    {
        Fixture f;
        f.start();
        f.serial.exitOk = false;
        f.nav.stop();
        f.pump();
        f.pump();
        assert(f.status("cleanup_failed") && f.nav.isBusy());
        assert(f.nav.start(ROUTE) == 409);
        f.serial.exitOk = true;
        f.nav.stop();
        f.pump();
        f.pump();
        assert(!f.nav.isBusy());
    }
    {
        Fixture f;
        assert(f.nav.start(ROUTE) == 202);
        f.nav.loop();
        testMillis += 11000;
        f.nav.loop();
        f.serial.drain();
        assert(f.status("command_timeout") && f.movements() == 0);
    }
    {
        testMillis = std::numeric_limits<uint32_t>::max() - 1000;
        Fixture f;
        f.start();
        testMillis += 3000;
        f.serial.pose.raw = "X=0.1 Y=0 Theta=0 Time=2";
        f.pump();
        f.pump();
        assert(f.status("\"state\":\"done\""));
    }
    {
        Fixture f;
        f.start();
        f.serial.motors.navigationValid = false;
        testMillis += 1000;
        f.pump();
        f.pump();
        assert(f.status("safety_feedback_lost") && f.movements() == 1);
    }
    {
        Fixture f;
        f.start();
        f.serial.sensors.navigationValid = false;
        testMillis += 1000;
        f.pump();
        f.pump();
        assert(f.status("safety_feedback_lost") && f.movements() == 1);
    }
    {
        Fixture f;
        f.serial.enableOk = false;
        f.start();
        f.pump();
        assert(f.status("enable_failed") && !f.nav.isBusy());
        f.serial.enableOk = true;
        assert(f.manual.enable(true, nullptr));
        f.serial.drain();
        testMillis += 6000;
        WebServer::lastApiActivity = millis();
        size_t before = f.serial.wheels.size();
        f.manual.loop();
        f.serial.drain();
        assert(f.serial.wheels.size() == before); // Fresh HTTP heartbeat must control ordinary manual mode.
        testMillis += 6000;
        f.manual.loop();
        f.serial.drain();
        assert(f.serial.wheels.size() > before); // Disconnect watchdog must still stop wheels.
    }
    {
        Fixture f;
        f.serial.rejectMove = true;
        f.start();
        f.pump();
        assert(f.status("move_failed") && f.movements() == 0 && !f.nav.isBusy());
    }
    {
        Fixture f;
        f.serial.rejectEnable = true;
        f.start();
        f.pump();
        assert(f.status("enable_failed") && f.movements() == 0 && !f.nav.isBusy());
    }
    {
        Fixture f;
        f.start();
        f.serial.rejectStop = true;
        f.nav.stop();
        f.pump();
        f.pump();
        assert(f.status("cleanup_failed") && f.nav.isBusy());
        f.serial.rejectStop = false;
        f.nav.stop();
        f.pump();
        f.pump();
        assert(!f.nav.isBusy());
    }
    // Simulate finite differential-drive wheel motion, including under-travel, against real managers.
    for (double travelScale: {1.0, 0.85, 0.0}) {
        Fixture f;
        const char *square = "[{\"x\":0,\"y\":0,\"t\":0},{\"x\":0.2,\"y\":0,\"t\":90},"
                             "{\"x\":0.2,\"y\":0.2,\"t\":180},{\"x\":0,\"y\":0.2,\"t\":270},"
                             "{\"x\":0,\"y\":0,\"t\":360}]";
        assert(f.nav.start(square) == 202);
        double x = 0, y = 0, heading = 0, left = 0, right = 0;
        size_t applied = 0;
        int ticks = 0;
        for (; ticks < 4000 && f.nav.isBusy(); ++ticks) {
            double maxWheel = std::max(std::fabs(left), std::fabs(right));
            double fraction = maxWheel > 0 ? std::min(5.0 / maxWheel, 1.0) : 0; // 50mm/s, 100ms tick
            double dl = left * fraction, dr = right * fraction;
            left -= dl;
            right -= dr;
            double turn = (dr - dl) * travelScale / 248.0;
            double distance = (dl + dr) * travelScale / 2000.0;
            x += distance * std::cos(heading + turn / 2);
            y += distance * std::sin(heading + turn / 2);
            heading += turn;
            char raw[128];
            std::snprintf(raw, sizeof(raw), "X=%.6f Y=%.6f Theta=%.6f Time=%.3f", x, y,
                          std::remainder(heading * 180.0 / 3.141592653589793, 360.0), (ticks + 1) / 10.0);
            f.serial.pose.raw = raw;
            testMillis += 100;
            f.pump();
            for (; applied < f.serial.wheels.size(); ++applied) {
                const auto& w = f.serial.wheels[applied];
                assert(std::abs(w.left) <= 100 && std::abs(w.right) <= 100 && w.speed <= 50);
                left = w.left;
                right = w.right;
            }
        }
        assert(!f.nav.isBusy());
        if (travelScale > 0) {
            assert(f.status("\"state\":\"done\""));
            assert(std::hypot(x, y) <= 0.05);
            assert(std::fabs(std::remainder(heading * 180.0 / 3.141592653589793, 360.0)) <= 5);
        } else {
            assert(f.status("progress_timeout"));
        }
        std::printf("square simulation: travel scale %.2f, %.1fs, final (%.3f, %.3f), %s\n", travelScale, ticks / 10.0,
                    x, y, travelScale > 0 ? "arrived" : "stopped on no progress");
    }
    std::puts("navigation lifecycle: completion, cancellation, safety, faults and rollover passed");
}
