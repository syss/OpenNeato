#ifndef NAVIGATION_MATH_H
#define NAVIGATION_MATH_H

#include <cstddef>

struct NavigationPoint {
    float x = 0; // metres
    float y = 0;
    float t = 0; // degrees, counter-clockwise
};

struct NavigationPose : NavigationPoint {
    double time = 0; // robot's Time field, used to reject repeated feedback
};

struct NavigationRoute {
    static constexpr size_t MAX_POINTS = 32;
    static constexpr size_t MAX_BODY = 4096;
    NavigationPoint points[MAX_POINTS];
    size_t count = 0;
};

struct NavigationMove {
    int left = 0; // millimetres
    int right = 0;
    int speed = 50; // millimetres/second
    bool arrived = false;
};

struct NavigationBody {
    char data[NavigationRoute::MAX_BODY + 1] = {};
    size_t received = 0;
    size_t expected = 0;
    bool invalid = false;
    bool append(const char *chunk, size_t length, size_t index, size_t total);
    bool complete() const { return !invalid && expected > 0 && received == expected; }
};

// Strict bounded input for motion commands; unlike settings, partial objects are unsafe.
bool parseNavigationRoute(const char *json, NavigationRoute& route);
bool parseNavigationPose(const char *raw, NavigationPose& pose);
NavigationMove navigationMove(const NavigationPose& pose, const NavigationPoint& target);
float navigationDistance(const NavigationPoint& a, const NavigationPoint& b);

#endif // NAVIGATION_MATH_H
