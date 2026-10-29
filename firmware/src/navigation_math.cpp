#include "navigation_math.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace {
    bool whitespace(char c) {
        return c == ' ' || c == '\t' || c == '\r' || c == '\n';
    }

    void skipSpace(const char *& p) {
        while (whitespace(*p))
            ++p;
    }

    bool take(const char *& p, char c) {
        skipSpace(p);
        if (*p != c)
            return false;
        ++p;
        return true;
    }

    bool digit(char c) {
        return c >= '0' && c <= '9';
    }

    bool number(const char *& p, float& value) {
        skipSpace(p);
        const char *begin = p;
        if (*p == '-')
            ++p;
        if (*p == '0') {
            ++p;
        } else {
            if (*p < '1' || *p > '9')
                return false;
            while (digit(*p))
                ++p;
        }
        if (*p == '.') {
            ++p;
            if (!digit(*p))
                return false;
            while (digit(*p))
                ++p;
        }
        if (*p == 'e' || *p == 'E') {
            ++p;
            if (*p == '+' || *p == '-')
                ++p;
            if (!digit(*p))
                return false;
            while (digit(*p))
                ++p;
        }
        char *end = nullptr;
        value = std::strtof(begin, &end);
        return end == p && std::isfinite(value);
    }

    bool validPoint(const NavigationPoint& p) {
        return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.t) && std::fabs(p.x) <= 100 &&
               std::fabs(p.y) <= 100 && std::fabs(p.t) <= 360;
    }

    bool poseField(const char *raw, const char *key, double& value) {
        const char *field = std::strstr(raw, key);
        if (!field || (field != raw && !whitespace(field[-1]) && field[-1] != ','))
            return false;
        const char *begin = field + std::strlen(key);
        if (std::strstr(begin, key))
            return false;
        char *end = nullptr;
        value = std::strtod(begin, &end);
        return end != begin && std::isfinite(value) &&
               (*end == '\0' || whitespace(*end) || *end == ',' || *end == '\x1a');
    }

    float angleDifference(float target, float current) {
        return std::remainder(target - current, 360.0f);
    }
} // namespace

bool parseNavigationRoute(const char *json, NavigationRoute& route) {
    route.count = 0;
    if (!json || std::strlen(json) > NavigationRoute::MAX_BODY)
        return false;
    const char *p = json;
    if (!take(p, '['))
        return false;
    size_t count = 0;
    float length = 0;
    do {
        if (count == NavigationRoute::MAX_POINTS || !take(p, '{'))
            return false;
        NavigationPoint point;
        unsigned int seen = 0;
        do {
            if (!take(p, '"'))
                return false;
            char key = *p;
            if (key != 'x' && key != 'y' && key != 't')
                return false;
            ++p;
            unsigned int bit = key == 'x' ? 1U : (key == 'y' ? 2U : 4U);
            if ((seen & bit) != 0 || !take(p, '"') || !take(p, ':'))
                return false;
            seen |= bit;
            float& value = key == 'x' ? point.x : (key == 'y' ? point.y : point.t);
            if (!number(p, value))
                return false;
        } while (take(p, ','));
        if (seen != 7 || !take(p, '}') || !validPoint(point))
            return false;
        if (count > 0) {
            float distance = navigationDistance(route.points[count - 1], point);
            length += distance;
            if (distance > 2.0f || length > 20.0f)
                return false;
        }
        route.points[count++] = point;
    } while (take(p, ','));
    if (!take(p, ']'))
        return false;
    skipSpace(p);
    if (*p != '\0')
        return false;
    route.count = count;
    return true;
}

bool NavigationBody::append(const char *chunk, size_t length, size_t index, size_t total) {
    if (invalid || !chunk || total == 0 || total > NavigationRoute::MAX_BODY || index != received ||
        (expected != 0 && expected != total) || index > total || length > total - index ||
        std::memchr(chunk, '\0', length)) {
        invalid = true;
        return false;
    }
    expected = total;
    std::memcpy(data + received, chunk, length);
    received += length;
    data[received] = '\0';
    return true;
}

bool parseNavigationPose(const char *raw, NavigationPose& pose) {
    if (!raw)
        return false;
    double x, y, theta, time;
    if (!poseField(raw, "X=", x) || !poseField(raw, "Y=", y) || !poseField(raw, "Theta=", theta) ||
        !poseField(raw, "Time=", time) || time < 0)
        return false;
    NavigationPose parsed;
    parsed.x = static_cast<float>(x);
    parsed.y = static_cast<float>(y);
    parsed.t = static_cast<float>(theta);
    parsed.time = time;
    if (!validPoint(parsed))
        return false;
    pose = parsed;
    return true;
}

float navigationDistance(const NavigationPoint& a, const NavigationPoint& b) {
    return std::hypot(b.x - a.x, b.y - a.y);
}

NavigationMove navigationMove(const NavigationPose& pose, const NavigationPoint& target) {
    NavigationMove move;
    float distance = navigationDistance(pose, target);
    constexpr float PI = 3.14159265358979323846f;
    float heading = distance <= 0.05f ? target.t : std::atan2(target.y - pose.y, target.x - pose.x) * 180.0f / PI;
    float error = angleDifference(heading, pose.t);
    if (std::fabs(error) > 5.0f) {
        // shortcut: nominal 248 mm track width, calibrate against supervised hardware trials.
        float rotation = std::max(-15.0f, std::min(error, 15.0f));
        int wheel = static_cast<int>(std::lround(rotation * PI / 180.0f * 124.0f));
        move.left = -wheel;
        move.right = wheel;
    } else if (distance > 0.05f) {
        move.left = static_cast<int>(std::lround(std::min(distance, 0.1f) * 1000.0f));
        move.right = move.left;
    } else {
        move.arrived = true;
    }
    return move;
}
