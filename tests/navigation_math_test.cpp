#include "navigation_math.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>

int main() {
    NavigationBody body;
    assert(body.append("[", 1, 0, 2));
    assert(!body.complete());
    assert(body.append("]", 1, 1, 2));
    assert(body.complete() && std::string(body.data) == "[]");
    NavigationBody invalidBody;
    assert(!invalidBody.append("[]", 2, 1, 2));
    assert(!invalidBody.complete());
    NavigationBody nulBody;
    assert(!nulBody.append("\0]", 2, 0, 2));
    NavigationBody hugeBody;
    assert(!hugeBody.append("[]", 2, 0, 4097));
    NavigationBody changedBody;
    assert(changedBody.append("[", 1, 0, 2));
    assert(!changedBody.append("]", 1, 1, 3));
    NavigationRoute route;
    assert(parseNavigationRoute("[{\"x\":0,\"y\":0,\"t\":0},{\"t\":90,\"x\":1e-1,\"y\":0.2}]", route));
    assert(route.count == 2 && std::fabs(route.points[1].x - 0.1f) < 0.0001f);
    for (const char *bad:
         {"", "[]", "{}", "[{}]", "[{\"x\":0,\"y\":0}]", "[{\"x\":0,\"y\":0,\"t\":0,}]",
          "[{\"x\":0,\"y\":0,\"t\":0}]trailing", "[{\"x\":0 \"y\":0,\"t\":0}]", "[{\"x\":0,\"x\":1,\"y\":0,\"t\":0}]",
          "[{\"x\":NaN,\"y\":0,\"t\":0}]", "[{\"x\":1e99,\"y\":0,\"t\":0}]", "[{\"x\":\"0\",\"y\":0,\"t\":0}]",
          "[{\"x\":+1,\"y\":0,\"t\":0}]", "[{\"x\":01,\"y\":0,\"t\":0}]", "[{\"x\":1.,\"y\":0,\"t\":0}]",
          "[{\"x\":0,\"y\":0,\"t\":361}]", "[{\"x\":101,\"y\":0,\"t\":0}]",
          "[{\"x\":0,\"y\":0,\"t\":0},{\"x\":3,\"y\":0,\"t\":0}]"}) {
        assert(!parseNavigationRoute(bad, route));
        assert(route.count == 0);
    }
    std::string large = "[";
    for (int i = 0; i < 33; ++i)
        large += (i ? "," : "") + std::string("{\"x\":0,\"y\":0,\"t\":0}");
    large += "]";
    assert(!parseNavigationRoute(large.c_str(), route));
    assert(!parseNavigationRoute((std::string(4097, ' ') + "[]").c_str(), route));

    NavigationPose pose;
    assert(parseNavigationPose("GetRobotPos Smooth\r\nX=0.12 Y=-0.25 Theta=359.0 Time=123.5\r\n\x1a", pose));
    assert(pose.x == 0.12f && pose.y == -0.25f && pose.t == 359 && pose.time == 123.5);
    for (const char *bad: {"", "Unknown Cmd", "X=1 Y=2 Theta=3", "X=no Y=2 Theta=3 Time=4", "X=nan Y=2 Theta=3 Time=4",
                           "X=1 Y=2 Theta=3 Time=inf", "X=1junk Y=2 Theta=3 Time=4", "NotX=1 Y=2 Theta=3 Time=4",
                           "X=1 X=2 Y=2 Theta=3 Time=4", "X=1 Y=2 Theta=3 Time=-1"}) {
        assert(!parseNavigationPose(bad, pose));
    }
    NavigationMove move = navigationMove({0, 0, 0, 0}, {1, 0, 0});
    assert(move.left == 100 && move.right == 100 && move.speed == 50 && !move.arrived);
    move = navigationMove({0, 0, 0, 0}, {0, 1, 90});
    assert(move.left == -32 && move.right == 32); // 15 degrees on a 248 mm track
    move = navigationMove({0, 0, 359, 0}, {1, 0, 0});
    assert(move.left == 100 && move.right == 100); // shortest turn is 1 degree
    move = navigationMove({0, 0, 0, 0}, {0, -1, -90});
    assert(move.left == 32 && move.right == -32);
    assert(navigationMove({0, 0, 0, 0}, {0.03f, 0, 0}).arrived);
    assert(!navigationMove({0, 0, 0, 0}, {0.03f, 0, 90}).arrived);
    move = navigationMove({0, 0, 179, 0}, {0, 0, -179});
    assert(move.arrived);
    std::puts("navigation math: strict route/pose parsing and steering passed");
}
