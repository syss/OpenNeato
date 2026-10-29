# Experimental waypoint navigation

Implement upstream issue #70 on a feature branch of syss/OpenNeato. User authorized
offline development and publication to the fork; never contact, flash, or move the
robot during this work. No Vibeplanner task or upstream discussion is requested.

Use the existing C++17 firmware, ManualCleanManager, serial queue and Preact build.
Keep navigation disabled in ordinary builds. An explicit experimental build enables
POST /api/navigate with a bounded JSON array of {x,y,t} (metres, metres, degrees),
GET /api/navigate/status, and POST /api/navigate/stop. A start only queues work;
serial actions execute on the firmware loop. Use small finite wheel movements,
fresh valid pose/safety feedback, bounded runtime, cancellation, and checked teardown.
Keep cleaning motors off. Require an idle robot and a start pose near the first
waypoint; reject charging/docked starts (automatic undocking is out of scope).

Prefer firmware-owned navigation over an external host driver: it uses the intended
upstream architecture and can stop without a network round trip. A host-only
simulation validates the same parser, geometry and lifecycle without hardware.
Keep short steps and generous timing margins until hardware calibration is possible.

No zones, no-go enforcement, autonomous avoidance, docking, charging/resume, UI,
or claims of real localization accuracy. No route persistence or new dependencies.
Existing recorded paths are not proof of free space. Supervised hardware tests are
required before enabling movement for real use.
