# Experimental waypoint navigation

Work toward upstream [issue #70](https://github.com/renjfk/OpenNeato/issues/70).
This is a supervised proof of concept, **not a validated autonomous cleaner**.
No robot was contacted, flashed, or moved during development. Tests use simulated
serial responses and simulated differential-drive motion.

The firmware follows a bounded list of positions using `GetRobotPos Smooth`, the
existing manual-mode safety polling, and short wheel commands. It stops on contact,
stall, invalid/incomplete/stale feedback, failed commands, excessive pose jumps,
cancellation, or lack of progress. It checks fuel and dock state before starting.
Cleaning motors remain off. Nothing is persisted to flash.

This does not implement zones, no-go enforcement, obstacle avoidance, undocking,
return-to-dock, or battery recharge/resume. A recorded path does not prove a route
is currently clear. Bumpers do not replace cliff protection. Use a clear, level,
bounded floor area for future supervised trials.

## Build and offline checks

Ordinary builds expose status but return HTTP 501 on navigation start. The only
opt-in target added here is `c3-navigation-debug`; it is not a release target.

```sh
bash scripts/test_navigation.sh
SANITIZE=1 bash scripts/test_navigation.sh
cd frontend
npm ci
npm run check
npm run build
cd ..
pio run -e c3-debug -e c3-navigation-debug
python scripts/check_format.py
pio check -e c3-debug --fail-on-defect=low
```

Use Node 22+ and the project's PlatformIO requirements. These commands only build
and test. They do not upload firmware. In a ptrace-based sandbox, LeakSanitizer may
be unsupported; use `ASAN_OPTIONS=detect_leaks=0` there while retaining address and
undefined-behavior checks. CI runs with the usual sanitizer defaults.

The host suite compiles the production navigation math, navigation manager, manual
manager, serial queue and serial parsers. The hardware boundary is a fake UART or
fake serial service. It checks malformed and fragmented input, metre/mm conversion,
angle wrap, arrival, startup cancellation, stale callbacks, partial telemetry,
timeouts, synchronous queue rejection, cleanup retry, and manual watchdog ownership.

The square-route simulation uses finite wheel distances at 50 mm/s and a nominal
248 mm track. It covers nominal motion, 15% under-travel and a completely stuck
robot. Its poses are simulated; it does **not** validate real localization,
wheel calibration, serial latency, traction, or cliff behavior.

### Offline validation recorded 2026-10-29

- Five host test executables passed, also with AddressSanitizer and UBSan (sandbox
  LeakSanitizer disabled as described above).
- Square simulation: nominal motion finished in 82.7 simulated seconds, within
  13 mm of the origin; 15% under-travel finished in 105.2 seconds, within 24 mm.
  Zero travel stopped on the progress deadline in 60.3 seconds.
- Normal `c3-debug` and opt-in `c3-navigation-debug` compiled. The experimental
  image used 1,358,662 of 1,638,400 flash bytes and 57,436 of 327,680 static RAM bytes.
  These figures do not measure runtime heap or real-time performance.
- Frontend check/build, API path parity, firmware formatting and actionlint passed.
- `pio check -e c3-debug --fail-on-defect=low` completed with four existing medium
  warnings, in `cleaning_history.cpp:522`, `firmware_manager.cpp:11`,
  `neato_serial.cpp:77`, and `settings_manager.cpp:402`. It is not a zero-warning run;
  no findings were reported in the navigation implementation.
- Independent review found incomplete safety telemetry and a watchdog ownership
  leak on failed startup. Both were fixed with regression tests. Real serial tests
  also verify stop failures and synchronous queue rejection.

## API for future supervised trials

Do not use these endpoints until an experimental build has been deliberately
installed and an operator is present. No command examples below select a real host.

| Endpoint | Behavior |
| --- | --- |
| `POST /api/navigate` | Accept an `application/json` array of `{x,y,t}` waypoints; return 202 for queued startup checks. |
| `GET /api/navigate/status` | Read cached run state, reason, current waypoint, pose, and cleanup requirement. No serial query. |
| `POST /api/navigate/stop` | Cancel, or retry a previously failed cleanup. Does not return to dock. |

Coordinates use **metres**, heading uses **degrees counter-clockwise**. The robot's
current coordinate frame must match the route. The first point must be within
0.25 m of the initial reported pose. Each point's heading is reached before advancing.

An illustrative route, meaningful only if the robot is already at this pose:

```json
[{"x":0,"y":0,"t":0},{"x":0.1,"y":0,"t":0}]
```

Limits: 1–32 points; 4096-byte request body; X/Y within ±100 m; heading within ±360°;
each segment at most 2 m; total route at most 20 m. Unknown/duplicate keys, missing
values, non-numeric/non-finite numbers, trailing data and incomplete bodies are rejected.
The robot must be idle, off external power, not charging, and have at least 20% fuel.

Movement is limited to 100 mm straight or 15° rotation per command at 50 mm/s.
After command acknowledgement it waits at least 2.5 seconds before another pose read.
Arrival tolerances are 50 mm and 5°. These are conservative starting assumptions,
not calibrated values. A waypoint has a 60-second deadline and a run a 10-minute
deadline. Commands have a 10-second deadline; safety feedback a 2-second freshness
window. The robot's `Time` field must advance between pose samples.

Status states: `idle`, `starting`, `running`, `stopping`, `done`, `cancelled`, `error`.
HTTP 202 means accepted, not movement or success. Poll until a terminal state **and**
`cleanupRequired:false`. A failed stop/teardown retains control and rejects new
runs; `/api/navigate/stop` retries cleanup. HTTP stop acknowledgement only confirms
the request was queued, not that the physical robot has stopped.

Other HTTP writes are rejected while navigation owns control, including manual
drive, cleaning, settings and uploads. Native cleaning, power and raw serial
requests are also blocked, including scheduler-triggered cleaning/restarts. Status
reads remain available. Navigation does not depend on browser heartbeats; bounded
wheel steps and feedback deadlines limit movement if feedback is lost.

## Hardware acceptance still required

Before considering everyday use, verify these with an operator beside the robot:

1. Capture the exact pose, digital-sensor, motor-load and charger responses on the
   target model; confirm units, heading sign, complete fields, and advancing `Time`.
2. Verify TestMode/LDS startup and teardown, wheel disabling, and emergency stop.
3. Measure a short straight move and rotation against floor marks; calibrate wheel
   separation and timing. Check overshoot and arrival tolerance at low speed.
4. Validate fresh safety reads, bumper/stall stops, cancellation during startup and
   movement, and behavior when serial communication fails.
5. Compare repeated-route poses with physical position, especially after TestMode
   transitions. Never assume a saved run and a new run share a coordinate frame.

A fixed overhead camera showing the dock and whole test area, with visible floor
reference marks and recordings, would help measure drift and overshoot. It cannot
replace an operator or establish physical safety by itself.
