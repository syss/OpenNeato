# Waypoint Navigation Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans. User requested autonomous offline execution.

**Goal:** Deliver a compile-tested, experimentally gated waypoint follower with reproducible offline safety tests.

**Architecture:** Portable bounded route/pose parsing and steering math; a loop-driven NavigationManager adapts the existing manual lifecycle and serial feedback. HTTP only submits commands and reads snapshots.

**Tech Stack:** Existing C++17/Arduino/PlatformIO; standard C++ host tests; existing OpenAPI build.

**Spec:** docs/superpowers/specs/2026-10-29-waypoint-navigation.md

## Global Constraints

- Never contact, flash or move the robot. Work only against fakes and local build tools.
- No external runtime dependencies. No flash writes for routes.
- Default builds reject navigation start; opt in with OPENNEATO_EXPERIMENTAL_NAVIGATION=1.
- Preserve API/schema parity, manual safety and the serial command queue.

## Review Focus

- Malformed, oversized, non-finite or incomplete route and pose input must never move wheels.
- Cancel during startup or a pending callback must not allow a late callback to restart movement.
- Serial queue rejection, missing feedback and safety events must stop and expose an error.
- Coordinate units, angle wrap and timer rollover must not generate incorrect movement.
- Other control requests must not take over a navigation run; teardown failure must remain visible.

### Task 1: Bounded input and steering

**Files:** firmware/src/navigation_math.{h,cpp}, tests/navigation_math_test.cpp, scripts/test_navigation.sh

**Interfaces:** NavigationPose, NavigationRoute, NavigationMove; parseNavigationRoute, parseNavigationPose, navigationMove.

- [x] Write and run failing tests for strict input, limits, metres-to-mm conversion, shortest turns and arrival.
- [x] Implement bounded parsing and finite short wheel steps, no Arduino dependency.
- [x] Run host tests with warnings and sanitizers; commit.

### Task 2: Firmware lifecycle and safety

**Files:** firmware/src/navigation_manager.{h,cpp}, manual_clean_manager.{h,cpp}, neato_serial.cpp, data_logger.{h,cpp}; tests/navigation_manager_test.cpp and host doubles.

**Interfaces:** NavigationManager::start(route), stop(), isBusy(), getStatusJson(); loop-only I/O, async completion guards.

- [x] Write failing lifecycle tests: start, normal finish, cancel, invalid feedback, command failures, timeout, safety, stale callback.
- [x] Implement lifecycle and fresh safety reporting using ManualCleanManager; preserve stop failures.
- [x] Exercise deterministic simulated trajectories and faults; commit.

### Task 3: API, build integration and handover

**Files:** firmware/src/{main.cpp,web_server.h,web_server.cpp,config.h}, frontend/api/openapi.yaml, platformio.ini, docs/navigation.md, .github/workflows/ci.yml.

- [x] Connect bounded request handling, status and stop routes; block competing writes during navigation.
- [x] Document experimental build, offline checks and future supervised hardware acceptance checks.
- [x] Run host suite, frontend checks/build, firmware normal/experimental builds, formatting, static analysis and workflow validation.
- [x] Obtain independent review and fix important findings with regression tests.
- [x] Publish the verified feature branch to the existing fork.
