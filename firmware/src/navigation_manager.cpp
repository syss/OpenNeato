#include "navigation_manager.h"
#include "config.h"
#include "data_logger.h"
#include "manual_clean_manager.h"
#include "neato_serial.h"
#include <cmath>
#include <cstdio>

NavigationManager::NavigationManager(NeatoSerial& serial, ManualCleanManager& manual, DataLogger& logger) :
    serial(serial), manual(manual), logger(logger) {
    TaskRegistry::add(this);
}

const char *NavigationManager::stateName(State value) {
    switch (value) {
        case State::IDLE:
            return "idle";
        case State::STARTING:
            return "starting";
        case State::RUNNING:
            return "running";
        case State::STOPPING:
            return "stopping";
        case State::DONE:
            return "done";
        case State::CANCELLED:
            return "cancelled";
        case State::ERROR:
            return "error";
    }
    return "error";
}

void NavigationManager::setState(State next) {
    state = next;
    logger.logNavigation(stateName(state), reason, waypoint);
}

bool NavigationManager::isBusy() const {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    return state == State::STARTING || state == State::RUNNING || state == State::STOPPING || cleanupRequired;
}

int NavigationManager::start(const char *json) {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    if (!OPENNEATO_EXPERIMENTAL_NAVIGATION)
        return 501;
    if (isBusy())
        return 409;
    if (!parseNavigationRoute(json, route))
        return 400;
    ++generation;
    waypoint = 0;
    pose = {};
    hasPose = false;
    hasMoved = false;
    enteredManual = false;
    cancelRequested = false;
    pending = false;
    reason = "";
    startedMs = millis();
    waypointMs = startedMs;
    serial.setNavigationActive(true);
    // Logging and all hardware operations belong to the main loop.
    state = State::STARTING;
    return 202;
}

void NavigationManager::stop() {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    if (isBusy())
        cancelRequested = true;
}

void NavigationManager::beginStart() {
    if (manual.isBusy()) {
        beginStop(State::ERROR, "manual_busy");
        return;
    }
    pending = true;
    pendingMs = millis();
    uint32_t token = generation;
    logger.logNavigation("starting", "", 0);
    serial.invalidateAll();
    serial.getState([this, token](bool ok, const RobotState& robot) {
        std::lock_guard<std::recursive_mutex> lock(mutex);
        if (token != generation || cancelRequested)
            return;
        if (!ok || !robot.isIdle()) {
            beginStop(State::ERROR, ok ? "robot_not_idle" : "state_unavailable");
            return;
        }
        serial.getCharger([this, token](bool chargerOk, const ChargerData& charger) {
            std::lock_guard<std::recursive_mutex> lock(mutex);
            if (token != generation || cancelRequested)
                return;
            if (!chargerOk || !charger.navigationValid || charger.extPwrPresent || charger.chargingActive ||
                charger.fuelPercent < 20) {
                beginStop(State::ERROR, (!chargerOk || !charger.navigationValid)
                                                ? "charger_unavailable"
                                                : (charger.extPwrPresent || charger.chargingActive ? "robot_on_dock"
                                                                                                   : "low_battery"));
                return;
            }
            enteredManual = true;
            manual.navigationHeartbeat();
            bool accepted = manual.enable(true, [this, token](bool enabled) {
                std::lock_guard<std::recursive_mutex> lock(mutex);
                if (token != generation || cancelRequested)
                    return;
                pending = false;
                if (!enabled) {
                    beginStop(State::ERROR, "enable_failed");
                    return;
                }
                setState(State::RUNNING);
            });
            if (!accepted && token == generation && pending)
                beginStop(State::ERROR, "enable_rejected");
        });
    });
}

void NavigationManager::beginStop(State terminal, const char *message) {
    ++generation; // Invalidate queued position/setup/movement callbacks before sending a stop.
    pending = false;
    cancelRequested = false;
    terminalState = terminal;
    reason = message;
    cleanupRequired = enteredManual;
    if (!cleanupRequired) {
        serial.setNavigationActive(false);
        setState(terminalState);
        return;
    }
    setState(State::STOPPING);
    cleanup();
}

void NavigationManager::cleanup() {
    pending = true;
    pendingMs = millis();
    uint32_t token = generation;
    auto done = [this, token](bool ok) {
        std::lock_guard<std::recursive_mutex> lock(mutex);
        if (token == generation)
            finishCleanup(ok);
    };
    if (manual.isBusy()) {
        if (!manual.enable(false, done) && pending)
            finishCleanup(false);
        return;
    }
    // Failed setup may have entered TestMode without becoming active. Tear it down explicitly.
    serial.setMotorWheels(0, 0, 0, [this, token, done](bool stopped) {
        std::lock_guard<std::recursive_mutex> lock(mutex);
        if (token != generation)
            return;
        serial.setLdsRotation(false, [this, token, done, stopped](bool ldsOff) {
            std::lock_guard<std::recursive_mutex> lock(mutex);
            if (token != generation)
                return;
            serial.testMode(false, [done, stopped, ldsOff](bool off) { done(stopped && ldsOff && off); });
        });
    });
}

void NavigationManager::finishCleanup(bool ok) {
    pending = false;
    cleanupRequired = !ok;
    if (!ok) {
        reason = "cleanup_failed";
        setState(State::ERROR);
        return;
    }
    enteredManual = false;
    manual.releaseNavigationControl();
    serial.setNavigationActive(false);
    setState(terminalState);
}

void NavigationManager::pollPose() {
    pending = true;
    pendingMs = millis();
    uint32_t token = generation;
    serial.getRobotPos(true, [this, token](bool ok, const RobotPosData& data) {
        std::lock_guard<std::recursive_mutex> lock(mutex);
        if (token != generation || cancelRequested)
            return;
        pending = false;
        NavigationPose next;
        if (!ok || !parseNavigationPose(data.raw.c_str(), next)) {
            beginStop(State::ERROR, "invalid_pose");
            return;
        }
        if (hasPose && next.time <= pose.time) {
            beginStop(State::ERROR, "stale_pose");
            return;
        }
        if (hasPose &&
            (navigationDistance(pose, next) > 0.3f || std::fabs(std::remainder(next.t - pose.t, 360.0f)) > 45)) {
            beginStop(State::ERROR, "pose_jump");
            return;
        }
        if (!hasPose && navigationDistance(next, route.points[0]) > 0.25f) {
            beginStop(State::ERROR, "start_too_far");
            return;
        }
        pose = next;
        hasPose = true;
        if (!manual.navigationSafetyFresh() || manual.navigationBlocked()) {
            beginStop(State::ERROR, "unsafe_feedback");
            return;
        }
        NavigationMove move;
        while (waypoint < route.count) {
            move = navigationMove(pose, route.points[waypoint]);
            if (!move.arrived)
                break;
            ++waypoint;
            waypointMs = millis();
        }
        if (waypoint == route.count) {
            beginStop(State::DONE, "");
            return;
        }
        pending = true;
        pendingMs = millis();
        hasMoved = true;
        bool accepted = manual.move(move.left, move.right, move.speed, [this, token](bool moved) {
            std::lock_guard<std::recursive_mutex> lock(mutex);
            if (token != generation || cancelRequested)
                return;
            pending = false;
            if (!moved) {
                beginStop(State::ERROR, "move_failed");
                return;
            }
            lastMoveMs = millis();
        });
        if (!accepted && token == generation && pending)
            beginStop(State::ERROR, "move_rejected");
    });
}

void NavigationManager::tick() {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    uint32_t now = millis();
    if (cancelRequested) {
        // A stop while cleanup is pending does not invalidate its callback or enqueue a second teardown.
        if (state == State::STOPPING) {
            cancelRequested = false;
        } else {
            beginStop(State::CANCELLED, "");
        }
        return;
    }
    if (pending && static_cast<uint32_t>(now - pendingMs) > 10000) {
        if (state == State::STOPPING) {
            ++generation;
            finishCleanup(false);
        } else {
            beginStop(State::ERROR, "command_timeout");
        }
        return;
    }
    if (state == State::STARTING && !pending) {
        beginStart();
        return;
    }
    if (state != State::RUNNING)
        return;
    manual.navigationHeartbeat();
    if (!manual.isActive() || manual.navigationBlocked()) {
        beginStop(State::ERROR, "obstacle");
        return;
    }
    if (!manual.navigationSafetyFresh()) {
        if (hasPose || static_cast<uint32_t>(now - startedMs) > 10000)
            beginStop(State::ERROR, "safety_feedback_lost");
        return;
    }
    if (static_cast<uint32_t>(now - startedMs) > 600000 || static_cast<uint32_t>(now - waypointMs) > 60000) {
        beginStop(State::ERROR, "progress_timeout");
        return;
    }
    // Wait for a finite <=100 mm move at 50 mm/s to settle before asking for fresh feedback.
    if (!pending && (!hasMoved || static_cast<uint32_t>(now - lastMoveMs) >= 2500))
        pollPose();
}

String NavigationManager::getStatusJson() const {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    char json[512];
    std::snprintf(json, sizeof(json),
                  "{\"enabled\":%s,\"state\":\"%s\",\"reason\":\"%s\",\"waypoint\":%u,\"count\":%u,"
                  "\"poseValid\":%s,\"x\":%.4f,\"y\":%.4f,\"t\":%.2f,\"cleanupRequired\":%s}",
                  OPENNEATO_EXPERIMENTAL_NAVIGATION ? "true" : "false", stateName(state), reason,
                  static_cast<unsigned>(waypoint), static_cast<unsigned>(route.count), hasPose ? "true" : "false",
                  static_cast<double>(pose.x), static_cast<double>(pose.y), static_cast<double>(pose.t),
                  cleanupRequired ? "true" : "false");
    return String(json);
}
