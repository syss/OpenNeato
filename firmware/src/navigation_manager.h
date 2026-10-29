#ifndef NAVIGATION_MANAGER_H
#define NAVIGATION_MANAGER_H

#include "loop_task.h"
#include "navigation_math.h"
#include <mutex>

class NeatoSerial;
class ManualCleanManager;
class DataLogger;

class NavigationManager : public LoopTask {
public:
    NavigationManager(NeatoSerial& serial, ManualCleanManager& manual, DataLogger& logger);
    int start(const char *json); // HTTP status; only submits work, never touches hardware.
    void stop();
    bool isBusy() const;
    String getStatusJson() const;

private:
    enum class State { IDLE, STARTING, RUNNING, STOPPING, DONE, CANCELLED, ERROR };
    NeatoSerial& serial;
    ManualCleanManager& manual;
    DataLogger& logger;
    // HTTP callbacks and the firmware loop share submissions/status. Serial callbacks may be synchronous.
    mutable std::recursive_mutex mutex;
    NavigationRoute route;
    NavigationPose pose;
    State state = State::IDLE;
    State terminalState = State::IDLE;
    const char *reason = "";
    size_t waypoint = 0;
    uint32_t generation = 0;
    uint32_t startedMs = 0;
    uint32_t pendingMs = 0;
    uint32_t lastMoveMs = 0;
    uint32_t waypointMs = 0;
    bool pending = false;
    bool cancelRequested = false;
    bool cleanupRequired = false;
    bool enteredManual = false;
    bool hasPose = false;
    bool hasMoved = false;

    void tick() override;
    void beginStart();
    void pollPose();
    void beginStop(State terminal, const char *message);
    void cleanup();
    void finishCleanup(bool ok);
    void setState(State next);
    static const char *stateName(State value);
};

#endif // NAVIGATION_MANAGER_H
