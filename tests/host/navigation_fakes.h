#pragma once
#include "Arduino.h"
#include <deque>
#include <functional>
#include <vector>
#include <utility>

// Replace only hardware/storage boundaries; compile the real manual and navigation managers.
#define NEATO_SERIAL_H
#define JSON_FIELDS_H
#define WEB_SERVER_H
#define DATA_LOGGER_H
enum FieldType { FIELD_INT, FIELD_FLOAT, FIELD_BOOL, FIELD_STRING };
struct Field {
    String key;
    String value;
    FieldType type;
};
inline String fieldsToJson(const std::vector<Field>& fields) {
    String result = "{";
    for (const auto& f: fields) {
        if (result.size() > 1)
            result += ',';
        result += '"' + f.key + "\":";
        if (f.type == FIELD_STRING)
            result += '"' + f.value + '"';
        else
            result += f.value;
    }
    return result + '}';
}
struct DigitalSensorData {
    bool navigationValid = true;
    bool lFrontBit = false, rFrontBit = false, lLdsBit = false, rLdsBit = false;
    bool lSideBit = false, rSideBit = false, leftWheelExtended = false, rightWheelExtended = false;
};
struct MotorData {
    bool navigationValid = true;
    int leftWheelLoad = 0, rightWheelLoad = 0;
};
struct RobotState {
    bool idle = true;
    bool isIdle() const { return idle; }
};
struct ChargerData {
    bool navigationValid = true;
    int fuelPercent = 90;
    bool extPwrPresent = false, chargingActive = false;
};
struct RobotPosData {
    String raw;
};
enum CommandPriority { PRIORITY_HIGH };
class NeatoSerial {
public:
    struct Wheels {
        int left;
        int right;
        int speed;
    };
    std::deque<std::function<void()>> pending;
    std::vector<Wheels> wheels;
    std::vector<bool> modes;
    DigitalSensorData sensors;
    MotorData motors;
    RobotState state;
    ChargerData charger;
    RobotPosData pose{"X=0 Y=0 Theta=0 Time=1"};
    bool ok = true, safetyOk = true, moveOk = true, stopOk = true, exitOk = true, enableOk = true;
    bool navigationActive = false;
    bool rejectMove = false, rejectStop = false, rejectEnable = false;
    int queueDepth() const { return 0; }
    void setManualCleanActive(bool) {}
    void setNavigationActive(bool active) { navigationActive = active; }
    void invalidateAll() {}
    void getState(std::function<void(bool, const RobotState&)> cb) {
        pending.push_back([=] { cb(ok, state); });
    }
    void getCharger(std::function<void(bool, const ChargerData&)> cb) {
        pending.push_back([=] { cb(ok, charger); });
    }
    void getRobotPos(bool, std::function<void(bool, const RobotPosData&)> cb) {
        pending.push_back([=] { cb(ok, pose); });
    }
    void getDigitalSensors(std::function<void(bool, const DigitalSensorData&)> cb, CommandPriority) {
        pending.push_back([=] { cb(safetyOk, sensors); });
    }
    void getMotors(std::function<void(bool, const MotorData&)> cb, CommandPriority) {
        pending.push_back([=] { cb(ok, motors); });
    }
    bool testMode(bool active, std::function<void(bool)> cb) {
        if (active && rejectEnable) {
            if (cb)
                cb(false);
            return false;
        }
        modes.push_back(active);
        pending.push_back([=] {
            if (cb)
                cb(active ? (ok && enableOk) : exitOk);
        });
        return true;
    }
    bool setLdsRotation(bool, std::function<void(bool)> cb) {
        pending.push_back([=] {
            if (cb)
                cb(ok);
        });
        return true;
    }
    bool setMotorWheels(int l, int r, int speed, std::function<void(bool)> cb) {
        if (((l || r) && rejectMove) || ((!l && !r) && rejectStop)) {
            if (cb)
                cb(false);
            return false;
        }
        wheels.push_back({l, r, speed});
        bool result = l == 0 && r == 0 ? stopOk : moveOk;
        pending.push_back([=] {
            if (cb)
                cb(result);
        });
        return true;
    }
    bool setMotorBrush(int, std::function<void(bool)> cb) {
        if (cb)
            cb(ok);
        return ok;
    }
    bool setMotorVacuum(bool, int, std::function<void(bool)> cb) {
        if (cb)
            cb(ok);
        return ok;
    }
    bool setMotorSideBrush(bool, int, std::function<void(bool)> cb) {
        if (cb)
            cb(ok);
        return ok;
    }
    void drain() {
        unsigned count = 0;
        while (!pending.empty()) {
            auto cb = std::move(pending.front());
            pending.pop_front();
            cb();
            if (++count > 100)
                std::abort();
        }
    }
};
class WebServer {
public:
    static inline unsigned long lastApiActivity = 0;
};
class DataLogger {
public:
    void logNavigation(const char *, const char *, size_t) {}
};
