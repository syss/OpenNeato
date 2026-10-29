#include "neato_commands.h"
#include "neato_serial.h"
#include <cassert>
#include <cstdio>
uint32_t testMillis = 100;
int main() {
    DigitalSensorData digital;
    assert(parseDigitalSensorData("SNSR_DC_JACK_IS_IN,0\r\n", digital));
    assert(!digital.navigationValid);
    const char *complete = "SNSR_DC_JACK_IS_IN,0\r\nSNSR_LEFT_WHEEL_EXTENDED,0\r\n"
                           "SNSR_RIGHT_WHEEL_EXTENDED,0\r\nLSIDEBIT,0\r\nLFRONTBIT,0\r\nLLDSBIT,0\r\n"
                           "RSIDEBIT,0\r\nRFRONTBIT,0\r\nRLDSBIT,0\r\n";
    assert(parseDigitalSensorData(complete, digital) && digital.navigationValid);
    String malformed = complete;
    malformed.replace("LFRONTBIT,0", "LFRONTBIT,bad");
    assert(parseDigitalSensorData(malformed, digital) && !digital.navigationValid);
    malformed = complete;
    malformed.replace("LFRONTBIT,0\r\n", "");
    assert(parseDigitalSensorData(malformed, digital) && !digital.navigationValid);
    ChargerData charger;
    assert(parseChargerData("FuelPercent,90\r\n", charger) && !charger.navigationValid);
    assert(parseChargerData("FuelPercent,90\r\nExtPwrPresent,0\r\nChargingActive,0\r\n", charger));
    assert(charger.navigationValid);
    MotorData motors;
    parseMotorData("Brush_RPM,0\r\n", motors);
    assert(!motors.navigationValid);
    parseMotorData("LeftWheel_Load%,0\r\nRightWheel_Load%,0\r\n", motors);
    assert(motors.navigationValid);
    {
        NeatoSerial serial;
        bool called = false, result = true;
        assert(serial.setMotorWheels(0, 0, 0, [&](bool ok) {
            called = true;
            result = ok;
        }));
        serial.loop();
        serial.loop();
        assert(Serial1.output.back().indexOf("LWheelDisable") >= 0);
        testMillis += 3001;
        serial.loop(); // wheel-disable timeout
        assert(called && !result);
        testMillis += 51;
        serial.loop();
        serial.loop();
        serial.loop();
        for (const auto& cmd: Serial1.output)
            assert(cmd.indexOf("LWheelEnable") < 0);
    }
    {
        Serial1.output.clear();
        Serial1.input.clear();
        NeatoSerial serial;
        bool called = false, result = false;
        assert(serial.setMotorWheels(0, 0, 0, [&](bool ok) {
            called = true;
            result = ok;
        }));
        serial.loop();
        serial.loop();
        Serial1.reply("SetMotor\r\nOK\r\n\x1a");
        serial.loop();
        assert(!called); // disable alone is not completion; re-enable must succeed too
        testMillis += 51;
        serial.loop();
        serial.loop();
        serial.loop();
        assert(Serial1.output.back().indexOf("LWheelEnable") >= 0);
        Serial1.reply("SetMotor\r\nOK\r\n\x1a");
        serial.loop();
        assert(called && result);
    }
    {
        NeatoSerial serial;
        for (int i = 0; i < NEATO_QUEUE_MAX_SIZE; ++i)
            assert(serial.setMotorWheels(1, 1, 50, nullptr));
        bool rejected = false;
        assert(!serial.setMotorWheels(0, 0, 0, [&](bool ok) { rejected = !ok; }));
        assert(rejected); // full queues fail synchronously, never report a successful stop
        serial.setNavigationActive(true);
        assert(!serial.clean("house"));
        assert(!serial.powerControl("restart"));
        assert(!serial.sendRaw("TestMode Off", nullptr));
    }
    std::puts("real serial parsers: incomplete/invalid navigation safety data rejected");
    std::puts("real serial queue: failed stop, disable/enable sequence, full queue and ownership passed");
}
