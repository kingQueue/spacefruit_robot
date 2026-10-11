#include "motion_protocol.hpp"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <initializer_list>

static bool send(robot::MotionProtocol& protocol, const char* body, std::uint32_t now,
                 const robot::MotionSensors& sensors) {
    char frame[96];
    const auto checksum = robot::crc16(body, static_cast<unsigned>(std::strlen(body)));
    std::snprintf(frame, sizeof(frame), "%s*%04X", body, checksum);
    return protocol.receive(frame, now, sensors);
}
int main() {
    robot::MotionSensors sensors{1000,1000,0,0,false,false};
    robot::MotionProtocol protocol;
    assert(!send(protocol,"D 1 20 20 200",0,sensors));
    assert(send(protocol,"A 2",0,sensors));
    assert(send(protocol,"D 3 20 -20 200",1,sensors));
    assert(protocol.poll(2,sensors).left_pwm == 20);
    assert(!send(protocol,"D 3 20 20 200",2,sensors));
    assert(!protocol.poll(2,sensors).armed);
    assert(send(protocol,"A 4",3,sensors));
    assert(!send(protocol,"D 5 20 20 200 trailing",4,sensors));
    assert(send(protocol,"A 6",5,sensors));
    assert(!send(protocol,"D 42949672960 20 20 200",6,sensors));
    assert(send(protocol,"A 7",7,sensors));
    assert(!send(protocol,"D 8 -2147483648 20 200",8,sensors));
    assert(send(protocol,"A 9",9,sensors));
    assert(!send(protocol,"S 1",10,sensors));
    assert(!protocol.poll(10,sensors).armed);
    assert(protocol.acknowledged_sequence() == 9);
    assert(send(protocol,"A 10",11,sensors));
    char corrupt[] = "D 11 20 20 200*0000";
    assert(!protocol.receive(corrupt,12,sensors));
    assert(!protocol.poll(12,sensors).armed);
    assert(send(protocol,"A 12",13,sensors));
    assert(send(protocol,"D 13 20 20 100",14,sensors));
    assert(!protocol.poll(114,sensors).armed);
    assert(!send(protocol,"D 14 20 20 100",114,sensors));
    assert(protocol.acknowledged_sequence()==14);
    assert(send(protocol,"A 15",115,sensors)); // recover using telemetry ACK+1
    assert(!protocol.receive(nullptr,115,sensors));
    for (const char* bad : {"A", "A 0", "A +16", "A  16", "D 16 101 0 100", "D 16 0 0 99", "P 16"})
        assert(!send(protocol,bad,116,sensors));
}
