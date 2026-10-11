#include "motion_guard.hpp"
#include <cstdio>
#include <cstring>
#define CHECK(x) do { if (!(x)) { std::fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x); return 1; } } while(0)
int main() {
    using namespace robot;
    MotionSensors s{1000,1000,0,0,false,false};
    MotionGuard g;
    CHECK(!g.drive(1,50,50,300,0,s));
    CHECK(g.arm(2,0,s)); CHECK(g.drive(3,50,50,300,10,s));
    CHECK(g.poll(309,s).left_pwm == 50);
    CHECK(g.poll(310,s).fault == MotionFault::timeout);
    CHECK(!g.drive(4,50,50,300,320,s));
    CHECK(g.arm(5,330,s)); CHECK(!g.drive(5,50,50,300,340,s));
    CHECK(g.poll(350,s).fault == MotionFault::command);
    CHECK(g.arm(6,400,s)); CHECK(!g.drive(7,101,0,300,410,s));
    CHECK(g.arm(8,500,s)); CHECK(!g.drive(9,1,1,99,510,s));
    CHECK(g.arm(10,600,s)); s.front_mm=300;
    CHECK(!g.drive(11,50,50,300,610,s));
    CHECK(g.poll(620,s).fault == MotionFault::obstacle);
    s.front_mm=1000; CHECK(g.arm(12,700,s)); s.rear_mm=200;
    CHECK(!g.drive(13,-10,-10,300,710,s));
    s.rear_mm=1000; CHECK(g.arm(14,800,s)); s.front_age_ms=201;
    CHECK(g.poll(810,s).fault == MotionFault::sensor);
    s.front_age_ms=0; s.bumper=true;
    CHECK(!g.arm(15,900,s)); CHECK(g.poll(900,s).fault == MotionFault::emergency);
    s.bumper=false; s.emergency=true; CHECK(!g.arm(16,950,s));
    s.emergency=false; s.front_mm=-1; CHECK(!g.arm(17,960,s));
    s.front_mm=1000; CHECK(g.arm(18,0xffffff00u,s));
    CHECK(g.drive(19,20,20,300,0xfffffff0u,s));
    CHECK(g.poll(0x20,s).armed); CHECK(!g.poll(0x200,s).armed);
    g.stop(); CHECK(g.poll(0x210,s).left_pwm==0);
    CHECK(crc16("123456789",9)==0x29b1);
    char frame[96]; const char* body="D 22 30 30 300";
    std::snprintf(frame,sizeof(frame),"%s*%04X",body,crc16(body,static_cast<unsigned>(std::strlen(body))));
    CHECK(validate_frame(frame)); CHECK(std::strcmp(frame,body)==0);
    char bad[]="S 2*0000"; CHECK(!validate_frame(bad));
    char malformed[]="S 2*ZZZZ"; CHECK(!validate_frame(malformed));
    puts("PASS: motion arming, sequences, TTL, sensor gates, obstacle/E-stop, CRC");
}
