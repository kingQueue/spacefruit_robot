#include "mission.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using namespace robot;
static HardwareResult good() {
    HardwareResult result;
    result.success = result.verified = true;
    result.plot = {6,8};
    result.environment = {22,60};
    return result;
}
static bool done(Mission& m, HardwareResult r = good(), std::uint64_t now = 1) {
    assert(m.pending());
    return m.complete_action(m.pending()->id,r,now);
}
static void setup(Mission& m) {
    assert(m.start(0));
    assert(!m.start(0));
    assert(m.pending()->kind == ActionKind::map_plot);
    assert(done(m));
    assert(m.pending()->kind == ActionKind::read_environment);
    assert(done(m));
    assert(m.state() == MissionState::waiting_for_confirmation);
    assert(m.environment().temperature_c == 22);
}
static std::vector<MissionCell> plan() {
    return {{{"Radish",0.15,0.01},0.25,0.5},{{"Radish",0.15,0.01},0.5,0.5}};
}
static void plant(Mission& m) {
    setup(m);
    assert(!m.confirm_plan());
    assert(m.set_plan(plan()));
    assert(m.confirm_plan());
    assert(!m.start_planting(0));
    assert(!m.set_plan(plan()));
    assert(!m.load_seeds({{"Radish",1}}));
    assert(!m.load_seeds({{"Radish",2},{"Carrot",1}}));
    assert(m.load_seeds({{"Radish",2}}));
    assert(m.start_planting(1));
    assert(!m.start_planting(1));
    for (unsigned i=0;i<2;++i) {
        assert(m.pending()->kind == ActionKind::navigate);
        assert(m.pending()->cell == i);
        assert(done(m));
        assert(m.pending()->kind == ActionKind::plant_seed);
        assert(m.pending()->crop.name == "Radish");
        assert(m.cells()[i].state == CellState::planned);
        assert(done(m));
        assert(m.cells()[i].state == CellState::planted);
    }
    assert(m.state() == MissionState::complete && !m.pending());
    assert(m.progress().planted == 2);
}
int main() {
    Mission m;
    assert(!m.start_monitoring(0) && !m.start_harvest(0,0) && !m.confirm_plan());
    plant(m);
    assert(m.start_monitoring(2));
    assert(!m.start_harvest(0,2));
    assert(done(m)); // navigate
    auto r=good(); r.weeds=2;
    assert(done(m,r));
    assert(m.pending()->kind == ActionKind::remove_weeds && m.pending()->count == 2);
    assert(done(m,r));
    r=good(); r.health_issue=true;
    assert(done(m,r));
    assert(done(m)); // next navigate
    assert(done(m)); // no weeds
    assert(m.pending()->kind == ActionKind::inspect_health);
    assert(done(m));
    assert(m.state() == MissionState::complete);
    assert(m.progress().checked==2 && m.progress().weed_locations==2);
    assert(m.progress().weeds_removed==2 && m.progress().issues==1);
    assert(m.cells()[0].health_issue && !m.cells()[1].health_issue);
    assert(!m.start_harvest(2,3));
    assert(m.start_harvest(0,3));
    assert(done(m));
    r=good(); r.readiness=Readiness::not_ready;
    assert(done(m,r));
    assert(m.inventory().empty() && m.cells()[0].state==CellState::planted);
    assert(m.start_harvest(0,4));
    assert(done(m));
    r.readiness=Readiness::ready;
    assert(done(m,r));
    assert(m.inventory().empty());
    const auto collect_id=m.pending()->id;
    assert(done(m));
    assert(!m.complete_action(collect_id,good(),5)); // no duplicate inventory
    assert(m.inventory().at("Radish")==1 && m.cells()[0].state==CellState::empty);
    assert(!m.start_harvest(0,5));
    assert(!m.start_replant(0,{"Carrot",0.1,0.02},{},5));
    assert(!m.start_replant(0,{"Tomato",1,0.01},{{"Tomato",1}},5)); // spacing
    assert(m.start_replant(0,{"Carrot",0.1,0.02},{{"Carrot",1}},5));
    assert(m.pending()->crop.name=="Carrot");
    assert(done(m));
    assert(done(m));
    assert(m.cells()[0].crop.name=="Carrot" && m.cells()[0].generation==2);
    assert(m.inventory().at("Radish")==1);

    // Invalid measured inputs, malformed plans and exact seed gates.
    Mission bad;
    setup(bad);
    assert(!bad.set_plan({}));
    auto invalid=plan(); invalid[0].x_m=0.1;
    assert(!bad.set_plan(invalid));
    invalid=plan(); invalid[0].crop.depth_m=std::numeric_limits<double>::quiet_NaN();
    assert(!bad.set_plan(invalid));
    invalid=plan(); invalid[1].x_m=invalid[0].x_m;
    assert(!bad.set_plan(invalid));
    invalid=plan(); invalid[1].crop.depth_m=0.02;
    assert(!bad.set_plan(invalid));
    for (const auto kind : {ActionKind::map_plot,ActionKind::read_environment}) {
        Mission failure; assert(failure.start(0));
        if (kind==ActionKind::read_environment) assert(done(failure));
        r=good();
        if (kind==ActionKind::map_plot) r.plot.width_m=0.2;
        else r.environment.humidity_percent=101;
        assert(!done(failure,r));
        assert(failure.state()==MissionState::error && failure.pending()->kind==ActionKind::stop);
    }
    // Every issued hardware step fails closed on a rejected/unverified result.
    for (const auto kind : {ActionKind::navigate,ActionKind::plant_seed,
        ActionKind::inspect_weeds,ActionKind::remove_weeds,ActionKind::inspect_health,
        ActionKind::inspect_readiness,ActionKind::harvest}) {
        Mission failure; plant(failure);
        if (kind==ActionKind::navigate || kind==ActionKind::plant_seed) {
            assert(failure.start_replant(0,{"Radish",0.15,0.01},{{"Radish",1}},2)==false);
            assert(failure.start_harvest(0,2));
            if (kind==ActionKind::plant_seed) {
                assert(done(failure)); r=good(); r.readiness=Readiness::ready;
                assert(done(failure,r)); assert(done(failure));
                assert(failure.start_replant(0,{"Radish",0.15,0.01},{{"Radish",1}},2));
                assert(done(failure));
            }
        } else if (kind==ActionKind::inspect_readiness || kind==ActionKind::harvest) {
            assert(failure.start_harvest(0,2)); assert(done(failure));
            if (kind==ActionKind::harvest) {r=good();r.readiness=Readiness::ready;assert(done(failure,r));}
        } else {
            assert(failure.start_monitoring(2)); assert(done(failure));
            if (kind!=ActionKind::inspect_weeds) {
                r=good(); r.weeds=kind==ActionKind::remove_weeds?1:0; assert(done(failure,r));
            }
        }
        assert(failure.pending()->kind==kind);
        r=good(); r.verified=false;
        const auto old=failure.pending()->id;
        assert(!done(failure,r));
        assert(failure.state()==MissionState::error && failure.pending()->kind==ActionKind::stop);
        assert(!failure.complete_action(old,good(),2));
        assert(!failure.start_monitoring(2));
        assert(done(failure)); // verified stop leaves error latched
        assert(failure.state()==MissionState::error);
    }
    Mission timed; assert(timed.start(0));
    const auto old=timed.pending()->id;
    timed.tick(30000,true);
    assert(timed.state()==MissionState::error);
    assert(!timed.complete_action(old,good(),30000));
    Mission emergency; plant(emergency); emergency.tick(2,false);
    assert(emergency.state()==MissionState::error && emergency.pending()->kind==ActionKind::stop);
    Mission cancelled; plant(cancelled); cancelled.cancel(3);
    assert(cancelled.state()==MissionState::error);
    Mission unknown; plant(unknown); assert(unknown.start_harvest(0,2));assert(done(unknown));
    assert(!done(unknown)); assert(unknown.inventory().empty());
    Mission partial; plant(partial); assert(partial.start_monitoring(2));assert(done(partial));
    r=good();r.weeds=2;assert(done(partial,r));r.weeds=1;
    assert(!done(partial,r)); assert(partial.progress().weed_locations==0);
    std::cout << "PASS: complete mission flow, validation, exact seeds, verified tools, failures, cancellation, timeout\n";
}
