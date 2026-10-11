#include "mission.hpp"
extern "C" {
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
}
#include <cstdio>
#include <cstring>
#include <type_traits>

namespace {
using namespace robot;
struct Request { std::uint64_t id; ActionKind kind; unsigned count; };
struct Reply { std::uint64_t id; HardwareResult result; };
static_assert(std::is_trivially_copyable_v<Request> && std::is_trivially_copyable_v<Reply>);
QueueHandle_t requests, replies;
int outcome = 1;
const char* names[] = {"map_plot","read_environment","navigate","plant_seed",
    "inspect_weeds","remove_weeds","inspect_health","inspect_readiness","harvest","stop"};
void fake_hardware(void*) {
    Request request{};
    for (;;) {
        configASSERT(xQueueReceive(requests,&request,portMAX_DELAY)==pdPASS);
        HardwareResult result{};
        result.success = result.verified = true; // Explicit simulation only.
        result.plot = {6,8};
        result.environment = {22,60};
        result.readiness = Readiness::ready;
        result.weeds = request.kind==ActionKind::inspect_weeds ? 1 : request.count;
        const Reply reply{request.id,result};
        configASSERT(xQueueSend(replies,&reply,portMAX_DELAY)==pdPASS);
    }
}
std::uint64_t now() { return static_cast<std::uint64_t>(xTaskGetTickCount()) * portTICK_PERIOD_MS; }
void mission_task(void*) {
    Mission mission;
    configASSERT(mission.start(now()));
    unsigned phase=0;
    for (;;) {
        mission.tick(now(),true); // Real adapter must supply actual safety state.
        if (mission.state()==MissionState::error) {
            std::fprintf(stderr,"mission_error=%s\n",mission.error().c_str());
            vTaskEndScheduler();
        }
        if (mission.pending()) {
            const auto action=*mission.pending();
            std::printf("SIM action=%s cell=%zu crop=%s count=%u\n",names[static_cast<unsigned>(action.kind)],
                action.cell,action.crop.name.c_str(),action.count);
            const Request request{action.id,action.kind,action.count};
            configASSERT(xQueueSend(requests,&request,portMAX_DELAY)==pdPASS);
            Reply reply{};
            configASSERT(xQueueReceive(replies,&reply,pdMS_TO_TICKS(1000))==pdPASS);
            configASSERT(mission.complete_action(reply.id,reply.result,now()));
        } else if (mission.state()==MissionState::waiting_for_confirmation) {
            configASSERT(mission.set_plan({{{"Radish",0.15,0.01},0.25,0.5},
                {{"Onion",0.15,0.02},0.25,1.0}}));
            configASSERT(mission.confirm_plan());
            configASSERT(mission.load_seeds({{"Radish",1},{"Onion",1}}));
            configASSERT(mission.start_planting(now()));
        } else if (mission.state()==MissionState::complete) {
            if (phase==0) configASSERT(mission.start_monitoring(now()));
            else if (phase==1) configASSERT(mission.start_harvest(0,now()));
            else if (phase==2) configASSERT(mission.start_replant(0,{"Carrot",0.1,0.02},{{"Carrot",1}},now()));
            else {
                std::printf("SIM complete growing=2 harvested_radish=%u checked=%u replanted=Carrot\n",
                    mission.inventory().at("Radish"),mission.progress().checked);
                std::fflush(stdout);
                outcome=0;
                vTaskEndScheduler();
            }
            ++phase;
        }
    }
}
}
int main(int argc,char** argv) {
    if (argc!=2 || std::strcmp(argv[1],"--simulate")!=0) {
        std::fprintf(stderr,"Usage: mission_demo --simulate (no hardware actuation)\n");
        return 2;
    }
    requests=xQueueCreate(1,sizeof(Request));
    replies=xQueueCreate(1,sizeof(Reply));
    configASSERT(requests && replies);
    configASSERT(xTaskCreate(fake_hardware,"sim_hw",configMINIMAL_STACK_SIZE,nullptr,1,nullptr)==pdPASS);
    configASSERT(xTaskCreate(mission_task,"mission",configMINIMAL_STACK_SIZE*2,nullptr,2,nullptr)==pdPASS);
    vTaskStartScheduler();
    return outcome;
}
