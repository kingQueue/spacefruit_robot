#include "robot_control.hpp"
extern "C" {
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
}
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>

namespace {
QueueHandle_t sensor_queue;
QueueHandle_t output_queue;
unsigned long sample_limit = 0;
struct Report { unsigned long sequence; robot::Decision decision; };
constexpr unsigned inputs[] = {20, 35, 50, 101};
constexpr unsigned period_ms = 200;

void sensor_task(void*) {
    TickType_t wake = xTaskGetTickCount();
    unsigned index = 0;
    for (;;) {
        const unsigned moisture = inputs[index];
        configASSERT(xQueueSend(sensor_queue, &moisture, portMAX_DELAY) == pdPASS);
        index = (index + 1) % 4;
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(period_ms));
    }
}
void control_task(void*) {
    robot::Controller controller;
    unsigned moisture;
    unsigned long sequence = 0;
    for (;;) {
        configASSERT(xQueueReceive(sensor_queue, &moisture, portMAX_DELAY) == pdPASS);
        const Report report{++sequence, controller.update(moisture)};
        configASSERT(xQueueSend(output_queue, &report, portMAX_DELAY) == pdPASS);
    }
}
void logger_task(void*) {
    Report report{};
    // Only this task uses stdio: the POSIX port can preempt inside libc locks.
    for (;;) {
        configASSERT(xQueueReceive(output_queue, &report, portMAX_DELAY) == pdPASS);
        std::printf("sample=%lu moisture=%u pump=%u valid=%u\n", report.sequence,
                    report.decision.moisture, report.decision.pump_requested ? 1u : 0u,
                    report.decision.valid ? 1u : 0u);
        std::fflush(stdout);
        if (sample_limit != 0 && report.sequence >= sample_limit) {
            vTaskEndScheduler();
        }
    }
}
}
int main(int argc, char** argv) {
    if (argc != 1) {
        if (argc != 3 || std::strcmp(argv[1], "--samples") != 0) {
            std::fprintf(stderr, "Usage: robot_demo [--samples positive_integer]\n");
            return 2;
        }
        char* end = nullptr;
        errno = 0;
        sample_limit = std::strtoul(argv[2], &end, 10);
        if (errno || end == argv[2] || *end || argv[2][0] == '-' || sample_limit == 0) {
            std::fprintf(stderr, "Invalid sample count\n");
            return 2;
        }
    }
    sensor_queue = xQueueCreate(8, sizeof(unsigned));
    output_queue = xQueueCreate(8, sizeof(Report));
    configASSERT(sensor_queue && output_queue);
    configASSERT(xTaskCreate(sensor_task, "sensor", configMINIMAL_STACK_SIZE,
                            nullptr, 1, nullptr) == pdPASS);
    configASSERT(xTaskCreate(control_task, "control", configMINIMAL_STACK_SIZE,
                            nullptr, 2, nullptr) == pdPASS);
    configASSERT(xTaskCreate(logger_task, "logger", configMINIMAL_STACK_SIZE,
                            nullptr, 3, nullptr) == pdPASS);
    vTaskStartScheduler();
    return sample_limit == 0 ? 1 : 0;
}
