#pragma once
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace robot {
enum class MissionState { waiting_for_start, mapping, reading_environment,
    waiting_for_confirmation, waiting_for_seeds, planting, monitoring,
    harvesting, replanting, complete, error };
enum class ActionKind { map_plot, read_environment, navigate, plant_seed,
    inspect_weeds, remove_weeds, inspect_health, inspect_readiness, harvest, stop };
enum class CellState { planned, planted, empty };
enum class Readiness { unknown, not_ready, ready };
struct Crop { std::string name; double spacing_m = 0, depth_m = 0; };
struct MissionCell {
    Crop crop;
    double x_m = 0, y_m = 0;
    CellState state = CellState::planned;
    bool health_issue = false;
    unsigned generation = 0;
};
struct PlotBounds { double width_m = 0, length_m = 0; };
struct Environment { double temperature_c = 0, humidity_percent = 0; };
struct HardwareAction {
    std::uint64_t id;
    ActionKind kind;
    std::size_t cell;
    unsigned count;
    Crop crop;
    double x_m, y_m;
    std::uint64_t deadline_ms;
};
struct HardwareResult {
    bool success = false;
    // Verification is physical feedback/model/operator evidence, not serial ACK.
    bool verified = false;
    PlotBounds plot;
    Environment environment;
    unsigned weeds = 0;
    bool health_issue = false;
    Readiness readiness = Readiness::unknown;
};
struct MissionProgress { unsigned planted = 0, checked = 0, weed_locations = 0,
    weeds_removed = 0, issues = 0; };

// Single-owner Pi task. Hardware work is asynchronous and completed by ID.
// The app supplies crop recommendations/grid positions; this core validates them.
class Mission {
public:
    bool start(std::uint64_t now);
    bool set_plan(std::vector<MissionCell> cells);
    bool confirm_plan();
    bool load_seeds(const std::map<std::string, unsigned>& counts);
    bool start_planting(std::uint64_t now);
    bool start_monitoring(std::uint64_t now);
    bool start_harvest(std::size_t cell, std::uint64_t now);
    bool start_replant(std::size_t cell, Crop crop,
                       const std::map<std::string, unsigned>& loaded, std::uint64_t now);
    bool complete_action(std::uint64_t id, const HardwareResult& result, std::uint64_t now);
    void tick(std::uint64_t now, bool safe);
    void cancel(std::uint64_t now);
    MissionState state() const noexcept { return state_; }
    const std::optional<HardwareAction>& pending() const noexcept { return pending_; }
    const std::vector<MissionCell>& cells() const noexcept { return cells_; }
    const std::map<std::string, unsigned>& inventory() const noexcept { return inventory_; }
    const MissionProgress& progress() const noexcept { return progress_; }
    const std::string& error() const noexcept { return error_; }
    const Environment& environment() const noexcept { return environment_; }
private:
    void issue(ActionKind kind, std::uint64_t now, unsigned count = 0);
    void fail(const char* message, std::uint64_t now);
    bool valid_plan(const std::vector<MissionCell>& plan) const;
    std::map<std::string, unsigned> needed() const;
    void next_plant(std::uint64_t now);
    void next_monitor(std::uint64_t now);
    MissionState state_ = MissionState::waiting_for_start;
    PlotBounds plot_;
    Environment environment_;
    std::vector<MissionCell> cells_;
    std::map<std::string, unsigned> loaded_, inventory_;
    MissionProgress progress_;
    std::optional<HardwareAction> pending_;
    std::uint64_t sequence_ = 0;
    std::size_t current_ = 0;
    Crop replant_crop_;
    unsigned found_weeds_ = 0;
    std::string error_;
};
}
