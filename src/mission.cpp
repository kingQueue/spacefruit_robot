#include "mission.hpp"
#include <cmath>
#include <utility>

namespace robot {
namespace {
bool valid_crop(const Crop& c) {
    return !c.name.empty() && c.name.size() <= 64 && std::isfinite(c.spacing_m)
        && std::isfinite(c.depth_m) && c.spacing_m > 0 && c.depth_m > 0 && c.depth_m <= 0.2;
}
}
void Mission::issue(ActionKind kind, std::uint64_t now, unsigned count) {
    const auto timeout = kind == ActionKind::navigate ? 120000u : 30000u;
    const bool targeted = current_ < cells_.size();
    const Crop crop = state_ == MissionState::replanting ? replant_crop_
        : targeted ? cells_[current_].crop : Crop{};
    pending_ = HardwareAction{++sequence_, kind, current_, count, crop,
        targeted ? cells_[current_].x_m : 0, targeted ? cells_[current_].y_m : 0, now + timeout};
}
void Mission::fail(const char* message, std::uint64_t now) {
    error_ = message;
    state_ = MissionState::error;
    issue(ActionKind::stop, now);
}
bool Mission::start(std::uint64_t now) {
    if (state_ != MissionState::waiting_for_start || pending_) return false;
    state_ = MissionState::mapping;
    issue(ActionKind::map_plot, now);
    return true;
}
bool Mission::valid_plan(const std::vector<MissionCell>& plan) const {
    if (plan.empty() || plan.size() > 1000) return false;
    for (std::size_t i = 0; i < plan.size(); ++i) {
        const auto& c = plan[i];
        if (!valid_crop(c.crop) || c.state != CellState::planned || !std::isfinite(c.x_m)
            || !std::isfinite(c.y_m) || c.x_m < 0.25 || c.y_m < 0.25
            || c.x_m > plot_.width_m - 0.25 || c.y_m > plot_.length_m - 0.25) return false;
        for (std::size_t j = 0; j < i; ++j) {
            const double spacing = std::fmax(c.crop.spacing_m, plan[j].crop.spacing_m);
            if (std::hypot(c.x_m - plan[j].x_m, c.y_m - plan[j].y_m) + 0.001 < spacing) return false;
            if (c.crop.name == plan[j].crop.name && (c.crop.spacing_m != plan[j].crop.spacing_m
                || c.crop.depth_m != plan[j].crop.depth_m)) return false;
        }
    }
    return true;
}
bool Mission::set_plan(std::vector<MissionCell> cells) {
    if (state_ != MissionState::waiting_for_confirmation || !valid_plan(cells)) return false;
    cells_ = std::move(cells);
    loaded_.clear();
    return true;
}
std::map<std::string, unsigned> Mission::needed() const {
    std::map<std::string, unsigned> counts;
    for (const auto& cell : cells_) ++counts[cell.crop.name];
    return counts;
}
bool Mission::confirm_plan() {
    if (state_ != MissionState::waiting_for_confirmation || !valid_plan(cells_)) return false;
    state_ = MissionState::waiting_for_seeds;
    return true;
}
bool Mission::load_seeds(const std::map<std::string, unsigned>& counts) {
    if (state_ != MissionState::waiting_for_seeds || counts != needed()) return false;
    loaded_ = counts;
    return true;
}
bool Mission::start_planting(std::uint64_t now) {
    if (state_ != MissionState::waiting_for_seeds || loaded_ != needed() || cells_.empty()) return false;
    current_ = 0;
    state_ = MissionState::planting;
    issue(ActionKind::navigate, now);
    return true;
}
void Mission::next_plant(std::uint64_t now) {
    ++current_;
    if (current_ == cells_.size()) state_ = MissionState::complete;
    else issue(ActionKind::navigate, now);
}
bool Mission::start_monitoring(std::uint64_t now) {
    if (state_ != MissionState::complete || pending_) return false;
    current_ = 0;
    while (current_ < cells_.size() && cells_[current_].state != CellState::planted) ++current_;
    if (current_ == cells_.size()) return false;
    progress_.checked = progress_.weed_locations = progress_.weeds_removed = progress_.issues = 0;
    state_ = MissionState::monitoring;
    issue(ActionKind::navigate, now);
    return true;
}
void Mission::next_monitor(std::uint64_t now) {
    do { ++current_; } while (current_ < cells_.size() && cells_[current_].state != CellState::planted);
    if (current_ == cells_.size()) state_ = MissionState::complete;
    else issue(ActionKind::navigate, now);
}
bool Mission::start_harvest(std::size_t cell, std::uint64_t now) {
    if (state_ != MissionState::complete || pending_ || cell >= cells_.size()
        || cells_[cell].state != CellState::planted) return false;
    current_ = cell;
    state_ = MissionState::harvesting;
    issue(ActionKind::navigate, now);
    return true;
}
bool Mission::start_replant(std::size_t cell, Crop crop,
                          const std::map<std::string, unsigned>& loaded, std::uint64_t now) {
    if (state_ != MissionState::complete || pending_ || cell >= cells_.size()
        || cells_[cell].state != CellState::empty || !valid_crop(crop)
        || loaded != std::map<std::string,unsigned>{{crop.name,1}}) return false;
    for (std::size_t i = 0; i < cells_.size(); ++i) {
        if (i == cell || cells_[i].state != CellState::planted) continue;
        const auto& other = cells_[i];
        if (std::hypot(cells_[cell].x_m-other.x_m,cells_[cell].y_m-other.y_m)+0.001
            < std::fmax(crop.spacing_m,other.crop.spacing_m)) return false;
    }
    current_ = cell;
    replant_crop_ = std::move(crop);
    loaded_ = loaded;
    state_ = MissionState::replanting;
    issue(ActionKind::navigate, now);
    return true;
}
bool Mission::complete_action(std::uint64_t id, const HardwareResult& result, std::uint64_t now) {
    if (!pending_ || pending_->id != id) return false; // Late/duplicate result.
    const auto action = *pending_;
    if (action.kind == ActionKind::stop) {
        if (!result.success || !result.verified) return false;
        pending_.reset(); return true;
    }
    if (now >= action.deadline_ms) { fail("Hardware operation timed out", now); return false; }
    if (!result.success || !result.verified) { fail("Hardware operation failed or was not verified", now); return false; }
    pending_.reset();
    switch (action.kind) {
    case ActionKind::map_plot:
        if (!std::isfinite(result.plot.width_m) || !std::isfinite(result.plot.length_m)
            || result.plot.width_m <= 0.5 || result.plot.length_m <= 0.5) {
            fail("Invalid measured plot bounds", now); return false;
        }
        plot_ = result.plot;
        state_ = MissionState::reading_environment;
        issue(ActionKind::read_environment, now);
        break;
    case ActionKind::read_environment:
        if (!std::isfinite(result.environment.temperature_c) || !std::isfinite(result.environment.humidity_percent)
            || result.environment.temperature_c < -40 || result.environment.temperature_c > 85
            || result.environment.humidity_percent < 0 || result.environment.humidity_percent > 100) {
            fail("Invalid environment reading", now); return false;
        }
        environment_ = result.environment;
        state_ = MissionState::waiting_for_confirmation;
        break;
    case ActionKind::navigate:
        if (state_ == MissionState::planting || state_ == MissionState::replanting) issue(ActionKind::plant_seed,now,1);
        else if (state_ == MissionState::monitoring) issue(ActionKind::inspect_weeds,now);
        else if (state_ == MissionState::harvesting) issue(ActionKind::inspect_readiness,now);
        break;
    case ActionKind::plant_seed: {
        auto& cell = cells_[current_];
        if (state_ == MissionState::replanting) cell.crop = replant_crop_;
        --loaded_.at(cell.crop.name);
        cell.state = CellState::planted;
        cell.health_issue = false;
        ++cell.generation;
        ++progress_.planted;
        if (state_ == MissionState::replanting) state_ = MissionState::complete;
        else next_plant(now);
        break;
    }
    case ActionKind::inspect_weeds:
        if (result.weeds > 20) { fail("Unbounded weed-removal request", now); return false; }
        found_weeds_ = result.weeds;
        if (found_weeds_) issue(ActionKind::remove_weeds,now,found_weeds_);
        else { ++progress_.weed_locations; issue(ActionKind::inspect_health,now); }
        break;
    case ActionKind::remove_weeds:
        if (result.weeds != found_weeds_) { fail("Weed removal was incomplete",now); return false; }
        progress_.weeds_removed += result.weeds;
        ++progress_.weed_locations;
        issue(ActionKind::inspect_health,now);
        break;
    case ActionKind::inspect_health:
        cells_[current_].health_issue = result.health_issue;
        progress_.issues += result.health_issue ? 1u : 0u;
        ++progress_.checked;
        next_monitor(now);
        break;
    case ActionKind::inspect_readiness:
        if (result.readiness == Readiness::unknown) { fail("Harvest readiness is unknown",now); return false; }
        if (result.readiness == Readiness::not_ready) state_ = MissionState::complete;
        else issue(ActionKind::harvest,now,1);
        break;
    case ActionKind::harvest:
        ++inventory_[cells_[current_].crop.name];
        cells_[current_].state = CellState::empty;
        state_ = MissionState::complete;
        break;
    case ActionKind::stop: break;
    }
    return true;
}
void Mission::tick(std::uint64_t now, bool safe) {
    if (state_ == MissionState::error) return;
    if (!safe) fail("Emergency stop, stale sensing or localization loss",now);
    else if (pending_ && now >= pending_->deadline_ms) fail("Hardware operation timed out",now);
}
void Mission::cancel(std::uint64_t now) { fail("Operation cancelled",now); }
}
