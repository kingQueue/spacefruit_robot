#pragma once
#include <cstdint>
namespace robot {
constexpr unsigned dry_percent = 30;
constexpr unsigned wet_percent = 45;
struct Decision { unsigned moisture; bool pump_requested; bool valid; };
class Controller {
public:
    Decision update(unsigned moisture) noexcept;
    void reset() noexcept { pump_ = false; samples_ = invalid_ = 0; }
    std::uint32_t samples() const noexcept { return samples_; }
    std::uint32_t invalid_samples() const noexcept { return invalid_; }
private:
    bool pump_ = false;
    std::uint32_t samples_ = 0;
    std::uint32_t invalid_ = 0;
};
}
