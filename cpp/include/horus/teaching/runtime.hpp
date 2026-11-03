#pragma once
#include "horus/teaching/session.hpp"

namespace horus::teaching {
class TeachRuntime {
  public:
    TeachRuntime(const TeachConfig &config, const std::string &robot_name,
                 std::shared_ptr<TeachAdapter> adapter = {});
    ~TeachRuntime();
    TeachRuntime(const TeachRuntime &) = delete;
    TeachRuntime &operator=(const TeachRuntime &) = delete;
    TeachSession &session() noexcept { return session_; }
    Json status() const;

  private:
    TeachSession session_;
    void *runtime_ = nullptr;
};
} // namespace horus::teaching
