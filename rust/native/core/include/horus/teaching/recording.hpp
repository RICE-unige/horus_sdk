#pragma once
#include "config.hpp"
#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace horus::teaching {
struct Sample {
    std::string stream_id;
    std::vector<std::uint8_t> payload;
    std::uint64_t received_unix_ns = unix_ns();
    std::uint64_t received_monotonic_ns = monotonic_ns();
    std::optional<std::uint64_t> source_ns;
    std::optional<std::uint64_t> source_sequence;
    std::string encoding = "cdr";
    std::string schema;
    std::string schema_encoding;
};

class EpisodeRecorder {
  public:
    EpisodeRecorder(const TeachConfig &, const std::string &robot_name,
                    const std::string &episode_id, const std::string &task);
    ~EpisodeRecorder();
    EpisodeRecorder(const EpisodeRecorder &) = delete;
    EpisodeRecorder &operator=(const EpisodeRecorder &) = delete;
    bool append(Sample sample);
    bool event(const std::string &kind, const Json &data);
    Json finish(const std::string &outcome = "unknown", bool interrupted = false);
    std::string error() const;
    std::uint64_t dropped() const;
    std::string path() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

class Episode {
  public:
    explicit Episode(std::string path);
    const Json &manifest() const noexcept { return manifest_; }
    std::vector<std::string> validate() const;
    // Callback data is valid only during that call; false stops the streaming read.
    void
    samples(const std::function<bool(const Json &, const std::uint8_t *, std::size_t)> &visitor,
            const std::vector<std::string> &streams = {}, std::uint64_t start_ns = 0,
            std::uint64_t end_ns = UINT64_MAX) const;

  private:
    std::string path_;
    Json manifest_;
};
std::string file_digest(const std::string &path);
// Keeps the source evidence untouched and returns a new, explicitly
// interrupted/unacceptable episode. Refuses captures whose lock is still held.
std::string recover_episode(const std::string &inprogress_path);
} // namespace horus::teaching
