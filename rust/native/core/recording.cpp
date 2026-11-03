#include "horus/teaching/recording.hpp"
#define MCAP_IMPLEMENTATION
#include <algorithm>
#include <cerrno>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <linux/fs.h>
#include <mcap/mcap.hpp>
#include <mutex>
#include <openssl/evp.h>
#include <set>
#include <sstream>
#include <sys/file.h>
#include <sys/syscall.h>
#include <thread>
#include <unistd.h>

namespace horus::teaching {
namespace fs = std::filesystem;
namespace {
void check(const mcap::Status &s) {
  if (!s.ok())
    throw std::runtime_error(s.message);
}
void sync_path(const fs::path &p, bool directory = false) {
  int fd = ::open(p.c_str(), O_RDONLY | (directory ? O_DIRECTORY : 0));
  if (fd < 0)
    throw std::runtime_error("Cannot open path for durability sync: " +
                             p.string());
  int result = ::fsync(fd);
  ::close(fd);
  if (result)
    throw std::runtime_error("Durability sync failed: " + p.string());
}
void atomic_json(const fs::path &p, const Json &value) {
  auto tmp = p.string() + "." + new_uuid() + ".tmp";
  std::ofstream out(tmp, std::ios::binary);
  out.exceptions(std::ios::badbit | std::ios::failbit);
  out << canonical(value);
  out.close();
  sync_path(tmp);
  fs::rename(tmp, p);
  sync_path(p.parent_path(), true);
}
fs::path root_path(std::string root) {
  if (root.rfind("~/", 0) == 0) {
    const char *home = std::getenv("HOME");
    if (!home)
      throw std::runtime_error("HOME is unset");
    root = std::string(home) + root.substr(1);
  }
  return fs::path(root);
}
void validate_outcome(const std::string &v) {
  if (v != "unknown" && v != "success" && v != "failure" && v != "abort")
    throw std::invalid_argument("Invalid episode outcome");
}
Json read_json(const fs::path &p) {
  std::ifstream file(p);
  if (!file)
    throw std::runtime_error("Cannot read " + p.string());
  return Json::parse(file);
}
void publish_directory(const fs::path &source, const fs::path &destination) {
  if (::syscall(SYS_renameat2, AT_FDCWD, source.c_str(), AT_FDCWD,
                destination.c_str(), RENAME_NOREPLACE) == 0) {
    sync_path(destination.parent_path(), true);
    return;
  }
  const int error = errno;
  if (error != EINVAL && error != ENOSYS && error != EOPNOTSUPP)
    throw std::runtime_error(std::string("Cannot publish episode: ") +
                             std::strerror(error));

  // NTFS/FUSE may not implement RENAME_NOREPLACE. Reserve a new directory,
  // retain all staging artifacts, and publish the manifest last. Hard links
  // avoid copying the MCAP and preserve its inode/lock until publication ends.
  std::vector<fs::path> files;
  for (const auto &entry : fs::directory_iterator(source)) {
    if (!fs::is_regular_file(entry.symlink_status()))
      throw std::runtime_error(
          "Episode publication requires regular artifacts");
    if (entry.path().filename() != "manifest.json")
      files.push_back(entry.path().filename());
  }
  if (!fs::is_regular_file(fs::symlink_status(source / "manifest.json")))
    throw std::runtime_error("Episode publication requires its manifest");
  files.push_back("manifest.json");
  if (!fs::create_directory(destination))
    throw std::runtime_error(
        "Episode destination already exists; refusing overwrite");
  std::vector<fs::path> created;
  try {
    for (const auto &name : files) {
      fs::create_hard_link(source / name, destination / name);
      created.push_back(name);
      if (name != "manifest.json")
        sync_path(destination, true);
    }
    sync_path(destination, true);
    sync_path(destination.parent_path(), true);
  } catch (...) {
    // Only unlink artifacts installed by this attempt; staging is intact.
    std::error_code ignored;
    for (const auto &name : created)
      fs::remove(destination / name, ignored);
    fs::remove(destination, ignored);
    throw;
  }
  // A cleanup failure must not turn an already durable saved episode into a
  // reported failed capture. Remaining staging links can be inspected later.
  std::error_code ignored;
  for (const auto &name : files)
    fs::remove(source / name, ignored);
  fs::remove(source, ignored);
}
} // namespace
std::string file_digest(const std::string &path) {
  std::ifstream file(path, std::ios::binary);
  if (!file)
    throw std::runtime_error("Cannot read artifact");
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> ctx(EVP_MD_CTX_new(),
                                                              EVP_MD_CTX_free);
  if (!ctx || EVP_DigestInit_ex(ctx.get(), EVP_sha256(), nullptr) != 1)
    throw std::runtime_error("Digest initialization failed");
  char buffer[64 * 1024];
  while (file) {
    file.read(buffer, sizeof(buffer));
    if (EVP_DigestUpdate(ctx.get(), buffer, file.gcount()) != 1)
      throw std::runtime_error("Digest failed");
  }
  if (!file.eof())
    throw std::runtime_error("Artifact read failed");
  unsigned char bytes[EVP_MAX_MD_SIZE];
  unsigned int count = 0;
  if (EVP_DigestFinal_ex(ctx.get(), bytes, &count) != 1)
    throw std::runtime_error("Digest finalization failed");
  std::ostringstream out;
  for (unsigned int i = 0; i < count; ++i)
    out << std::hex << std::setfill('0') << std::setw(2) << unsigned(bytes[i]);
  return out.str();
}

struct EpisodeRecorder::Impl {
  TeachConfig config;
  fs::path root, current, destination;
  mutable std::mutex mutex;
  std::mutex finish_mutex;
  std::condition_variable wake;
  std::deque<std::pair<Sample, std::size_t>> queue;
  std::size_t queued = 0;
  std::uint64_t drops = 0, started = monotonic_ns();
  bool accepting = true;
  std::string failure;
  Json manifest, counts = Json::object(), stats = Json::object();
  std::optional<Json> finished;
  std::map<std::string, mcap::Channel> channels;
  std::map<std::string, std::string> contracts;
  mcap::McapWriter writer;
  int lock_fd = -1;
  std::thread worker;
  Impl(const TeachConfig &c, const std::string &robot, const std::string &id,
       const std::string &task)
      : config(c) {
    validate_uuid(id);
    validate_identifier(robot);
    root = root_path(c.to_dict().at("recording_root"));
    fs::create_directories(root);
    if (fs::space(root).available <
        c.to_dict().at("minimum_free_bytes").get<std::uint64_t>())
      throw std::runtime_error("Insufficient recording disk headroom");
    destination = root / id;
    current = root / (id + ".inprogress");
    if (fs::exists(destination) || !fs::create_directory(current))
      throw std::runtime_error("Episode already exists; refusing overwrite");
    try {
      lock_fd = ::open((current / "capture.lock").c_str(),
                       O_CREAT | O_RDWR | O_CLOEXEC, 0600);
      if (lock_fd < 0 || flock(lock_fd, LOCK_EX | LOCK_NB))
        throw std::runtime_error("Cannot reserve episode");
      std::string boot = "unknown";
      std::ifstream boot_file("/proc/sys/kernel/random/boot_id");
      if (boot_file)
        std::getline(boot_file, boot);
      manifest = {{"schema", "horus.teach.episode.v1"},
                  {"episode_id", id},
                  {"robot_name", robot},
                  {"task", task},
                  {"provenance", c.to_dict().at("provenance")},
                  {"config", c.to_dict()},
                  {"config_hash", c.digest()},
                  {"capture", "recording"},
                  {"quality", "pending"},
                  {"outcome", "unknown"},
                  {"export", "not_requested"},
                  {"started_unix_ns", unix_ns()},
                  {"clock",
                   {{"receipt", "recorder_unix"},
                    {"intervals", "recorder_monotonic"},
                    {"clock_offset_ns", nullptr},
                    {"clock_error_ns", nullptr},
                    {"boot_id", boot}}},
                  {"counter_origin", "recorder_subscription"},
                  {"upstream_drops", nullptr},
                  {"required_artifacts", {"data.mcap"}}};
      atomic_json(current / "journal.json", manifest);
      mcap::McapWriterOptions options("");
      options.library = "horus-teach-native/1";
      options.chunkSize = 256 * 1024;
      options.compression = mcap::Compression::Zstd;
      options.enableDataCRC = true;
      check(writer.open((current / "data.mcap").string(), options));
      worker = std::thread([this] { run(); });
    } catch (...) {
      if (lock_fd >= 0)
        ::close(lock_fd);
      lock_fd = -1;
      throw;
    }
  }
  ~Impl() {
    {
      std::lock_guard guard(mutex);
      accepting = false;
      wake.notify_all();
    }
    if (worker.joinable())
      worker.join();
    if (lock_fd >= 0)
      ::close(lock_fd);
  }
  mcap::ChannelId channel(const Sample &s) {
    auto contract = canonical(Json{{"encoding", s.encoding},
                                   {"schema", s.schema},
                                   {"schema_encoding", s.schema_encoding}});
    auto found = channels.find(s.stream_id);
    if (found != channels.end()) {
      if (contracts.at(s.stream_id) != contract)
        throw std::runtime_error("Stream schema/encoding changed: " +
                                 s.stream_id);
      return found->second.id;
    }
    std::string type = "horus.teach.event.v1", topic = "internal";
    for (auto &stream : config.to_dict().at("streams"))
      if (stream.at("stream_id") == s.stream_id) {
        type = stream.at("message_type");
        topic = stream.at("topic");
      }
    mcap::Schema schema(type, s.schema_encoding, s.schema);
    writer.addSchema(schema);
    mcap::Channel ch(s.stream_id, s.encoding, schema.id,
                     {{"source_topic", topic}});
    writer.addChannel(ch);
    channels.emplace(s.stream_id, ch);
    contracts.emplace(s.stream_id, std::move(contract));
    return ch.id;
  }
  void write(const Sample &s) {
    auto index = counts.value(s.stream_id, std::uint64_t(0));
    mcap::Message message;
    message.channelId = channel(s);
    message.sequence = static_cast<std::uint32_t>(index);
    message.logTime = s.received_unix_ns;
    message.publishTime = s.source_ns.value_or(0);
    message.data = reinterpret_cast<const std::byte *>(s.payload.data());
    message.dataSize = s.payload.size();
    check(writer.write(message));
    counts[s.stream_id] = index + 1;
    auto timing = channels.find("__timing");
    if (timing == channels.end()) {
      mcap::Channel ch("__timing", "json", 0);
      writer.addChannel(ch);
      timing = channels.emplace("__timing", ch).first;
    }
    std::string data = canonical(
        Json{{"stream_id", s.stream_id},
             {"receipt_index", index},
             {"received_monotonic_ns", s.received_monotonic_ns},
             {"source_ns", s.source_ns ? Json(*s.source_ns) : Json(nullptr)},
             {"source_sequence",
              s.source_sequence ? Json(*s.source_sequence) : Json(nullptr)}});
    message.channelId = timing->second.id;
    message.data = reinterpret_cast<const std::byte *>(data.data());
    message.dataSize = data.size();
    check(writer.write(message));
    if (!stats.contains(s.stream_id))
      stats[s.stream_id] = {{"first_ns", s.received_monotonic_ns},
                            {"last_ns", s.received_monotonic_ns},
                            {"max_gap_ns", 0},
                            {"bytes", 0},
                            {"source_time_regressions", 0},
                            {"source_time_duplicates", 0},
                            {"source_last_ns", nullptr}};
    auto &stat = stats[s.stream_id];
    auto last = stat["last_ns"].get<std::uint64_t>();
    if (s.received_monotonic_ns < last)
      throw std::runtime_error("Receipt monotonic time regressed");
    stat["max_gap_ns"] = std::max(stat["max_gap_ns"].get<std::uint64_t>(),
                                  s.received_monotonic_ns - last);
    stat["last_ns"] = s.received_monotonic_ns;
    stat["bytes"] = stat["bytes"].get<std::uint64_t>() + s.payload.size();
    if (s.source_ns) {
      if (!stat["source_last_ns"].is_null()) {
        auto previous = stat["source_last_ns"].get<std::uint64_t>();
        for (auto key : {"source_time_regressions", "source_time_duplicates"}) {
          bool yes = std::string(key) == "source_time_regressions"
                         ? *s.source_ns < previous
                         : *s.source_ns == previous;
          stat[key] = stat[key].get<std::uint64_t>() + (yes ? 1 : 0);
        }
      }
      stat["source_last_ns"] = *s.source_ns;
    }
  }
  void run() noexcept {
    try {
      double disk_check = monotonic_seconds();
      for (;;) {
        Sample sample;
        {
          std::unique_lock guard(mutex);
          wake.wait(guard, [this] { return !accepting || !queue.empty(); });
          if (queue.empty())
            break;
          auto item = std::move(queue.front());
          queue.pop_front();
          queued -= item.second;
          sample = std::move(item.first);
        }
        if (monotonic_seconds() - disk_check > 1) {
          if (fs::space(root).available <
              config.to_dict().at("minimum_free_bytes").get<std::uint64_t>())
            throw std::runtime_error("Recording disk reserve reached");
          disk_check = monotonic_seconds();
        }
        write(sample);
      }
      writer.close();
      sync_path(current / "data.mcap");
    } catch (const std::exception &e) {
      std::lock_guard guard(mutex);
      failure = e.what();
    } catch (...) {
      std::lock_guard guard(mutex);
      failure = "Unknown recording failure";
    }
    std::lock_guard guard(mutex);
    accepting = false;
    queue.clear();
    queued = 0;
  }
};
EpisodeRecorder::EpisodeRecorder(const TeachConfig &c, const std::string &robot,
                                 const std::string &id, const std::string &task)
    : impl_(std::make_unique<Impl>(c, robot, id, task)) {}
EpisodeRecorder::~EpisodeRecorder() {
  try {
    finish("abort", true);
  } catch (...) {
  }
}
bool EpisodeRecorder::append(Sample s) {
  if (s.stream_id != "__events") {
    bool found = false;
    for (auto &stream : impl_->config.to_dict().at("streams"))
      if (stream.at("stream_id") == s.stream_id)
        found = true;
    if (!found)
      throw std::invalid_argument("Undeclared recording stream");
  }
  if (s.encoding == "cdr" &&
      (s.schema.empty() ||
       (s.schema_encoding != "ros2msg" && s.schema_encoding != "ros2idl")))
    throw std::invalid_argument("CDR recording requires its ROS schema");
  auto size = s.payload.size() + s.schema.size() + 512;
  std::lock_guard guard(impl_->mutex);
  if (!impl_->accepting || !impl_->failure.empty())
    return false;
  if (size >
      impl_->config.to_dict().at("queue_budget_bytes").get<std::uint64_t>() -
          impl_->queued) {
    ++impl_->drops;
    return false;
  }
  impl_->queue.emplace_back(std::move(s), size);
  impl_->queued += size;
  impl_->wake.notify_one();
  return true;
}
bool EpisodeRecorder::event(const std::string &kind, const Json &data) {
  auto value = canonical(Json{{"kind", kind}, {"data", data}});
  Sample s;
  s.stream_id = "__events";
  s.encoding = "json";
  s.payload.assign(value.begin(), value.end());
  return append(std::move(s));
}
std::string EpisodeRecorder::error() const {
  std::lock_guard guard(impl_->mutex);
  return impl_->failure;
}
std::uint64_t EpisodeRecorder::dropped() const {
  std::lock_guard guard(impl_->mutex);
  return impl_->drops;
}
std::string EpisodeRecorder::path() const {
  std::lock_guard guard(impl_->mutex);
  return impl_->current.string();
}
Json EpisodeRecorder::finish(const std::string &outcome, bool interrupted) {
  validate_outcome(outcome);
  std::lock_guard finishing(impl_->finish_mutex);
  if (impl_->finished)
    return *impl_->finished;
  {
    std::lock_guard guard(impl_->mutex);
    impl_->accepting = false;
    impl_->wake.notify_all();
  }
  auto ended = monotonic_ns();
  auto ended_unix = unix_ns();
  if (impl_->worker.joinable())
    impl_->worker.join();
  Json issues = Json::array();
  auto failure = error();
  auto drops = dropped();
  if (drops)
    issues.push_back("recorder_queue_drops:" + std::to_string(drops));
  for (auto &s : impl_->config.to_dict().at("streams")) {
    std::string id = s.at("stream_id");
    if (!s.at("required").get<bool>())
      continue;
    if (!impl_->stats.contains(id)) {
      issues.push_back("missing:" + id);
      continue;
    }
    auto stat = impl_->stats.at(id);
    auto first = stat["first_ns"].get<std::uint64_t>();
    auto last = stat["last_ns"].get<std::uint64_t>();
    auto gap = std::max({stat["max_gap_ns"].get<std::uint64_t>(),
                         first >= impl_->started ? first - impl_->started : 0,
                         ended >= last ? ended - last : 0});
    if (!retained_calibration(s) && gap > s.at("max_gap_seconds").get<double>() * 1e9)
      issues.push_back("stream_gap:" + id);
    if (stat["source_time_regressions"].get<std::uint64_t>())
      issues.push_back("source_time_regression:" + id);
  }
  if (!failure.empty())
    issues.push_back(failure);
  if (interrupted)
    issues.push_back("interrupted");
  auto &m = impl_->manifest;
  m.update(Json{
      {"capture", interrupted || !failure.empty() ? "interrupted" : "saved"},
      {"quality", issues.empty() ? "acceptable" : "unacceptable"},
      {"quality_issues", issues},
      {"outcome", outcome},
      {"ended_unix_ns", ended_unix},
      {"duration_ns", ended - impl_->started},
      {"stream_counts", impl_->counts},
      {"stream_statistics", impl_->stats},
      {"recorder_drops", drops}});
  if (failure.empty()) {
    mcap::McapReader reader;
    check(reader.open((impl_->current / "data.mcap").string()));
    check(reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan));
    for (const auto &message :
         reader.readMessages([](const mcap::Status &s) { check(s); })) {
      (void)message;
    }
    reader.close();
    m["artifacts"] = Json::array(
        {Json{{"path", "data.mcap"},
              {"sha256", file_digest((impl_->current / "data.mcap").string())},
              {"bytes", fs::file_size(impl_->current / "data.mcap")}}});
  }
  atomic_json(impl_->current / "manifest.json", m);
  if (failure.empty()) {
    publish_directory(impl_->current, impl_->destination);
    std::lock_guard guard(impl_->mutex);
    impl_->current = impl_->destination;
  }
  if (impl_->lock_fd >= 0) {
    ::close(impl_->lock_fd);
    impl_->lock_fd = -1;
  }
  impl_->finished = m;
  return m;
}

Episode::Episode(std::string path)
    : path_(std::move(path)),
      manifest_(read_json(fs::path(path_) / "manifest.json")) {
  if (manifest_.value("schema", "") != "horus.teach.episode.v1")
    throw std::invalid_argument("Unsupported episode schema");
}
void Episode::samples(
    const std::function<bool(const Json &, const std::uint8_t *, std::size_t)>
        &visitor,
    const std::vector<std::string> &streams, std::uint64_t start,
    std::uint64_t end) const {
  mcap::McapReader reader;
  check(reader.open((fs::path(path_) / "data.mcap").string()));
  check(reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan));
  mcap::ReadMessageOptions options(start, end);
  options.topicFilter = [&](std::string_view topic) {
    return streams.empty() ||
           std::find(streams.begin(), streams.end(), topic) != streams.end();
  };
  for (const auto &view :
       reader.readMessages([](const mcap::Status &s) { check(s); }, options)) {
    Json meta = {{"stream_id", view.channel->topic},
                 {"encoding", view.channel->messageEncoding},
                 {"received_unix_ns", view.message.logTime},
                 {"source_ns", view.message.publishTime},
                 {"sequence", view.message.sequence},
                 {"schema_encoding", view.schema ? view.schema->encoding : ""},
                 {"schema", view.schema
                                ? std::string(reinterpret_cast<const char *>(
                                                  view.schema->data.data()),
                                              view.schema->data.size())
                                : ""}};
    if (!visitor(meta,
                 reinterpret_cast<const std::uint8_t *>(view.message.data),
                 view.message.dataSize))
      break;
  }
}
std::vector<std::string> Episode::validate() const {
  std::vector<std::string> issues;
  try {
    if (TeachConfig(manifest_.at("config")).digest() !=
        manifest_.at("config_hash").get<std::string>())
      issues.push_back("Recording configuration hash mismatch");
    auto inventory = manifest_.at("artifacts");
    if (!inventory.is_array() || inventory.size() != 1 ||
        inventory[0].at("path") != "data.mcap")
      throw std::runtime_error("Missing or unsupported artifact inventory");
    auto p = fs::path(path_) / "data.mcap";
    if (fs::file_size(p) != inventory[0].at("bytes").get<std::uint64_t>() ||
        file_digest(p.string()) != inventory[0].at("sha256").get<std::string>())
      issues.push_back("Artifact integrity mismatch: data.mcap");
    if (!issues.empty())
      return issues;
    std::map<std::string, std::uint64_t> counts;
    samples([&](const Json &info, const std::uint8_t *, std::size_t) {
      ++counts[info.at("stream_id")];
      if (info.at("encoding") == "cdr" &&
          (info.at("schema") == "" ||
           (info.at("schema_encoding") != "ros2msg" &&
            info.at("schema_encoding") != "ros2idl")))
        throw std::runtime_error("Missing ROS schema");
      return true;
    });
    for (auto it = manifest_.at("stream_counts").begin();
         it != manifest_.at("stream_counts").end(); ++it)
      if (counts[it.key()] != it.value().get<std::uint64_t>())
        issues.push_back("Stream count mismatch: " + it.key());
  } catch (const std::exception &e) {
    issues.push_back(e.what());
  }
  return issues;
}

std::string recover_episode(const std::string &path) {
  auto source = fs::canonical(root_path(path));
  if (source.extension() != ".inprogress")
    throw std::invalid_argument(
        "Recovery accepts .inprogress directories only");
  struct Lock {
    int fd;
    explicit Lock(const fs::path &path)
        : fd(::open(path.c_str(), O_RDWR | O_CLOEXEC)) {
      if (fd < 0)
        throw std::runtime_error(
            "Missing capture lock; refusing unverified recovery source");
      if (::flock(fd, LOCK_EX | LOCK_NB)) {
        ::close(fd);
        fd = -1;
        throw std::runtime_error("Capture is still active");
      }
    }
    ~Lock() {
      if (fd >= 0)
        ::close(fd);
    }
  } lock(source / "capture.lock");
  auto manifest = read_json(source / "journal.json");
  TeachConfig config(manifest.at("config"));
  if (manifest.value("schema", "") != "horus.teach.episode.v1" ||
      config.digest() != manifest.at("config_hash").get<std::string>())
    throw std::runtime_error("Invalid recovery journal");
  if (fs::space(source.parent_path()).available <
      config.to_dict().at("minimum_free_bytes").get<std::uint64_t>())
    throw std::runtime_error("Insufficient recovery disk headroom");
  auto id = new_uuid();
  auto staging = source.parent_path() / (id + ".inprogress");
  if (!fs::create_directory(staging))
    throw std::runtime_error("Cannot reserve recovery directory");
  int staging_fd = ::open((staging / "capture.lock").c_str(),
                          O_CREAT | O_EXCL | O_RDWR | O_CLOEXEC, 0600);
  if (staging_fd < 0)
    throw std::runtime_error("Cannot create recovery lock");
  ::close(staging_fd);
  Lock recovery_lock(staging / "capture.lock");
  atomic_json(staging / "journal.json", manifest);

  // MCAP's C++ reader does not check chunk CRCs. Verify each complete chunk
  // before allowing it to contribute recovered messages; bound allocations.
  struct BoundedReader : mcap::IReadable {
    std::ifstream stream;
    mcap::FileStreamReader reader;
    explicit BoundedReader(const fs::path &path)
        : stream(path, std::ios::binary), reader(stream) {
      if (!stream)
        throw std::runtime_error("Cannot read recovery source");
    }
    std::uint64_t size() const override { return reader.size(); }
    std::uint64_t read(std::byte **out, std::uint64_t offset,
                       std::uint64_t size) override {
      if (size > 256ULL * 1024 * 1024)
        throw std::runtime_error("MCAP record exceeds recovery limit");
      return reader.read(out, offset, size);
    }
  } input(source / "data.mcap");
  mcap::McapWriter writer;
  mcap::McapWriterOptions options("");
  options.library = "horus-teach-native-recovery/1";
  options.enableDataCRC = true;
  options.chunkSize = 256 * 1024;
  check(writer.open((staging / "data.mcap").string(), options));
  std::map<mcap::SchemaId, mcap::SchemaId> schemas{{0, 0}};
  std::map<mcap::ChannelId, mcap::Channel> channels;
  Json counts = Json::object();
  std::string error;
  auto last = manifest.at("started_unix_ns").get<std::uint64_t>();
  bool complete = false;
  try {
    std::byte *magic = nullptr;
    if (input.read(&magic, 0, sizeof(mcap::Magic)) != sizeof(mcap::Magic) ||
        std::memcmp(magic, mcap::Magic, sizeof(mcap::Magic)))
      throw std::runtime_error("Invalid MCAP header");
    mcap::TypedRecordReader reader(input, sizeof(mcap::Magic), input.size());
    reader.onChunk = [](const mcap::Chunk &chunk, auto) {
      if (chunk.uncompressedSize > 256ULL * 1024 * 1024)
        throw std::runtime_error("MCAP chunk exceeds recovery limit");
      const std::byte *bytes = chunk.records;
      mcap::ByteArray uncompressed;
      if (chunk.compression == "zstd") {
        check(mcap::ZStdReader::DecompressAll(
            chunk.records, chunk.compressedSize, chunk.uncompressedSize,
            &uncompressed));
        bytes = uncompressed.data();
      } else if (!chunk.compression.empty())
        throw std::runtime_error("Unsupported recovery compression");
      else if (chunk.compressedSize != chunk.uncompressedSize)
        throw std::runtime_error("Invalid uncompressed chunk length");
      if (chunk.uncompressedCrc &&
          mcap::internal::crc32Final(mcap::internal::crc32Update(
              mcap::internal::CRC32_INIT, bytes, chunk.uncompressedSize)) !=
              chunk.uncompressedCrc)
        throw std::runtime_error("MCAP chunk CRC mismatch");
    };
    reader.onSchema = [&](const mcap::SchemaPtr &schema, auto, auto) {
      if (schemas.count(schema->id))
        return;
      auto copy = *schema;
      writer.addSchema(copy);
      schemas.emplace(schema->id, copy.id);
    };
    reader.onChannel = [&](const mcap::ChannelPtr &channel, auto, auto) {
      if (channels.count(channel->id))
        return;
      auto copy = *channel;
      copy.schemaId = schemas.at(channel->schemaId);
      writer.addChannel(copy);
      channels.emplace(channel->id, std::move(copy));
    };
    reader.onMessage = [&](const mcap::Message &message, auto, auto) {
      if (fs::space(staging).available <
          config.to_dict().at("minimum_free_bytes").get<std::uint64_t>())
        throw std::runtime_error("Recovery disk reserve reached");
      const auto &channel = channels.at(message.channelId);
      auto copy = message;
      copy.channelId = channel.id;
      check(writer.write(copy));
      counts[channel.topic] = counts.value(channel.topic, std::uint64_t(0)) + 1;
      last = std::max(last, message.logTime);
    };
    reader.onDataEnd = [&](auto &, auto) { complete = true; };
    while (!complete && reader.next()) {
      check(reader.status());
    }
    check(reader.status());
    if (!complete)
      error = "Source ended before DataEnd; lost tail unknown";
  } catch (const std::exception &exc) {
    error = exc.what();
  }
  writer.close();
  sync_path(staging / "data.mcap");
  manifest.update(
      Json{{"episode_id", id},
           {"capture", "interrupted"},
           {"quality", "unacceptable"},
           {"outcome", "unknown"},
           {"quality_issues", {"recovered_interrupted_capture"}},
           {"export", "not_requested"},
           {"ended_unix_ns", last},
           {"duration_ns",
            last - manifest.at("started_unix_ns").get<std::uint64_t>()},
           {"stream_counts", counts},
           {"stream_statistics", Json::object()},
           {"recorder_drops", nullptr},
           {"recovery",
            {{"source_episode", source.filename().string()},
             {"source_sha256", file_digest((source / "data.mcap").string())},
             {"read_error", error},
             {"recovered_unix_ns", unix_ns()},
             {"lost_tail", "unknown"}}},
           {"artifacts",
            Json::array({Json{
                {"path", "data.mcap"},
                {"bytes", fs::file_size(staging / "data.mcap")},
                {"sha256", file_digest((staging / "data.mcap").string())}}})}});
  atomic_json(staging / "manifest.json", manifest);
  auto issues = Episode(staging.string()).validate();
  if (!issues.empty())
    throw std::runtime_error("Recovered episode validation failed: " +
                             issues.front());
  auto destination = source.parent_path() / id;
  publish_directory(staging, destination);
  return destination.string();
}
} // namespace horus::teaching
