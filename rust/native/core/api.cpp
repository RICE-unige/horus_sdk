#include "api.h"
#include "horus/description/compiler.hpp"
#include "horus/monitoring/dashboard.hpp"
#include "horus/remote_rendering/protocol.hpp"
#include "horus/teaching/cartesian.hpp"
#include "horus/teaching/session.hpp"
#include <cstdlib>
#include <cstring>

using namespace horus::teaching;
namespace {
char *copy(const std::string &s) {
    auto *p = static_cast<char *>(std::malloc(s.size() + 1));
    if (!p)
        throw std::bad_alloc();
    std::memcpy(p, s.c_str(), s.size() + 1);
    return p;
}
Json parse(const char *value) {
    if (!value)
        throw std::invalid_argument("Missing JSON");
    return Json::parse(value);
}
template <class F> char *result(F f) noexcept {
    try {
        return copy(Json{{"ok", true}, {"value", f()}}.dump());
    } catch (const std::exception &e) {
        try {
            return copy(Json{{"ok", false}, {"error", e.what()}}.dump());
        } catch (...) {
            return nullptr;
        }
    } catch (...) {
        return nullptr;
    }
}
template <class F> void *create(char **error, F f) noexcept {
    if (error)
        *error = nullptr;
    try {
        return f();
    } catch (const std::exception &e) {
        if (error)
            try {
                *error = copy(e.what());
            } catch (...) {
            }
        return nullptr;
    } catch (...) {
        return nullptr;
    }
}
class CallbackAdapter final : public TeachAdapter {
    void *context_;
    horus_teach_callback callback_;
    horus_teach_callback_free release_;

  public:
    Json call(const char *op, const Json &value) const {
        auto input = value.dump();
        auto *out = callback_(context_, op, input.c_str());
        if (!out)
            throw std::runtime_error("Adapter returned no result");
        Json parsed;
        try {
            parsed = Json::parse(out);
        } catch (...) {
            release_(context_, out);
            throw;
        }
        release_(context_, out);
        if (!parsed.value("ok", false))
            throw std::runtime_error(parsed.value("error", "Adapter callback failed"));
        return parsed.at("value");
    }

  public:
    CallbackAdapter(void *context, horus_teach_callback callback, horus_teach_callback_free release)
        : context_(context), callback_(callback), release_(release) {}
    AdapterCapabilities capabilities() const override {
        return AdapterCapabilities::from_dict(call("capabilities", nullptr));
    }
    std::pair<bool, std::string> ready() override {
        auto r = call("ready", nullptr);
        return {r.at("ready"), r.value("reason", "")};
    }
    void anchor(const Json &s) override { call("anchor", s); }
    Json apply(const Json &s) override { return call("apply", s); }
    bool hold(const Json &s) override { return call("hold", s).get<bool>(); }
    bool pause(const Json &s) override { return call("pause", s).get<bool>(); }
};
Sample sample(const char *metadata, const std::uint8_t *data, std::size_t size) {
    if (!data && size)
        throw std::invalid_argument("Missing sample bytes");
    auto m = parse(metadata);
    Sample s;
    s.stream_id = m.at("stream_id");
    s.received_unix_ns = m.value("received_unix_ns", unix_ns());
    s.received_monotonic_ns = m.value("received_monotonic_ns", monotonic_ns());
    if (m.contains("source_ns") && !m["source_ns"].is_null())
        s.source_ns = m["source_ns"].get<std::uint64_t>();
    if (m.contains("source_sequence") && !m["source_sequence"].is_null())
        s.source_sequence = m["source_sequence"].get<std::uint64_t>();
    s.encoding = m.value("encoding", "cdr");
    s.schema = m.value("schema", "");
    s.schema_encoding = m.value("schema_encoding", "");
    if (size)
        s.payload.assign(data, data + size);
    return s;
}
} // namespace
extern "C" {
char *horus_remote_contract(const char *operation, const char *value) {
    return result([&]() -> Json {
        using namespace horus::remote_rendering;
        if (!operation)
            throw std::invalid_argument("Missing remote operation");
        if (!value || std::strlen(value) > maximum_metadata_bytes)
            throw std::invalid_argument("Remote contract metadata exceeds 64 KiB");
        const std::string op(operation);
        const auto j = parse(value);
        if (op == "view")
            return ViewState::from_dict(j).to_dict();
        if (op == "metadata")
            return validate_frame_metadata(j);
        if (op == "matrices") {
            const auto v = ViewState::from_dict(j.at("view"));
            const double n = j.at("near"), f = j.at("far");
            return {{"left_view", eye_view_matrix(v.left_eye)},
                    {"right_view", eye_view_matrix(v.right_eye)},
                    {"left_projection", eye_projection_matrix(v.left_eye, n, f)},
                    {"right_projection", eye_projection_matrix(v.right_eye, n, f)},
                    {"left_vp", eye_view_projection_matrix(v.left_eye, n, f)},
                    {"right_vp", eye_view_projection_matrix(v.right_eye, n, f)}};
        }
        throw std::invalid_argument("Unknown remote contract operation");
    });
}
char *horus_dashboard_render(const char *snapshot, unsigned width, unsigned rows, unsigned page,
                             int color) {
    return result([&] {
        return horus::monitoring::render_dashboard(parse(snapshot), width, rows, page, color != 0);
    });
}
char *horus_description_compile(const char *xml, const char *options) {
    return result([&] {
        return (xml ? horus::description::compile_urdf(xml, parse(options))
                    : horus::description::compile_file(parse(options)))
            .to_dict();
    });
}
uint64_t horus_teach_monotonic_ns(void) { return monotonic_ns(); }
char *horus_teach_config(const char *config, const char *robot) {
    return result([&] {
        TeachConfig c(parse(config));
        return Json{{"config", c.to_dict()},
                    {"digest", c.digest()},
                    {"registration", robot ? c.registration(robot) : Json(nullptr)}};
    });
}
void *horus_teach_session_create(const char *config, const char *robot, void *context,
                                 horus_teach_callback callback, horus_teach_callback_free release,
                                 char **error) {
    return create(error, [&] {
        if (!robot || (callback && !release))
            throw std::invalid_argument("Missing robot or callback release");
        std::shared_ptr<TeachAdapter> a;
        if (callback)
            a = std::make_shared<CallbackAdapter>(context, callback, release);
        return new TeachSession(TeachConfig(parse(config)), robot, std::move(a));
    });
}
char *horus_teach_session_call(void *session, const char *operation, const char *value) {
    return result([&]() -> Json {
        if (!session || !operation)
            throw std::invalid_argument("Missing session/operation");
        auto &s = *static_cast<TeachSession *>(session);
        std::string op(operation);
        auto v = parse(value);
        if (op == "state")
            return s.state();
        if (op == "request")
            return s.request(v);
        if (op == "input")
            return s.input(v);
        if (op == "lease") {
            s.update_lease(v.value("owner", ""), v.value("generation", ""), v.value("ttl", 0.0));
            return nullptr;
        }
        if (op == "note_stream") {
            s.note_stream(v.get<std::string>());
            return nullptr;
        }
        if (op == "missing_streams")
            return s.missing_streams(v.get<bool>());
        if (op == "readiness") {
            auto [ready, reason] = s.readiness();
            return {{"ready", ready}, {"reason", reason}};
        }
        if (op == "tick") {
            s.tick();
            return nullptr;
        }
        if (op == "close") {
            s.close();
            return nullptr;
        }
        throw std::invalid_argument("Unknown Teach operation");
    });
}
char *horus_teach_session_append(void *s, const char *meta, const std::uint8_t *bytes,
                                 std::size_t size) {
    return result([&] {
        if (!s)
            throw std::invalid_argument("Missing session");
        return static_cast<TeachSession *>(s)->append(sample(meta, bytes, size));
    });
}
void horus_teach_session_destroy(void *s) { delete static_cast<TeachSession *>(s); }
void *horus_teach_cartesian_create(const char *capabilities, const char *limits, const char *hands,
                                   const char *offsets, void *context,
                                   horus_teach_callback callback, horus_teach_callback_free release,
                                   char **error) {
    return create(error, [&] {
        if (!callback || !release)
            throw std::invalid_argument("Missing Cartesian callbacks");
        auto callbacks = std::make_shared<CallbackAdapter>(context, callback, release);
        auto l = parse(limits);
        CartesianLimits value{l.at("minimum").get<std::array<double, 3>>(),
                              l.at("maximum").get<std::array<double, 3>>(),
                              l.at("max_speed_m_s").get<double>(),
                              l.at("max_rotation_rad_s").get<double>()};
        value.feedback_timeout_s = l.value("feedback_timeout_s", 0.2);
        value.translation_scale = l.value("translation_scale", 1.0);
        value.floor_to_command_rotation =
            l.value("floor_to_command_rotation", std::array<double, 4>{0, 0, 0, 1});
        return new CartesianAdapter(
            AdapterCapabilities::from_dict(parse(capabilities)), value,
            [callbacks] { return callbacks->call("feedback", nullptr); },
            [callbacks](const Json &v) { return callbacks->call("publish", v); },
            [callbacks](const Json &v) { return callbacks->call("hold", v).get<bool>(); },
            parse(hands).get<std::vector<std::string>>(), parse(offsets));
    });
}
char *horus_teach_cartesian_call(void *adapter, const char *operation, const char *value) {
    return result([&]() -> Json {
        if (!adapter || !operation)
            throw std::invalid_argument("Missing Cartesian adapter");
        auto &a = *static_cast<CartesianAdapter *>(adapter);
        auto v = parse(value);
        std::string op(operation);
        if (op == "ready") {
            auto [ready, reason] = a.ready();
            return {{"ready", ready}, {"reason", reason}};
        }
        if (op == "anchor") {
            a.anchor(v);
            return nullptr;
        }
        if (op == "apply")
            return a.apply(v);
        if (op == "hold")
            return a.hold(v);
        throw std::invalid_argument("Unknown Cartesian operation");
    });
}
void horus_teach_cartesian_destroy(void *adapter) {
    delete static_cast<CartesianAdapter *>(adapter);
}
void *horus_teach_recorder_create(const char *config, const char *robot, const char *episode,
                                  const char *task, char **error) {
    return create(error, [&] {
        if (!robot || !episode || !task)
            throw std::invalid_argument("Missing recorder arguments");
        return new EpisodeRecorder(TeachConfig(parse(config)), robot, episode, task);
    });
}
char *horus_teach_recorder_call(void *recorder, const char *operation, const char *value) {
    return result([&]() -> Json {
        if (!recorder || !operation)
            throw std::invalid_argument("Missing recorder/operation");
        auto &r = *static_cast<EpisodeRecorder *>(recorder);
        std::string op(operation);
        auto v = parse(value);
        if (op == "finish")
            return r.finish(v.value("outcome", "unknown"), v.value("interrupted", false));
        if (op == "event")
            return r.event(v.at("kind"), v.at("data"));
        if (op == "status")
            return {{"error", r.error()}, {"dropped", r.dropped()}, {"path", r.path()}};
        throw std::invalid_argument("Unknown recorder operation");
    });
}
char *horus_teach_recorder_append(void *r, const char *meta, const std::uint8_t *bytes,
                                  std::size_t size) {
    return result([&] {
        if (!r)
            throw std::invalid_argument("Missing recorder");
        return static_cast<EpisodeRecorder *>(r)->append(sample(meta, bytes, size));
    });
}
void horus_teach_recorder_destroy(void *r) { delete static_cast<EpisodeRecorder *>(r); }
char *horus_teach_episode_validate(const char *path) {
    return result([&] {
        if (!path)
            throw std::invalid_argument("Missing path");
        Episode e(path);
        return Json{{"manifest", e.manifest()}, {"issues", e.validate()}};
    });
}
char *horus_teach_episode_recover(const char *path) {
    return result([&] {
        if (!path)
            throw std::invalid_argument("Missing path");
        return recover_episode(path);
    });
}
char *horus_teach_episode_read(const char *path, const char *options, void *context,
                               horus_teach_sample_callback callback) {
    return result([&]() -> Json {
        if (!path || !callback)
            throw std::invalid_argument("Missing reader arguments");
        auto v = parse(options);
        Episode(path).samples(
            [&](const Json &meta, const std::uint8_t *data, std::size_t size) {
                auto text = meta.dump();
                return callback(context, text.c_str(), data, size) != 0;
            },
            v.value("streams", std::vector<std::string>{}), v.value("start_ns", std::uint64_t(0)),
            v.value("end_ns", UINT64_MAX));
        return nullptr;
    });
}
void horus_teach_free(char *value) { std::free(value); }
}
