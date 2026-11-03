#include "bridge_process.hpp"
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fcntl.h>
#include <spawn.h>
#include <stdexcept>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

extern char** environ;

std::string BridgeProcess::locate() {
    const char* prefix = std::getenv("AMENT_PREFIX_PATH");
    if (!prefix) return {};
    const std::string paths(prefix);
    std::size_t start = 0;
    while (start <= paths.size()) {
        const auto end = paths.find(':', start);
        const auto entry = paths.substr(start, end == std::string::npos ? end : end - start);
        if (!entry.empty()) {
            const auto binary = std::filesystem::path(entry) / "lib/horus_unity_bridge/horus_unity_bridge_node";
            if (::access(binary.c_str(), X_OK) == 0) return binary.string();
        }
        if (end == std::string::npos) break;
        start = end + 1;
    }
    return {};
}

void BridgeProcess::start(const std::string& executable, const std::vector<std::string>& arguments) {
    if (started_) throw std::logic_error("Bridge launch has already been attempted");
    if (!std::filesystem::path(executable).is_absolute() || executable.find('\0') != std::string::npos)
        throw std::invalid_argument("Bridge executable must be an absolute path");
    std::vector<std::string> values{executable};
    values.insert(values.end(), arguments.begin(), arguments.end());
    std::vector<char*> argv;
    argv.reserve(values.size() + 1);
    for (auto& value : values) {
        if (value.find('\0') != std::string::npos) throw std::invalid_argument("Bridge argument contains NUL");
        argv.push_back(value.data());
    }
    argv.push_back(nullptr);
    int pipe_fd[2];
    if (::pipe2(pipe_fd, O_CLOEXEC) != 0) throw std::runtime_error(std::strerror(errno));
    posix_spawn_file_actions_t actions;
    posix_spawnattr_t attributes;
    const int init_actions = ::posix_spawn_file_actions_init(&actions);
    const int init_attributes = ::posix_spawnattr_init(&attributes);
    int error = init_actions ? init_actions : init_attributes;
    auto checked = [&](int result) { if (!error) error = result; };
    if (!error) {
        checked(::posix_spawn_file_actions_addopen(&actions, STDIN_FILENO, "/dev/null", O_RDONLY, 0));
        checked(::posix_spawn_file_actions_adddup2(&actions, pipe_fd[1], STDOUT_FILENO));
        checked(::posix_spawn_file_actions_adddup2(&actions, pipe_fd[1], STDERR_FILENO));
        checked(::posix_spawn_file_actions_addclose(&actions, pipe_fd[0]));
        checked(::posix_spawn_file_actions_addclose(&actions, pipe_fd[1]));
        checked(::posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETPGROUP));
        checked(::posix_spawnattr_setpgroup(&attributes, 0));
    }
    pid_t child = -1;
    if (!error) error = ::posix_spawn(&child, executable.c_str(), &actions, &attributes, argv.data(), environ);
    if (!init_actions) ::posix_spawn_file_actions_destroy(&actions);
    if (!init_attributes) ::posix_spawnattr_destroy(&attributes);
    ::close(pipe_fd[1]);
    if (error) {
        ::close(pipe_fd[0]);
        throw std::runtime_error("Cannot start HORUS bridge: " + std::string(std::strerror(error)));
    }
    pid_ = child;
    output_ = pipe_fd[0];
    started_ = true;
    const int flags = ::fcntl(output_, F_GETFL);
    if (flags < 0 || ::fcntl(output_, F_SETFL, flags | O_NONBLOCK) < 0) {
        ::close(output_);
        output_ = -1;
        stop();
        throw std::runtime_error("Cannot configure bridge log capture");
    }
}

void BridgeProcess::drain() {
    if (output_ < 0) return;
    char buffer[4096];
    for (int batch = 0; batch < 16; ++batch) {
        const auto size = ::read(output_, buffer, sizeof(buffer));
        if (size > 0) {
            tail_.append(buffer, static_cast<std::size_t>(size));
            if (tail_.size() > 65536) tail_.erase(0, tail_.size() - 65536);
        } else if (size == 0) {
            ::close(output_);
            output_ = -1;
            break;
        } else if (errno != EINTR) break;
    }
}

void BridgeProcess::poll() {
    drain();
    if (pid_ <= 0) return;
    int status = 0;
    const auto result = ::waitpid(pid_, &status, WNOHANG);
    if (result == pid_) {
        pid_ = -1;
        error_ = WIFEXITED(status) ? "HORUS bridge exited with status " + std::to_string(WEXITSTATUS(status))
                                 : "HORUS bridge terminated by signal " + std::to_string(WTERMSIG(status));
        drain();
    } else if (result < 0 && errno == ECHILD) {
        pid_ = -1;
        error_ = "HORUS bridge child was reaped outside its owner";
    }
}

void BridgeProcess::stop() noexcept {
    // Only signal our still-unreaped child group; never find processes by name/port.
    if (pid_ > 0) {
        for (const int signal : {SIGINT, SIGTERM, SIGKILL}) {
            ::kill(-pid_, signal);
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
            do {
                int status = 0;
                const auto result = ::waitpid(pid_, &status, WNOHANG);
                if (result == pid_ || (result < 0 && errno == ECHILD)) { pid_ = -1; break; }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            } while (std::chrono::steady_clock::now() < deadline);
            if (pid_ <= 0) break;
        }
        if (pid_ > 0) {
            int status;
            while (::waitpid(pid_, &status, 0) < 0 && errno == EINTR) {}
            pid_ = -1;
        }
    }
    if (output_ >= 0) { ::close(output_); output_ = -1; }
}
BridgeProcess::~BridgeProcess() { stop(); }
