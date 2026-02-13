#ifndef HORUS_TOPICS_HPP
#define HORUS_TOPICS_HPP

#include <string>

namespace horus {
namespace topics {

inline constexpr const char* kRegistration = "/horus/registration";
inline constexpr const char* kRegistrationAck = "/horus/registration_ack";
inline constexpr const char* kHeartbeat = "/horus/heartbeat";

} // namespace topics
} // namespace horus

#endif // HORUS_TOPICS_HPP

