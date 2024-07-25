#ifndef PHYSICS_FORMATTING_HPP
#define PHYSICS_FORMATTING_HPP

#include "physics.hpp"
#include "spdlog/fmt/ostr.h" // must be included
#include "spdlog/sinks/stdout_sinks.h"
#include "spdlog/spdlog.h"

std::ostream &operator<<(std::ostream &os, const Physics &physics);

std::ostream &operator<<(std::ostream &os, const JPH::Vec3 &vector);

// fmt v10 and above requires `fmt::formatter<T>` extends
// `fmt::ostream_formatter`. See: https://github.com/fmtlib/fmt/issues/3318
template<>
struct fmt::formatter<Physics> : fmt::ostream_formatter {
};

template<>
struct fmt::formatter<JPH::Vec3> : fmt::ostream_formatter {
};

#endif // PHYSICS_FORMATTING_HPP