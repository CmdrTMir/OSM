#pragma once

#include <list>

#include "ProblemReporter.h"
#include "assembler_stats.h"
#include "assembler_types.h"

namespace assembler {
constexpr std::size_t max_split_locations_ = 100ULL;
constexpr int max_depth = 20;

struct State {
  State() = delete;
  State(const State&) = delete;
  State& operator=(const State&) = delete;
  explicit State(std::ostream* out, bool enable_debug = true)
      : problem_reporter(out, enable_debug), debug(enable_debug) {}

  bool debug;
  SegmentList segment_list{};
  area_stats stats{};
  ProblemReporter problem_reporter;
  std::list<ProtoRing> rings{};
  std::vector<slocation> slocations{};
  std::vector<osm::Location> split_locations{};
};

}  // namespace assembler