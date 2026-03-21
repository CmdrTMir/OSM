#pragma once

#include <list>

#include "ProblemReporter.h"
#include "assembler_stats.h"
#include "assembler_types.h"

namespace assembler {
constexpr std::size_t max_split_locations_ = 100ULL;
constexpr int max_depth = 20;

struct State {
  bool debug = true;
  SegmentList segment_list = SegmentList{};
  area_stats stats = area_stats{};
  ProblemReporter problem_reporter = ProblemReporter{};
  std::list<ProtoRing> rings = std::list<ProtoRing>{};
  std::vector<slocation> slocations = std::vector<slocation>{};
  std::vector<osm::Location> split_locations = std::vector<osm::Location>{};
};

}  // namespace assembler