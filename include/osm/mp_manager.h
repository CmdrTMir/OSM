#pragma once

#include <optional>

#include "osm/decoder.h"

namespace osm {

struct polygon {};

struct multi_polygons {
  std::vector<std::int64_t> ways_;
  std::vector<std::vector<std::int64_t>> way_nodes_;

  std::vector<polygon> multipolygon_;
};

// gemacht um immer wieder zu verwenden... noch füllen
struct polygon_area {
  int lng;
  // void reset() {}
};

template <typename Tags>
inline bool is_area(Tags&& tags) {
  for (auto const& [key, value] : tags) {
    // std::strcmp(type, "multipolygon")
    if (key == "type" && (value == "multipolygon" || value == "boundary")) {
      return true;
    }
  }
  return false;
}

template <typename Members, typename Tags>
void save_ways_of_relation(multi_polygons& mp,
                           std::int64_t const id,
                           Members&& members,
                           Tags&& tags) {
  if (!is_area(tags)) {
    return;
  }
  for (auto const [ref, role, type] : members) {
    if (type != osm::member_type::kWay) {
      continue;
    }
    mp.ways_.emplace_back(ref);
  }
}

template <typename Members, typename Tags>
std::optional<polygon_area> assemble_area(multi_polygons& mp,
                                          std::int64_t const id,
                                          Members&& members,
                                          Tags&& tags) {
  if (!is_area(tags)) {
    return {};
  }

  // ... build area (assembler code from libosmium)
  polygon_area a = polygon_area{1};
  return a;
}

}  // namespace osm