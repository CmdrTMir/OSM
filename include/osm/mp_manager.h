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

// WIP
template <typename Tags>
inline bool is_area(Tags&& tags) {
  int count = 0;
  for (auto const& [key, value] : tags) {
    if (key == "type") {
      count++;
    }
    std::cout << "key: " << key << "   value: " << value << "\n";
  }
  std::cout << "counted " << count << " type-tags \n";
  const char* type = "";  // = tags.get_value_by_key("type");
  if (type == nullptr) {
    return false;
  }
  if ((!std::strcmp(type, "multipolygon")) ||
      (!std::strcmp(type, "boundary"))) {
    return false;
  }
  return true;
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