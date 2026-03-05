#pragma once

#include <optional>

#include "osm/assembler.h"
#include "osm/decoder.h"

namespace osm {

// struct polygon {};

struct multi_polygon {
  std::vector<std::int64_t> ways_refs;
  std::vector<std::vector<std::int64_t>> way_nodes_refs;

  // std::vector<polygon> multipolygon_;
};

// gemacht um immer wieder zu verwenden... noch füllen
struct polygon_area {
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
void save_ways_of_relation(multi_polygon& mp,
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
    mp.ways_refs.emplace_back(ref);
  }
}

// nicht sicher!
// 1 - ways gebaut und diese dann in den hybrid_node_idx mit update location
// nur einzeln
// 2 - noch nicht überprüft
template <typename References, typename Tags>
osm::Way save_nodes_of_ways(tiles::hybrid_node_idx& nodes,
                            std::int64_t const id,
                            References&& refs) {
  std::vector<NodeRef> way_node_refs;
  for (auto r : refs) {
    NodeRef roderef = r;
    auto const& coords = get_coords(nodes, r);
    roderef.set_location(coords);
    way_node_refs.emplace_back(roderef);
  }
  return Way{id, way_node_refs};
}

template <typename Members, typename Tags>
std::optional<polygon_area> assemble_area(multi_polygon& mp,
                                          std::int64_t const id,
                                          Members&& members,
                                          Tags&& tags) {
  if (!is_area(tags)) {
    return {};
  }

  // ... build area (assembler code from libosmium)
  polygon_area a =
      polygon_area{};  // das ist das was dann der out_buffer sein soll
  assembler::assembling_area();
  return a;
}

}  // namespace osm