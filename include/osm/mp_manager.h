#pragma once

#include <algorithm>
#include <mutex>
#include <optional>

#include "osm/assembler.h"
#include "osm/assembler/assemble_rings.h"
#include "osm/assembler/assembler_types.h"
#include "osm/decoder.h"

namespace osm {

struct multi_polygon {
  std::int64_t relation_id;
  std::vector<std::int64_t> ways_refs;
};

std::mutex mp_vec_mtx;
std::vector<osm::multi_polygon> mp_vec_ = std::vector<osm::multi_polygon>{};
std::mutex ways_vec_mtx;
std::vector<osm::Way> all_ways_ = std::vector<osm::Way>{};  // speicherbedarf!?

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
void save_ways_of_relation(std::int64_t const id,
                           Members&& members,
                           Tags&& tags) {
  if (!is_area(tags)) {
    return;
  }
  auto mp = osm::multi_polygon{};
  mp.relation_id = id;
  for (auto const [ref, role, type] : members) {
    if (type != osm::member_type::kWay) {
      continue;
    }
    mp.ways_refs.emplace_back(ref);
  }
  std::lock_guard<std::mutex> lock(mp_vec_mtx);
  mp_vec_.emplace_back(std::move(mp));
}

void save_ways(osm::Way way) {
  std::lock_guard<std::mutex> lock(ways_vec_mtx);
  all_ways_.push_back(way);
}

std::vector<const osm::Way*> make_const_way_ptrs(
    const std::vector<object_id_type>& ids) {
  std::vector<const osm::Way*> ptrs = {};
  ptrs.reserve(ids.size());
  for (const auto& id : ids) {
    auto it = std::find_if(all_ways_.begin(), all_ways_.end(),
                           [id](const osm::Way& w) { return w.id == id; });
    if (it != all_ways_.end()) {
      ptrs.push_back(&(*it));
    }
  }
  return ptrs;
}

template <typename Members, typename Tags>
assembler::polygon_area assemble_area(std::int64_t const id,
                                      Members&& members,
                                      Tags&& tags) {
  assembler::polygon_area a(id);
  if (!is_area(tags)) {
    return a;
  }
  assembler::assembly assemble = assembler::assembly{};
  bool worked = false;
  osm::Relation r = {id, members};
  std::vector<const osm::Way*> ways = {};
  for (auto elem : mp_vec_) {
    if (elem.relation_id == id) {
      ways = make_const_way_ptrs(elem.ways_refs);
      break;
    }
  }
  worked = assemble.assembling_area_from_relation(r, ways, a);
  std::cout << "Worked? " << worked << " Relation id: " << id << std::endl;
  //  add (way.tags()) to area ?
  return a;
}

}  // namespace osm