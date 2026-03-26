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

bool first = true;
std::mutex mp_vec_mtx;
std::vector<osm::multi_polygon> mp_vec_ = std::vector<osm::multi_polygon>{};
std::mutex ways_vec_mtx;
std::unordered_map<object_id_type, osm::Way> all_ways_;

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
int count_non_areas = 0;
template <typename Members, typename Tags>
void save_ways_of_relation(std::int64_t const id,
                           Members&& members,
                           Tags&& tags) {
  if (!is_area(tags)) {
    count_non_areas++;
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

void reserve_way_map(size_t expected_count) {
  all_ways_.reserve(expected_count);
}

void save_ways(osm::Way way) {
  std::lock_guard<std::mutex> lock(ways_vec_mtx);
  all_ways_.insert({way.id, way});
}

std::vector<const osm::Way*> make_const_way_ptrs(
    const std::vector<object_id_type>& ids) {
  std::vector<const osm::Way*> ptrs = {};
  ptrs.reserve(ids.size());
  for (const auto& id : ids) {
    auto it = all_ways_.find(id);
    if (it != all_ways_.end()) {
      ptrs.push_back(&(*it).second);
    }
  }
  return ptrs;
}

int cannot = 0;
int last = 0;

template <typename Members, typename Tags>
assembler::polygon_area assemble_area(std::int64_t const id,
                                      Members&& members,
                                      Tags&& tags,
                                      int count) {
  last++;
  if (first) {
    std::cout << "vectorsize: " << mp_vec_.size()
              << " not areas: " << count_non_areas << std::endl;
    first = false;
  }
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
  if (!a.valid && a.missing_flag == true) {
    cannot++;
    // std::cout
    //<< "This realtion couldn't be assembled into an area, because ways "
    //    "are missing in the dataset: "
    //  << id << std::endl;
  }
  // 5197022 id die hier gebaut wird und in libosmium nicht! monaco

  if (last == count) {
    std::cout << "couldn't assemble: " << cannot << std::endl;
  }
  //  add (way.tags()) to area ?
  return a;
}

}  // namespace osm