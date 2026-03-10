#pragma once

#include <list>

#include "ProtoRing.h"
#include "osm/types.h"
#include "segment_list.h"

namespace assembler {

using open_ring_its_type = std::list<std::list<ProtoRing>::iterator>;

struct location_to_ring_map {
  osm::Location location;
  open_ring_its_type::iterator ring_it;
  bool start{false};

  location_to_ring_map(osm::Location l,
                       open_ring_its_type::iterator r,
                       const bool s) noexcept
      : location(l), ring_it(r), start(s) {}

  explicit location_to_ring_map(osm::Location l) noexcept : location(l) {}
  const ProtoRing& ring() const noexcept { return **ring_it; }

};  // struct location_to_ring_map

inline bool operator==(const location_to_ring_map& lhs,
                       const location_to_ring_map& rhs) noexcept {
  return lhs.location == rhs.location;
}

inline bool operator<(const location_to_ring_map& lhs,
                      const location_to_ring_map& rhs) noexcept {
  return lhs.location < rhs.location;
}

struct candidate {
  int64_t sum;
  std::vector<std::pair<location_to_ring_map, bool>> rings;
  osm::Location start_location;
  osm::Location stop_location;
  explicit candidate(location_to_ring_map& ring, bool reverse)
      : sum(ring.ring().sum()),
        start_location(ring.ring().get_node_ref_start().location()),
        stop_location(ring.ring().get_node_ref_stop().location()) {
    rings.emplace_back(ring, reverse);
  }
  bool closed() const noexcept { return start_location == stop_location; }
};

struct slocation {
  enum { invalid_item = 1U << 30U };
  std::uint32_t item : 31;
  std::uint32_t reverse : 1;
  slocation() noexcept : item(invalid_item), reverse(false) {}
  explicit slocation(uint32_t n, bool r = false) noexcept
      : item(n), reverse(r) {}
  osm::Location location(const SegmentList& segment_list) const noexcept {
    const auto& segment = segment_list[item];
    return reverse ? segment.second().location() : segment.first().location();
  }
  const osm::NodeRef& node_ref(const SegmentList& segment_list) const noexcept {
    const auto& segment = segment_list[item];
    return reverse ? segment.second() : segment.first();
  }
  osm::Location location(const SegmentList& segment_list,
                         const osm::Location& default_location) const noexcept {
    if (item == invalid_item) {
      return default_location;
    }
    return location(segment_list);
  }
};  // struct slocation

struct rings_stack_element {
  double m_y;
  ProtoRing* m_ring_ptr;

  rings_stack_element(double y, ProtoRing* ring_ptr)
      : m_y(y), m_ring_ptr(ring_ptr) {}
  double y() const noexcept { return m_y; }
  const ProtoRing& ring() const noexcept { return *m_ring_ptr; }
  ProtoRing* ring_ptr() noexcept { return m_ring_ptr; }
  bool operator==(const rings_stack_element& rhs) const noexcept {
    return m_ring_ptr == rhs.m_ring_ptr;
  }
  bool operator<(const rings_stack_element& rhs) const noexcept {
    return m_y < rhs.m_y;
  }
};  // struct rings_stack_element

}  // namespace assembler
