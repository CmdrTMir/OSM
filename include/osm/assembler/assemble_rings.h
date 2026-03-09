#pragma once

#include "osm/types.h"
#include "segment_list.h"

// TODO:    1. rings und segments als übergabeparameter Referenz
//          2. output

namespace assembler {
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

std::uint32_t add_new_ring(const slocation& node, bool debug) {
  NodeRefSegment* segment = &m_segment_list[node.item];
  assert(!segment->is_done());

  if (debug) {
    std::cerr << "  Starting new ring at location "
              << node.location(m_segment_list) << " with segment " << *segment
              << "\n";
  }

  if (node.reverse) {
    segment->reverse();
  }

  ProtoRing* outer_ring = nullptr;

  if (segment != &m_segment_list.front()) {
    outer_ring = find_enclosing_ring(segment);
  }
  segment->mark_direction_done();

  m_rings.emplace_back(segment);
  ProtoRing* ring = &m_rings.back();
  if (outer_ring) {
    if (debug) {
      std::cerr << "    This is an inner ring. Outer ring is " << *outer_ring
                << "\n";
    }
    outer_ring->add_inner_ring(ring);
    ring->set_outer_ring(outer_ring);
  } else if (debug) {
    std::cerr << "    This is an outer ring\n";
  }

  const osm::Location& first_location = node.location(m_segment_list);
  osm::Location last_location = segment->stop().location();

  uint32_t nodes = 1;
  while (!first_location.equal_to(last_location)) {
    ++nodes;
    NodeRefSegment* next_segment = get_next_segment(last_location);
    next_segment->mark_direction_done();
    if (!next_segment->start().location().equal_to(last_location)) {
      next_segment->reverse();
    }
    ring->add_segment_back(next_segment);
    if (debug) {
      std::cerr << "    Next segment is " << *next_segment << "\n";
    }
    last_location = next_segment->stop().location();
  }

  ring->fix_direction();

  if (debug) {
    std::cerr << "    Completed ring: " << *ring << "\n";
  }
  return nodes;
}

bool create_rings_complex_case() {
  //   // First create all the (partial) rings starting at the split locations
  //   auto count_remaining = m_segment_list.size();
  //   for (const osmium::Location& location : m_split_locations) {
  //     const auto locs = make_range(std::equal_range(
  //         m_locations.begin(), m_locations.end(), slocation{},
  //         [this, &location](const slocation& lhs, const slocation& rhs) {
  //           return lhs.location(m_segment_list, location) <
  //                  rhs.location(m_segment_list, location);
  //         }));
  //     for (auto& loc : locs) {
  //       if (!m_segment_list[loc.item].is_done()) {
  //         count_remaining -= add_new_ring_complex(loc);
  //         if (count_remaining == 0) {
  //           break;
  //         }
  //       }
  //     }
  //   }

  //   // Now find all the rest of the rings (ie not starting at split
  //   locations) if (count_remaining > 0) {
  //     for (const slocation& sl : m_locations) {
  //       const NodeRefSegment& segment = m_segment_list[sl.item];
  //       if (!segment.is_done()) {
  //         count_remaining -= add_new_ring_complex(sl);
  //         if (count_remaining == 0) {
  //           break;
  //         }
  //       }
  //     }
  //   }

  //   // Now all segments are in exactly one (partial) ring.

  //   // If there are open rings, try to join them to create closed
  //   // rings.
  //   if (there_are_open_rings()) {
  //     ++m_stats.area_really_complex_case;

  //     open_ring_its_type open_ring_its;
  //     for (auto it = m_rings.begin(); it != m_rings.end(); ++it) {
  //       if (!it->closed()) {
  //         open_ring_its.push_back(it);
  //       }
  //     }

  //     while (!open_ring_its.empty()) {
  //       if (debug()) {
  //         std::cerr << "  There are " << open_ring_its.size() << " open
  //         rings\n";
  //       }
  //       while (try_to_merge(open_ring_its)) {
  //         // intentionally left blank
  //       }

  //       if (!open_ring_its.empty()) {
  //         if (debug()) {
  //           std::cerr << "  After joining obvious cases there are still "
  //                     << open_ring_its.size() << " open rings\n";
  //         }
  //         if (!join_connected_rings(open_ring_its)) {
  //           return false;
  //         }
  //       }
  //     }

  //     if (debug()) {
  //       std::cerr << "  Joined all open rings\n";
  //     }
  //   }

  //   // Now all rings are complete.

  //   find_inner_outer_complex();

  //   return true;
}

}  // namespace assembler
