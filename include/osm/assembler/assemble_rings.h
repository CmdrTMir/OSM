#pragma once

#include <list>

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

//
/*
 *HELPER FUNCTIONS
 */
//
NodeRefSegment* get_next_segment(const osm::Location& location,
                                 std::vector<slocation>& locations,
                                 SegmentList& segment_list) {
  auto it = std::lower_bound(
      locations.begin(), locations.end(), slocation{},
      [&segment_list, &location](const slocation& lhs, const slocation& rhs) {
        return lhs.location(segment_list, location)
            .smaller_than(rhs.location(segment_list, location));
      });

  assert(it != locations.end());
  if (segment_list[it->item].is_done()) {
    ++it;
  }
  assert(it != locations.end());
  assert(!segment_list[it->item].is_done());
  return &segment_list[it->item];
}

void remove_duplicates(std::vector<rings_stack_element>& outer_rings) {
  while (true) {
    const auto it = std::adjacent_find(outer_rings.begin(), outer_rings.end());
    if (it == outer_rings.end()) {
      return;
    }
    outer_rings.erase(it, std::next(it, 2));
  }
}

ProtoRing* find_enclosing_ring(NodeRefSegment* segment,
                               SegmentList& segment_list,
                               bool debug) {
  if (debug) {
    std::cerr << "    Looking for ring enclosing \n";
  }

  const auto location = segment->first().location();
  const auto end_location = segment->second().location();

  while (segment->first().location().equal_to(location)) {
    if (segment == &segment_list.back()) {
      break;
    }
    ++segment;
  }

  int nesting = 0;
  std::vector<rings_stack_element> outer_rings;
  while (segment >= &segment_list.front()) {
    if (!segment->is_direction_done()) {
      --segment;
      continue;
    }
    if (debug) {
      std::cerr << "      Checking against " << segment << "\n";
    }
    const osm::Location& a = segment->first().location();
    const osm::Location& b = segment->second().location();

    if (segment->first().location().equal_to(location)) {
      const std::int64_t ax = a.x();
      const std::int64_t bx = b.x();
      const std::int64_t lx = end_location.x();
      const std::int64_t ay = a.y();
      const std::int64_t by = b.y();
      const std::int64_t ly = end_location.y();
      const auto z = ((bx - ax) * (ly - ay)) - ((by - ay) * (lx - ax));
      if (debug) {
        std::cerr << "      Segment z=" << z << '\n';
      }
      if (z > 0) {
        nesting += segment->is_reverse() ? -1 : 1;
        if (debug) {
          std::cerr << "        Segment is below (nesting=" << nesting << ")\n";
        }
        if (segment->ring()->is_outer()) {
          if (debug) {
            std::cerr << "        Segment belongs to outer ring (y=" << a.y()
                      << " ring=" << segment->ring() << ")\n";
          }
          outer_rings.emplace_back(a.y(), segment->ring());
        }
      }
    } else if (a.x() <= location.x() && location.x() < b.x()) {
      if (debug) {
        std::cerr << "        Is in x range\n";
      }

      const std::int64_t ax = a.x();
      const std::int64_t bx = b.x();
      const std::int64_t lx = location.x();
      const std::int64_t ay = a.y();
      const std::int64_t by = b.y();
      const std::int64_t ly = location.y();
      const auto z = ((bx - ax) * (ly - ay)) - ((by - ay) * (lx - ax));

      if (z >= 0) {
        nesting += segment->is_reverse() ? -1 : 1;
        if (debug) {
          std::cerr << "        Segment is below (nesting=" << nesting << ")\n";
        }
        if (segment->ring()->is_outer()) {
          const double y = static_cast<double>(ay) +
                           (static_cast<double>((by - ay) * (lx - ax)) /
                            static_cast<double>(bx - ax));
          if (debug) {
            std::cerr << "        Segment belongs to outer ring (y=" << y
                      << " ring=" << segment->ring() << ")\n";
          }
          outer_rings.emplace_back(y, segment->ring());
        }
      }
    }
    --segment;
  }

  if (nesting % 2 == 0) {
    if (debug) {
      std::cerr << "    Decided that this is an outer ring\n";
    }
    return nullptr;
  }
  if (debug) {
    std::cerr << "    Decided that this is an inner ring\n";
  }
  assert(!outer_rings.empty());
  std::stable_sort(outer_rings.rbegin(), outer_rings.rend());
  if (debug) {
    for (const auto& o : outer_rings) {
      std::cerr << "        y=" << o.y()
                << std::endl;  // " " << o.ring() << "\n";
    }
  }

  remove_duplicates(outer_rings);
  if (debug) {
    std::cerr << "      after remove duplicates:\n";
    for (const auto& o : outer_rings) {
      std::cerr << "        y=" << o.y()
                << std::endl;  //" " << o.ring() << "\n";
    }
  }

  assert(!outer_rings.empty());
  return outer_rings.front().ring_ptr();
}

std::uint32_t add_new_ring(const slocation& node,
                           SegmentList& segment_list,
                           std::list<ProtoRing>& rings,
                           std::vector<slocation>& locations,
                           bool debug) {
  NodeRefSegment* segment = &segment_list[node.item];
  assert(!segment->is_done());
  if (debug) {
    std::cerr << "  Starting new ring at location "
              << node.location(segment_list).x() << ","
              << node.location(segment_list).y() << " with segment " << segment
              << "\n";
  }
  if (node.reverse) {
    segment->reverse();
  }

  ProtoRing* outer_ring = nullptr;
  if (segment != &segment_list.front()) {
    outer_ring = find_enclosing_ring(segment, segment_list, debug);
  }
  segment->mark_direction_done();
  rings.emplace_back(segment);
  ProtoRing* ring = &rings.back();
  if (outer_ring) {
    if (debug) {
      std::cerr << "    This is an inner ring:\n";
    }
    outer_ring->add_inner_ring(ring);
    ring->set_outer_ring(outer_ring);
  } else if (debug) {
    std::cerr << "    This is an outer ring\n";
  }

  const osm::Location& first_location = node.location(segment_list);
  osm::Location last_location = segment->stop().location();

  uint32_t nodes = 1;
  while (!first_location.equal_to(last_location)) {
    ++nodes;
    NodeRefSegment* next_segment =
        get_next_segment(last_location, locations, segment_list);
    next_segment->mark_direction_done();
    if (!next_segment->start().location().equal_to(last_location)) {
      next_segment->reverse();
    }
    ring->add_segment_back(next_segment);
    if (debug) {
      std::cerr << "    Next segment is " << next_segment->first_noderef_.ref()
                << "\n";
    }
    last_location = next_segment->stop().location();
  }

  ring->fix_direction();

  if (debug) {
    std::cerr << "    Completed ring \n";
  }
  return nodes;
}

bool create_rings_complex_case(SegmentList& segment_list,
                               std::list<ProtoRing>& rings,
                               std::vector<slocation>& locations,
                               std::vector<osm::Location>& split_locations,
                               bool debug) {
  //   // First create all the (partial) rings starting at the split locations
  //   auto count_remaining = segment_list.size();
  //   for (const osm::Location& location : split_locations) {
  //     const auto locs = make_range(std::equal_range(
  //         locations.begin(), locations.end(), slocation{},
  //         [&segment_list, &location](const slocation& lhs, const slocation&
  //         rhs) {
  //           return lhs.location(segment_list, location)
  //               .smaller_than(rhs.location(segment_list, location));
  //         }));
  //     for (auto& loc : locs) {
  //       if (!segment_list[loc.item].is_done()) {
  //         count_remaining -= add_new_ring_complex(loc);
  //         if (count_remaining == 0) {
  //           break;
  //         }
  //       }
  //     }
  //   }

  //   // Now find all the rest of the rings (ie not starting at split
  //   locations) if (count_remaining > 0) {
  //     for (const slocation& sl : locations) {
  //       const NodeRefSegment& segment = segment_list[sl.item];
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
  //     for (auto it = rings.begin(); it != rings.end(); ++it) {
  //       if (!it->closed()) {
  //         open_ring_its.push_back(it);
  //       }
  //     }

  //     while (!open_ring_its.empty()) {
  //       if (debug) {
  //         std::cerr << "  There are " << open_ring_its.size() << " open
  //         rings\n";
  //       }
  //       while (try_to_merge(open_ring_its)) {
  //         // intentionally left blank
  //       }
  //       if (!open_ring_its.empty()) {
  //         if (debug) {
  //           std::cerr << "  After joining obvious cases there are still "
  //                     << open_ring_its.size() << " open rings\n";
  //         }
  //         if (!join_connected_rings(open_ring_its)) {
  //           return false;
  //         }
  //       }
  //     }

  //     if (debug) {
  //       std::cerr << "  Joined all open rings\n";
  //     }
  //   }

  //   // Now all rings are complete.
  //   find_inner_outer_complex();

  //   return true;
}

}  // namespace assembler
