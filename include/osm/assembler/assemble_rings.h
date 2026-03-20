#pragma once

#include <cassert>
#include <list>

#include "assembler_types.h"
#include "merge_rings_helper.h"
#include "osm/types.h"
#include "state.h"

namespace assembler {

uint32_t add_new_ring_complex(const slocation& node) {
  NodeRefSegment* segment = &state_.segment_list[node.item];
  assert(!segment->is_done());
  if (state_.debug) {
    std::cerr << "  Starting new ring at location "
              << node.location(state_.segment_list).x() << ","
              << node.location(state_.segment_list).y() << " with segment "
              << segment << "\n";
  }
  if (node.reverse) {
    segment->reverse();
  }
  state_.rings.emplace_back(segment);
  ProtoRing* ring = &state_.rings.back();

  const osm::Location& first_location = node.location(state_.segment_list);
  osm::Location last_location = segment->stop().location();

  auto is_split_location = [&](osm::Location loc) {
    return std::find(state_.split_locations.cbegin(),
                     state_.split_locations.cend(),
                     loc) != state_.split_locations.cend();
  };

  uint32_t nodes = 1;
  while (first_location != last_location && !is_split_location(last_location)) {
    ++nodes;
    NodeRefSegment* next_segment = get_next_segment(last_location);
    if (next_segment->start().location() != last_location) {
      next_segment->reverse();
    }
    ring->add_segment_back(next_segment);
    if (state_.debug) {
      std::cerr << "    Next segment is " << next_segment << "\n";
    }
    last_location = next_segment->stop().location();
  }
  if (state_.debug) {
    if (first_location == last_location) {
      std::cerr << "    Completed ring: " << ring << "\n";
    } else {
      std::cerr << "    Completed partial ring: " << ring << "\n";
    }
  }
  return nodes;
}

std::uint32_t add_new_ring(const slocation& node) {
  NodeRefSegment* segment = &state_.segment_list[node.item];
  assert(!segment->is_done());
  if (state_.debug) {
    std::cerr << "  Starting new ring at location "
              << node.location(state_.segment_list).x() << ","
              << node.location(state_.segment_list).y() << " with segment "
              << segment << "\n";
  }
  if (node.reverse) {
    segment->reverse();
  }

  ProtoRing* outer_ring = nullptr;
  if (segment != &state_.segment_list.front()) {
    outer_ring = find_enclosing_ring(segment);
  }
  segment->mark_direction_done();
  state_.rings.emplace_back(segment);
  ProtoRing* ring = &state_.rings.back();
  if (outer_ring) {
    if (state_.debug) {
      std::cerr << "    This is an inner ring:\n";
    }
    outer_ring->add_inner_ring(ring);
    ring->set_outer_ring(outer_ring);
  } else if (state_.debug) {
    std::cerr << "    This is an outer ring\n";
  }

  const osm::Location& first_location = node.location(state_.segment_list);
  osm::Location last_location = segment->stop().location();

  uint32_t nodes = 1;
  while (first_location != last_location) {
    ++nodes;
    NodeRefSegment* next_segment = get_next_segment(last_location);
    next_segment->mark_direction_done();
    if (next_segment->start().location() != last_location) {
      next_segment->reverse();
    }
    ring->add_segment_back(next_segment);
    if (state_.debug) {
      std::cerr << "    Next segment is " << next_segment->first_noderef_.ref()
                << "\n";
    }
    last_location = next_segment->stop().location();
  }

  ring->fix_direction();

  if (state_.debug) {
    std::cerr << "    Completed ring \n";
  }
  return nodes;
}

bool create_rings_complex_case() {
  // First create all the (partial) rings starting at the split locations
  auto count_remaining = state_.segment_list.size();
  for (const osm::Location& location : state_.split_locations) {
    const auto locs = make_range(std::equal_range(
        state_.slocations.begin(), state_.slocations.end(), slocation{},
        [&location](const slocation& lhs, const slocation& rhs) {
          return lhs.location(state_.segment_list, location) <
                 rhs.location(state_.segment_list, location);
        }));
    for (auto& loc : locs) {
      if (!state_.segment_list[loc.item].is_done()) {
        count_remaining -= add_new_ring_complex(loc);
        if (count_remaining == 0) {
          break;
        }
      }
    }
  }
  // Now find all the rest of the rings (ie not starting at split
  // locations)
  if (count_remaining > 0) {
    for (const slocation& sl : state_.slocations) {
      const NodeRefSegment& segment = state_.segment_list[sl.item];
      if (!segment.is_done()) {
        count_remaining -= add_new_ring_complex(sl);
        if (count_remaining == 0) {
          break;
        }
      }
    }
  }

  auto there_are_open_rings = [&]() {
    return std::any_of(state_.rings.cbegin(), state_.rings.cend(),
                       [](const ProtoRing& ring) { return !ring.closed(); });
  };

  // Now all segments are in exactly one (partial) ring.
  // If there are open rings, try to join them to create closed
  // rings.
  if (there_are_open_rings()) {
    ++state_.stats.area_really_complex_case;

    open_ring_its_type open_ring_its;
    for (auto it = state_.rings.begin(); it != state_.rings.end(); ++it) {
      if (!it->closed()) {
        open_ring_its.push_back(it);
      }
    }

    while (!open_ring_its.empty()) {
      if (state_.debug) {
        std::cerr << "There are " << open_ring_its.size() << " open rings\n";
      }
      while (try_to_merge(open_ring_its)) {
        // intentionally left blank
      }
      if (!open_ring_its.empty()) {
        if (state_.debug) {
          std::cerr << "  After joining obvious cases there are still "
                    << open_ring_its.size() << " open rings\n";
        }
        if (!join_connected_rings(open_ring_its)) {
          return false;
        }
      }
    }

    if (state_.debug) {
      std::cerr << "  Joined all open rings\n";
    }
  }

  // Now all rings are complete.
  find_inner_outer_complex();

  return true;
}

}  // namespace assembler
