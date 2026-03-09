#pragma once

#include "assembler/ProtoRing.h"
#include "assembler/assemble_rings.h"
#include "assembler/assembler_stats.h"
#include "assembler/problem_reporter.h"
#include "assembler/segment_list.h"

#include <cassert>
#include <iostream>
#include <list>
#include <vector>

namespace assembler {
struct assembly {

  bool create_rings(bool debug) {
    stats_.nodes += segment_list_.size();
    // Sort the list of segments (from left to right and bottom
    // to top).
    segment_list_.sort();
    // Remove duplicate segments. Removal is in pairs, so if there
    // are two identical segments, they will both be removed. If
    // there are three, two will be removed and one remains.
    segment_list_.erase_duplicate_segments(&problem_reporter_,
                                           stats_.duplicate_segments,
                                           stats_.overlapping_segments);
    // If there are no segments left at this point, this isn't
    // a valid area.
    if (segment_list_.empty()) {
      if (debug) {
        std::cerr << "  No segments left\n";
      }
      return false;
    }
    // Now we look for segments crossing each other. If there are
    // any, the multipolygon is invalid.
    // In the future this could be improved by trying to fix those
    // cases.
    stats_.intersections = segment_list_.find_intersections(&problem_reporter_);
    if (stats_.intersections) {
      return false;
    }
    // locations: An ordered list of locations of both endpoints
    // of all segments with pointers back to the segments. We will
    // use this list later to quickly find which segment(s) fits
    // onto a known segment.
    std::vector<slocation> locations;
    locations.reserve(segment_list_.size() * 2);
    // static_cast is okay here: The 32bit limit is way past
    // anything that makes sense here and even if there are
    // 2^32 segments here, it would simply not go through
    // all of them not building the multipolygon correctly.
    /////assert(segment_list_.size() < std::numeric_limits<uint32_t>::max());
    for (uint32_t n = 0; n < static_cast<uint32_t>(segment_list_.size()); ++n) {
      locations.emplace_back(n, false);
      locations.emplace_back(n, true);
    }
    std::stable_sort(locations.begin(), locations.end(),
                     [this](const slocation& lhs, const slocation& rhs) {
                       return lhs.location(segment_list_)
                           .smaller_than(rhs.location(segment_list_));
                     });
    // Find all locations where more than two segments start or
    // end. We call those "split" locations. If there are any
    // "spike" segments found while doing this, we know the area
    // geometry isn't valid and return.
    /**
     * If there are any open rings found along the way, they are reported
     * and the function returns false.
     */
    bool found_split_locations = false;  // richtig initialisiert?
    std::vector<osm::Location> split_locations;
    osm::Location previous_location;
    for (auto it = locations.cbegin(); it != locations.cend(); ++it) {
      const osm::NodeRef& nr = it->node_ref(segment_list_);
      const osm::Location& loc = nr.location();
      if (std::next(it) == locations.cend() ||
          !loc.equal_to(std::next(it)->location(segment_list_))) {
        if (debug) {
          std::cerr << " Found open ring at " << nr.ref() << "\n";
        }
        const auto& segment = segment_list_[it->item];
        problem_reporter_.report_ring_not_closed(nr, segment.way());
        ++stats_.open_rings;
      } else {
        if (loc.equal_to(previous_location) &&
            (split_locations.empty() ||
             !split_locations.back().equal_to(previous_location))) {
          split_locations.push_back(previous_location);
        }
        ++it;
        if (it == locations.end()) {
          break;
        }
      }
      previous_location = loc;
    }
    found_split_locations = stats_.open_rings == 0;
    //?????
    // if (!find_split_locations()) {
    //   return false;
    // }
    if (!found_split_locations) {
      return false;
    }
    // Now report all split locations to the problem reporter.
    stats_.touching_rings += split_locations.size();
    if (!split_locations.empty()) {
      if (debug) {
        std::cerr << "  Found split locations:\n";
      }
      for (const auto& location : split_locations) {
        auto it = std::lower_bound(
            locations.cbegin(), locations.cend(), slocation{},
            [this, &location](const slocation& lhs, const slocation& rhs) {
              return lhs.location(segment_list_, location)
                  .smaller_than(rhs.location(segment_list_, location));
            });
        ////assert(it != m_locations.cend());
        const osm::object_id_type id = it->node_ref(segment_list_).ref();
        problem_reporter_.report_touching_ring(id, location);
        if (debug) {
          std::cerr << "    " << location.x() << "," << location.y() << "\n";
        }
      }
    }
    // From here on we use two different algorithms depending on
    // whether there were any split locations or not. If there
    // are no splits, we use the faster "simple algorithm", if
    // there are, we use the slower "complex algorithm".
    std::list<ProtoRing> rings;
    if (split_locations.empty()) {
      if (debug) {
        std::cerr << " No split locations -> using simple algorithm\n";
      }
      ++stats_.area_simple_case;
      // create_rings_simple_case:
      auto count_remaining = segment_list_.size();
      for (const slocation& sl : locations) {
        const NodeRefSegment& segment = segment_list_[sl.item];
        if (!segment.is_done()) {
          count_remaining -=
              add_new_ring(sl, segment_list_, rings, locations, debug);
          if (count_remaining == 0) {
            return;
          }
        }
      }
    } else if (split_locations.size() > max_split_locations_) {
      if (debug) {
        std::cerr << " Ignoring polygon with " << split_locations.size()
                  << " split locations (>" << max_split_locations_ << ")\n";
      }
      return false;
    } else {
      if (debug) {
        std::cerr << " Found " << split_locations.size()
                  << " split locations -> using complex algorithm\n";
      }
      ++stats_.area_touching_rings_case;
      if (!create_rings_complex_case(segment_list_, rings, locations,
                                     split_locations, debug)) {  // TODO
        return false;
      }
    }
    // If the assembler was so configured, now check whether the
    // member roles are correctly tagged. --> check always
    // check_inner_outer_roles:
    if (debug) {
      std::cerr << "    Checking inner/outer roles\n";
    }
    int count_segments_for_debug = 0;
    std::unordered_map<const osm::Way*, const ProtoRing*> way_rings;
    std::unordered_set<const osm::Way*> ways_in_multiple_rings;
    for (const ProtoRing& ring : rings) {
      for (const auto& segment : ring.segments()) {
        count_segments_for_debug++;
        ////assert(segment->way());
        if (!segment->role_empty() &&
            (ring.is_outer() ? !segment->role_outer()
                             : !segment->role_inner())) {
          ++stats_.wrong_role;
          if (debug) {
            std::cerr << " Segment: " << count_segments_for_debug
                      << " from way " << segment->way()->id << " has role '"
                      << segment->role_name() << "', but should have role '"
                      << (ring.is_outer() ? "outer" : "inner") << "'\n ";
          }
          if (ring.is_outer()) {
            problem_reporter_.report_role_should_be_outer(
                segment->way()->id, segment->first().location(),
                segment->second().location());
          } else {
            problem_reporter_.report_role_should_be_inner(
                segment->way()->id, segment->first().location(),
                segment->second().location());
          }
        }
        auto& r = way_rings[segment->way()];
        if (!r) {
          r = &ring;
        } else if (r != &ring) {
          ways_in_multiple_rings.insert(segment->way());
        }
      }
      count_segments_for_debug = 0;
    }
    for (const osm::Way* way :
         ways_in_multiple_rings) {  // NOLINT(bugprone - nondeterministic -
                                    // pointer - iteration - order)
      ++stats_.ways_in_multiple_rings;
      if (debug) {
        std::cerr << " Way " << way->id << " is in multiple rings\n ";
      }
      problem_reporter_.report_way_in_multiple_rings(*way);
    }
    // check_inner_outer_roles - finished

    stats_.outer_rings =
        std::count_if(rings.cbegin(), rings.cend(),
                      [](const ProtoRing& ring) { return ring.is_outer(); });
    stats_.inner_rings = rings.size() - stats_.outer_rings;
    return true;
  }

  ////
  /*
   * This is the assembler ---
   */
  ////
  /**
   * Assembles area objects from closed ways or multipolygon relations
   * and their members.
   */
  // HIER STEHEN GEBLIEBEN
  bool create_area_from_way(std::vector<int>& out_buffer,
                            const osm::Way& way,
                            bool debug) {
    osm::AreaBuilder builder{out_buffer};
    builder.initialize_from_object(way);

    const bool area_okay = create_rings(debug);
    if (area_okay || stats_.create_empty_areas) {
      builder.add_item(way.tags());
    }
    if (area_okay) {
      add_rings_to_area(builder);
    }
    return area_okay || stats_.create_empty_areas;
  }

  //   bool create_area_from_relation(
  //       osmium::memory::Buffer& out_buffer,
  //       const osmium::Relation& relation,
  //       const std::vector<const osmium::Way*>& members) {
  //     set_num_members(members.size());
  //     osm::AreaBuilder builder{out_buffer};
  //     builder.initialize_from_object(relation);

  //     const bool area_okay = create_rings();
  //     if (area_okay || config().create_empty_areas) {
  //       if (config().keep_type_tag) {
  //         builder.add_item(relation.tags());
  //       } else {
  //         copy_tags_without_type(builder, relation.tags());
  //       }
  //     }
  //     if (area_okay) {
  //       add_rings_to_area(builder);
  //     }

  //     if (report_ways()) {
  //       for (const osmium::Way* way : members) {
  //         config().problem_reporter->report_way(*way);
  //       }
  //     }

  //     return area_okay || config().create_empty_areas;
  //   }

  /**
   * Assemble an area from the given way.
   * The resulting area is put into the out_buffer.
   *
   * @returns false if there was some kind of error building the
   *          area, true otherwise.
   */
  bool assembling_area_from_way(const osm::Way& way,
                                std::vector<int>& out_buffer,
                                bool report_problems = true,
                                bool debug) {
    if (!stats_.create_way_polygons) {
      return true;
    }
    // Ignore (but count) ways without segments.
    if (way.nodes().size() < 2) {
      ++stats_.short_ways;
      return false;
    }

    if (!way.ends_have_same_id()) {
      ++stats_.duplicate_nodes;
      if (report_problems) {
        problem_reporter_.report_duplicate_node(way.nodes().front().ref(),
                                                way.nodes().back().ref(),
                                                way.nodes().front().location());
      }
    }

    ++stats_.from_ways;
    stats_.invalid_locations = segment_list_.extract_segments_from_way(
        &problem_reporter_, stats_.duplicate_nodes, way);
    // if (!config().ignore_invalid_locations && stats_.invalid_locations > 0) {
    if (stats_.invalid_locations > 0) {
      return false;
    }
    if (debug) {
      std::cerr << "\nAssembling way " << way.id << " containing "
                << segment_list_.size() << " nodes\n";
    }
    const bool okay = create_area_from_way(out_buffer, way, debug);
    if (debug) {
      std::cerr << "Done: " << std::endl;
      stats_.print_stats();
    }
    return okay;
  }

  /**
   * Assemble an area from the given relation and its members.
   * The resulting area is put into the out_buffer.
   *
   * @returns false if there was some kind of error building the
   *          area(s), true otherwise.
   */
  template <typename Members>
  bool assembling_area_from_relation(const osm::Relation<Members>& relation,
                                     const std::vector<const osm::Way*>& ways,
                                     std::vector<int>& out_buffer,
                                     bool report_problems = true,
                                     bool debug) {
    // if (!config().create_new_style_polygons) {
    //   return true;
    // }

    // assert(relation.cmembers().size() >= members.size());

    // if (config().problem_reporter) {
    //   config().problem_reporter->set_object(osm::item_type::relation,
    //                                         relation.id);
    // }

    if (relation.members().empty()) {
      ++stats_.no_way_in_mp_relation;
      return false;
    }
    ++stats_.from_relations;
    stats_.invalid_locations = segment_list_.extract_segments_from_ways(
        &problem_reporter_, stats_.duplicate_nodes, stats_.duplicate_ways,
        relation, ways);
    // if (!config().ignore_invalid_locations && stats_.invalid_locations > 0) {
    if (stats_.invalid_locations > 0) {
      return false;
    }
    stats_.member_ways = ways.size();

    if (stats_.member_ways == 1) {
      ++stats_.single_way_in_mp_relation;
    }

    if (debug) {
      std::cerr << "\nAssembling relation " << relation.id() << "containing "
                << ways.size() << " way members with " << segment_list_.size()
                << " nodes\n";
    }
    const bool okay = create_area_from_relation(out_buffer, relation, ways);
    // if (okay) {
    //   out_buffer.commit();
    // } else {
    //   out_buffer.rollback();
    // }
    return okay;
  }

  SegmentList segment_list_ = SegmentList{};  // init?
  area_stats stats_ = area_stats{};
  problem_reporter problem_reporter_ = problem_reporter{};
  static constexpr const std::size_t max_split_locations_ = 100ULL;
};  // class Assembly

}  // namespace assembler
