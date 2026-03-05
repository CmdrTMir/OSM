#include "assembler/assembler_stats.h"
#include "assembler/problem_reporter.h"
#include "assembler/segment_list.h"

#include <cassert>
#include <iostream>
#include <vector>

namespace assembler {
struct assembly {
  /**
   * Assembles area objects from closed ways or multipolygon relations
   * and their members.
   */
  // HIER STEHEN GEBLIEBEN
  bool create_area_from_ways(std::vector<int>& out_buffer,
                             const osm::Way& way) {
    //     osm::AreaBuilder builder{out_buffer};
    //     builder.initialize_from_object(way);

    //     const bool area_okay = create_rings();
    //     if (area_okay || stats_.create_empty_areas) {
    //       builder.add_item(way.tags());
    //     }
    //     if (area_okay) {
    //       add_rings_to_area(builder);
    //     }

    //     return area_okay || stats_.create_empty_areas;
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
  // hier fängts an
  bool assembling_area_from_way(const osm::Way& way,
                                std::vector<int>& out_buffer,
                                bool report_problems) {
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
        reporter_.report_duplicate_node(way.nodes().front().ref(),
                                        way.nodes().back().ref(),
                                        way.nodes().front().location());
      }
    }

    // SegmentList needed
    ++stats_.from_ways;
    stats_.invalid_locations = segment_list_.extract_segments_from_way(
        &reporter_, stats_.duplicate_nodes, way);
    // if (!config().ignore_invalid_locations && stats_.invalid_locations > 0) {
    if (stats_.invalid_locations > 0) {
      return false;
    }

    if (report_problems) {
      std::cerr << "\nAssembling way " << way.id << " containing "
                << segment_list_.size() << " nodes\n";
    }
    // Now create the Area object and add the attributes and tags
    // from the way.
    const bool okay = create_area_from_ways(out_buffer, way);
    if (report_problems) {
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
  bool assembling_area_from_relation(
      const osm::Relation<Members>& relation,
      const std::vector<const osm::Way*>& members,
      std::vector<int>& out_buffer,
      bool report_problems) {
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
        &reporter_, stats_.duplicate_nodes, stats_.duplicate_ways, relation,
        members);
    // if (!config().ignore_invalid_locations && stats_.invalid_locations > 0) {
    if (stats_.invalid_locations > 0) {
      return false;
    }
    stats_.member_ways = members.size();

    if (stats_.member_ways == 1) {
      ++stats_.single_way_in_mp_relation;
    }

    if (report_problems) {
      std::cerr << "\nAssembling relation " << relation.id() << "containing "
                << members.size() << " way members with "
                << segment_list().size() << " nodes\n";
    }

    // Now create the Area object and add the attributes and tags
    // from the relation.
    const bool okay = create_area(out_buffer, relation, members);
    // if (okay) {
    //   out_buffer.commit();
    // } else {
    //   out_buffer.rollback();
    // }

    return okay;
  }

  SegmentList segment_list_ = SegmentList{};  // init?
  area_stats stats_ = area_stats{};
  problem_reporter reporter_ = problem_reporter{};
};  // class Assembly

}  // namespace assembler
