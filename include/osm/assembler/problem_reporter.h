#include <iostream>

#include "osm/types.h"

namespace assembler {

struct problem_reporter {

  void header(const char* msg) {
    *out_stream_ << "DATA PROBLEM: " << msg << " ON ";
  }

  void report_duplicate_node(osm::object_id_type node_id1,
                             osm::object_id_type node_id2,
                             osm::Location location) {
    header("duplicate node");
    *out_stream_ << "node_id1=" << node_id1 << " node_id2=" << node_id2
                 << " location=" << location.x() << "," << location.y() << "\n";
  }

  void report_touching_ring(osm::object_id_type node_id,
                            osm::Location location) {
    header("touching ring");
    *out_stream_ << "node_id=" << node_id << " location=" << location.x() << ","
                 << location.y() << "\n";
  }

  void report_intersection(osm::object_id_type way1_id,
                           osm::Location way1_seg_start,
                           osm::Location way1_seg_end,
                           osm::object_id_type way2_id,
                           osm::Location way2_seg_start,
                           osm::Location way2_seg_end,
                           osm::Location intersection) {
    header("intersection");
    *out_stream_ << "way1_id=" << way1_id
                 << " way1_seg_start=" << way1_seg_start.x() << ","
                 << way1_seg_start.y() << " way1_seg_end=" << way1_seg_end.x()
                 << "," << way1_seg_end.y() << " way2_id=" << way2_id
                 << " way2_seg_start=" << way2_seg_start.x() << ","
                 << way2_seg_start.y() << " way2_seg_end=" << way2_seg_end.x()
                 << "," << way2_seg_end.y()
                 << " intersection=" << intersection.x() << ","
                 << intersection.y() << "\n";
  }

  void report_duplicate_segment(const osm::NodeRef& nr1,
                                const osm::NodeRef& nr2) {
    header("duplicate segment");
    *out_stream_ << "node_id1=" << nr1.ref()
                 << " location1=" << nr1.location().x() << ","
                 << nr1.location().y() << " node_id2=" << nr2.ref()
                 << " location2=" << nr2.location().x() << ","
                 << nr2.location().y() << "\n";
  }

  void report_overlapping_segment(const osm::NodeRef& nr1,
                                  const osm::NodeRef& nr2) {
    header("overlapping segment");
    *out_stream_ << "node_id1=" << nr1.ref()
                 << " location1=" << nr1.location().x() << ","
                 << nr1.location().y() << " node_id2=" << nr2.ref()
                 << " location2=" << nr2.location().x() << ","
                 << nr2.location().y() << "\n";
  }

  void report_ring_not_closed(const osm::NodeRef& nr, const osm::Way* way) {
    header("ring not closed");
    *out_stream_ << "node_id=" << nr.ref() << " location=" << nr.location().x()
                 << "," << nr.location().y();
    if (way) {
      *out_stream_ << " on way " << way->id;
    }
    *out_stream_ << "\n";
  }

  void report_role_should_be_outer(osm::object_id_type way_id,
                                   osm::Location seg_start,
                                   osm::Location seg_end) {
    header("role should be outer");
    *out_stream_ << "way_id=" << way_id << " seg_start=" << seg_start.x() << ","
                 << seg_start.y() << " seg_end=" << seg_end.x() << ","
                 << seg_end.y() << "\n";
  }

  void report_role_should_be_inner(osm::object_id_type way_id,
                                   osm::Location seg_start,
                                   osm::Location seg_end) {
    header("role should be inner");
    *out_stream_ << "way_id=" << way_id << " seg_start=" << seg_start.x() << ","
                 << seg_start.y() << " seg_end=" << seg_end.x() << ","
                 << seg_end.y() << "\n";
  }

  void report_way_in_multiple_rings(const osm::Way& way) {
    header("way in multiple rings");
    *out_stream_ << "way_id=" << way.id << '\n';
  }

  void report_inner_with_same_tags(const osm::Way& way) {
    header("inner way with same tags as relation or outer");
    *out_stream_ << "way_id=" << way.id << '\n';
  }

  void report_invalid_location(osm::object_id_type way_id,
                               osm::object_id_type node_id) {
    header("invalid location");
    *out_stream_ << "way_id=" << way_id << " node_id=" << node_id << '\n';
  }

  void report_duplicate_way(const osm::Way& way) {
    header("duplicate way");
    *out_stream_ << "way_id=" << way.id << '\n';
  }

  std::ostream* out_stream_;
};
}  // namespace assembler