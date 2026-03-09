#pragma once

#include <cassert>
#include <iostream>
#include <set>
#include <vector>

#include "NodeRefSegment.h"

namespace assembler {
struct ProtoRing {
  std::vector<NodeRefSegment*> segments_;
  std::vector<ProtoRing*> inner_;
  NodeRefSegment* min_segment_;
  ProtoRing* outer_ring_ = nullptr;
  std::int64_t sum_ = 0;

  explicit ProtoRing(NodeRefSegment* segment) noexcept : min_segment_(segment) {
    add_segment_back(segment);
  }

  void add_segment_back(NodeRefSegment* segment) {
    assert(segment);
    if (*segment < *min_segment_) {
      min_segment_ = segment;
    }
    segments_.push_back(segment);
    segment->set_ring(this);
    sum_ += segment->det();
  }

  NodeRefSegment* min_segment() const noexcept { return min_segment_; }
  ProtoRing* outer_ring() const noexcept { return outer_ring_; }
  const std::vector<ProtoRing*>& inner_rings() const noexcept { return inner_; }

  void set_outer_ring(ProtoRing* outer_ring) noexcept {
    assert(outer_ring);
    assert(inner_.empty());
    outer_ring_ = outer_ring;
  }

  void add_inner_ring(ProtoRing* ring) {
    assert(ring);
    assert(!outer_ring_);
    inner_.push_back(ring);
  }

  bool is_outer() const noexcept { return !outer_ring_; }

  const std::vector<NodeRefSegment*>& segments() const noexcept {
    return segments_;
  }
  const osm::NodeRef& get_node_ref_start() const noexcept {
    return segments_.front()->start();
  }
  const osm::NodeRef& get_node_ref_stop() const noexcept {
    return segments_.back()->stop();
  }

  bool closed() const noexcept {
    return get_node_ref_start().location().equal_to(
        get_node_ref_stop().location());
  }

  void reverse() {
    std::for_each(segments_.begin(), segments_.end(),
                  [](NodeRefSegment* segment) { segment->reverse(); });
    std::reverse(segments_.begin(), segments_.end());
    sum_ = -sum_;
  }

  void mark_direction_done() {
    std::for_each(
        segments_.begin(), segments_.end(),
        [](NodeRefSegment* segment) { segment->mark_direction_done(); });
  }

  bool is_cw() const noexcept { return sum_ <= 0; }
  int64_t sum() const noexcept { return sum_; }

  void fix_direction() noexcept {
    if (is_cw() == is_outer()) {
      reverse();
    }
  }

  void reset() {
    inner_.clear();
    outer_ring_ = nullptr;
    std::for_each(
        segments_.begin(), segments_.end(),
        [](NodeRefSegment* segment) { segment->mark_direction_not_done(); });
  }

  void get_ways(std::set<const osm::Way*>& ways) const {
    for (const auto& segment : segments_) {
      ways.insert(segment->way());
    }
  }

  void join_forward(ProtoRing& other) {
    segments_.reserve(segments_.size() + other.segments_.size());
    for (NodeRefSegment* segment : other.segments_) {
      add_segment_back(segment);
    }
  }

  void join_backward(ProtoRing& other) {
    segments_.reserve(segments_.size() + other.segments_.size());
    for (auto it = other.segments_.rbegin(); it != other.segments_.rend();
         ++it) {
      (*it)->reverse();
      add_segment_back(*it);
    }
  }

  void print(std::ostream& out) const {
    out << "Ring [";
    if (!segments_.empty()) {
      out << segments_.front()->start().ref();
    }
    for (const auto& segment : segments_) {
      out << ',' << segment->stop().ref();
    }
    out << "]-" << (is_outer() ? "OUTER" : "INNER");
  }

};  // class ProtoRing
}  // namespace assembler
