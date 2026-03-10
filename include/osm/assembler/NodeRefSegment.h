#pragma once

#include <cassert>

#include "ProtoRing.h"
#include "osm/types.h"
#include "vec.h"

namespace assembler {

enum role_type : uint8_t { unknown = 0, outer = 1, inner = 2, empty = 3 };
/**
 * This helper for the Assembler models a segment,
 * the connection between two nodes.
 *
 * Internally segments have their smaller coordinate at the
 * beginning of the segment. Smaller, in this case, means smaller
 * x coordinate, and, if they are the same, smaller y coordinate.
 */
struct NodeRefSegment {
  osm::NodeRef first_noderef_;
  osm::NodeRef second_noderef_;
  const osm::Way* way_ = nullptr;
  // The ring this segment is part of. Initially nullptr, this
  // will be filled in once we know which ring the segment is in.
  ProtoRing* ring_ = nullptr;
  role_type role_ = role_type::unknown;
  // Nodes have to be reversed to get the intended order.
  bool reverse_ = false;
  // We found the right direction for this segment in the ring.
  // (This depends on whether it is an inner or outer ring.)
  bool direction_done_ = false;

  NodeRefSegment() noexcept = default;
  NodeRefSegment(const osm::NodeRef& nr1,
                 const osm::NodeRef& nr2,
                 role_type role,
                 const osm::Way* way) noexcept
      : first_noderef_(nr1.location().smaller_than(nr2.location()) ? nr1 : nr2),
        second_noderef_(nr1.location().smaller_than(nr2.location()) ? nr2
                                                                    : nr1),
        way_(way),
        role_(role) {}

  ProtoRing* ring() const noexcept { return ring_; }
  bool is_done() const noexcept { return ring_ != nullptr; }
  void set_ring(ProtoRing* ring) noexcept {
    assert(ring);
    ring_ = ring;
  }

  bool is_reverse() const noexcept { return reverse_; }
  void reverse() noexcept { reverse_ = !reverse_; }
  bool is_direction_done() const noexcept { return direction_done_; }
  void mark_direction_done() noexcept { direction_done_ = true; }
  void mark_direction_not_done() noexcept { direction_done_ = false; }

  const osm::NodeRef& first() const noexcept { return first_noderef_; }
  const osm::NodeRef& second() const noexcept { return second_noderef_; }
  const osm::NodeRef& start() const noexcept {
    return reverse_ ? second_noderef_ : first_noderef_;
  }
  const osm::NodeRef& stop() const noexcept {
    return reverse_ ? first_noderef_ : second_noderef_;
  }

  bool role_outer() const noexcept { return role_ == role_type::outer; }
  bool role_inner() const noexcept { return role_ == role_type::inner; }
  bool role_empty() const noexcept { return role_ == role_type::empty; }
  const char* role_name() const noexcept {
    static const std::array<const char*, 4> names = {
        {"unknown", "outer", "inner", "empty"}};
    return names[static_cast<int>(role_)];
  }
  const osm::Way* way() const noexcept { return way_; }

  /**
   * The "determinant" of this segment. Used for calculating
   * the winding order of a ring.
   */
  int64_t det() const noexcept {
    const vec a{start()};
    const vec b{stop()};
    return a * b;
  }

};  // struct NodeRefSegment

inline bool operator==(const NodeRefSegment& lhs,
                       const NodeRefSegment& rhs) noexcept {
  return lhs.first().location().equal_to(rhs.first().location()) &&
         lhs.second().location().equal_to(rhs.second().location());
}
inline bool operator!=(const NodeRefSegment& lhs,
                       const NodeRefSegment& rhs) noexcept {
  return !(lhs == rhs);
}
/**
 * A NodeRefSegment is "smaller" if the first point is to the
 * left and down of the first point of the second segment.
 * If both first points are the same, the segment with the higher
 * slope comes first. If the slope is the same, the shorter
 * segment comes first.
 */
inline bool operator<(const NodeRefSegment& lhs,
                      const NodeRefSegment& rhs) noexcept {
  if (lhs.first().location().equal_to(rhs.first().location())) {
    const vec p0{lhs.first().location()};
    const vec p1{lhs.second().location()};
    const vec q0{rhs.first().location()};
    const vec q1{rhs.second().location()};
    const vec p = p1 - p0;
    const vec q = q1 - q0;
    if (p.x == 0 && q.x == 0) {
      return p.y < q.y;
    }
    const auto a = p.y * q.x;
    const auto b = q.y * p.x;
    if (a == b) {
      return p.x < q.x;
    }
    return a > b;
  }
  return lhs.first().location().smaller_than(rhs.first().location());
}
}  // namespace assembler