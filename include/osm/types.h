#pragma once

#include <cstdint>
#include <vector>

#include "osm/varint.h"

namespace osm {

enum member_type : std::uint32_t { kNode, kWay, kRelation };
static const int undefined_coordinate = 2147483647;
static const int coordinate_precision = 10000000;
using object_id_type = std::int64_t;

// use geo? or change to double?
struct Location {
  std::int32_t x_;
  std::int32_t y_;
  Location() : x_(undefined_coordinate), y_(undefined_coordinate) {}
  constexpr Location(const std::int32_t x, const std::int32_t y) noexcept
      : x_(x), y_(y) {}
  constexpr std::int32_t x() const noexcept { return x_; }
  constexpr std::int32_t y() const noexcept { return y_; }
  Location& set_x(const std::int32_t x) noexcept {
    x_ = x;
    return *this;
  }
  Location& set_y(const std::int32_t y) noexcept {
    y_ = y;
    return *this;
  }
  constexpr bool valid() const noexcept {
    return x_ >= -180 * coordinate_precision &&
           x_ <= 180 * coordinate_precision &&
           y_ >= -90 * coordinate_precision && y_ <= 90 * coordinate_precision;
  }
  inline bool operator==(const Location& other_location) const {
    return x_ == other_location.x() && y_ == other_location.y();
  }
  inline bool operator!=(const Location& other_location) const {
    return !(*this == other_location);
  }
  inline bool operator<(const Location& other_location) const {
    return (x_ == other_location.x() && y_ < other_location.y()) ||
           x_ < other_location.x();
  }
  inline bool operator>(const Location& other_location) const {
    return (other_location.x() == x_ && other_location.y() < y_) ||
           other_location.x() < x_;
  }
  bool is_there() const {
    return x_ != undefined_coordinate && y_ != undefined_coordinate;
  }
};

struct Node {
  object_id_type id_;
  osm::Location node_loc_;
  osm::Location location() const noexcept { return node_loc_; }
  object_id_type id() const noexcept { return id_; }
  Node& set_location(const osm::Location& location) noexcept {
    node_loc_ = location;
    return *this;
  }
};

struct NodeRef {
  object_id_type ref_id;
  osm::Location loc;
  constexpr object_id_type ref() const { return ref_id; }
  const osm::Location& location() const noexcept { return loc; }
  osm::Location& location() noexcept { return loc; }
  void set_location(osm::Location l) { loc = l; }
};

struct Way {
  object_id_type id;
  std::vector<NodeRef> node_refs;
  std::vector<NodeRef> nodes() const { return node_refs; }
  bool ends_have_same_id() const noexcept {
    // assert(!nodes().empty());
    return nodes().front().ref() == nodes().back().ref();
  }
};

// NICHT SICHER
template <typename Members>
struct Relation {
  object_id_type id;
  Members members_;
  constexpr Members members() const { return members_; }
};

}  // namespace osm