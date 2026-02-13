#pragma once

#include <cstdint>
#include <vector>

namespace osm {

static const int coordinate_precision = 10000000;
using object_id_type = std::int64_t;

// use geo?
struct Location {
  std::int32_t x_;  // change to double?
  std::int32_t y_;
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
  object_id_type id;
  osm::Location loc;
  constexpr object_id_type ref() const { return id; }
  constexpr osm::Location location() const { return loc; }
  void set_location(osm::Location l) { loc = l; }
};

struct Way {
  std::vector<NodeRef> node_refs;

  std::vector<NodeRef> nodes() const { return node_refs; }
};

}  // namespace osm