#include <cassert>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <array>
#include <iostream>
#include <iterator>
#include <numeric>
#include <unordered_set>
#include <utility>
#include <vector>

#include "assembler/problem_reporter.h"
#include "assembler/vec.h"
#include "osm/types.h"

namespace assembler {

enum class role_type : uint8_t { unknown = 0, outer = 1, inner = 2, empty = 3 };
/**
 * This helper class for the Assembler class models a segment,
 * the connection between two nodes.
 *
 * Internally segments have their smaller coordinate at the
 * beginning of the segment. Smaller, in this case, means smaller
 * x coordinate, and, if they are the same, smaller y coordinate.
 */
struct NodeRefSegment {

  // First node in order described above.
  osm::NodeRef first_noderef_;
  // Second node in order described above.
  osm::NodeRef second_noderef_;
  // Way this segment was from.
  const osm::Way* way_ = nullptr;

  // The ring this segment is part of. Initially nullptr, this
  // will be filled in once we know which ring the segment is in.
  // ProtoRing* m_ring = nullptr;

  // The role of this segment from the member role.
  role_type m_role = role_type::unknown;

  // Nodes have to be reversed to get the intended order.
  bool reverse_ = false;

  // We found the right direction for this segment in the ring.
  // (This depends on whether it is an inner or outer ring.)
  bool m_direction_done = false;

public:
  NodeRefSegment() noexcept = default;
  NodeRefSegment(const osm::NodeRef& nr1,
                 const osm::NodeRef& nr2,
                 role_type role,
                 const osm::Way* way) noexcept
      : first_noderef_(nr1.location().smaller_than(nr2.location()) ? nr1 : nr2),
        second_noderef_(nr1.location().smaller_than(nr2.location()) ? nr2
                                                                    : nr1),
        way_(way),
        m_role(role) {}

  /**
   * The ring this segment is a part of. nullptr if we don't
   * have the ring yet.
   */
  // ProtoRing* ring() const noexcept { return m_ring; }

  /**
   * Returns true if the segment has already been placed in a
   * ring.
   */
  //   bool is_done() const noexcept { return m_ring != nullptr; }

  //   void set_ring(ProtoRing* ring) noexcept {
  //     assert(ring);
  //     m_ring = ring;
  //   }

  bool is_reverse() const noexcept { return reverse_; }
  void reverse() noexcept { reverse_ = !reverse_; }
  bool is_direction_done() const noexcept { return m_direction_done; }
  void mark_direction_done() noexcept { m_direction_done = true; }
  void mark_direction_not_done() noexcept { m_direction_done = false; }

  const osm::NodeRef& first() const noexcept { return first_noderef_; }
  const osm::NodeRef& second() const noexcept { return second_noderef_; }
  const osm::NodeRef& start() const noexcept {
    return reverse_ ? second_noderef_ : first_noderef_;
  }
  const osm::NodeRef& stop() const noexcept {
    return reverse_ ? first_noderef_ : second_noderef_;
  }

  bool role_outer() const noexcept { return m_role == role_type::outer; }
  bool role_inner() const noexcept { return m_role == role_type::inner; }
  bool role_empty() const noexcept { return m_role == role_type::empty; }
  //   const char* role_name() const noexcept {
  //     static const std::array<const char*, 4> names = {
  //         {"unknown", "outer", "inner", "empty"}};
  //     return names[static_cast<int>(m_role)];
  //   }
  const osm::Way* way() const noexcept { return way_; }

  /**
   * The "determinant" of this segment. Used for calculating
   * the winding order of a ring.
   */
  //   int64_t det() const noexcept {
  //     const vec a{start()};
  //     const vec b{stop()};
  //     return a * b;
  //   }

};  // struct NodeRefSegment

////----
// HELPER FUNCTIONS
////----
/**
 * Iterate over all relation members and the vector of ways at the
 * same time and call given function with the relation member and
 * way as parameter. This takes into account that there might be
 * non-way members in the relation.
 */
template <typename TFunc, typename Members>
inline void for_each_member(const osm::Relation<Members>& relation,
                            const std::vector<const osm::Way*>& ways,
                            TFunc&& func) {
  auto way_it = ways.cbegin();
  for (const auto& member : relation.members()) {
    if (member.type() == osmium::item_type::way) {
      assert(way_it != ways.cend());
      std::forward<TFunc>(func)(member, **way_it);
      ++way_it;
    }
  }
}

inline bool outside_x_range(const NodeRefSegment& s1,
                            const NodeRefSegment& s2) noexcept {
  return s1.first().location().x() > s2.second().location().x();
}

inline bool y_range_overlap(const NodeRefSegment& s1,
                            const NodeRefSegment& s2) noexcept {
  const std::pair<int32_t, int32_t> m1 =
      std::minmax(s1.first().location().y(), s1.second().location().y());
  const std::pair<int32_t, int32_t> m2 =
      std::minmax(s2.first().location().y(), s2.second().location().y());
  return !(m1.first > m2.second || m2.first > m1.second);
}

/**
 * Calculate the intersection between two NodeRefSegments. The
 * result is returned as a Location. Note that because the Location
 * uses integers with limited precision internally, the result
 * might be slightly different than the numerically correct
 * location.
 *
 * This function uses integer arithmetic as much as possible and
 * will not work if the segments are longer than about half the
 * planet. This shouldn't happen with real data, so it isn't a big
 * problem.
 *
 * If the segments touch in one or both of their endpoints, it
 * doesn't count as an intersection.
 *
 * If the segments intersect not in a single point but in multiple
 * points, ie if they are collinear and overlap, the smallest
 * of the endpoints that is in the overlapping section is returned.
 *
 * @returns Undefined osmium::Location if there is no intersection
 *          or a defined Location if the segments intersect.
 */
inline osm::Location calculate_intersection(const NodeRefSegment& s1,
                                            const NodeRefSegment& s2) noexcept {
  // See
  // https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
  // for some hints about how the algorithm works.
  const vec p0{s1.first()};
  const vec p1{s1.second()};
  const vec q0{s2.first()};
  const vec q1{s2.second()};

  if ((p0 == q0 && p1 == q1) || (p0 == q1 && p1 == q0)) {
    // segments are the same
    return osm::Location{};
  }

  const vec pd = p1 - p0;
  const std::int64_t d = pd * (q1 - q0);

  if (d != 0) {
    // segments are not collinear

    if (p0 == q0 || p0 == q1 || p1 == q0 || p1 == q1) {
      // touching at an end point
      return osm::Location{};
    }

    // intersection in a point
    const std::int64_t na =
        ((q1.x - q0.x) * (p0.y - q0.y)) - ((q1.y - q0.y) * (p0.x - q0.x));

    const std::int64_t nb =
        ((p1.x - p0.x) * (p0.y - q0.y)) - ((p1.y - p0.y) * (p0.x - q0.x));

    if ((d > 0 && na >= 0 && na <= d && nb >= 0 && nb <= d) ||
        (d < 0 && na <= 0 && na >= d && nb <= 0 && nb >= d)) {
      const double ua = static_cast<double>(na) / static_cast<double>(d);
      const vec i = p0 + ua * (p1 - p0);
      return osm::Location{static_cast<int32_t>(i.x),
                           static_cast<int32_t>(i.y)};
    }

    return osm::Location{};
  }

  // segments are collinear
  if (pd * (q0 - p0) == 0) {
    // segments are on the same line

    struct seg_loc {
      int segment;
      osm::Location location;
    };

    std::array<seg_loc, 4UL> sl = {{
        {0, s1.first().location()},
        {0, s1.second().location()},
        {1, s2.first().location()},
        {1, s2.second().location()},
    }};

    std::sort(sl.begin(), sl.end(), [](const seg_loc& lhs, const seg_loc& rhs) {
      return lhs.location.smaller_than(rhs.location);
    });

    if (sl[1].location.equal_to(sl[2].location)) {
      return osm::Location{};
    }

    if (sl[0].segment != sl[1].segment) {
      if (sl[0].location.equal_to(sl[1].location)) {
        return sl[2].location;
      }
      return sl[1].location;
    }
  }

  return osm::Location{};
}

struct SegmentList {
  std::vector<NodeRefSegment> segments_;
  static role_type parse_role(const char* role) noexcept {
    if (role[0] == '\0') {
      return role_type::empty;
    }
    if (!std::strcmp(role, "outer")) {
      return role_type::outer;
    }
    if (!std::strcmp(role, "inner")) {
      return role_type::inner;
    }
    return role_type::unknown;
  }

  /**
   * Calculate the number of segments in all the ways together.
   */
  static std::size_t get_num_segments(
      const std::vector<const osm::Way*>& members) noexcept {
    return std::accumulate(members.cbegin(), members.cend(),
                           static_cast<std::size_t>(0),
                           [](std::size_t sum, const osm::Way* way) {
                             if (way->nodes().empty()) {
                               return sum;
                             }
                             return sum + way->nodes().size() - 1;
                           });
  }

  uint32_t extract_segments_from_way_impl(problem_reporter* problem_reporter,
                                          uint64_t& duplicate_nodes,
                                          const osm::Way& way,
                                          role_type role) {
    uint32_t invalid_locations = 0;

    auto previous_nr = osm::NodeRef{};
    for (const osm::NodeRef& nr : way.nodes()) {
      if (!nr.location().valid()) {
        ++invalid_locations;
        if (problem_reporter) {
          problem_reporter->report_invalid_location(way.id, nr.ref());
        }
        continue;
      }
      if (previous_nr.location().equal_to(nr.location())) {
        segments_.emplace_back(previous_nr, nr, role, &way);
      } else {
        ++duplicate_nodes;
        if (problem_reporter) {
          problem_reporter->report_duplicate_node(previous_nr.ref(), nr.ref(),
                                                  nr.location());
        }
      }

      previous_nr = nr;
    }

    return invalid_locations;
  }

public:
  // see if and when segments_ is initialised?
  //  SegmentList(const SegmentList&) = delete;
  //  SegmentList(SegmentList&&) = delete;
  //  SegmentList() { segments_ = {}; };

  SegmentList& operator=(const SegmentList&) = delete;
  SegmentList& operator=(SegmentList&&) = delete;

  ~SegmentList() noexcept = default;

  std::size_t size() const noexcept { return segments_.size(); }
  bool empty() const noexcept { return segments_.empty(); }
  NodeRefSegment& front() { return segments_.front(); }
  NodeRefSegment& back() { return segments_.back(); }

  const NodeRefSegment& operator[](std::size_t n) const noexcept {
    assert(n < segments_.size());
    return segments_[n];
  }

  NodeRefSegment& operator[](const std::size_t n) noexcept {
    assert(n < segments_.size());
    return segments_[n];
  }

  auto begin() noexcept { return segments_.begin(); }
  auto end() noexcept { return segments_.end(); }
  auto begin() const noexcept { return segments_.begin(); }
  auto end() const noexcept { return segments_.end(); }

  /// Sort the list of segments.
  void sort() { std::sort(segments_.begin(), segments_.end()); }

  /**
   * Extract segments from given way and add them to the list.
   *
   * Segments connecting two nodes with the same location (ie
   * same node or different nodes with same location) are
   * removed after reporting the duplicate node.
   */
  uint32_t extract_segments_from_way(problem_reporter* problem_reporter,
                                     uint64_t& duplicate_nodes,
                                     const osm::Way& way) {
    if (way.nodes().empty()) {
      return 0;
    }
    segments_.reserve(way.nodes().size() - 1);
    return extract_segments_from_way_impl(problem_reporter, duplicate_nodes,
                                          way, role_type::outer);
  }

  /**
   * Extract all segments from all ways that make up this
   * multipolygon relation and add them to the list.
   */
  template <typename Members>
  uint32_t extract_segments_from_ways(
      problem_reporter* problem_reporter,
      uint64_t& duplicate_nodes,
      uint64_t& duplicate_ways,
      const osm::Relation<Members>& relation,
      const std::vector<const osm::Way*>& members) {
    // assert(relation.cmembers().size() >= members.size());

    const std::size_t num_segments = get_num_segments(members);
    // if (problem_reporter) {
    //   problem_reporter->set_nodes(num_segments);
    // }
    segments_.reserve(num_segments);

    std::unordered_set<osm::object_id_type> ids;
    ids.reserve(members.size());
    uint32_t invalid_locations = 0;
    for_each_member(
        relation, members,
        [&](const osmium::RelationMember& member, const osm::Way& way) {
          if (ids.count(way.id) == 0) {
            ids.insert(way.id);
            const auto role = parse_role(member.role());
            invalid_locations += extract_segments_from_way_impl(
                problem_reporter, duplicate_nodes, way, role);
          } else {
            ++duplicate_ways;
            if (problem_reporter) {
              problem_reporter->report_duplicate_way(way);
            }
          }
        });

    return invalid_locations;
  }

  /**
   * Find duplicate segments (ie same start and end point) in the
   * list and remove them. This will always remove pairs of the
   * same segment. So if there are three, for instance, two will
   * be removed and one will be left.
   */
  void erase_duplicate_segments(problem_reporter* problem_reporter,
                                uint64_t& duplicate_segments,
                                uint64_t& overlapping_segments) {
    while (true) {
      auto it = std::adjacent_find(segments_.begin(), segments_.end());
      if (it == segments_.end()) {
        break;
      }

      // Only count and report duplicate segments if they
      // belong to the same way or if they don't both have
      // the role "inner". Those cases are definitely wrong.
      // If the duplicate segments belong to different
      // "inner" ways, they could be touching inner rings
      // which are perfectly okay. Note that for this check
      // the role has to be correct in the member data.
      if (it->way() == std::next(it)->way() || !it->role_inner() ||
          !std::next(it)->role_inner()) {
        ++duplicate_segments;
        if (problem_reporter) {
          problem_reporter->report_duplicate_segment(it->first(), it->second());
        }
      }

      // if (it + 2 != segments_.end() && *it == *(it + 2)) {
      if (it + 2 != segments_.end() && it == (it + 2)) {
        ++overlapping_segments;
        if (problem_reporter) {
          problem_reporter->report_overlapping_segment(it->first(),
                                                       it->second());
        }
      }

      segments_.erase(it, it + 2);
    }
  }

  /**
   * Find intersection between segments.
   *
   * @param problem_reporter Any intersections found are
   *                         reported to this object.
   * @returns true if there are intersections.
   */
  uint32_t find_intersections(problem_reporter* problem_reporter) const {
    if (segments_.empty()) {
      return 0;
    }

    uint32_t found_intersections = 0;

    for (auto it1 = segments_.cbegin(); it1 != segments_.cend() - 1; ++it1) {
      const NodeRefSegment& s1 = *it1;
      for (auto it2 = it1 + 1; it2 != segments_.end(); ++it2) {
        const NodeRefSegment& s2 = *it2;
        // erase_duplicate_segments() should have made sure of that
        // assert(s1 != s2);

        if (outside_x_range(s2, s1)) {
          break;
        }

        if (y_range_overlap(s1, s2)) {
          const osm::Location intersection{calculate_intersection(s1, s2)};
          if (intersection.is_there()) {
            ++found_intersections;
            if (problem_reporter) {
              problem_reporter->report_intersection(
                  s1.way()->id, s1.first().location(), s1.second().location(),
                  s2.way()->id, s2.first().location(), s2.second().location(),
                  intersection);
            }
          }
        }
      }
    }

    return found_intersections;
  }

};  // struct SegmentList
}  // namespace assembler