#include <chrono>
#include <ranges>
#include <string_view>

#include "fmt/ranges.h"

#include "gtest/gtest.h"

#include "utl/progress_tracker.h"

#include "osm/assembler/assembler_types.h"
#include "osm/decoder.h"
#include "osm/filing/tmp_file.h"
#include "osm/hnidx/hybrid_node_index.h"
#include "osm/inflate.h"
#include "osm/memory.h"
#include "osm/mp_manager.h"
#include "osm/osm.h"
#include "osm/parallel.h"

TEST(relation_tests, relation_areas) {
  auto r = osm::raw_reader{
      .file_ = cista::mmap{"/home/tmir/OSM/monaco-260324.osm.pbf",
                           cista::mmap::protection::READ}};

  auto bars = utl::global_progress_bars{false};
  auto pt = utl::activate_progress_tracker("parse");
  pt->in_high(r.rest_.size());

  // auto vec_mtx = std::mutex{};
  // auto mp_vec = std::vector<osm::multi_polygon>{};
  // auto mp = osm::multi_polygon{};
  std::atomic_uint64_t relations_count2 = 0;
  std::atomic_uint64_t relations_count = 0;
  std::atomic_uint64_t ways_count2 = 0;
  std::atomic_bool first = true;
  std::atomic_uint64_t worked_count = 0;

  std::atomic<size_t> ways_processed{0};
  size_t total_ways = 0;
  std::mutex rel_mutex;
  std::condition_variable rel_cv;
  bool ways_done = false;
  osm::PolygonManager mp_manager(false);

  auto tmp_dname = std::filesystem::temp_directory_path();
  auto const node_idx_file = tiles::tmp_file{
      (std::filesystem::path{tmp_dname} / "idx.bin").generic_string()};
  auto const node_dat_file = tiles::tmp_file{
      (std::filesystem::path{tmp_dname} / "dat.bin").generic_string()};
  tiles::hybrid_node_idx node_idx{node_idx_file.fileno(),
                                  node_dat_file.fileno()};
  tiles::hybrid_node_idx_builder node_idx_builder{node_idx};

  //   PASS 1: nodes & ways
  osm::decode_primitive_parallel(
      r, node_idx_builder, true, true, true,
      [&](std::int64_t const id, geo::latlng const& pos, auto&& tags) {},
      [&](std::int64_t, auto&&, auto&&) { total_ways++; },
      [&](std::int64_t const id, auto&& members, auto&& tags) {
        relations_count++;
        mp_manager.save_ways_of_relation(id, members, tags);
      },
      pt);

  mp_manager.reserve_way_map(total_ways);
  std::cout << " \t In the middle: " << std::endl;

  r.reset_reader();
  // PASS 2: areas
  osm::decode_primitive_parallel(
      r, node_idx_builder, false, true, true,
      [&](std::int64_t, geo::latlng const&, auto&&) {},
      [&](std::int64_t const id, auto&& refs, auto&& tags) {
        ways_count2++;
        std::vector<osm::NodeRef> way_node_refs;
        std::int64_t acc = 0;
        for (auto r : refs) {
          acc += r;
          way_node_refs.emplace_back(acc);
        }
        osm::Way tempway{id, way_node_refs};
        tiles::update_locations_of_way(node_idx, tempway);
        auto area_result = mp_manager.save_ways(tempway, tags);
        if (area_result.has_value()) {
          // not set in this test
        }
        if (++ways_processed == total_ways) {
          std::lock_guard lock(rel_mutex);
          ways_done = true;
          rel_cv.notify_all();
        }
      },
      [&](std::int64_t const id, auto&& members, auto&& tags) {
        std::unique_lock lock(rel_mutex);
        rel_cv.wait(lock, [&] { return ways_done; });
        lock.unlock();
        relations_count2++;
        // std::optional<assembler::polygon_area> a =
        assembler::polygon_area a =
            mp_manager.assemble_area(id, members, tags, relations_count);
        if (a.valid) {
          worked_count++;
          //  do something with final area
          // std::cout << "AREA BUILT! area id: " << a.relation_id
          //          << " num outer;inner(vec size) "
          //          << a.get_all_outers().size() << ";"
          //          << a.get_all_inners_as_vec(0).size() << std::endl;
        }
      },
      pt);

  std::cout << " \t At the end: " << std::endl;
  std::cout << " \t area count " << worked_count << "/" << relations_count2
            << " (realtions2) \n \t ways_count: " << ways_count2 << std::endl;
}