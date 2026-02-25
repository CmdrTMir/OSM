#include <ranges>
#include <string_view>

#include "fmt/ranges.h"

#include "gtest/gtest.h"

#include "utl/progress_tracker.h"

#include "osm/decoder.h"
#include "osm/filing/tmp_file.h"
#include "osm/hnidx/hybrid_node_index.h"
#include "osm/inflate.h"
#include "osm/memory.h"
#include "osm/mp_manager.h"
#include "osm/osm.h"
#include "osm/parallel.h"

#include "boost/fiber/all.hpp"

namespace bf = boost::fibers;
/*
TEST(osm, varint) {
  auto buf = std::array<char, protozero::max_varint_length * 10U>{};
  auto const n1 =
      protozero::write_varint(buf.data(), protozero::encode_zigzag64(123));
  auto const n2 =
      protozero::write_varint(buf.data() + n1, protozero::encode_zigzag64(456));
  auto const n3 = protozero::write_varint(buf.data() + n1 + n2,
                                          protozero::encode_zigzag64(789));
  auto const n4 = protozero::write_varint(buf.data() + n1 + n2 + n3, 0);

  EXPECT_EQ(0, buf[n1 + n2 + n3]);

  auto const size = static_cast<std::size_t>(n1 + n2 + n3);
  auto x = static_cast<char const*>(buf.data());
  EXPECT_EQ(123,
            protozero::decode_zigzag64(protozero::decode_varint(&x, x + size)));

  auto const empty = osm::varint<std::int64_t>{std::string_view{}};
  EXPECT_EQ(empty.begin(), empty.end());

  auto const v = osm::varint<std::int64_t>{
      std::string_view{buf.data(), static_cast<std::size_t>(n1 + n2 + n3)}};
  auto it = v.begin();
  EXPECT_NE(it, v.end());
  EXPECT_EQ(123, *it);

  ++it;
  EXPECT_NE(it, v.end());
  EXPECT_EQ(456, *it);

  ++it;
  EXPECT_NE(it, v.end());
  EXPECT_EQ(789, *it);

  ++it;
  EXPECT_EQ(it, v.end());
}

TEST(a, b) {
  auto r = osm::raw_reader{
      .file_ = cista::mmap{"/home/tmir/OSM/berlin-251113.osm.pbf",
                           cista::mmap::protection::READ}};

  auto bars = utl::global_progress_bars{false};
  auto pt = utl::activate_progress_tracker("parse");
  pt->in_high(r.rest_.size());

  auto const n_threads = std::thread::hardware_concurrency();

  auto n_nodes = std::atomic_uint64_t{0U};
  auto n_ways = std::atomic_uint64_t{0U};
  auto n_rels = std::atomic_uint64_t{0U};

  auto print_mtx = std::mutex{};

  auto pool = std::vector<std::thread>{n_threads};
  auto fin = std::atomic_bool{false};
  auto fin_cv = bf::condition_variable_any{};
  auto fin_mutex = std::mutex{};

  auto ch = bf::buffered_channel<osm::buf>{64U};
  for (auto i = 0U; i != n_threads; ++i) {
    bf::fiber([&]() {
      auto decompressor = osm::inflate{};
      auto out = std::string{};
      auto strings = std::vector<std::string_view>{};

      for (auto const& b : ch) {
        out.resize(b.raw_size_);
        decompressor.decompress(b.compressed_, out);

        osm::decode_primitive(
            out, strings, true, true, true,
            [&](std::int64_t const id, geo::latlng const& pos, auto&& tags) {
              ++n_nodes;
            },
            [&](std::int64_t const id, auto&& refs, auto&& tags) { ++n_ways; },
            [&](std::int64_t const id, auto&& members, auto&& tags) {
              ++n_rels;
            });
      }
    }).detach();
  }

  for (auto& t : pool) {
    t = std::thread{[&]() {
      bf::use_scheduling_algorithm<bf::algo::work_stealing>(n_threads + 1U);
      auto l = std::unique_lock{fin_mutex};
      fin_cv.wait(l, [&]() { return fin.load(); });
    }};
  }

  bf::use_scheduling_algorithm<bf::algo::work_stealing>(n_threads + 1U);

  auto buf = std::optional<osm::buf>{};
  while ((buf = r.read()).has_value()) {
    ch.push(*buf);
    pt->update(r.file_.size() - r.rest_.size());
  }
  ch.close();
  fin.store(true);
  fin_cv.notify_all();

  for (auto& t : pool) {
    t.join();
  }

  std::cout << "number of nodes: " << n_nodes << "\n";  // 7674356
  std::cout << "number of ways: " << n_ways << "\n";  // 1276568
  std::cout << "number of relations: " << n_rels << "\n";  // 17929

  const ium::MemoryUsage memory;
  std::cout << "\nMemory used: " << memory.peak() << " MBytes\n";
}
*/
TEST(c, d) {
  auto r = osm::raw_reader{
      .file_ = cista::mmap{"/home/tmir/OSM/berlin-251113.osm.pbf",
                           cista::mmap::protection::READ}};

  auto bars = utl::global_progress_bars{false};
  auto pt = utl::activate_progress_tracker("parse");
  pt->in_high(r.rest_.size());

  auto vec_mtx = std::mutex{};
  auto mp_vec = std::vector<osm::multi_polygon>{};
  // auto mp = osm::multi_polygon{};
  std::atomic_uint64_t relations_count = 0;
  std::atomic_uint64_t ways_count = 0;
  std::atomic_uint64_t relations_count2 = 0;
  std::atomic_uint64_t ways_count2 = 0;

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
      r, node_idx_builder, true, false, true,
      [&](std::int64_t const id, geo::latlng const& pos, auto&& tags) {},
      [&](std::int64_t, auto&&, auto&&) { ways_count++; },
      [&](std::int64_t const id, auto&& members, auto&& tags) {
        relations_count++;
        auto mp = osm::multi_polygon{};
        osm::save_ways_of_relation(mp, id, members, tags);
        std::lock_guard<std::mutex> lock(vec_mtx);
        mp_vec.emplace_back(mp);
      },
      pt);

  std::cout << " \t In the middle: " << std::endl;
  std::cout << "vec_size: " << mp_vec.size()
            << " relations_count: " << relations_count
            << " ways_count (0): " << ways_count << std::endl;

  r.reset_reader();
  // PASS 2: areas
  osm::decode_primitive_parallel(
      r, node_idx_builder, false, true, true,
      [&](std::int64_t, geo::latlng const&, auto&&) {},
      [&](std::int64_t const id, auto&& refs, auto&& tags) {
        ways_count2++;
        // TODO save nodes of ways => update_locations (wo noch ein Fehler
        // ist)
      },
      [&](std::int64_t const id, auto&& members, auto&& tags) {
        relations_count2++;
        // auto mp = osm::multi_polygons{};
        // auto a = osm::assemble_area(mp, id, members, tags);
        //  TODO do something with final area
        //  a.outer_rings()
      },
      pt);

  std::cout << " \t At the end: " << std::endl;
  std::cout << "relations_count2: " << relations_count2
            << " ways_count2 (1276568): " << ways_count2 << std::endl;
}
