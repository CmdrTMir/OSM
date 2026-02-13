#pragma once

#include <string_view>

#include "osm.h"
#include "osm/decoder.h"
#include "osm/inflate.h"
#include "osm/memory.h"

#include "utl/progress_tracker.h"

#include "boost/fiber/all.hpp"

namespace osm {

template <typename NodeFn, typename WayFn, typename RelFn>
void decode_primitive_parallel(osm::raw_reader& r,
                               bool const read_nodes,
                               bool const read_ways,
                               bool const read_relations,
                               NodeFn&& on_node,
                               WayFn&& on_way,
                               RelFn&& on_rel,
                               utl::progress_tracker_ptr pt) {
  namespace bf = boost::fibers;

  auto const n_threads = std::thread::hardware_concurrency();
  auto pool = std::vector<std::thread>{n_threads};
  auto await_fibers_mtx = std::mutex{};
  auto await_fibers_cv = bf::condition_variable_any{};
  std::atomic<int> active{n_threads};

  std::atomic<std::uint64_t> next_block_id{0};
  struct WorkItem {
    std::uint64_t block_id;
    osm::buf buffer;
  };
  struct MergeItem {
    std::uint64_t block_id;
    std::shared_ptr<std::vector<osm::Node>> nodes;
  };
  auto merge_channel = bf::buffered_channel<MergeItem>{64U};

  auto ch = bf::buffered_channel<WorkItem>{64U};
  // for (auto i = 0U; i != n_threads; ++i) {
  for (auto& t : pool) {
    t = std::thread{[&]() {
      bf::use_scheduling_algorithm<bf::algo::work_stealing>(n_threads);
      bf::fiber([&]() {
        auto decompressor = osm::inflate{};
        auto out = std::string{};
        auto strings = std::vector<std::string_view>{};

        for (auto work : ch) {
          // auto local_nodes = std::vector<osm::Node>{};
          auto local_nodes = std::make_shared<std::vector<osm::Node>>();
          auto on_node_local = [&](std::int64_t const id,
                                   geo::latlng const& pos, auto&& tags) {
            osm::Location temp_loc{static_cast<int>(pos.lat()),
                                   static_cast<int>(pos.lng())};
            osm::Node temp_node{id, temp_loc};
            local_nodes->emplace_back(temp_node);
          };

          out.resize(work.buffer.raw_size_);
          decompressor.decompress(work.buffer.compressed_, out);
          osm::decode_primitive(out, strings, read_nodes, read_ways,
                                read_relations, on_node_local, on_way, on_rel);

          // Optional: sortiere lokal nach ID (sicher)
          // std::sort(local_nodes.begin(), local_nodes.end(),
          //          [](auto const& a, auto const& b) { return a.id < b.id; });
          // merge_channel.push({work.block_id, std::move(local_nodes)});
          merge_channel.push({work.block_id, local_nodes});
          std::lock_guard lk(await_fibers_mtx);
          active--;
        }
        await_fibers_cv.notify_one();
        std::cout << "worker exiting\n";
      }).detach();
      boost::this_fiber::yield();
    }};
  }

  auto tmp_dname = std::filesystem::temp_directory_path();
  auto const node_idx_file = tiles::tmp_file{
      (std::filesystem::path{tmp_dname} / "idx.bin").generic_string()};
  auto const node_dat_file = tiles::tmp_file{
      (std::filesystem::path{tmp_dname} / "dat.bin").generic_string()};
  tiles::hybrid_node_idx node_idx{node_idx_file.fileno(),
                                  node_dat_file.fileno()};
  tiles::hybrid_node_idx_builder node_idx_builder{node_idx};

  std::thread merger_thread([&]() {
    auto next_expected = std::uint64_t{0};
    auto items_pending =
        std::map<std::uint64_t, std::shared_ptr<std::vector<osm::Node>>>{};
    for (auto const& item : merge_channel) {
      items_pending.emplace(item.block_id, std::move(item.nodes));

      // normal map: if key there count = 1
      while (items_pending.count(next_expected)) {
        auto& vec = items_pending[next_expected];
        for (auto& node : *vec) {
          node_idx_builder.node(node);
        }
        items_pending.erase(next_expected);
        ++next_expected;
      }
    }
  });

  // auto pool = std::vector<std::thread>{n_threads};
  //  auto fin = std::atomic_bool{false};
  //  auto fin_cv = bf::condition_variable_any{};
  //  auto fin_mutex = std::mutex{};

  // for (auto& t : pool) {
  //   t = std::thread{[&]() {
  //     bf::use_scheduling_algorithm<bf::algo::work_stealing>(n_threads + 1U);
  //     auto l = std::unique_lock{fin_mutex};
  //     fin_cv.wait(l, [&]() { return fin.load(); });
  //   }};
  // }

  // bf::use_scheduling_algorithm<bf::algo::work_stealing>(n_threads + 1U);

  // update file-buffer, hier ändern wenn ich nicht immer das ganze file
  // druchgehe
  auto buf = std::optional<osm::buf>{};
  while ((buf = r.read()).has_value()) {
    WorkItem item;
    item.block_id = next_block_id++;
    item.buffer = std::move(*buf);
    ch.push(std::move(item));
    // ch.push(*buf);
    pt->update(r.file_.size() - r.rest_.size());
  }

  ch.close();
  std::cout << "ch closed\n";
  {
    std::unique_lock lk(await_fibers_mtx);
    await_fibers_cv.wait(lk, [&] { return active == 0; });
  }
  merge_channel.close();
  std::cout << "merge ch closed\n";
  // fin.store(true);
  // fin_cv.notify_all();

  for (auto& t : pool) {
    t.join();
  }
  merger_thread.join();
}

}  // namespace osm