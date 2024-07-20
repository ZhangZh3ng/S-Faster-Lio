#ifndef VOXMAP_VOXEL_MAP_H
#define VOXMAP_VOXEL_MAP_H

#include <execution>
#include <list>
#include <thread>

#include "voxmap/basic_voxel_node.h"
#include "voxmap/voxel_index.h"

// Faster-LIO中IVox类的简化版，移除了原版的前两个模板参数。其中ivox类型固定使用DEFAULT版（即论文中的Linear版），因此不再支持PHC版本。此外，还对KeyType进行了替换，用更加人性化的VoxelIndex类代替了原版的Eigen Matrix，但哈希函数保留了原版实现
template <typename PointType = pcl::PointXYZ>
class VoxelMap {
 public:
  using KeyType = VoxelIndex;
  using NodeType = BasicVoxelNode<PointType>;
  using PointVector =
      std::vector<PointType, Eigen::aligned_allocator<PointType>>;
  using DistPoint = typename NodeType::DistPoint;

  enum class NearbyType {
    CENTER,  // center only
    NEARBY6,
    NEARBY18,
    NEARBY26,
  };

  struct Options {
    float resolution_ = 0.5;                        // ivox resolution
    float inv_resolution_ = 10.0;                   // inverse resolution
    NearbyType nearby_type_ = NearbyType::NEARBY6;  // nearby range
    std::size_t capacity_ = 1000000;                // capacity
  };

  /**
   * constructor
   * @param options  VoxelMap options
   */
  explicit VoxelMap(Options options) : options_(options) {
    options_.inv_resolution_ = 1.0 / options_.resolution_;
    GenerateNearbyGrids();
  }

  /**
   * add points
   * @param points_to_add
   */
  void AddPoints(const PointVector& points_to_add);

  /// get nn
  bool GetClosestPoint(const PointType& pt, PointType& closest_pt);

  /// get nn with condition
  bool GetClosestPoint(const PointType& pt,
                       PointVector& closest_pt,
                       int max_num = 5,
                       double max_range = 5.0);

  /// get nn in cloud
  bool GetClosestPoint(const PointVector& cloud, PointVector& closest_cloud);

  /// get number of points
  size_t NumPoints() const;

  /// get number of valid grids
  size_t NumValidGrids() const;

  /// get statistics of the points
  std::vector<float> StatGridPoints() const;

 private:
  /// generate the nearby grids according to the given options
  void GenerateNearbyGrids();

  /// position to grid
  KeyType Pos2Grid(const PointType& pt) const;

  Options options_;
  std::unordered_map<KeyType,
                     typename std::list<std::pair<KeyType, NodeType>>::iterator>
      grids_map_;                                        // voxel hash map
  std::list<std::pair<KeyType, NodeType>> grids_cache_;  // voxel cache
  std::vector<KeyType> nearby_grids_;                    // nearbys
};

template <typename PointType>
VoxelIndex VoxelMap<PointType>::Pos2Grid(const PointType& pt) const {
  return VoxelIndex(
      static_cast<int>(std::round(pt.x * options_.inv_resolution_)),
      static_cast<int>(std::round(pt.y * options_.inv_resolution_)),
      static_cast<int>(std::round(pt.z * options_.inv_resolution_)));
}

template <typename PointType>
bool VoxelMap<PointType>::GetClosestPoint(const PointType& pt,
                                          PointType& closest_pt) {
  std::vector<DistPoint> candidates;
  // auto key = Pos2Grid(pt);
  auto key = KeyType::FromPclPoint(pt, options_.inv_resolution_);
  std::for_each(nearby_grids_.begin(), nearby_grids_.end(),
                [&key, &candidates, &pt, this](const KeyType& delta) {
                  auto dkey = key + delta;
                  auto iter = grids_map_.find(dkey);
                  if (iter != grids_map_.end()) {
                    DistPoint dist_point;
                    bool found = iter->second->second.NNPoint(pt, dist_point);
                    if (found) {
                      candidates.emplace_back(dist_point);
                    }
                  }
                });

  if (candidates.empty()) {
    return false;
  }

  auto iter = std::min_element(candidates.begin(), candidates.end());
  closest_pt = iter->Get();
  return true;
}

template <typename PointType>
bool VoxelMap<PointType>::GetClosestPoint(const PointType& pt,
                                          PointVector& closest_pt,
                                          int max_num,
                                          double max_range) {
  std::vector<DistPoint> candidates;
  candidates.reserve(max_num * nearby_grids_.size());

  auto key = Pos2Grid(pt);

// #define INNER_TIMER
#ifdef INNER_TIMER
  static std::unordered_map<std::string, std::vector<int64_t>> stats;
  if (stats.empty()) {
    stats["knn"] = std::vector<int64_t>();
    stats["nth"] = std::vector<int64_t>();
  }
#endif

  for (const KeyType& delta : nearby_grids_) {
    auto dkey = key + delta;
    auto iter = grids_map_.find(dkey);
    if (iter != grids_map_.end()) {
#ifdef INNER_TIMER
      auto t1 = std::chrono::high_resolution_clock::now();
#endif
      auto tmp = iter->second->second.KNNPointByCondition(candidates, pt,
                                                          max_num, max_range);
#ifdef INNER_TIMER
      auto t2 = std::chrono::high_resolution_clock::now();
      auto knn =
          std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
      stats["knn"].emplace_back(knn);
#endif
    }
  }

  if (candidates.empty()) {
    return false;
  }

#ifdef INNER_TIMER
  auto t1 = std::chrono::high_resolution_clock::now();
#endif

  if (candidates.size() <= max_num) {
  } else {
    std::nth_element(candidates.begin(), candidates.begin() + max_num - 1,
                     candidates.end());
    candidates.resize(max_num);
  }
  std::nth_element(candidates.begin(), candidates.begin(), candidates.end());

#ifdef INNER_TIMER
  auto t2 = std::chrono::high_resolution_clock::now();
  auto nth =
      std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
  stats["nth"].emplace_back(nth);

  constexpr int STAT_PERIOD = 100000;
  if (!stats["nth"].empty() && stats["nth"].size() % STAT_PERIOD == 0) {
    for (auto& it : stats) {
      const std::string& key = it.first;
      std::vector<int64_t>& stat = it.second;
      int64_t sum_ = std::accumulate(stat.begin(), stat.end(), 0);
      int64_t num_ = stat.size();
      stat.clear();
      std::cout << "inner_" << key << "(ns): sum=" << sum_ << " num=" << num_
                << " ave=" << 1.0 * sum_ / num_
                << " ave*n=" << 1.0 * sum_ / STAT_PERIOD << std::endl;
    }
  }
#endif

  closest_pt.clear();
  for (auto& it : candidates) {
    closest_pt.emplace_back(it.Get());
  }
  return closest_pt.empty() == false;
}

template <typename PointType>
size_t VoxelMap<PointType>::NumValidGrids() const {
  return grids_map_.size();
}

template <typename PointType>
void VoxelMap<PointType>::GenerateNearbyGrids() {
  if (options_.nearby_type_ == NearbyType::CENTER) {
    nearby_grids_.emplace_back(KeyType());
  } else if (options_.nearby_type_ == NearbyType::NEARBY6) {
    nearby_grids_ = {KeyType(0, 0, 0), KeyType(-1, 0, 0), KeyType(1, 0, 0),
                     KeyType(0, 1, 0), KeyType(0, -1, 0), KeyType(0, 0, -1),
                     KeyType(0, 0, 1)};
  } else if (options_.nearby_type_ == NearbyType::NEARBY18) {
    nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0),  KeyType(1, 0, 0),
                     KeyType(0, 1, 0),  KeyType(0, -1, 0),  KeyType(0, 0, -1),
                     KeyType(0, 0, 1),  KeyType(1, 1, 0),   KeyType(-1, 1, 0),
                     KeyType(1, -1, 0), KeyType(-1, -1, 0), KeyType(1, 0, 1),
                     KeyType(-1, 0, 1), KeyType(1, 0, -1),  KeyType(-1, 0, -1),
                     KeyType(0, 1, 1),  KeyType(0, -1, 1),  KeyType(0, 1, -1),
                     KeyType(0, -1, -1)};
  } else if (options_.nearby_type_ == NearbyType::NEARBY26) {
    nearby_grids_ = {
        KeyType(0, 0, 0),   KeyType(-1, 0, 0),  KeyType(1, 0, 0),
        KeyType(0, 1, 0),   KeyType(0, -1, 0),  KeyType(0, 0, -1),
        KeyType(0, 0, 1),   KeyType(1, 1, 0),   KeyType(-1, 1, 0),
        KeyType(1, -1, 0),  KeyType(-1, -1, 0), KeyType(1, 0, 1),
        KeyType(-1, 0, 1),  KeyType(1, 0, -1),  KeyType(-1, 0, -1),
        KeyType(0, 1, 1),   KeyType(0, -1, 1),  KeyType(0, 1, -1),
        KeyType(0, -1, -1), KeyType(1, 1, 1),   KeyType(-1, 1, 1),
        KeyType(1, -1, 1),  KeyType(1, 1, -1),  KeyType(-1, -1, 1),
        KeyType(-1, 1, -1), KeyType(1, -1, -1), KeyType(-1, -1, -1)};
  } else {
    // LOG(ERROR) << "Unknown nearby_type!";
  }
}

template <typename PointType>
bool VoxelMap<PointType>::GetClosestPoint(const PointVector& cloud,
                                          PointVector& closest_cloud) {
  std::vector<size_t> index(cloud.size());
  for (int i = 0; i < cloud.size(); ++i) {
    index[i] = i;
  }
  closest_cloud.resize(cloud.size());

  std::for_each(std::execution::par_unseq, index.begin(), index.end(),
                [&cloud, &closest_cloud, this](size_t idx) {
                  PointType pt;
                  if (GetClosestPoint(cloud[idx], pt)) {
                    closest_cloud[idx] = pt;
                  } else {
                    closest_cloud[idx] = PointType();
                  }
                });
  return true;
}

template <typename PointType>
void VoxelMap<PointType>::AddPoints(const PointVector& points_to_add) {
  std::for_each(std::execution::unseq, points_to_add.begin(),
                points_to_add.end(), [this](const auto& pt) {
                  auto key = Pos2Grid(pt);

                  auto iter = grids_map_.find(key);
                  if (iter == grids_map_.end()) {
                    PointType center;
                    center.x = key.x() * options_.resolution_;
                    center.y = key.y() * options_.resolution_;
                    center.z = key.z() * options_.resolution_;

                    grids_cache_.push_front(
                        {key, NodeType(center, options_.resolution_)});
                    grids_map_.insert({key, grids_cache_.begin()});

                    grids_cache_.front().second.InsertPoint(pt);

                    if (grids_map_.size() >= options_.capacity_) {
                      grids_map_.erase(grids_cache_.back().first);
                      grids_cache_.pop_back();
                    }
                  } else {
                    iter->second->second.InsertPoint(pt);
                    grids_cache_.splice(grids_cache_.begin(), grids_cache_,
                                        iter->second);
                    grids_map_[key] = grids_cache_.begin();
                  }
                });
}

template <typename PointType>
std::vector<float> VoxelMap<PointType>::StatGridPoints() const {
  int num = grids_cache_.size(), valid_num = 0, max = 0, min = 100000000;
  int sum = 0, sum_square = 0;
  for (auto& it : grids_cache_) {
    int s = it.second.Size();
    valid_num += s > 0;
    max = s > max ? s : max;
    min = s < min ? s : min;
    sum += s;
    sum_square += s * s;
  }
  float ave = float(sum) / num;
  float stddev =
      num > 1 ? sqrt((float(sum_square) - num * ave * ave) / (num - 1)) : 0;
  return std::vector<float>{valid_num, ave, max, min, stddev};
}

#endif  // VOXMAP_VOXEL_MAP_H