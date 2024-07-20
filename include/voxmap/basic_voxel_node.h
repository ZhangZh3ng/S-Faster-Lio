#ifndef VOXMAP_BASIC_VOXEL_NODE
#define VOXMAP_BASIC_VOXEL_NODE

#include <algorithm>
#include <cmath>
#include <list>
#include <vector>

#include <pcl/point_types.h>

// 对pcl库中3D PointType通用的'距离平方'计算函数
template <typename PointT>
inline double distance2(const PointT& pt1, const PointT& pt2) {
  Eigen::Vector3f d = pt1.getVector3fMap() - pt2.getVector3fMap();
  return d.squaredNorm();
}

// 'ivox node'的default版本
template <typename PointT>
class BasicVoxelNode {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  struct DistPoint;

  BasicVoxelNode() = default;
  BasicVoxelNode(const PointT& center, const float& side_length) {
  } 

  void InsertPoint(const PointT& pt);

  inline bool Empty() const;

  inline std::size_t Size() const;

  inline PointT GetPoint(const std::size_t idx) const;

  int KNNPointByCondition(std::vector<DistPoint>& dis_points,
                          const PointT& point,
                          const int& K,
                          const double& max_range);

 private:
  std::vector<PointT> points_;
};

template <typename PointT>
struct BasicVoxelNode<PointT>::DistPoint {
  double dist = 0;
  BasicVoxelNode* node = nullptr;
  int idx = 0;

  DistPoint() = default;
  DistPoint(const double d, BasicVoxelNode* n, const int i)
      : dist(d), node(n), idx(i) {}

  PointT Get() { return node->GetPoint(idx); }

  inline bool operator<(const DistPoint& rhs) { return dist < rhs.dist; }
};

template <typename PointT>
void BasicVoxelNode<PointT>::InsertPoint(const PointT& pt) {
  points_.template emplace_back(pt);
}

template <typename PointT>
bool BasicVoxelNode<PointT>::Empty() const {
  return points_.empty();
}

template <typename PointT>
std::size_t BasicVoxelNode<PointT>::Size() const {
  return points_.size();
}

template <typename PointT>
PointT BasicVoxelNode<PointT>::GetPoint(const std::size_t idx) const {
  return points_[idx];
}

template <typename PointT>
int BasicVoxelNode<PointT>::KNNPointByCondition(
    std::vector<DistPoint>& dis_points,
    const PointT& point,
    const int& K,
    const double& max_range) {
  std::size_t old_size = dis_points.size();
// #define INNER_TIMER
#ifdef INNER_TIMER
  static std::unordered_map<std::string, std::vector<int64_t>> stats;
  if (stats.empty()) {
    stats["dis"] = std::vector<int64_t>();
    stats["put"] = std::vector<int64_t>();
    stats["nth"] = std::vector<int64_t>();
  }
#endif

  for (const auto& pt : points_) {
#ifdef INNER_TIMER
    auto t0 = std::chrono::high_resolution_clock::now();
#endif
    double d = distance2(pt, point);
#ifdef INNER_TIMER
    auto t1 = std::chrono::high_resolution_clock::now();
#endif
    if (d < max_range * max_range) {
      dis_points.template emplace_back(
          DistPoint(d, this, &pt - points_.data()));
    }
#ifdef INNER_TIMER
    auto t2 = std::chrono::high_resolution_clock::now();

    auto dis =
        std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
    stats["dis"].emplace_back(dis);
    auto put =
        std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
    stats["put"].emplace_back(put);
#endif
  }

#ifdef INNER_TIMER
  auto t1 = std::chrono::high_resolution_clock::now();
#endif
  // sort by distance
  if (old_size + K >= dis_points.size()) {
  } else {
    std::nth_element(dis_points.begin() + old_size,
                     dis_points.begin() + old_size + K - 1, dis_points.end());
    dis_points.resize(old_size + K);
  }

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

  return dis_points.size();
}

#endif  // VOXMAP_BASIC_VOXEL_NODE
