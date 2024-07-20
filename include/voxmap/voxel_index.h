#ifndef VOXMAP_VOXEL_INDEX_H
#define VOXMAP_VOXEL_INDEX_H

#include <functional>
#include <iostream>

class VoxelIndex {
 public:
  VoxelIndex() : coords_{0, 0, 0} {}

  VoxelIndex(int x, int y, int z) : coords_{x, y, z} {}

  int x() const { return coords_[0]; }
  int y() const { return coords_[1]; }
  int z() const { return coords_[2]; }

  bool operator==(const VoxelIndex& other) const {
    return coords_[0] == other.coords_[0] && coords_[1] == other.coords_[1] &&
           coords_[2] == other.coords_[2];
  }

  bool operator!=(const VoxelIndex& other) const { return !(*this == other); }

  // for sort
  bool operator<(const VoxelIndex& other) const {
    if (coords_[0] != other.coords_[0])
      return coords_[0] < other.coords_[0];
    if (coords_[1] != other.coords_[1])
      return coords_[1] < other.coords_[1];
    return coords_[2] < other.coords_[2];
  }

  // for adding delta index
  VoxelIndex operator+(const VoxelIndex& other) const {
    return VoxelIndex(coords_[0] + other.coords_[0],
                      coords_[1] + other.coords_[1],
                      coords_[2] + other.coords_[2]);
  }

  // same with faster-lio implementation
  std::size_t hash() const {
    return size_t(((coords_[0]) * 73856093) ^ ((coords_[1]) * 471943) ^
                  ((coords_[2]) * 83492791)) %
           10000000;
  }

  friend std::ostream& operator<<(std::ostream& os, const VoxelIndex& index) {
    os << "VoxelIndex(" << index.coords_[0] << ", " << index.coords_[1] << ", "
       << index.coords_[2] << ")";
    return os;
  }

  template <typename EigenVector>
  static VoxelIndex FromEigenVector(const EigenVector& eigen,
                                    const double& s = 1.0) {
    return VoxelIndex(static_cast<int>(std::round(eigen[0] * s)),
                      static_cast<int>(std::round(eigen[1] * s)),
                      static_cast<int>(std::round(eigen[2] * s)));
  }

  template <typename PclPointType>
  static VoxelIndex FromPclPoint(const PclPointType& point,
                                 const double& s = 1.0) {
    return VoxelIndex(static_cast<int>(std::round(point.x * s)),
                      static_cast<int>(std::round(point.y * s)),
                      static_cast<int>(std::round(point.z * s)));
  }

 private:
  int coords_[3];
};

namespace std {
template <>
struct hash<VoxelIndex> {
  std::size_t operator()(const VoxelIndex& index) const { return index.hash(); }
};
}  // namespace std

#endif  // VOXMAP_VOXEL_INDEX_H
