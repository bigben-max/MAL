/*
 * @Date: 2019-08-28 17:56:20
 * @Author: max.zhong
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-28 02:56:14
 */

#include "base/log/log.h"
#include "base/map/ndt_struct/map_block_index.h"

namespace base::map {
  
MapBlockIndex::MapBlockIndex() {}

bool MapBlockIndex::operator<(const MapBlockIndex& index) const {
  if (resolution_id_ < index.resolution_id_) {
    return true;
  } else if (resolution_id_ == index.resolution_id_) {
    if (zone_id_ < index.zone_id_) {
      return true;
    } else if (zone_id_ == index.zone_id_) {
      if (m_ < index.m_) {
        return true;
      } else if (m_ == index.m_) {
        if (n_ < index.n_) {
          return true;
        } else {
          return false;
        }
      } else {
        return false;
      }
    } else {
      return false;
    }
  } else {
    return false;
  }
}

bool MapBlockIndex::operator==(const MapBlockIndex& index) const {
  return resolution_id_ == index.resolution_id_ && zone_id_ == index.zone_id_ &&
         m_ == index.m_ && n_ == index.n_;
}

bool MapBlockIndex::operator!=(const MapBlockIndex& index) const {
  return !(*this == index);
}

std::string MapBlockIndex::ToString() const {
  std::ostringstream ss;
  ss << "Map node (Resolution ID: " << resolution_id_
     << " Zone ID: " << zone_id_ << " Easting: " << n_ << " Northing: " << m_
     << ")";
  return ss.str();
}

MapBlockIndex MapBlockIndex::GetMapBlockIndex(const MapConfig& option,
                                              const Eigen::Vector3d& coordinate,
                                              uint32_t resolution_id,
                                              int zone_id) {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  return GetMapBlockIndex(option, coord2d, resolution_id, zone_id);
}

MapBlockIndex MapBlockIndex::GetMapBlockIndex(const MapConfig& option,
                                              const Eigen::Vector2d& coordinate,
                                              uint32_t resolution_id,
                                              int zone_id) {
  DCHECK_LT(resolution_id, option.map_resolutions_.size());
  MapBlockIndex index;
  index.resolution_id_ = resolution_id;
  index.zone_id_ = zone_id;
  int n = static_cast<int>((coordinate[0] - option.map_range_.GetMinX()) /
                           (static_cast<float>(option.map_block_size_x_) *
                            option.map_resolutions_[resolution_id]));
  int m = static_cast<int>((coordinate[1] - option.map_range_.GetMinY()) /
                           (static_cast<float>(option.map_block_size_y_) *
                            option.map_resolutions_[resolution_id]));
  if (n >= 0 && m >= 0 &&
      n < static_cast<int>(GetMapIndexRangeEast(option, resolution_id)) &&
      m < static_cast<int>(GetMapIndexRangeNorth(option, resolution_id))) {
    index.m_ = m;
    index.n_ = n;
  } else {
    DCHECK(false);  // should never reach here
  }
  return index;
}

uint32_t MapBlockIndex::GetMapIndexRangeEast(const MapConfig& option,
                                                 uint32_t resolution_id) {
  return static_cast<uint32_t>(
      (option.map_range_.GetMaxX() - option.map_range_.GetMinX()) /
      (static_cast<float>(option.map_block_size_x_) *
       option.map_resolutions_[resolution_id]));
}

uint32_t MapBlockIndex::GetMapIndexRangeNorth(const MapConfig& option,
                                                  uint32_t resolution_id) {
  return static_cast<uint32_t>(
      (option.map_range_.GetMaxY() - option.map_range_.GetMinY()) /
      (static_cast<float>(option.map_block_size_y_) *
       option.map_resolutions_[resolution_id]));
}

std::ostream& operator<<(std::ostream& cerr, const MapBlockIndex& index) {
  cerr << index.ToString();
  return cerr;
}

}  // namespace base::map
