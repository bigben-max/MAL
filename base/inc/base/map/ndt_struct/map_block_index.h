
/*
 * @Date: 2019-08-28 17:35:33
 * @Author: max.zhong
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-28 02:46:05
 */
#pragma once

#include <string>
//
#include "base/map/ndt_struct/map_config.h"

namespace base::map {

class MapBlockIndex;

std::ostream& operator<<(std::ostream& cout, const MapBlockIndex& index);

class MapBlockIndex {
 public:
  /**@brief The constructor. */
  MapBlockIndex();
  /**@brief Overload the less than operator. */
  bool operator<(const MapBlockIndex& index) const;
  /**@brief Overload the equal operator. */
  bool operator==(const MapBlockIndex& index) const;
  /**@brief Overload the unequal operator. */
  bool operator!=(const MapBlockIndex& index) const;
  std::string ToString() const;

  /**@brief Construct a map node index, given a global coordinate, eigen
   * version. */
  static MapBlockIndex GetMapBlockIndex(const MapConfig& option,
                                        const Eigen::Vector3d& coordinate,
                                        unsigned int resolution_id,
                                        int zone_id);
  static MapBlockIndex GetMapBlockIndex(const MapConfig& option,
                                        const Eigen::Vector2d& coordinate,
                                        unsigned int resolution_id,
                                        int zone_id);

  /**@brief Get the index range (maximum possible index + 1) in the east
   * direction. */
  static unsigned int GetMapIndexRangeEast(const MapConfig& option,
                                           unsigned int resolution_id);
  /**@brief Get the index range (maximum possible index + 1) in the north
   * direction. */
  static unsigned int GetMapIndexRangeNorth(const MapConfig& option,
                                            unsigned int resolution_id);

  friend std::ostream& operator<<(std::ostream& cout,
                                  const MapBlockIndex& index);

  /**@brief The ID of the resolution.
   * Should be less than MapConfig::map_resolutions_.size(). */
  unsigned int resolution_id_ = 0;
  /**@brief The zone ID. 1 - 60 and -1 - -60.
   * The positive value is the zone at the north hemisphere. */
  int zone_id_ = 50;
  /**@brief The map node ID at the northing direction. */
  unsigned int m_ = 0;
  /**@brief The map node ID at the easting direction. */
  unsigned int n_ = 0;
};

}  // namespace locator

