/*
 * @Date: 2019-08-28 17:49:11
 * @Author: max.zhong
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-28 02:08:17
 */
#pragma once

#include <boost/property_tree/xml_parser.hpp>
#include <string>
#include <vector>
//
#include "base/map_struct/rect2d.h"

namespace base::map {

/**@brief The options of the reflectance map. */
class MapConfig {
 public:
  /**@brief The constructor gives the default map settings. */
  explicit MapConfig(std::string map_version = "0.1");
  /**@brief Save the map option to a XML file. */
  bool Save(const std::string file_path);
  /**@brief Load the map option from a XML file. */
  bool Load(const std::string file_path);
  /**@brief Resize map range by range and resolutions. */
  void ResizeMapRange();
  /**@brief Set single resolutions. */
  void SetSingleResolutions(float resolution = 1.0);
  /**@brief Set multi resolutions. */
  void SetMultiResolutions();

  /**@brief The version of map. */
  std::string map_version_;
  /**@brief The pixel resolutions in the map in meters. */
  std::vector<float> map_resolutions_;
  /**@brief The map block size in pixels. */
  unsigned int map_block_size_x_;
  /**@brief The map block size in pixels. */
  unsigned int map_block_size_y_;
  /**@brief The minimum and maximum UTM range in the map.
   *
   * The x direction is the easting in UTM coordinate.
   * The y direction is the northing in UTM coordinate.
   */
  Rect2D<double> map_range_;

  /**@brief Velodyne's height to the ground. Estimate the Velodyne's height
   * based on the ground height. */
  float map_ground_height_offset_;
  /**@brief Enable the compression. */
  bool map_is_compression_;

  /**@brief The map folder path. */
  std::string map_folder_path_;
  /**@brief The datasets that contributed to the map. */
  std::vector<std::string> map_datasets_;

 protected:
  /**@brief Create the XML structure. */
  virtual void CreateXml(boost::property_tree::ptree* config) const;
  /**@brief Load the map options from a XML structure. */
  virtual void LoadXml(const boost::property_tree::ptree& config);
};

}  // namespace base::map
