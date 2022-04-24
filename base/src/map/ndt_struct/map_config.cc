/*
 * @Date: 2019-08-28 17:52:17
 * @Author: max.zhong
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-28 02:43:32
 */
#include <boost/foreach.hpp>
//
#include "base/map/ndt_struct/map_config.h"
#include "base/log/log_helper.h"

namespace base::map {

MapConfig::MapConfig(std::string map_version) {
  map_resolutions_.push_back(1.0);  // Resolution: 0.125m in level 2
  map_block_size_x_ = 100;           // in pixels
  map_block_size_y_ = 100;           // in pixels
  map_range_ = Rect2D<double>(0, 0, 1000448.0, 10000384.0);  // in meters

  map_version_ = map_version;
  map_folder_path_ = ".";
}

bool MapConfig::Save(const std::string file_path) {
  boost::property_tree::ptree config;
  CreateXml(&config);
  boost::property_tree::write_xml(file_path, config);
  AINFO << "Saved the map configuration to: " << file_path;
  return true;
}

bool MapConfig::Load(const std::string file_path) {
  boost::property_tree::ptree config;
  boost::property_tree::read_xml(file_path, config);

  std::string map_version = config.get<std::string>("map.map_config.version");
  if (map_version_ == map_version) {
    LoadXml(config);
    AINFO << "Loaded the map configuration from: " << file_path;
    return true;
  } else {
    AERROR << "[Fatal Error] Expect v" << map_version_
                  << " map, but found v" << map_version << " map.";
    return false;
  }
}

void MapConfig::CreateXml(boost::property_tree::ptree* config) const {
  config->put("map.map_config.version", map_version_);
  config->put("map.map_config.node_size.x", map_block_size_x_);
  config->put("map.map_config.node_size.y", map_block_size_y_);
  config->put("map.map_config.range.min_x", map_range_.GetMinX());
  config->put("map.map_config.range.min_y", map_range_.GetMinY());
  config->put("map.map_config.range.max_x", map_range_.GetMaxX());
  config->put("map.map_config.range.max_y", map_range_.GetMaxY());
  // config->put("map.map_config.compression", map_is_compression_);
  config->put("map.map_runtime.map_ground_height_offset",
              map_ground_height_offset_);
  for (size_t i = 0; i < map_resolutions_.size(); ++i) {
    config->add("map.map_config.resolutions.resolution", map_resolutions_[i]);
  }
  config->put("map.map_record.map_folder_path", map_folder_path_);
  for (size_t i = 0; i < map_datasets_.size(); ++i) {
    config->add("map.map_record.datasets.dataset", map_datasets_[i]);
  }
  return;
}

void MapConfig::LoadXml(const boost::property_tree::ptree& config) {
  map_resolutions_.clear();
  map_datasets_.clear();
  map_block_size_x_ = config.get<unsigned int>("map.map_config.node_size.x");
  map_block_size_y_ = config.get<unsigned int>("map.map_config.node_size.y");
  double min_x = config.get<double>("map.map_config.range.min_x");
  double min_y = config.get<double>("map.map_config.range.min_y");
  double max_x = config.get<double>("map.map_config.range.max_x");
  double max_y = config.get<double>("map.map_config.range.max_y");
  map_range_ = Rect2D<double>(min_x, min_y, max_x, max_y);
  // map_is_compression_ = config.get<bool>("map.map_config.compression");
  map_ground_height_offset_ =
      config.get<float>("map.map_runtime.map_ground_height_offset");
  BOOST_FOREACH (const boost::property_tree::ptree::value_type& v,  // NOLINT
                 config.get_child("map.map_config.resolutions")) {
    map_resolutions_.push_back(
        static_cast<float>(atof(v.second.data().c_str())));
    AINFO << "Resolution: " << v.second.data();
  }
  map_folder_path_ = config.get<std::string>("map.map_record.map_folder_path");
  AINFO << "map_folder_path: " << map_folder_path_;
  BOOST_FOREACH (const boost::property_tree::ptree::value_type& v,  // NOLINT
                 config.get_child("map.map_record.datasets")) {
    map_datasets_.push_back(v.second.data());
    AINFO << "Dataset: " << v.second.data();
  }
  return;
}

void MapConfig::ResizeMapRange() {
  double min_x = 0;
  double min_y = 0;
  double max_x = 0;
  double max_y = 0;

  double max_resolutions = 0.0;
  for (std::size_t i = 0; i < map_resolutions_.size(); ++i) {
    if (max_resolutions < map_resolutions_[i]) {
      max_resolutions = map_resolutions_[i];
    }
  }

  int n = 0;
  while (true) {
    if (min_x < map_range_.GetMinX()) {
      break;
    }
    ++n;
    min_x -= n * map_block_size_x_ * max_resolutions;
  }
  n = 0;
  while (true) {
    if (min_y < map_range_.GetMinY()) {
      break;
    }
    ++n;
    min_y -= n * map_block_size_y_ * max_resolutions;
  }
  n = 0;
  while (true) {
    if (max_x > map_range_.GetMaxX()) {
      break;
    }
    ++n;
    max_x += n * map_block_size_x_ * max_resolutions;
  }
  n = 0;
  while (true) {
    if (max_y > map_range_.GetMaxY()) {
      break;
    }
    ++n;
    max_y += n * map_block_size_y_ * max_resolutions;
  }
  map_range_ = Rect2D<double>(min_x, min_y, max_x, max_y);
}

void MapConfig::SetSingleResolutions(float resolution) {
  map_resolutions_.clear();
  map_resolutions_.push_back(resolution);
}

}  // namespace base::map
