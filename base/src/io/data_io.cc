#include <fstream>
#include <iostream>
//
#include "base/io/data_io.h"
#include "base/log/log.h"

namespace base::ldm {

bool loadKittiCloud(const std::string &file_name,
                    base::LidarMeasurement *lidar_mes) {
  CHECK_NOTNULL(lidar_mes);
  std::ifstream lidar_data_file(file_name,
                                std::ifstream::in | std::ifstream::binary);
  lidar_data_file.seekg(0, std::ios::end);
  const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
  lidar_data_file.seekg(0, std::ios::beg);

  std::vector<float> lidar_data_buffer(num_elements);
  lidar_data_file.read(reinterpret_cast<char *>(&lidar_data_buffer[0]),
                       num_elements * sizeof(float));

  for (size_t i = 0; i < num_elements; i += 4) {
    PointI pt;
    pt.x = lidar_data_buffer[i];
    pt.y = lidar_data_buffer[i + 1];
    pt.z = lidar_data_buffer[i + 2];
    pt.intensity = lidar_data_buffer[i + 3];
    lidar_mes->pointcloud->emplace_back(pt);
  }
  return true;
}

} // namespace base::ldm
