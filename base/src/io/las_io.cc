#include <cmath>
#include <fstream>
//
#include "base/io/las_io.h"
#include "base/log/log.h"

namespace base::ldm {

bool savePointCloudToLas(const base::LidarMeasurement &lidar_mes,
                         const std::string &file_name) {
  liblas::Header header;
  header.SetDataFormatId(liblas::ePointFormat2);
  double minPt[3] = {9999999, 9999999, 9999999};
  double maxPt[3] = {-9999999, -9999999, -9999999};
  header.SetPointRecordsCount(lidar_mes.pointcloud->size());
  header.SetPointRecordsByReturnCount(0, lidar_mes.pointcloud->size());
  std::ofstream ofs;
  ofs.open(file_name, std::ios::out | std::ios::binary);

  liblas::Writer writer(ofs, header);
  liblas::Point las_pt(&header);
  for (size_t j = 0; j < lidar_mes.pointcloud->size(); j++) {
    double pt_x = lidar_mes.pointcloud->points[j].x;
    double pt_y = lidar_mes.pointcloud->points[j].y;
    double pt_z = lidar_mes.pointcloud->points[j].z;
    minPt[0] = std::min(minPt[0], pt_x);
    minPt[1] = std::min(minPt[1], pt_y);
    minPt[2] = std::min(minPt[2], pt_z);
    maxPt[0] = std::max(maxPt[0], pt_x);
    maxPt[1] = std::max(maxPt[1], pt_y);
    maxPt[2] = std::max(maxPt[2], pt_z);
    las_pt.SetCoordinates(pt_x, pt_y, pt_z);
    las_pt.SetIntensity(lidar_mes.pointcloud->points[j].intensity);
    writer.WritePoint(las_pt);
  }
  header.SetMax(maxPt[0], maxPt[1], maxPt[2]);
  header.SetMin(minPt[0], minPt[1], minPt[2]);
  writer.SetHeader(header);
  writer.WriteHeader();
  return true;
}

} // namespace base::ldm
