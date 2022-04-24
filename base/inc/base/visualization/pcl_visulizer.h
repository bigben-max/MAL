#pragma once

#include <boost/thread/thread.hpp>
#include <iostream>
#include <map>
#include <string>
#include <vector>
//
#include <Eigen/Eigen>

#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
//
#include "base/log/log.h"

namespace base::viewer {

enum class ControlCmd : int32_t { STOP = 0, ONE_STEP, CONTINUE, FOCAL, OTHER };

struct CloudParam {
  std::string id = "cloud";
  std::vector<double> rgb = {255.0, 255.0, 255.0};
  bool use_z_color = false;
  int point_size = 3;
};

struct TextParam {
  std::string id = "text";
  std::vector<double> rgb = {1.0, 1.0, 1.0};
  int fontsize = 10;
};

class PlaneNormalVisualizer {
public:
  PlaneNormalVisualizer();
  void Spin(int stop_flag = 0, std::string name = "Viewer");
  void Init();
  void Clear();
  void UpdateCam(const Eigen::Matrix3d &rotation = Eigen::Matrix3d::Identity(),
                 const Eigen::Vector3d &pose = Eigen::Vector3d(0, 0, 0),
                 int v1 = 0);

  static void
  keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
                        void *viewer_void);
  void AddCoordinateSystem(Eigen::Vector3d pose);
  void AddText(const std::string &text, int xpos, int ypos,
               const std::string &id = "", int fontsize = 10, double r = 1,
               double g = 0, double b = 0, int viewport = 0);

  void UpdateCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                   std::string cloud_name = "cloud",
                   std::vector<double> cloud_color = {0.0, 1.0, 0.0},
                   bool use_z_color = false, int v1 = 0);
  void UpdateCloudI(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                    std::string cloud_name = "cloud",
                    std::vector<double> cloud_color = {0.0, 1.0, 0.0},
                    bool use_z_color = false, int v1 = 0);
  void UpdateCloudRGB(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                      std::string cloud_name = "cloud", int point_size = 3);

  void
  UpdateCloudAndNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                        pcl::PointCloud<pcl::Normal>::ConstPtr normals,
                        int ds_ratio = 10, std::string cloud_name = "cloud",
                        std::string normals_name = "normals",
                        std::vector<double> cloud_color = {1.0, 1.0, 1.0},
                        std::vector<double> normals_color = {1.0, 1.0, 0.0});

  void UpdateLines(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1,
                   pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2,
                   std::vector<double> line_color = {0.0, 1.0, 0.0});

  void UpdatePlanes(const std::vector<Eigen::Vector4d,
                                      Eigen::aligned_allocator<Eigen::Vector4d>>
                        &plane_coeffs);

private:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
  std::string name_;
  pcl::visualization::Camera cam_;
  static ControlCmd key_value_;
  //  pcl::visualization::PCLVisualizer* viewer;
  boost::mutex mtx_;
  bool init_ = false;
  bool first_ = false;
  bool remove_allpoint_ = false;

  std::vector<std::string> line_names_;
  std::vector<std::string> plane_names_;
};

} // namespace base::viewer
