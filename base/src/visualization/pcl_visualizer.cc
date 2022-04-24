

#include "base/visualization/pcl_visulizer.h"

namespace base::viewer {

ControlCmd PlaneNormalVisualizer::key_value_ = ControlCmd::STOP;
void PlaneNormalVisualizer::keyboardEventOccurred(
    const pcl::visualization::KeyboardEvent &event, void *viewer_void) {
  std::string key = event.getKeySym();
  if (event.getKeySym() == "a" && event.keyDown()) {
    key_value_ = ControlCmd::ONE_STEP;
  }
  if (event.getKeySym() == "s" && event.keyDown()) {
    key_value_ = ControlCmd::STOP;
  }
  if (event.getKeySym() == "d" && event.keyDown()) {
    key_value_ = ControlCmd::CONTINUE;
  }
  if (event.getKeySym() == "f" && event.keyDown()) {
    key_value_ = ControlCmd::FOCAL;
  }
}
void PlaneNormalVisualizer::Clear() { viewer_->removeAllPointClouds(); }

void PlaneNormalVisualizer::AddCoordinateSystem(Eigen::Vector3d pose) {
  viewer_->addCoordinateSystem(1.0, pose[0], pose[1], pose[2]);
}
void PlaneNormalVisualizer::AddText(const std::string &text, int xpos, int ypos,
                                    const std::string &id, int fontsize,
                                    double r, double g, double b,
                                    int viewport) {
  if (!viewer_->updateText(text, xpos, ypos, fontsize, r, g, b, id)) {
    viewer_->addText(text, xpos, ypos, fontsize, r, g, b, id, viewport);
  }
}

void PlaneNormalVisualizer::UpdateCloud(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, std::string cloud_name,
    std::vector<double> cloud_color, bool use_z_color, int v1) {
  boost::mutex::scoped_lock lk(mtx_);

  //  AINFO << ">>>>>>> update <<<<<<<";
  //  AINFO << cloud->size();

  if (cloud->size() == 0) {
    AINFO << ">>>>>>> no points <<<<<<<";
    return;
  }
  if (use_z_color) {
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ>
        fildColor(cloud, "z");
    if (!viewer_->updatePointCloud(cloud, fildColor, cloud_name)) {
      viewer_->addPointCloud<pcl::PointXYZ>(cloud, fildColor, cloud_name, v1);
      viewer_->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, cloud_name, v1);
    }
  } else {
    if (!viewer_->updatePointCloud(cloud, cloud_name)) {
      viewer_->addPointCloud<pcl::PointXYZ>(cloud, cloud_name, v1);
      viewer_->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, cloud_name, v1);
    }
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, cloud_color[0],
        cloud_color[1], cloud_color[2], cloud_name, v1);
  }
}

void PlaneNormalVisualizer::UpdateCloudI(
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, std::string cloud_name,
    std::vector<double> cloud_color, bool use_z_color, int v1) {
  boost::mutex::scoped_lock lk(mtx_);

  //  AINFO << ">>>>>>> update <<<<<<<";
  //  AINFO << cloud->size();

  if (cloud->size() == 0) {
    AINFO << ">>>>>>> no points <<<<<<<";
    return;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_show(
      new pcl::PointCloud<pcl::PointXYZRGB>());
  for (auto pt : cloud->points) {
    pcl::PointXYZRGB pt_rgb;
    pt_rgb.x = pt.x;
    pt_rgb.y = pt.y;
    pt_rgb.z = pt.z;
    float intensity = pt.intensity;
    // int color[3];
    // IntensityToColor(intensity, color);
    pt_rgb.r = intensity * 100; // color[0];
    pt_rgb.g = intensity * 100; // color[1];
    pt_rgb.b = intensity * 100; // color[2];
    cloud_show->push_back(pt_rgb);
  }
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
      cloud_show);
  if (!viewer_->updatePointCloud(cloud_show, rgb, cloud_name)) {
    viewer_->addPointCloud<pcl::PointXYZRGB>(cloud_show, rgb, cloud_name);
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, cloud_name);
  }
}

void PlaneNormalVisualizer::UpdateCloudRGB(
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, std::string cloud_name,
    int point_size) {
  boost::mutex::scoped_lock lk(mtx_);

  if (cloud->size() == 0) {
    AINFO << ">>>>>>> no points <<<<<<<";
    return;
  }
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
      cloud);
  if (!viewer_->updatePointCloud(cloud, rgb, cloud_name)) {
    viewer_->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, cloud_name);
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, cloud_name);
  }
}

void PlaneNormalVisualizer::UpdateCloudAndNormals(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
    pcl::PointCloud<pcl::Normal>::ConstPtr normals, int ds_ratio,
    std::string cloud_name, std::string normals_name,
    std::vector<double> cloud_color, std::vector<double> normals_color) {
  boost::mutex::scoped_lock lk(mtx_);

  //  AINFO << ">>>>>>> update <<<<<<<";
  //  AINFO << cloud->size();
  //  AINFO << normals->size();

  if (cloud->size() == 0 || normals->size() == 0) {
    AINFO << ">>>>>>> no points <<<<<<<";
    return;
  }

  if (!viewer_->updatePointCloud(cloud, cloud_name)) {
    viewer_->addPointCloud<pcl::PointXYZ>(cloud, cloud_name);
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud_name);
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, cloud_color[0],
        cloud_color[1], cloud_color[2], cloud_name);
  }
  viewer_->removePointCloud(normals_name, 0);
  viewer_->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(
      cloud, normals, ds_ratio, 0.5, normals_name);
  viewer_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_COLOR, normals_color[0],
      normals_color[1], normals_color[2], normals_name);
}

void PlaneNormalVisualizer::UpdateLines(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1,
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2,
    std::vector<double> line_color) {
  boost::mutex::scoped_lock lk(mtx_);

  //  AINFO << ">>>>>>> update <<<<<<<";

  int num_cloud1 = cloud1->size();
  int num_cloud2 = cloud2->size();
  if (num_cloud1 == 0 || num_cloud2 == 0 || num_cloud1 != num_cloud2) {
    AINFO << ">>>>>>> no points or sizes are not the same <<<<<<<";
    LOG_IF(INFO, num_cloud1 != num_cloud2)
        << num_cloud1 << " != " << num_cloud2;
    return;
  }

  for (const auto line_name : line_names_) {
    viewer_->removeShape(line_name);
  }

  line_names_.clear();

  for (int i = 0; i < num_cloud1; ++i) {
    std::stringstream line_name_ss;
    line_name_ss << "line" << i;
    std::string line_name = line_name_ss.str();
    viewer_->addLine(cloud1->at(i), cloud2->at(i), line_name);
    viewer_->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, line_color[0], line_color[1],
        line_color[2], line_name);
    line_names_.push_back(line_name);
  }
}

void PlaneNormalVisualizer::UpdatePlanes(
    const std::vector<Eigen::Vector4d,
                      Eigen::aligned_allocator<Eigen::Vector4d>>
        &plane_coeffs) {
  boost::mutex::scoped_lock lk(mtx_);

  //  AINFO << ">>>>>>> update <<<<<<<";

  int size = plane_coeffs.size();
  if (size == 0) {
    AINFO << ">>>>>>> no planes <<<<<<<";
    return;
  }

  for (const auto plane_name : plane_names_) {
    viewer_->removeShape(plane_name);
  }

  plane_names_.clear();

  for (int i = 0; i < size; ++i) {
    Eigen::Vector4d coeffs_eigen = plane_coeffs[i];
    std::stringstream plane_name_ss;
    plane_name_ss << "plane" << i;
    std::string plane_name = plane_name_ss.str();
    pcl::ModelCoefficients coeffs;
    coeffs.values.push_back(coeffs_eigen.x());
    coeffs.values.push_back(coeffs_eigen.y());
    coeffs.values.push_back(coeffs_eigen.z());
    coeffs.values.push_back(coeffs_eigen.w());
    viewer_->addPlane(coeffs, plane_name);
    viewer_->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.01, plane_name);
    plane_names_.push_back(plane_name);
  }
}

PlaneNormalVisualizer::PlaneNormalVisualizer() {
  //  boost::mutex::scoped_lock lk(mtx_);
  //  viewer_ = boost::make_shared<pcl::visualization::PCLVisualizer>("3D Debug
  //  Viewer"); viewer_->setBackgroundColor(0, 0, 0);
  //  viewer_->addCoordinateSystem(1.0);
  //  viewer_->addText("debugger by Kitkat7", 10, 10, "debugger text", 0);
  //  viewer_->initCameraParameters();
  //  init_ = true;
}
void PlaneNormalVisualizer::UpdateCam(const Eigen::Matrix3d &rotation,
                                      const Eigen::Vector3d &pose, int v1) {
  // Eigen::Matrix3d rotation = current_transform.Rotation;
  Eigen::Vector3d cam_pose = rotation * Eigen::Vector3d(-50, 0, 15);

  cam_.pos[0] = pose[0] + cam_pose[0];
  cam_.pos[1] = pose[1] + cam_pose[1];
  cam_.pos[2] = pose[2] - cam_pose[2];
  cam_.focal[0] = pose[0];
  cam_.focal[1] = pose[1];
  cam_.focal[2] = pose[2];
  //  0.286992, -0.0497761, 0.956639
  cam_.view[0] = 0.270176;  //-0.260317;
  cam_.view[1] = 0.270176;  // 0.0964286;
  cam_.view[2] = -0.923639; //-0.960696;
  cam_.clip[0] = 0.29291;
  cam_.clip[1] = 292.91;
  cam_.fovy = 0.785398;
  cam_.window_size[0] = 1855;
  cam_.window_size[1] = 1056;
  cam_.window_pos[0] = 0;
  cam_.window_pos[1] = 0;

  viewer_->setCameraParameters(cam_, v1);
  // viewer_->addCoordinateSystem(1.0, pose(0), pose(1), pose(2));
}

void PlaneNormalVisualizer::Spin(int stop_flag, std::string name) {
  if (!init_) {
    name_ = name;
    Init();
  }
  if (stop_flag != 0)
    key_value_ = static_cast<ControlCmd>(stop_flag);
  while (!viewer_->wasStopped()) {
    {
      boost::mutex::scoped_lock lk(mtx_);
      viewer_->spinOnce(1, true);
    }
    boost::this_thread::sleep(boost::posix_time::microseconds(1));
    if (key_value_ == ControlCmd::ONE_STEP) {
      key_value_ = ControlCmd::STOP;
      break;
    }
    if (key_value_ == ControlCmd::OTHER) {
      key_value_ = ControlCmd::STOP;
    }
    if (key_value_ == ControlCmd::CONTINUE) {
      break;
    }
  }
}

void PlaneNormalVisualizer::Init() {
  boost::mutex::scoped_lock lk(mtx_);
  viewer_.reset(new pcl::visualization::PCLVisualizer(name_));
  viewer_->setBackgroundColor(0, 0, 0);
  viewer_->addCoordinateSystem(1.0);
  viewer_->addText("Visualizer", 10, 10, "debugger text", 0);
  viewer_->initCameraParameters();
  viewer_->registerKeyboardCallback(keyboardEventOccurred,
                                    reinterpret_cast<void *>(viewer_.get()));
  init_ = true;
}

} // namespace base::viewer
