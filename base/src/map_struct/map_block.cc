/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-12-28 02:16:07
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-28 02:49:36
 */
#include "base/map_struct/map_block.h"

namespace base::map {

SingleGrid::SingleGrid() { Reset(); }

void SingleGrid::Reset() {
  update_time_s_ = 0.0;
  intensity_ = 0.0;
  intensity_var_ = 0.0;
  road_pt_count_ = 0;
  count_ = 0;
  centroid_ = Eigen::Vector3f::Zero();
  centroid_average_cov_ = Eigen::Matrix3f::Zero();
  centroid_icov_ = Eigen::Matrix3f::Identity();
  is_icov_available_ = false;
}

void SingleGrid::AddSample(const double& update_time_s, const float& intensity,
                           const float& altitude,
                           const Eigen::Vector3f& centroid,
                           const bool& is_road) {
  update_time_s_ = std::max(update_time_s, update_time_s_);
  ++count_;
  float v1 = intensity - intensity_;
  intensity_ += v1 / static_cast<float>(count_);
  float v2 = intensity - intensity_;
  intensity_var_ = (static_cast<float>(count_ - 1) * intensity_var_ + v1 * v2) /
                   static_cast<float>(count_);

  if (is_road) {
    ++road_pt_count_;
  }

  Eigen::Vector3f v3 = centroid - centroid_;
  centroid_ += v3 / static_cast<float>(count_);
  Eigen::Matrix3f v4 = centroid * centroid.transpose() - centroid_average_cov_;
  centroid_average_cov_ += v4 / static_cast<float>(count_);
  CentroidEigenSolver(centroid_average_cov_);
}

void SingleGrid::MergeGrid(const double& update_time_s, const float& intensity,
                           const float& intensity_var,
                           const uint32_t& road_pt_count, const uint32_t& count,
                           const Eigen::Vector3f& centroid,
                           const Eigen::Matrix3f& centroid_cov) {
  update_time_s_ = std::max(update_time_s, update_time_s_);
  uint32_t new_count = count_ + count;
  float p0 = static_cast<float>(count_) / static_cast<float>(new_count);
  float p1 = static_cast<float>(count) / static_cast<float>(new_count);
  float intensity_diff = intensity_ - intensity;

  intensity_ = intensity_ * p0 + intensity * p1;
  intensity_var_ = intensity_var_ * p0 + intensity_var * p1 +
                   intensity_diff * intensity_diff * p0 * p1;

  centroid_[0] = centroid_[0] * p0 + centroid[0] * p1;
  centroid_[1] = centroid_[1] * p0 + centroid[1] * p1;
  centroid_[2] = centroid_[2] * p0 + centroid[2] * p1;

  count_ = new_count;
  road_pt_count_ += road_pt_count;
}

void SingleGrid::MergeGrid(const SingleGrid& grid_new) {
  MergeGrid(grid_new.update_time_s_, grid_new.intensity_,
            grid_new.intensity_var_, grid_new.road_pt_count_, grid_new.count_,
            grid_new.centroid_, grid_new.centroid_average_cov_);
}

void SingleGrid::CentroidEigenSolver(const Eigen::Matrix3f& centroid_cov) {
  // Contain more than five points, we calculate the eigen vector/value of cov.
  // [Magnusson 2009]
  if (count_ >= minimum_points_threshold_) {
    Eigen::Matrix3f cov = centroid_cov - centroid_ * centroid_.transpose();
    cov *= static_cast<float>((count_ - 1.0) / count_);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver;
    eigen_solver.compute(cov);
    Eigen::Matrix3f eigen_val = eigen_solver.eigenvalues().asDiagonal();
    Eigen::Matrix3f centroid_evecs = eigen_solver.eigenvectors();
    if (eigen_val(0, 0) < 0 || eigen_val(1, 1) < 0 || eigen_val(2, 2) <= 0) {
      is_icov_available_ = 0;
      return;
    }
    // Avoid matrices near singularities (eq 6.11) [Magnusson 2009].
    float min_covar_eigvalue = 0.01f * eigen_val(2, 2);
    if (eigen_val(0, 0) < min_covar_eigvalue) {
      eigen_val(0, 0) = min_covar_eigvalue;
      if (eigen_val(1, 1) < min_covar_eigvalue) {
        eigen_val(1, 1) = min_covar_eigvalue;
      }
    }
    // Calculate inverse covariance
    centroid_icov_ =
        (centroid_evecs * eigen_val * centroid_evecs.inverse()).inverse();
    if (centroid_icov_.maxCoeff() == std::numeric_limits<float>::infinity() ||
        centroid_icov_.minCoeff() == -std::numeric_limits<float>::infinity()) {
      is_icov_available_ = false;
      return;
    }
    // Set icov available
    is_icov_available_ = true;
  }
}

SingleGrid& SingleGrid::operator=(const SingleGrid& ref) {
  update_time_s_ = ref.update_time_s_;
  count_ = ref.count_;
  intensity_ = ref.intensity_;
  intensity_var_ = ref.intensity_var_;
  road_pt_count_ = ref.road_pt_count_;
  centroid_ = ref.centroid_;
  centroid_average_cov_ = ref.centroid_average_cov_;
  centroid_icov_ = ref.centroid_icov_;
  is_icov_available_ = ref.is_icov_available_;
  return *this;
}

void SingleGrid::Reduce(SingleGrid* grid, const SingleGrid& grid_new) {
  grid->MergeGrid(grid_new);
}

MapGrids::MapGrids() {
  max_altitude_index_ = static_cast<int>(-1e10);
  min_altitude_index_ = static_cast<int>(1e10);
}

void MapGrids::Reset() {
  max_altitude_index_ = static_cast<int>(-1e10);
  min_altitude_index_ = static_cast<int>(1e10);
  grids_.clear();
  road_grid_indices_.clear();
}

int MapGrids::AddSample(const double& update_time_s, const float& intensity,
                        const float& altitude, const float& resolution,
                        const Eigen::Vector3f& centroid, const bool& is_road) {
  int altitude_index = CalAltitudeIndex(resolution, altitude);
  SingleGrid& grid = grids_[altitude_index];
  grid.AddSample(update_time_s, intensity, altitude, centroid, is_road);
  if (altitude_index > max_altitude_index_) {
    max_altitude_index_ = altitude_index;
  }
  if (altitude_index < min_altitude_index_) {
    min_altitude_index_ = altitude_index;
  }
  if (is_road) {
    auto got = std::find(road_grid_indices_.begin(), road_grid_indices_.end(),
                         altitude_index);
    if (got == road_grid_indices_.end()) {
      road_grid_indices_.push_back(altitude_index);
    }
  }

  return altitude_index;
}

int32_t MapGrids::CalAltitudeIndex(const float& resolution,
                                   const float& altitude) {
  return static_cast<int32_t>(altitude / resolution);
}

float MapGrids::CalAltitude(const float& resolution,
                            const int32_t& altitude_index) {
  return static_cast<float>(resolution * (static_cast<float>(altitude_index)));
}

void MapGrids::Reduce(MapGrids* grid, const MapGrids& grid_new) {
  // Reduce grids
  for (auto it = grid_new.grids_.begin(); it != grid_new.grids_.end(); ++it) {
    int altitude_index = it->first;
    auto got = grid->grids_.find(altitude_index);
    if (got != grid->grids_.end()) {
      grid->grids_[altitude_index].MergeGrid(it->second);
    } else {
      grid->grids_[altitude_index] = SingleGrid(it->second);
    }
  }

  if (grid_new.max_altitude_index_ > grid->max_altitude_index_) {
    grid->max_altitude_index_ = grid_new.max_altitude_index_;
  }

  if (grid_new.min_altitude_index_ < grid->min_altitude_index_) {
    grid->min_altitude_index_ = grid_new.min_altitude_index_;
  }

  for (auto it_new = grid_new.road_grid_indices_.begin();
       it_new != grid_new.road_grid_indices_.end(); ++it_new) {
    auto got_it = std::find(grid->road_grid_indices_.begin(),
                            grid->road_grid_indices_.end(), *it_new);
    if (got_it != grid->road_grid_indices_.end()) {
      *got_it += *it_new;
    } else {
      grid->road_grid_indices_.push_back(*it_new);
    }
  }
}

MapBlock::MapBlock() {
  rows_ = 0;
  cols_ = 0;
  map3d_grids_ = nullptr;
}
MapBlock::~MapBlock() {}

MapBlock::MapBlock(const MapBlock& grids) {
  Init(grids.rows_, grids.cols_);
  for (uint32_t y = 0; y < rows_; ++y) {
    for (uint32_t x = 0; x < cols_; ++x) {
      MapGrids& grid = GetMapGrid(y, x);
      const MapGrids& src_grid = grids.GetMapGrid(y, x);
      grid = MapGrids(src_grid);
    }
  }
}

void MapBlock::Init(const MapConfig* map_config, const MapBlockIndex& index,
                    bool create_map_grids) {
  map_config_ = map_config;
  index_ = index;
  left_top_corner_ = ComputeLeftTopCorner(*map_config_, index_);
  is_reserved_ = false;
  data_is_ready_ = false;
  is_changed_ = false;
  if (create_map_grids) {
    Init(map_config_);
  }
  return;
}
void MapBlock::Init(const MapConfig* config) {
  map_config_ = config;
  is_reserved_ = false;
  data_is_ready_ = false;
  is_changed_ = false;

  Init(config->map_block_size_y_, config->map_block_size_x_);

  return;
}

void MapBlock::Reset(const MapConfig* config) {
  Reset(config->map_block_size_y_, config->map_block_size_x_);
}

void MapBlock::Init(uint32_t rows, uint32_t cols) {
  map3d_grids_.reset(new MapGrids[rows * cols]);
  rows_ = rows;
  cols_ = cols;
}

void MapBlock::Clear() {
  map3d_grids_ = nullptr;
  map_config_ = nullptr;
}

void MapBlock::Reset(uint32_t rows, uint32_t cols) {
  uint32_t length = rows * cols;
  for (uint32_t i = 0; i < length; ++i) {
    map3d_grids_[i].Reset();
  }
}

Eigen::Vector2d MapBlock::ComputeLeftTopCorner(const MapConfig& config,
                                               const MapBlockIndex& index) {
  Eigen::Vector2d coord;
  coord[0] = config.map_range_.GetMinX() +
             static_cast<float>(config.map_block_size_x_) *
                 config.map_resolutions_[index.resolution_id_] *
                 static_cast<float>(index.n_);
  coord[1] = config.map_range_.GetMinY() +
             static_cast<float>(config.map_block_size_y_) *
                 config.map_resolutions_[index.resolution_id_] *
                 static_cast<float>(index.m_);
  if (coord[0] >= config.map_range_.GetMaxX()) {
    throw "[MapBlock::ComputeLeftTopCorner] coord[0]"
                " >= config.map_range_.GetMaxX()";
  }
  if (coord[1] >= config.map_range_.GetMaxY()) {
    throw "[MapBlock::compute_left_top_corner] coord[1]"
                " >= config.map_range_.GetMaxY()";
  }
  return coord;
}

Eigen::Vector2d MapBlock::ComputeRightDownCorner(const MapConfig& config,
                                                 const MapBlockIndex& index) {
  Eigen::Vector2d coord;
  coord[0] = config.map_range_.GetMinX() +
             static_cast<float>(config.map_block_size_x_) *
                 config.map_resolutions_[index.resolution_id_] *
                 static_cast<float>(index.n_ + 1);
  coord[1] = config.map_range_.GetMinY() +
             static_cast<float>(config.map_block_size_y_) *
                 config.map_resolutions_[index.resolution_id_] *
                 static_cast<float>(index.m_ + 1);
  if (coord[0] >= config.map_range_.GetMaxX()) {
    throw "[MapBlock::ComputeLeftTopCorner] coord[0]"
                " >= config.map_range_.GetMaxX()";
  }
  if (coord[1] >= config.map_range_.GetMaxY()) {
    throw "[MapBlock::compute_left_top_corner] coord[1]"
                " >= config.map_range_.GetMaxY()";
  }
  return coord;
}

void MapBlock::SetMapBlockIndex(const MapBlockIndex& index) {
  index_ = index;
  left_top_corner_ = ComputeLeftTopCorner(*map_config_, index_);
  right_down_corner_ = ComputeRightDownCorner(*map_config_, index_);
}

void MapBlock::Reduce(MapBlock* grids, const MapBlock& grids_new) {
  for (uint32_t y = 0; y < grids->GetRows(); ++y) {
    for (uint32_t x = 0; x < grids->GetCols(); ++x) {
      MapGrids& grid = grids->GetMapGrid(y, x);
      const MapGrids& grid_new = grids_new.GetMapGrid(y, x);
      MapGrids::Reduce(&grid, grid_new);
    }
  }
}

bool MapBlock::GetCoordinate(const Eigen::Vector2d& coordinate, uint32_t* x,
                             uint32_t* y) const {
  const Eigen::Vector2d& left_top_corner = GetLeftTopCorner();
  int off_x = static_cast<int>((coordinate[0] - left_top_corner[0]) /
                               GetMapResolution());
  int off_y = static_cast<int>((coordinate[1] - left_top_corner[1]) /
                               GetMapResolution());
  if (off_x >= 0 &&
      off_x < static_cast<int>(this->map_config_->map_block_size_x_) &&
      off_y >= 0 &&
      off_y < static_cast<int>(this->map_config_->map_block_size_y_)) {
    *x = static_cast<uint32_t>(off_x);
    *y = static_cast<uint32_t>(off_y);
    return true;
  } else {
    return false;
  }
}

bool MapBlock::GetCoordinate(const Eigen::Vector3d& coordinate, uint32_t* x,
                             uint32_t* y) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  return GetCoordinate(coord2d, x, y);
}

Eigen::Vector2d MapBlock::GetCoordinate(uint32_t x, uint32_t y) const {
  const Eigen::Vector2d& left_top_corner = GetLeftTopCorner();
  Eigen::Vector2d coord(
      left_top_corner[0] + static_cast<float>(x) * GetMapResolution(),
      left_top_corner[1] + static_cast<float>(y) * GetMapResolution());
  return coord;
}

}  // namespace base::map
