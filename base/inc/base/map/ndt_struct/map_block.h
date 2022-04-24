/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-12-28 00:55:40
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-12-28 02:47:50
 */
#pragma once

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
//
#include <Eigen/Eigenvalues>
//
#include "base/map/ndt_struct/map_block_index.h"
#include "base/map/ndt_struct/map_config.h"
namespace base::map {

/**@brief The data structure of a single ndt map grid. */
class SingleGrid {
 public:
  SingleGrid(/* args */);
  ~SingleGrid() = default;
  
  void Reset();
  /**@brief Overloading the assign operator. */
  SingleGrid& operator=(const SingleGrid& ref);

  /**@brief Combine two SingleGrid instances (Reduce). */
  static void Reduce(SingleGrid* grid, const SingleGrid& grid_new);

  /**@brief Add an sample to the single 3d map grid. */
  inline void AddSample(const double& update_time_s, const float& intensity,
                        const float& altitude, const Eigen::Vector3f& centroid,
                        const bool& is_road);
  inline void AddSample(const double& update_time_s, const float& intensity,
                        const float& altitude,
                        const Eigen::Vector3f& centroid) {
    AddSample(update_time_s, intensity, altitude, centroid, false);
  }
  /**@brief Merge two grids. */
  inline void MergeGrid(const double& update_time_s, const float& intensity,
                        const float& intensity_var,
                        const uint32_t& road_pt_count, const uint32_t& count,
                        const Eigen::Vector3f& centroid,
                        const Eigen::Matrix3f& centroid_cov);

  inline void MergeGrid(const SingleGrid& grid_new);

  inline void CentroidEigenSolver(const Eigen::Matrix3f& centroid_cov);

 private:
  /**
   * @brief
   *  the last update timestamp in second.
   */
  double update_time_s_;
  /**@brief The average intensity value. */
  float intensity_ = 0;
  /**@brief The variance intensity value. */
  float intensity_var_ = 0;
  /**@brief The number of samples belonging to road surface. */
  uint32_t road_pt_count_ = 0;
  /**@brief The number of samples in the grid. */
  uint32_t count_ = 0;

  /**@brief the centroid of the grid. */
  Eigen::Vector3f centroid_;
  /**@brief the pose covariance of the grid. */
  Eigen::Matrix3f centroid_average_cov_;
  /**@brief the pose inverse covariance of the grid. */
  Eigen::Matrix3f centroid_icov_;
  /**@brief the inverse covariance available flag. */
  bool is_icov_available_ = false;
  /**@brief minimum number of points needed. */
  const uint32_t minimum_points_threshold_ = 6;
};

/**@brief The data structure of ndt Map grid. */
class MapGrids {
 public:
  /**@brief The default constructor. */
  MapGrids();
  /**@brief Reset to default value. */
  void Reset();
  /**@brief Add an sample. */
  int AddSample(const double& update_time_s, const float& intensity,
                const float& altitude, const float& resolution,
                const Eigen::Vector3f& centroid, const bool& is_road);
  int AddSample(const double& update_time_s, const float& intensity,
                const float& altitude, const float& resolution,
                const Eigen::Vector3f& centroid) {
    return AddSample(update_time_s, intensity, altitude, resolution, centroid,
                     false);
  }
  /**@brief Calculate altitude index from altitude. */
  static int32_t CalAltitudeIndex(const float& resolution,
                                  const float& altitude);

  /**@brief Calculate altitude from altitude index. */
  static float CalAltitude(const float& resolution,
                           const int32_t& altitude_index);

  /**@brief Combine two MapGrid instances (Reduce). */
  static void Reduce(MapGrids* grid, const MapGrids& grid_new);

 public:
  /**@brief The multiple altitudes of the grid. */
  std::unordered_map<int32_t, SingleGrid> grids_;
  /**@brief The index of biggest altitude. */
  int32_t max_altitude_index_;
  /**@brief The index of smallest altitude. */
  int32_t min_altitude_index_;
  /**@brief The indices of road surface. */
  std::vector<int32_t> road_grid_indices_;
};

class MapBlock {
 public:
  MapBlock();
  MapBlock(const MapBlock& grids);
  ~MapBlock();

  void Init(const MapConfig* config);
  void Reset(const MapConfig* config);

  /**@brief Initialize the matrix with the size of rows and columns. */
  void Init(uint32_t rows, uint32_t cols);
  void Init(const MapConfig* map_config, const MapBlockIndex& index,
            bool create_map_grids);
  /**@brief Reset the matrix item to default value. */
  void Reset(uint32_t rows, uint32_t cols);

  void Clear();

  /**@brief Set if the map block is reserved. */
  inline void SetIsReserved(bool is_reserved) { is_reserved_ = is_reserved; }
  /**@brief Get if the map block is reserved. */
  inline bool GetIsReserved() const { return is_reserved_; }
  /**@brief Get if the map data has changed. */
  inline bool GetIsChanged() const { return is_changed_; }
  /**@brief Set if the map block data has changed. */
  inline void SetIsChanged(bool is) { is_changed_ = is; }
  /**@brief Get if the map block data is ready*/
  inline bool GetIsReady() const { return data_is_ready_; }

  /**@brief Get a const map grid. */
  inline const MapGrids& GetMapGrid(uint32_t row, uint32_t col) const {
    assert(row < rows_);
    assert(col < cols_);
    return map3d_grids_[row * cols_ + col];
  }
  /**@brief Get a map grid. */
  inline MapGrids& GetMapGrid(uint32_t row, uint32_t col) {
    assert(row < rows_);
    assert(col < cols_);
    return map3d_grids_[row * cols_ + col];
  }
  /**@brief Get the size of row. */
  uint32_t GetRows() const { return rows_; }
  /**@brief Get the size of cols. */
  uint32_t GetCols() const { return cols_; }

  inline const Eigen::Vector2d& GetLeftTopCorner() const {
    return left_top_corner_;
  }

  inline const Eigen::Vector2d& GetRightDownCorner() const {
    return right_down_corner_;
  }

  inline void SetLeftTopCorner(double x, double y) {
    left_top_corner_[0] = x;
    left_top_corner_[1] = y;
  }

  /**@brief Given the global coordinate, get the local 2D coordinate of the map
   * cell matrix. <return> If global coordinate (x, y) belongs to this map node,
   * eigen version. */
  bool GetCoordinate(const Eigen::Vector2d& coordinate, uint32_t* x,
                     uint32_t* y) const;
  bool GetCoordinate(const Eigen::Vector3d& coordinate, uint32_t* x,
                     uint32_t* y) const;
  Eigen::Vector2d GetCoordinate(uint32_t x, uint32_t y) const;

  /**@brief Get the resolution of this map nodex. */
  inline float GetMapResolution() const {
    return this->map_config_->map_resolutions_[this->index_.resolution_id_];
  }

  void SetMapBlockIndex(const MapBlockIndex& index);

  Eigen::Vector2d ComputeLeftTopCorner(const MapConfig& config,
                                       const MapBlockIndex& index);
  Eigen::Vector2d ComputeRightDownCorner(const MapConfig& config,
                                         const MapBlockIndex& index);

  /**@brief Combine two MapBlock instances (Reduce). */
  static void Reduce(MapBlock* grids, const MapBlock& grids_new);

 private:
  /**@brief If the block is reserved in map. */
  bool is_reserved_ = false;
  /**@brief Has the map block been changed. */
  bool is_changed_ = false;
  /* *@brief Indicate map block data is ready*/
  bool data_is_ready_ = false;
  /**@brief The number of rows. */
  uint32_t rows_;
  /**@brief The number of columns. */
  uint32_t cols_;

  /**@brief The map settings. */
  const MapConfig* map_config_ = nullptr;

  /**@brief The index of this node*/
  MapBlockIndex index_;

  /**@brief The left top corner of the map node in the global coordinate system.
   */
  Eigen::Vector2d left_top_corner_;
  Eigen::Vector2d right_down_corner_;

  /**@brief The matrix data structure. */
  std::unique_ptr<MapGrids[]> map3d_grids_;
};

}  // namespace base::map
