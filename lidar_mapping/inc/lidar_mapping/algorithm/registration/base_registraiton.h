/*
 * @Descripttion:
 * @version:
 * @Author: max.zhong
 * @Date: 2021-08-12 01:17:08
 * @LastEditors: max.zhong
 * @LastEditTime: 2021-10-14 01:57:56
 */
#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/registration.h>

namespace mal {
enum class LSQ_OPTIMIZER_TYPE { GaussNewton, LevenbergMarquardt };

template <typename PointSource, typename PointTarget>
class BaseRegistraiton
    : public pcl::Registration<PointSource, PointTarget, float> {
 public:
  using Scalar = float;
  using Matrix4 =
      typename pcl::Registration<PointSource, PointTarget, Scalar>::Matrix4;

  using PointCloudSource = typename pcl::Registration<PointSource, PointTarget,
                                                      Scalar>::PointCloudSource;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

  using PointCloudTarget = typename pcl::Registration<PointSource, PointTarget,
                                                      Scalar>::PointCloudTarget;
  using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
  using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

#if PCL_VERSION >= PCL_VERSION_CALC(1, 10, 0)
  using Ptr = pcl::shared_ptr<LsqRegistration<PointSource, PointTarget>>;
  using ConstPtr =
      pcl::shared_ptr<const LsqRegistration<PointSource, PointTarget>>;
#else
  using Ptr = boost::shared_ptr<LsqRegistration<PointSource, PointTarget>>;
  using ConstPtr =
      boost::shared_ptr<const LsqRegistration<PointSource, PointTarget>>;
#endif

 protected:
  using pcl::Registration<PointSource, PointTarget, Scalar>::input_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::nr_iterations_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::max_iterations_;
  using pcl::Registration<PointSource, PointTarget,
                          Scalar>::final_transformation_;
  using pcl::Registration<PointSource, PointTarget,
                          Scalar>::transformation_epsilon_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::converged_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BaseRegistraiton();
  virtual ~BaseRegistraiton();

}
}  // namespace mal
