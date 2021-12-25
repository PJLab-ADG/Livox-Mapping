#ifndef PCL_UTILS_HPP
#define PCL_UTILS_HPP

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
namespace livox_slam_ware {
/*
 * @brief Get the plane coeffs.
 */
template <typename PointType>
Eigen::VectorXf get_plane_coeffs (typename pcl::PointCloud<PointType>::Ptr lidar_cloud) {
  Eigen::VectorXf coeff;
  typename pcl::PointCloud<PointType>::Ptr ground_cloud;
  ground_cloud.reset(new pcl::PointCloud<PointType>);
  for(const auto& p : lidar_cloud->points){
    if(std::fabs(p.normal_y + 1.0) < 1e-5) {
      ground_cloud->push_back(p);
    }
  }
  typename pcl::SampleConsensusModelPlane<PointType>::Ptr model(
    new pcl::SampleConsensusModelPlane<PointType>(ground_cloud));//定义待拟合平面的model，并使用待拟合点云初始化
  pcl::RandomSampleConsensus<PointType> ransac(model);//定义RANSAC算法模型
  ransac.setDistanceThreshold(0.05);//设定阈值
  ransac.computeModel();//拟合
  ransac.getModelCoefficients(coeff);//获取拟合平面参数，对于平面ax+by_cz_d=0，coeff分别按顺序保存a,b,c,d
  // make the normal upward
  // 法向量颠倒个方向
  if(coeff.head<3>().dot(Eigen::Vector3f::UnitZ()) < 0.0f) {
    coeff *= -1.0f;
  }
  return coeff;
}
}

#endif // PCL_UTILS_HPP