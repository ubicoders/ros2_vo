#ifndef __BACK_END_H__
#define __BACK_END_H__

#include "stereo_visual_slam/map.hpp"
#include "stereo_visual_slam/pinhole_camera.hpp"
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

// Select the linear solver library
// Use CSparse since it's what's available in CMakeLists.txt (g2o_solver_csparse
// is linked)
G2O_USE_OPTIMIZATION_LIBRARY(csparse);
// G2O_USE_OPTIMIZATION_LIBRARY(dense);

namespace StereoSLAM {
class Backend {

public:
  Backend(std::shared_ptr<PinholeCamera> camera, std::shared_ptr<Map> map);
  void updateMap();
  void optimize(Map::KeyFrameType &activeKeyframes,
                Map::MapPointType &activeMapPoints);

private:
  std::shared_ptr<PinholeCamera> camera_;
  std::shared_ptr<Map> map_;

  std::unique_ptr<g2o::SparseOptimizer> optimizer_;

  std::vector<g2o::VertexSE3Expmap *> poseVertexs_;
  std::vector<g2o::VertexPointXYZ *> pointVertexs_;
};
} // namespace StereoSLAM

#endif // __BACK_END_H__