#include "svo/backend.hpp"

namespace UbiSVO {
Backend::Backend(std::shared_ptr<StereoGeometry> camera,
                 std::shared_ptr<Map> map)
    : camera_(camera), map_(map) {
  optimizer_ = std::make_unique<g2o::SparseOptimizer>();

  // Choose the solver method (same approach as ba_demo.cpp, adapted for
  // available libraries) "lm_fix6_3" means: Levenberg-Marquardt, fixed-size
  // vertices (6 for pose, 3 for point) Using CSparse solver since it's linked
  // in CMakeLists.txt
  std::string solverName = "lm_fix6_3_csparse";
  g2o::OptimizationAlgorithmProperty solverProperty;
  optimizer_->setAlgorithm(
      g2o::OptimizationAlgorithmFactory::instance()->construct(solverName,
                                                               solverProperty));

  optimizer_->setVerbose(true);

  cv::Mat cvCamK = camera_->get_K();

  double focal_length = cvCamK.at<double>(0, 0);
  Eigen::Vector2d principal_point(cvCamK.at<double>(0, 2),
                                  cvCamK.at<double>(1, 2));

  g2o::CameraParameters *cam_params =
      new g2o::CameraParameters(focal_length, principal_point, 0.);
  cam_params->setId(0);

  optimizer_->addParameter(cam_params);
}

void Backend::updateMap() {
  // Retrieve the current "active" set of keyframes and map points from the Map
  // module. This constitutes the "Sliding Window" for Local Bundle Adjustment.
  // - Active Keyframes: Usually the most recent N frames (e.g., 7) to constrain
  // the latest trajectory.
  // - Active Map Points: 3D points observed by these active keyframes.
  auto &activeKeyFrames = map_->getActiveKeyFrames();
  auto &activeMapPoints = map_->getActiveMapPoints();

  if (activeKeyFrames.size() > 2) {
    optimize(activeKeyFrames, activeMapPoints);
  }
}

void Backend::optimize(Map::KeyFrameType &activeKeyframes,
                       Map::MapPointType &activeMapPoints) {
  // --- G2O Optimization Pattern Explanation ---
  // 1. Setup Optimizer: We use a SparseOptimizer with a specific solver
  // (Levenberg-Marquardt).
  // 2. Define Vertices (Nodes):
  //    - VertexSE3Expmap: Represents the Camera Pose (6DOF: Rotation +
  //    Translation).
  //    - VertexPointXYZ: Represents the 3D Map Point (3DOF: X, Y, Z).
  // 3. Define Edges (Constraints):
  //    - EdgeProjectXYZ2UV: Represents the observation of a 3D point in a 2D
  //    image.
  //      It connects a VertexSE3Expmap and a VertexPointXYZ.
  //      The "error" is the reprojection error: difference between projected 3D
  //      point and observed 2D pixel.
  // 4. Optimization: The solver adjusts the Vertices to minimize the total
  // error of all Edges.

  std::unordered_map<int32_t, g2o::VertexSE3Expmap *> vertices;
  std::unordered_map<int32_t, g2o::VertexPointXYZ *> mapPointVertices;
  u_int32_t maxKeyFrameId =
      0; // Track max ID to avoid collision with map point IDs
  u_int32_t edgeSize = 0;

  // Find the two oldest keyframe IDs in the active window to fix as anchor
  std::vector<uint32_t> frameIds;
  for (const auto &[id, frame] : activeKeyframes) {
    frameIds.push_back(id);
  }
  std::sort(frameIds.begin(), frameIds.end());

  // --- Step 1: Add KeyFrame Vertices (Camera Poses) ---
  for (auto &[id, frame] : activeKeyframes) {
    auto pose = frame->T_c2w;
    // Convert Sophus SE3 pose to G2O SE3Quat format
    g2o::SE3Quat g2oPose(pose.rotationMatrix(), pose.translation());

    g2o::VertexSE3Expmap *vertex = new g2o::VertexSE3Expmap();
    vertex->setId(id);
    vertex->setEstimate(g2oPose);

    // Fix the two OLDEST frames in the active window as anchor
    // This provides "Gauge Fixing":
    // In SLAM, the absolute position is arbitrary (gauge freedom).
    // Fixing some frames anchors the local map to a specific coordinate system,
    // preventing the solution from drifting indefinitely or floating around.
    if (frameIds.size() >= 2 && (id == frameIds[0] || id == frameIds[1])) {
      vertex->setFixed(true);
    }

    optimizer_->addVertex(vertex);
    vertices.insert({id, vertex});
    maxKeyFrameId = std::max(maxKeyFrameId, id);
  }

  // --- Step 2: Add MapPoint Vertices (3D Points) ---
  for (auto &[id, mapPoint] : activeMapPoints) {
    // ID Offset: We add 'maxKeyFrameId + 1' to the point ID to ensure unique
    // IDs in the graph (since G2O requires unique IDs for all vertices,
    // regardless of type).
    u_int32_t mapPointId = (maxKeyFrameId + 1) + id;

    g2o::VertexPointXYZ *vertex = new g2o::VertexPointXYZ();
    vertex->setId(mapPointId);

    // Marginalization: We set points to be marginalized.
    // This tells the solver to use the Schur Complement trick.
    // Since points only connect to frames (and not other points), the Hessian
    // matrix has a sparse structure. Marginalizing points allows us to solve a
    // much smaller system (only poses) first, then back-substitute for points.
    vertex->setMarginalized(true);
    vertex->setEstimate(mapPoint->getWorldPoint());

    optimizer_->addVertex(vertex);
    mapPointVertices.insert({id, vertex});

    if (mapPoint->getObserve().size() >= 2) {
      for (auto &observe : mapPoint->getObserve()) {
        if (observe.lock() == nullptr) {
          continue;
        }
        auto feature = observe.lock();
        if (!feature->isInlier || feature->framePtr.lock() == nullptr) {
          continue;
        }

        auto keyFrame = feature->framePtr.lock();

        auto landmarkId = id;
        auto keyFrameId = keyFrame->getFrameId();

        if (mapPointVertices.find(landmarkId) != mapPointVertices.end() &&
            vertices.find(keyFrameId) != vertices.end()) {

          // --- Step 3: Add Edges (Constraints) ---
          // Verify vertices exist in optimizer
          auto mapPointVertex = optimizer_->vertex(mapPointId);
          auto poseVertexIt = optimizer_->vertices().find(keyFrameId);

          if (mapPointVertex == nullptr ||
              poseVertexIt == optimizer_->vertices().end()) {
            continue;
          }

          // Create the edge connecting the Map Point and the KeyFrame
          g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
          g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
          Eigen::Vector2d measure = Eigen::Vector2d::Zero();

          // Measurement: The actual 2D pixel coordinates (u, v) observed in the
          // image
          measure(0) = feature->point.pt.x;
          measure(1) = feature->point.pt.y;

          edge->setId(edgeSize);
          edge->setVertex(
              0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(mapPointVertex));
          edge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                 poseVertexIt->second));
          edge->setMeasurement(measure);
          edge->setInformation(Eigen::Matrix2d::Identity());

          // Robust Kernel (Huber): Reduces the influence of outliers (wrong
          // matches) on the optimization If the error is large, the kernel
          // reduces the weight of this edge.
          edge->setRobustKernel(rk);

          edge->setParameterId(0, 0); // Set camera parameters (ID 0)
          optimizer_->addEdge(edge);

          ++edgeSize;
        }
      }
    }
  }

  std::cout << "Backend: Starting optimization with " << activeKeyframes.size()
            << " active keyframes (IDs: " << frameIds.front() << " to "
            << frameIds.back() << "), fixing frames " << frameIds[0] << " and "
            << (frameIds.size() >= 2 ? frameIds[1] : 0) << std::endl;
  std::cout << "Backend: " << mapPointVertices.size() << " map points, "
            << edgeSize << " edges" << std::endl;

  if (!optimizer_->initializeOptimization()) {
    std::cerr << "Backend: Failed to initialize optimization!" << std::endl;
    optimizer_->clear();
    return;
  }

  optimizer_->setVerbose(false);

  std::cout << "Backend: Running optimization..." << std::endl;
  optimizer_->optimize(10); // Reduced from 30 to 10 iterations for local BA
  std::cout << "Backend: Optimization completed successfully" << std::endl;

  // --- Step 4: Update Data Structures with Optimized Results ---

  // Update KeyFrame Poses
  for (const auto &[id, vertex] : vertices) {
    const auto &poseEstimate = vertex->estimate();
    Eigen::Quaterniond q = poseEstimate.rotation();
    Eigen::Vector3d t = poseEstimate.translation();

    // Write back the optimized pose to the KeyFrame object
    activeKeyframes[id]->T_c2w = Sophus::SE3d(q, t);
    activeKeyframes[id]->needUpdate =
        true; // Flag to notify other modules (e.g., Viewer)
  }

  // Update Map Point Positions
  for (const auto &[id, mapPointVertex] : mapPointVertices) {
    const auto &posEstimate = mapPointVertex->estimate();
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    pos(0) = posEstimate(0);
    pos(1) = posEstimate(1);
    pos(2) = posEstimate(2);

    // Write back the optimized position to the MapPoint object
    activeMapPoints[id]->setWorldPoint(pos);
  }

  map_->setRequiredViewerUpdated(true);
  optimizer_->clear();
}

} // namespace UbiSVO