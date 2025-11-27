# ECS Migration Plan for ROS2-Stereo-Visual-Odometry-G2O

## 1. Understanding ECS (Entity Component System)

**Entity Component System (ECS)** is an architectural pattern mostly used in game development (e.g., Unity DOTS, Bevy Engine) to improve performance and code decoupling. It favors **Composition over Inheritance** and **Data-Oriented Design**.

- **Entity**: A unique identifier (ID). It does not hold data or behavior itself. It's just a key to group components.
- **Component**: A pure data container (struct). It has no methods (logic). It holds the state of an aspect of an entity (e.g., `Position`, `Velocity`).
- **System**: The logic. It runs on groups of entities that have a specific set of components. It reads/writes data in components.

### Key Differences from OOP

| Feature           | OOP (Current Project)                                   | ECS (Proposed)                                           |
| :---------------- | :------------------------------------------------------ | :------------------------------------------------------- |
| **Data & Logic**  | Encapsulated together in Classes (`Frame`, `MapPoint`). | Separated. Data in Components, Logic in Systems.         |
| **Relationships** | Pointers/References (`std::shared_ptr<Feature>`).       | Entity IDs (Integers).                                   |
| **Memory**        | Objects scattered in heap (cache misses).               | Components packed in contiguous arrays (cache friendly). |
| **Polymorphism**  | Virtual functions, Inheritance.                         | Composition of components.                               |

---

## 2. Current Architecture Analysis (OOP)

The current `ROS2-Stereo-Visual-Odometry-G2O` project follows a standard OOP approach:

- **`Frame` Class**:
  - **Data**: `cv::Mat image`, `SE3d pose`, `vector<Feature>`.
  - **Logic**: `setKeyFrame()`, helpers.
- **`Feature` Class**:
  - **Data**: `cv::KeyPoint`, `weak_ptr<Frame>`, `weak_ptr<MapPoint>`.
  - **Logic**: None (mostly a struct).
- **`MapPoint` Class**:
  - **Data**: `Vector3d position`, `vector<weak_ptr<Feature>> observations`.
  - **Logic**: `addObserve()`, `removeObserve()`.
- **Managers (`Frontend`, `Backend`, `Map`)**:
  - These act somewhat like "Systems" but they own the data and manage the lifetime of objects directly.

---

## 3. Proposed ECS Architecture

To migrate to ECS, we decompose these classes into **Entities** (IDs), **Components** (Data), and **Systems** (Logic).

### A. Entities

We identify the distinct "things" in the simulation.

1.  **FrameEntity**: Represents a single captured moment in time.
2.  **FeatureEntity**: Represents a 2D point on an image.
3.  **MapPointEntity**: Represents a 3D point in the world.
4.  **CameraEntity**: Represents the camera hardware/parameters.

### B. Components

We break down the classes into granular data chunks.

#### 1. Core Components

- **`Pose3D`**: `{ Sophus::SE3d T_c2w; }`
  - _Used by:_ `FrameEntity`, `CameraEntity`.
- **`WorldPosition`**: `{ Eigen::Vector3d position; }`
  - _Used by:_ `MapPointEntity`.
- **`ImageResource`**: `{ cv::Mat left, right; }`
  - _Used by:_ `FrameEntity`.

#### 2. Feature & Tracking Components

- **`PixelCoordinate`**: `{ cv::KeyPoint point; bool isLeft; }`
  - _Used by:_ `FeatureEntity`.
- **`Descriptor`**: `{ cv::Mat descriptor; }` (if using ORB/AKAZE).
  - _Used by:_ `FeatureEntity`.
- **`ObservedBy`**: `{ EntityId frameId; }`
  - _Used by:_ `FeatureEntity` (Links feature to its source frame).
- **`Observes`**: `{ EntityId mapPointId; }`
  - _Used by:_ `FeatureEntity` (Links feature to the 3D point it represents).

#### 3. Graph & Optimization Components

- **`ObservationList`**: `{ std::vector<EntityId> featureIds; }`
  - _Used by:_ `MapPointEntity` (Reverse lookups for optimization).
- **`KeyFrameTag`**: `{ }` (Empty struct / Tag)
  - _Used by:_ `FrameEntity` (Marks a frame as a Keyframe).
- **`ActiveTag`**: `{ }`
  - _Used by:_ `FrameEntity`, `MapPointEntity` (Marks them for local bundle adjustment).
- **`OutlierTag`**: `{ }`
  - _Used by:_ `FeatureEntity`, `MapPointEntity` (Marks bad data).

### C. Systems

Logic is moved into systems that run every loop or on specific events.

#### 1. `InputSystem`

- **Role**: Interfaces with ROS2.
- **Action**: Receives images -> Creates a new `FrameEntity` -> Attaches `ImageResource` and initial `Pose3D` (prediction).

#### 2. `FeatureExtractionSystem`

- **Query**: Entities with `ImageResource` but NO `ProcessedTag`.
- **Action**:
  1.  Detect keypoints in `ImageResource`.
  2.  For each keypoint, create a `FeatureEntity`.
  3.  Attach `PixelCoordinate` and `ObservedBy` (pointing to the Frame).

#### 3. `TrackingSystem` (The "Frontend")

- **Query**: Active `FrameEntity` (current) and `MapPointEntity` (local map).
- **Action**:
  1.  Project `MapPointEntity` positions using `Pose3D` of current frame.
  2.  Match with `FeatureEntity` of the current frame.
  3.  Add `Observes` component to matched `FeatureEntity`.
  4.  Solve PnP to update `Pose3D` of the current `FrameEntity`.

#### 4. `MappingSystem`

- **Query**: `FrameEntity` (current).
- **Action**:
  1.  Decide if Keyframe (add `KeyFrameTag`).
  2.  If Keyframe, triangulate new points from unmatched features.
  3.  Create new `MapPointEntity` -> Attach `WorldPosition`.

#### 5. `OptimizationSystem` (The "Backend")

- **Query**: Entities with `ActiveTag` (`FrameEntity` + `MapPointEntity`).
- **Action**:
  1.  Construct G2O graph.
  2.  Vertices = `Pose3D` (Frames) and `WorldPosition` (MapPoints).
  3.  Edges = Derived from `Observes` components on `FeatureEntity`.
  4.  Optimize.
  5.  Write back results to `Pose3D` and `WorldPosition` components.

#### 6. `VisualizationSystem`

- **Query**: All `Pose3D`, `WorldPosition`, `ImageResource`.
- **Action**: Draw to Pangolin/Rviz.

---

## 4. Pros and Cons of ECS for this Project

### Pros

- **Cache Locality**: Iterating over all `FeatureEntity` to compute descriptors or match is much faster because data is contiguous in memory.
- **Decoupling**: The "Backend" doesn't need to know about the "Frame" class. It just asks for anything with a `Pose3D` and `ActiveTag`.
- **Parallelism**: Systems can run in parallel easily. e.g., `FeatureExtractionSystem` for Left and Right images can run on separate threads without mutexes if they write to different component arrays.

### Cons

- **Complexity for Graphs**: SLAM is inherently a graph problem (Hypergraph of Frames and Points). ECS is best for flat lists. Traversing the graph (e.g., "Get all features in this frame, then get all map points they observe, then get all other frames observing those points") can be cumbersome and require joins or auxiliary indices.
- **Overhead**: For a small number of entities (hundreds of keyframes), the ECS overhead might outweigh the benefits compared to simple pointers.
