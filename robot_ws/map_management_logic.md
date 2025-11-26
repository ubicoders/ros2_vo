# Map Management Logic

This document explains the lifecycle of **Keyframes** and **MapPoints** in the Stereo Visual Odometry system, specifically how they are added to and removed from the active map (sliding window).

## 1. Adding a Keyframe & MapPoints

When the Frontend decides to create a new Keyframe (e.g., enough movement occurred):

1.  **Create Keyframe**: The current frame is promoted to a Keyframe.
2.  **Create MapPoints**: New 3D points are triangulated from stereo matches.
3.  **Linkage (via Feature)**:
    - A `Feature` object is created to represent the 2D keypoint.
    - The `Feature` stores a pointer to the `MapPoint`.
    - The `MapPoint` adds the `Feature` as an observation.
    - The `Keyframe` stores the `Feature`.
4.  **Add to Map**:
    - `Map::addKeyframe()` adds the frame to the active window.
    - `Map::addMapPoint()` adds the new points to the active map.

```mermaid
sequenceDiagram
    participant Frontend
    participant Map
    participant Keyframe
    participant Feature
    participant MapPoint

    Note over Frontend: 1. Create New Keyframe
    Frontend->>Keyframe: setKeyFrame()

    Note over Frontend: 2. Create New MapPoints
    loop For each stereo match
        Frontend->>MapPoint: new MapPoint(3D Position)
        Frontend->>Feature: new Feature(2D Point)

        Note over Frontend: 3. Linkage
        Frontend->>Feature: feature->mapPointPtr = MapPoint
        Frontend->>MapPoint: addObserve(Feature)
        Frontend->>Keyframe: featurePtrs.push_back(Feature)

        Frontend->>Map: addMapPoint(MapPoint)
    end

    Note over Frontend: 4. Add Keyframe to Map
    Frontend->>Map: addKeyframe(Keyframe)

    rect rgb(200, 255, 200)
    Note over Map: Check Sliding Window Size
    alt Window Full?
        Map->>Map: removeActiveKeyframe(OldestFrame)
    end
    end
```

## 2. Removing a Keyframe (Marginalization)

When the sliding window is full (e.g., > 10 frames), the oldest keyframe is removed to keep optimization fast.

1.  **Remove Active Keyframe**: `Map::removeActiveKeyframe(OldestID)` is called.
2.  **Remove Observations**:
    - Iterate through all `Feature`s in the Oldest Keyframe.
    - For each `Feature`, find the corresponding `MapPoint`.
    - Call `MapPoint::removeObserve(Feature)`.
3.  **Clean Map**:
    - Check all active MapPoints.
    - If a MapPoint has **0 observations** left (meaning no other active keyframe sees it), it is removed from the active map (`isLocalPoint = false`).

```mermaid
sequenceDiagram
    participant Map
    participant OldKeyframe
    participant Feature
    participant MapPoint

    Note over Map: Window Size Exceeded
    Map->>Map: removeActiveKeyframe(OldestID)

    loop For each Feature in OldKeyframe
        Map->>OldKeyframe: get Feature list
        OldKeyframe->>Feature: get Feature
        Feature->>MapPoint: get MapPoint

        Note over Map: Remove Linkage
        Map->>MapPoint: removeObserve(Feature)
        activate MapPoint
        MapPoint->>MapPoint: Decrement observationCount_
        MapPoint->>MapPoint: Remove Feature from obsFeatures_
        deactivate MapPoint
    end

    Map->>Map: activeKeyFramePtrs_.erase(OldKeyframe)

    Note over Map: Clean Map (Garbage Collection)
    loop For each Active MapPoint
        alt observationCount == 0
            Map->>MapPoint: isLocalPoint = false
            Map->>Map: Remove from activeMapPointPtrs_
        end
    end
```

## 3. Data Structure Relationships (ER Diagram)

If we view the system as a relational database, the `Feature` class acts as the **Join Table** (or Association Entity) connecting `Keyframe`s and `MapPoint`s.

- **Map**: The container holding the global state.
- **Keyframe**: Represents a camera pose at a specific time.
- **MapPoint**: Represents a 3D landmark in the world.
- **Feature**: Represents a 2D observation of a `MapPoint` in a specific `Keyframe`.

```mermaid
erDiagram
    MAP ||--|{ KEYFRAME : manages
    MAP ||--|{ MAP_POINT : manages

    KEYFRAME ||--|{ FEATURE : contains

    MAP_POINT ||--|{ FEATURE : observed_by

    FEATURE }|--|| KEYFRAME : belongs_to
    FEATURE }|--o| MAP_POINT : points_to

    MAP {
        int localWindowSize
        list activeKeyFrames
        list activeMapPoints
    }

    KEYFRAME {
        int frameId
        Mat image
        Pose T_wc
    }

    MAP_POINT {
        int id
        Vector3d position
        bool isLocalPoint
    }

    FEATURE {
        Point2f pixelPosition
        ptr framePtr
        ptr mapPointPtr
    }
```
