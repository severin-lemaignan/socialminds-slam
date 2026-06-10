- realtime SLAM for a mobile robot
- target max linear velocity: 2m/s
- mobile base: approx 50cm x 70cm; omni-directional; 2 laser scanners in opposite corners; two RGB-D realsense camera front and rear; IMU
- on-board GPU: RTX5060 8GB VRAM; shared with other on-board processes
- the SLAM engine should be fully 3D, outputting point clouds or similar (voxels, etc)
- the engine should be heavily multi-threaded (CPU with 24+ cores), perfectly handling the different sensors refresh rates (from eg 1kHz for the IMU to eg 20fps for RGB-D cameras)
- loop closure is of utmost importance
- the primary environment is indoor. Expect lots of features (but sometimes corridors with regular repetitive patterns) and dynamic objects (open/closed doors, moving chairs, etc).
- people are expect to be often visible, possibly occulting some sensors

The core of the engine should be middleware-independent, but design decisions should always keep in mind that the engine will be integrated in a complete ROS 2 environment, and all the data I/O will ultimately flows through ROS (which means: the design should ensure very efficient data bindings -- ideally zero-copy -- for Python or C++, if these are not the native languages of the engine).

The project is an example in software engineering good practices: architecture decisions are always carefully documentated, extensive test coverage -- both unit tests and integration tests; performance benchmarks easily reproducible.

---- Additional requirements

- this is not a toy project: we aim at producing a production-ready SLAM, beating the SotA, and deployed on real-world robots.
- In addition to previous targets, fast re-localization is essential
- internally, the mobile base should be modeled as a 3D object (eg: lidar scan can not be assumed to be always horizontal: when the robot is accelerating, the lidar plane will tilt)
- as such, sensor registration should take place in a 3D map. OpenVDB is most likely the best option (considering future integration with VDB-based reMap)
- we target practical maps in the range 100m x 100m x 12m (3 floors): OpenVDB cell size and architecture (using sub map) should be tailored accordingly.
- for multi-sensor setups, synthetic test data should include both clean and noisy data: noisy data should include eg small extrinsic calibration errors, small laserscan offset mimicking a brief tilt of the laserscan plane, dynamic objects moving around (mimicking eg people walking by)
