# MOOS-iRealSense

Application for interfacing a realsense camera (D-series 435 tested) to MOOS.

Publishes depth cloud and camera data to MOOS. Optionally the user can set a reference frame for the published point cloud.
By default the realsense reference frame is used (z-axis pointing out from camera) however WORLD_YFORWARD (z up, y forward)
and XYZ_PLANE (z down, x forward) are possible. These transforms require more rigorous testing.

## Dependencies

* moos-ivp (see iPX4 readme for installation)
* realsense 2 library (`librealsense2`)
* PCL library (1.7.0 minimum requirement)
* math library
