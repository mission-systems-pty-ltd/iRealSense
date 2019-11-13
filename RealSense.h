/************************************************************/
/*    NAME: David Battle                                    */
/*    ORGN: MISSION SYSTEMS PTY LTD                         */
/*    FILE: RealSense.h                                     */
/*    DATE: 27 Jan 2019                                     */
/************************************************************/

#ifndef RealSense_HEADER
#define RealSense_HEADER

#include "MOOS/libMOOS/MOOSLib.h"

// Intel Realsense Headers
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

// pcl cloud
#include <pcl/common/common.h>

#include <vector>
#include <array>
//#include <atomic>

typedef pcl::PointXYZRGBA RGBA_point;
typedef pcl::PointCloud<RGBA_point> RGBA_cloud;
//typedef RGBA_cloud::Ptr RGBA_cloud_ptr;

// The realsense camera has z axis starting at camera and pointing out.
// The default position is assumed to be horizontal, not vertical, and
// so e.g. 90 degree rotation is required to transform to world frame.
// Will also translate vertically according to Camera_height (m)
// TODO: WORLD_YFORWARD and XYZ_FRAME require further testing
enum ReferenceFrame {
	REALSENSE = 0,		// RH, z out of cam / forward, y down
	WORLD_YFORWARD,	// RH, y forward, z up
	XYZ_PLANE,		// RH, x forward, z down
	NUM_FRAMES
};

class RealSense : public CMOOSApp
{
public:
	RealSense();
	~RealSense();

	std::string m_serial_number;
	float m_frame_rate;
	int m_resolution;
	bool m_video;
	bool m_RGB;

	ReferenceFrame m_reference_frame{REALSENSE};
	float m_camera_height{0.0}; // m

	// Body-relative position of Lidar
	std::vector<double> m_translation;
	std::vector<double> m_rotation;
	bool m_zeroTranslation{false};
	bool m_zeroRotation{false};

	// World-relative position of Lidar
	std::array<double, 3> m_bodyTranslation = {{0.0, 0.0, 0.0}};
	std::array<double, 3> m_bodyRotation = {{0.0, 0.0, 0.0}};

	// Estimator
	bool m_estimator_on{false};
	double m_timestamp_requested;
	bool m_is_timestamp_received{true}; // default true to save first point cloud

	// Verbose output flag
	bool m_verbose_output{false};

 protected: // Standard MOOSApp functions to overload
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected:
   void RegisterVariables();

 private: // Configuration variables

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline m_pipe;

	// Create a configuration for configuring the pipeline with a non default profile
	rs2::config m_cfg;

	// Hold the pointcloud in the memory space
	RGBA_cloud m_pcl_cloud;

 private: // State variables

};

#endif
