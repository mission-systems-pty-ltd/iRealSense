/************************************************************/
/*    NAME: David Battle                                    */
/*    ORGN: MISSION SYSTEMS PTY LTD                         */
/*    FILE: RealSense.cpp                                   */
/*    DATE: 27 Jan 2019                                     */
/************************************************************/

#include <pcl/common/transforms.h>
#include <iterator>
#include <algorithm>
//#include <chrono>
//#include <thread>
#include "MBUtils.h"
#include "RealSense.h"

using namespace std;

//---------------------------------------------------------
// Constructor

RealSense::RealSense()
{
  m_video       = true;
  m_resolution  = 0;
  m_frame_rate  = 30;
  m_translation = {0.0, 0.0, 0.0};
  m_rotation    = {0.0, 0.0, 0.0};
}

//---------------------------------------------------------
// Destructor

RealSense::~RealSense()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool RealSense::OnNewMail(MOOSMSG_LIST &NewMail)
{
  MOOSMSG_LIST::iterator p;

  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    std::string key = msg.GetKey();
    if (m_estimator_on) {
        if (key == "ESTIMATE_RESPONSE" && !m_is_timestamp_received) {
            if (m_verbose_output) {
		          std::cout << "Received an estimate response after request, parsing...\n";
            }
    		double * vector_ptr = reinterpret_cast<double *>(msg.GetBinaryData());
    		unsigned int vector_size = msg.GetBinaryDataSize()/sizeof(double);
            // Notify Iterate function
            m_is_timestamp_received = true;
    		if (vector_size == 7){
                // TODO fix hard-coded value
                if (m_timestamp_requested == *vector_ptr) {
                    m_bodyRotation[0] = *(vector_ptr+1);
                    m_bodyRotation[1] = *(vector_ptr+2);
                    m_bodyRotation[2] = *(vector_ptr+3);
                    /*m_bodyTranslation[0] = *(vector_ptr+4);
                    m_bodyTranslation[1] = *(vector_ptr+5);
                    m_bodyTranslation[2] = *(vector_ptr+6);*/
                    m_bodyTranslation[0] = 0;
                    m_bodyTranslation[1] = 0;
                    m_bodyTranslation[2] = 0;
                    if (m_verbose_output) {
    				    std::cout << "Rotation and translation set from estimate msg\n";
                    }
    			} else {
                    if (m_verbose_output) {
                        std::cout << "DEBUG MSG: TS_requested=" << m_timestamp_requested << " TS_received=" << *vector_ptr << "\n";
                    }
                }
    		} else {
    			std::cout << "Error: ESTIMATE_RESPONSE message does not have sufficient size\n";
                std::cout << "vector_size=" << vector_size << std::endl;
    		}
    	}
    } else {
        if (key == "NAV_ROLL") {
    		double roll = msg.GetDouble();
    		m_bodyRotation[0] = roll;
    	} else if (key == "NAV_PITCH") {
    		double pitch = msg.GetDouble();
    		m_bodyRotation[1] = pitch;
    	} else if (key == "NAV_YAW") {
    		double yaw = msg.GetDouble();
    		m_bodyRotation[2] = yaw;
    	} else if (key == "NAV_X") {
    		double x = msg.GetDouble();
    		m_bodyTranslation[0] = x;
    	} else if (key == "NAV_Y") {
    		double y = msg.GetDouble();
    		m_bodyTranslation[1] = y;
    	} else if (key == "NAV_Z") {
    		double z = msg.GetDouble();
    		m_bodyTranslation[2] = z;
    	}
    }
#if 0 // Keep these around just for template
    string key   = msg.GetKey();
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString();
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif
   }

   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool RealSense::OnConnectToServer()
{
   // register for variables here
   // possibly look at the mission file?
   // m_MissionReader.GetConfigurationParam("Name", <string>);
   // m_Comms.Register("VARNAME", 0);

   //RegisterVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool RealSense::Iterate()
{
	// Capture point cloud if estimator is off or last timestamp from estimator was received
	// After capture, request estimate
	if (!m_estimator_on || (m_estimator_on && m_is_timestamp_received)) {

		//// If not first pcl to be generated, transform last pcl and Notify
		if (!m_pcl_cloud.empty()) {
			// Transform to body reference
			Eigen::Affine3f transform_matrix_body = Eigen::Affine3f::Identity();
            // Check there is some transformation to perform
			if (!(m_zeroTranslation && m_zeroRotation)) {
				// x,y,z,roll,pitch,yaw in XYZ_FRAME reference
				transform_matrix_body = pcl::getTransformation(m_translation[1], m_translation[2], m_translation[0], m_rotation[1], m_rotation[2], m_rotation[0]);
			}

			// Transform to world reference (NAV_* messages or estimator return values)
			Eigen::Affine3f transform_matrix_world = Eigen::Affine3f::Identity();
			// The translation and rotation is being applied to the REALSENSE frame, but the NAV_* translation and rotation is in the XYZ_PLANE frame
            transform_matrix_world = pcl::getTransformation(m_bodyTranslation[1], m_bodyTranslation[2], m_bodyTranslation[0], m_bodyRotation[1], m_bodyRotation[2], 0);
            transform_matrix_world.rotate(Eigen::AngleAxisf(m_bodyRotation[0], Eigen::Vector3f(sin(m_bodyRotation[1]),0,cos(m_bodyRotation[1]))));

            // Transform according to chosen world frame
            Eigen::Affine3f transform_matrix_frame = Eigen::Affine3f::Identity();
			switch(m_reference_frame) {
				case REALSENSE:
					{
						transform_matrix_frame.translation() << 0.0, -m_camera_height, 0.0;
					}
					break;
				case WORLD_YFORWARD:
					{
						// Translate by camera height
						transform_matrix_frame.translation() << 0.0, 0.0, m_camera_height;
						// Rotate (-pi/2 about x axis)
						transform_matrix_frame.rotate(Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitX()));
					}
					break;
				case XYZ_PLANE:
					{
						// Translate by camera height
						transform_matrix_frame.translation() << 0.0, 0.0, -m_camera_height;
						// Rotate (pi/2 about x axis, then pi/2 about (old) y axis)
						transform_matrix_frame.rotate(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitX()));
						transform_matrix_frame.rotate(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitY()));
					}
					break;
				case NUM_FRAMES:
					//fallthrough
				default:
					std::cout << "No valid reference frame chosen. No transformations performed." << std::endl;
					break;
			}

            // Combine transformation matrices (note order matters)
            Eigen::Affine3f transform_matrix = transform_matrix_frame * transform_matrix_world * transform_matrix_body;
            pcl::transformPointCloud(m_pcl_cloud, m_pcl_cloud, transform_matrix);

			// Publish binary point cloud data
			Notify("LIDAR1_DATA", m_pcl_cloud.points.data(), m_pcl_cloud.size() * sizeof(RGBA_point));
		}
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//// Generate new point cloud data
		// Clear pcl cloud
		m_pcl_cloud.clear();

		// Capture a single frame and obtain depth + RGB values from it
		auto frames = m_pipe.wait_for_frames();
		double moosTime = MOOSTime(); // seconds, call this as soon as frames are received
		auto depth = frames.get_depth_frame();
		auto RGB = frames.get_color_frame();

		// Capture some timestamp data (ms since boot) and convert to seconds
        double timestamp; // = frames.get_timestamp()/1000.0; // seconds since boot

        // Analyse the metadata (look at rs_frame.h file for all metadata, as there is new stuff not documented)
        if (frames.supports_frame_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP)) { // this one chosen as most recently added
            // Device clock / sensor starts taking the image (measured by device clock)
            // Firmware-generated timestamp that marks middle of sensor exposure
            rs2_metadata_type sensor_ts = frames.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP);     // device usec

            // Device clock / frame starts to transfer to the driver
            // Generated in FW; designates the beginning of UVC frame transmission. (the first USB chunk sent towards host)
            rs2_metadata_type frame_ts = frames.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP);       // device usec

            // PC clock / frame starts to transfer to the driver (measured by kernal clock)
            // Host clock applied to kernel-space buffers (URB) when the data from USB controller is copied to OS kernel. (HW->Kernel transition)
            rs2_metadata_type backend_ts = frames.get_frame_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP);   // kernel unix usec

            // PC clock / frame is transfered to the driver (measured by kernal clock)
            // Host clock for the time when the frame buffer reaches Librealsense (kernel->user space transition)
            rs2_metadata_type arrival_ts = frames.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL);     // kernel unix usec

            // Processing
            //rs2_metadata_type kernel_device_offset_us = backend_ts - frame_ts;
            rs2_metadata_type device_intrinsic_delay_us = frame_ts - sensor_ts;
            rs2_metadata_type kernel_librealsense_delay_us = arrival_ts - backend_ts;
            //double librealsense_moostime_offset_s = moosTime - ((double)arrival_ts)/1000000.0;

            // Set timestamp (with respect to moosTime)
            timestamp = moosTime - ((double)(device_intrinsic_delay_us + kernel_librealsense_delay_us))/1000000.0;
        }

        // Print additional information
        if (m_verbose_output) {
          std::cout << "Camera Delay (s) = " << moosTime - timestamp << std::endl;
        }

    	// Map Color texture to each point
    	rs2::pointcloud pc;
    	pc.map_to(RGB);

    	// Generate Point Cloud
    	auto points = pc.calculate(depth);
    	size_t NPoints = points.size();

    	// Convert data captured from Realsense camera to Point Cloud
    	auto sp = points.get_profile().as<rs2::video_stream_profile>();

    	int width  = sp.width();
    	int height = sp.height();

    	auto uv = points.get_texture_coordinates();
    	auto vertices = points.get_vertices();

    	// Texture map
    	const auto tex = reinterpret_cast<const uint8_t*>(RGB.get_data());
    	int bytes_per_pixel = RGB.get_bytes_per_pixel();
    	int stride = RGB.get_stride_in_bytes();

        // Create the point cloud from points with non-zero z-values
    	for (int i = 0; i < NPoints; i++) {
    		if (vertices[i].z) {
    			// create the pcl point
    			RGBA_point pcl_point;

    			pcl_point.x = vertices[i].x;
    			pcl_point.y = vertices[i].y;
    			pcl_point.z = vertices[i].z;

    			// Calculate byte address of texture image
    			int col = min(max(int(uv[i].u * width  + .5f), 0), width - 1);
    			int row = min(max(int(uv[i].v * height + .5f), 0), height - 1);
    			int bytes = col * bytes_per_pixel; // Bytes per pixel
    			int strides = row * stride;        // Image width in bytes
    			int n = bytes + strides;

    			// Mapping Color (BGR due to Camera Model)
    			pcl_point.b = tex[n];
    			pcl_point.g = tex[n + 1];
    			pcl_point.r = tex[n + 2];

    			// add the pcl point to the cloud
    			m_pcl_cloud.push_back(pcl_point);
    		}
    	}

    	// Request estimate if estimator is on
    	if (m_estimator_on) {
    		m_timestamp_requested = timestamp;
    	    m_is_timestamp_received = false;
    	    Notify("ESTIMATE_REQUEST", timestamp);
            if (m_verbose_output) {
    		    std::cout << "Sent estimate request.\n";
            }
    	}

    	// Video feed
    	if (m_video) {
    		// Get colour frame data
    		void* color_frame = (void*)RGB.get_data();
    		// Publish binary point cloud data
    		Notify("CAMERA_DATA", color_frame, NPoints * 3);
    	}
    }
    return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool RealSense::OnStartUp()
{

    SetAppFreq(0); // As fast as we can
    SetIterateMode(REGULAR_ITERATE_AND_COMMS_DRIVEN_MAIL);

    list<string> sParams;
    m_MissionReader.EnableVerbatimQuoting(false);
    if (m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
        list<string>::iterator p;
        for(p=sParams.begin(); p!=sParams.end(); p++) {
            string original_line = *p;
            string param = stripBlankEnds(toupper(biteString(*p, '=')));
            string value = stripBlankEnds(*p);
            double dval  = atof(value.c_str());

            int nRows, nCols;

            if (param == "SERIAL_NUMBER") {
                m_serial_number = value;
                cout << "Device target = " << m_serial_number << endl;
            } else if (param == "VIDEO") {
                setBooleanOnString(m_video, value);
                cout << "Video = " << m_video << endl;
            } else if (param == "FRAME_RATE") {
                cout << "Frame rate = " << m_frame_rate << endl;
                m_frame_rate = dval;
            } else if (param == "TRANSLATION") {

                // Read a three-vector of x,y,z positions
                if (not m_MissionReader.GetConfigurationParam(param, m_translation, nRows, nCols)
                or nRows != 1 or nCols != 3)
                    cout << "WARNING: Lidar translation incorrectly set!" << endl;
                else
                    cout << "Translation set to " << m_translation[0] << endl;

                // Check if any translation
                if (std::all_of(m_translation.begin(), m_translation.end(), [](double i) { return i==0; })) {
                    m_zeroTranslation = true;
                }
            } else if (param == "ROTATION") {

                // Read a three-vector of x,y,z rotations
                if (not m_MissionReader.GetConfigurationParam(param, m_rotation, nRows, nCols)
                or nRows != 1 or nCols != 3)
                    cout << "WARNING: Lidar rotation incorrectly set!" << endl;
                else
                    cout << "Rotation set to " << m_rotation[0] << endl;

                // Check if any rotation
                if (std::all_of(m_rotation.begin(), m_rotation.end(), [](double i) { return i==0; })) {
                    m_zeroRotation = true;
                }
            } else if (param == "REFERENCE_FRAME") {
                value = toupper(value);
                if (value == "REALSENSE") {
                    m_reference_frame = REALSENSE;
                } else if (value == "WORLD_YFORWARD") {
                    m_reference_frame = WORLD_YFORWARD;
                } else if (value == "XYZ_PLANE") {
                    m_reference_frame = XYZ_PLANE;
                } else {
                    std::cout << "REFERENCE_FRAME could not be parse. Was: " << value << " Required:(see enum ReferenceFrame in RealSense.h)" << std::endl;
                }
            } else if (param == "CAMERA_HEIGHT") {
                try {
                    m_camera_height = std::stof(value);
                } catch (std::exception &e) {
                    std::cout << "Could not parse CAMERA_HEIGHT. Was: " << value << " Required: float" << std::endl;
                }
            } else if (param == "ESTIMATOR") {
                value = toupper(value);
                if (value == "ON" || value == "TRUE") {
                    m_estimator_on = true;
                } else if (value == "OFF" || value == "FALSE") {
                    m_estimator_on = false;
                }
            } else if (param == "VERBOSE_OUTPUT") {
              value = toupper(value);
              if (value == "ON" || value == "TRUE") {
              	m_verbose_output = true;
              } else if (value == "OFF" || value == "FALSE") {
              	m_verbose_output = false;
              }
            }
        }
    }

    rs2::context ctx;
    auto devices = ctx.query_devices(); // Currently connected devices
    cout << "Found " << devices.size() << " RealSense devices." << endl;

    rs2::device dev;
    bool found = false;

    for (auto const& d : devices) {
	if (d.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) == m_serial_number) {
		string name = d.get_info(RS2_CAMERA_INFO_NAME);
		cout << "Found " << name << endl;
		found = true;
		dev = d;
	}
    }

    if (not found)
    throw std::runtime_error("FATAL: RealSense device not found!");

    // Stream configuration
    std::cout << "Enabling cfg stream...\n";
    //m_cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, m_frame_rate);
    //m_cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16,  m_frame_rate);
    //m_cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, m_frame_rate);
    //m_cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16,  m_frame_rate);
    m_cfg.enable_stream(RS2_STREAM_COLOR, 424, 240, RS2_FORMAT_BGR8, m_frame_rate);
    m_cfg.enable_stream(RS2_STREAM_DEPTH, 424, 240, RS2_FORMAT_Z16,  m_frame_rate);
    std::cout << "\t... enabled.\n";

    std::cout << "rs2 pipeline profile configuring...\n";
    rs2::pipeline_profile selection = m_pipe.start(m_cfg);
    std::cout << "\t... configured.\n";

    RegisterVariables();
    return(true);
}

//---------------------------------------------------------
// Procedure: RegisterVariables

void RealSense::RegisterVariables()
{
    if (m_estimator_on) {
	Register("ESTIMATE_RESPONSE", 0);
	std::cout << "Registered to ESTIMATE_RESPONSE (estimator on)\n";
    } else {
        Register("NAV_ROLL", 0);
        Register("NAV_PITCH", 0);
        Register("NAV_YAW", 0);
        Register("NAV_X", 0);
        Register("NAV_Y", 0);
        Register("NAV_Z", 0);
	std::cout << "Registered to NAV_* messages (estimator off)\n";
    }
}
