/*
 * Copyright 2015 Aldebaran
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef CAMERA_INFO_DEF_HPP
#define CAMERA_INFO_DEF_HPP

#include <sensor_msgs/CameraInfo.h>
#include <boost/assign/list_of.hpp>

namespace naoqi
{
namespace converter
{
namespace camera_info_definitions
{

/**
* TOP CAMERA
*/
inline sensor_msgs::CameraInfo createCameraInfoTOPVGA()
{
  sensor_msgs::CameraInfo cam_info_msg;

  cam_info_msg.header.frame_id = "CameraTop_optical_frame";

  cam_info_msg.width = 640;
  cam_info_msg.height = 480;
  cam_info_msg.K = boost::array<double, 9>{{ 556.845054830986, 0, 309.366895338178, 0, 555.898679730161, 230.592233628776, 0, 0, 1 }};

  cam_info_msg.distortion_model = "plumb_bob";
  cam_info_msg.D = boost::assign::list_of(-0.0545211535376379)(0.0691973423510287)(-0.00241094929163055)(-0.00112245009306511)(0).convert_to_container<std::vector<double> >();

  cam_info_msg.R = boost::array<double, 9>{{ 1, 0, 0, 0, 1, 0, 0, 0, 1 }};

  cam_info_msg.P = boost::array<double, 12>{{ 551.589721679688, 0, 308.271132841983, 0, 0, 550.291320800781, 229.20143668168, 0, 0, 0, 1, 0 }};

  return cam_info_msg;
}


inline sensor_msgs::CameraInfo createCameraInfoTOPQVGA()
{
  sensor_msgs::CameraInfo cam_info_msg;

  cam_info_msg.header.frame_id = "CameraTop_optical_frame";

  cam_info_msg.width = 320;
  cam_info_msg.height = 240;
  cam_info_msg.K = boost::array<double, 9>{{ 278.623049, 0, 164.015178, 0, 278.677742, 114.137019, 0, 0, 1 }};

  cam_info_msg.distortion_model = "plumb_bob";
  cam_info_msg.D = boost::assign::list_of(-0.071892)(0.057165)(0.000300)(-0.003705)(0.000000).convert_to_container<std::vector<double> >();

  cam_info_msg.R = boost::array<double, 9>{{ 1, 0, 0, 0, 1, 0, 0, 0, 1 }};

  cam_info_msg.P = boost::array<double, 12>{{ 272.849884, 0, 162.516240, 0, 0, 274.324738, 113.595419, 0, 0, 0, 1, 0 }};

  return cam_info_msg;
}


inline sensor_msgs::CameraInfo createCameraInfoTOPQQVGA()
{
  sensor_msgs::CameraInfo cam_info_msg;

  cam_info_msg.header.frame_id = "CameraTop_optical_frame";

  cam_info_msg.width = 160;
  cam_info_msg.height = 120;
  cam_info_msg.K = boost::array<double, 9>{{ 139.424539568966, 0, 76.9073669920582, 0, 139.25542782325, 59.5554242026743, 0, 0, 1 }};

  cam_info_msg.distortion_model = "plumb_bob";
  cam_info_msg.D = boost::assign::list_of(-0.0843564504845967)(0.125733083790192)(0.00275901756247071)(-0.00138645823460527)(0).convert_to_container<std::vector<double> >();

  cam_info_msg.R = boost::array<double, 9>{{ 1, 0, 0, 0, 1, 0, 0, 0, 1 }};

  cam_info_msg.P = boost::array<double, 12>{{ 137.541534423828, 0, 76.3004646597892, 0, 0, 136.815216064453, 59.3909799751191, 0, 0, 0, 1, 0 }};

  return cam_info_msg;
}


/**
* BOTTOM CAMERA
*/
inline sensor_msgs::CameraInfo createCameraInfoBOTTOMVGA()
{
  sensor_msgs::CameraInfo cam_info_msg;

  cam_info_msg.header.frame_id = "CameraBottom_optical_frame";

  cam_info_msg.width = 640;
  cam_info_msg.height = 480;
  cam_info_msg.K = boost::array<double, 9>{{ 558.570339530768, 0, 308.885375457296, 0, 556.122943034837, 247.600724811385, 0, 0, 1 }};

  cam_info_msg.distortion_model = "plumb_bob";
  cam_info_msg.D = boost::assign::list_of(-0.0648763971625288)(0.0612520196884308)(0.0038281538281731)(-0.00551104078371959)(0).convert_to_container<std::vector<double> >();

  cam_info_msg.R = boost::array<double, 9>{{ 1, 0, 0, 0, 1, 0, 0, 0, 1 }};

  cam_info_msg.P = boost::array<double, 12>{{ 549.571655273438, 0, 304.799679526441, 0, 0, 549.687316894531, 248.526959297022, 0, 0, 0, 1, 0 }};

  return cam_info_msg;
}


inline sensor_msgs::CameraInfo createCameraInfoBOTTOMQVGA()
{
  sensor_msgs::CameraInfo cam_info_msg;

  cam_info_msg.header.frame_id = "CameraBottom_optical_frame";

  cam_info_msg.width = 320;
  cam_info_msg.height = 240;
  cam_info_msg.K = boost::array<double, 9>{{ 277.226670, 0, 162.400240, 0, 277.011916, 117.516717, 0, 0, 1 }};

  cam_info_msg.distortion_model = "plumb_bob";
  cam_info_msg.D = boost::assign::list_of(-0.078184)(0.108829)(-0.001420)(0.000786)(0).convert_to_container<std::vector<double> >();

  cam_info_msg.R = boost::array<double, 9>{{ 1, 0, 0, 0, 1, 0, 0, 0, 1 }};
  cam_info_msg.P = boost::array<double, 12>{{ 273.613983, 0, 162.089417, 0, 0, 273.000916, 116.713638, 0, 0, 0, 1, 0 }};

  return cam_info_msg;
}


inline sensor_msgs::CameraInfo createCameraInfoBOTTOMQQVGA()
{
  sensor_msgs::CameraInfo cam_info_msg;

  cam_info_msg.header.frame_id = "CameraBottom_optical_frame";

  cam_info_msg.width = 160;
  cam_info_msg.height = 120;
  cam_info_msg.K = boost::array<double, 9>{{ 141.611855886672, 0, 78.6494086288656, 0, 141.367163830175, 58.9220646201529, 0, 0, 1 }};

  cam_info_msg.distortion_model = "plumb_bob";
  cam_info_msg.D = boost::assign::list_of(-0.0688388724945936)(0.0697453843669642)(0.00309518737071049)(-0.00570486993696543)(0).convert_to_container<std::vector<double> >();

  cam_info_msg.R = boost::array<double, 9>{{ 1, 0, 0, 0, 1, 0, 0, 0, 1 }};

  cam_info_msg.P = boost::array<double, 12>{{ 138.705535888672, 0, 77.2544255212306, 0, 0, 138.954086303711, 58.7000861760043, 0, 0, 0, 1, 0 }};

  return cam_info_msg;
}


/**
* DEPTH CAMERA
*/
inline sensor_msgs::CameraInfo createCameraInfoDEPTHVGA()
{
  sensor_msgs::CameraInfo cam_info_msg;

  cam_info_msg.header.frame_id = "CameraDepth_optical_frame";

  cam_info_msg.width = 640;
  cam_info_msg.height = 480;
  cam_info_msg.K = boost::array<double, 9>{{ 525, 0, 319.5000000, 0, 525, 239.5000000000000, 0, 0, 1  }};

  //cam_info_msg.distortion_model = "plumb_bob";
  //cam_info_msg.D = boost::assign::list_of(-0.0688388724945936)(0.0697453843669642)(0.00309518737071049)(-0.00570486993696543)(0);

  cam_info_msg.R = boost::array<double, 9>{{ 1, 0, 0, 0, 1, 0, 0, 0, 1 }};

  cam_info_msg.P = boost::array<double, 12>{{ 525, 0, 319.500000, 0, 0, 525, 239.5000000000, 0, 0, 0, 1, 0 }};

  return cam_info_msg;
}


inline sensor_msgs::CameraInfo createCameraInfoDEPTHQVGA()
{
  sensor_msgs::CameraInfo cam_info_msg;

  cam_info_msg.header.frame_id = "CameraDepth_optical_frame";

  cam_info_msg.width = 320;
  cam_info_msg.height = 240;
  cam_info_msg.K = boost::array<double, 9>{{ 525/2.0f, 0, 319.5000000/2.0f, 0, 525/2.0f, 239.5000000000000/2.0f, 0, 0, 1  }};

  //cam_info_msg.distortion_model = "plumb_bob";
  //cam_info_msg.D = boost::assign::list_of(-0.0688388724945936)(0.0697453843669642)(0.00309518737071049)(-0.00570486993696543)(0);

  cam_info_msg.R = boost::array<double, 9>{{ 1, 0, 0, 0, 1, 0, 0, 0, 1 }};

  cam_info_msg.P = boost::array<double, 12>{{ 525/2.0f, 0, 319.500000/2.0f, 0, 0, 525/2.0f, 239.5000000000/2.0f, 0, 0, 0, 1, 0 }};

  return cam_info_msg;
}

inline sensor_msgs::CameraInfo createCameraInfoDEPTHQQVGA()
{
  sensor_msgs::CameraInfo cam_info_msg;

  cam_info_msg.header.frame_id = "CameraDepth_optical_frame";

  cam_info_msg.width = 160;
  cam_info_msg.height = 120;
  cam_info_msg.K = boost::array<double, 9>{{ 525/4.0f, 0, 319.5000000/4.0f, 0, 525/4.0f, 239.5000000000000/4.0f, 0, 0, 1  }};

  //cam_info_msg.distortion_model = "plumb_bob";
  //cam_info_msg.D = boost::assign::list_of(-0.0688388724945936)(0.0697453843669642)(0.00309518737071049)(-0.00570486993696543)(0);

  cam_info_msg.R = boost::array<double, 9>{{ 1, 0, 0, 0, 1, 0, 0, 0, 1 }};

  cam_info_msg.P = boost::array<double, 12>{{ 525/4.0f, 0, 319.500000/4.0f, 0, 0, 525/4.0f, 239.5000000000/4.0f, 0, 0, 0, 1, 0 }};

  return cam_info_msg;
}
} // camera_info_definitions
} //publisher
} //naoqi


#endif
