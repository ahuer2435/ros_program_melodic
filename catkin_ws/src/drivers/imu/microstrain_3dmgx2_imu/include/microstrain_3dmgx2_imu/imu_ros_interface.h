/*
 * Software License Agreement (BSD License)
 *
 *  Microstrain 3DM-GX2 node
 *  Copyright (c) 2008-2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <assert.h>
#include <math.h>
#include <iostream>

#include <boost/format.hpp>
#include "microstrain_3dmgx2_imu/3dmgx2.h"

#include "ros/time.h"
#include "self_test/self_test.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"

#include "sensor_msgs/Imu.h"
#include "std_srvs/Empty.h"
#include "tf/transform_datatypes.h"
#include "microstrain_3dmgx2_imu/AddOffset.h"
#include "std_msgs/Bool.h"

using namespace std;

//基于IMU sdk封装的ros驱动接口类
class ImuNode 
{
public:
  ImuNode(ros::NodeHandle h);   //初始化ros系统和参数。
  ~ImuNode();
  bool spin();                  //数据读取，数据解析，数据发布。

private:
  microstrain_3dmgx2_imu::IMU imu;  //IMU sdk的封装接口
  sensor_msgs::Imu reading;     //要发布的imu数据。
  string port;                  //imu所对应的设备文件。
  microstrain_3dmgx2_imu::IMU::cmd cmd;

  self_test::TestRunner self_test_;         //测试
  diagnostic_updater::Updater diagnostic_;
  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  ros::Publisher imu_data_pub_;
  ros::ServiceServer add_offset_serv_;
  ros::ServiceServer calibrate_serv_;
  ros::Publisher is_calibrated_pub_;

  bool running;
  bool autocalibrate_;
  bool calibrate_requested_;
  bool calibrated_;  
  int error_count_;
  int slow_count_;
  std::string was_slow_;
  std::string error_status_;
  string frameid_;  
  double offset_;    
  double bias_x_;
  double bias_y_;
  double bias_z_;

  double angular_velocity_stdev_, angular_velocity_covariance_;
  double linear_acceleration_covariance_, linear_acceleration_stdev_;
  double orientation_covariance_, orientation_stdev_;
  double max_drift_rate_;
  double desired_freq_;
  diagnostic_updater::FrequencyStatus freq_diag_;  
  
  void setErrorStatusf(const char *format, ...);
  // Prints an error message if it isn't the same old error message.
  void setErrorStatus(const std::string msg);
  void clearErrorStatus();
  int start();
  std::string getID(bool output_info = false);
  int stop();
  int publish_datum();
  
  void publish_is_calibrated();
  void pretest(diagnostic_updater::DiagnosticStatusWrapper& status);
  void InterruptionTest(diagnostic_updater::DiagnosticStatusWrapper& status);
  void ConnectTest(diagnostic_updater::DiagnosticStatusWrapper& status);
  void ReadIDTest(diagnostic_updater::DiagnosticStatusWrapper& status);
  void GyroBiasTest(diagnostic_updater::DiagnosticStatusWrapper& status); 
  void getData(sensor_msgs::Imu& data);  
  void StreamedDataTest(diagnostic_updater::DiagnosticStatusWrapper& status);  
  void GravityTest(diagnostic_updater::DiagnosticStatusWrapper& status);  
  void DisconnectTest(diagnostic_updater::DiagnosticStatusWrapper& status);  
  void ResumeTest(diagnostic_updater::DiagnosticStatusWrapper& status);  
  void deviceStatus(diagnostic_updater::DiagnosticStatusWrapper &status);  
  void calibrationStatus(diagnostic_updater::DiagnosticStatusWrapper& status);  
  bool addOffset(microstrain_3dmgx2_imu::AddOffset::Request &req, microstrain_3dmgx2_imu::AddOffset::Response &resp);  
  bool calibrate(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);  
  void doCalibrate();  
};
