/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_ROS_SENSOR_BRIDGE_H_
#define CARTOGRAPHER_ROS_SENSOR_BRIDGE_H_

#include "cartographer/mapping/trajectory_builder.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"


 #include "tf_bridge.h"
 
 #include "MapPoint3D.h"
 #include "MapPoint2D.h"
 #include "IMUData.h"

 using namespace My::map3d;
 using namespace My::map2d;
using namespace My;
 
// #include "geometry_msgs/Transform.h"
// #include "geometry_msgs/TransformStamped.h"
// #include "nav_msgs/OccupancyGrid.h"
// #include "nav_msgs/Odometry.h"
// #include "sensor_msgs/Imu.h"
// #include "sensor_msgs/LaserScan.h"
// #include "sensor_msgs/MultiEchoLaserScan.h"
// #include "sensor_msgs/PointCloud2.h"

namespace cartographer_ros {

struct SensorBridgeOptions {
  double constant_odometry_translational_variance;
  double constant_odometry_rotational_variance;
};

SensorBridgeOptions CreateSensorBridgeOptions(  ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);

// Converts ROS messages into SensorData in tracking frame for the MapBuilder.
class SensorBridge {
 public:
  explicit SensorBridge( const SensorBridgeOptions& options, const TfBridge* tf_bridge,   ::cartographer::mapping::TrajectoryBuilder* trajectory_builder);

  SensorBridge(const SensorBridge&) = delete;
  SensorBridge& operator=(const SensorBridge&) = delete;

//   void HandleOdometryMessage(const string& topic,  const nav_msgs::Odometry::ConstPtr& msg);
//   void HandleImuMessage(const string& topic,  const sensor_msgs::Imu::ConstPtr& msg);
//   void HandleLaserScanMessage(const string& topic, const sensor_msgs::LaserScan::ConstPtr& msg);
//   void HandleMultiEchoLaserScanMessage( const string& topic, const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg);
//   void HandlePointCloud2Message(const string& topic,   const sensor_msgs::PointCloud2::ConstPtr& msg);
  
  void HandleMultiEchoLaserScanData(const string& topic,MapPoint* dat);

  void HandlePointCloudData(   const string& topic, const pcl::PointCloud<pcl::PointXYZ>& pcl_point_cloud,double tm) ;
  
  cartographer::common::Time Rostime2CartoTime(double tm);
  
  void HandleImuData(const string& topic,IMUData& p) ;
  
  
 private:
  //void HandleLaserScanProto(
  //    const string& topic, const ::cartographer::common::Time time,
  //    const string& frame_id,
  //    const ::cartographer::sensor::proto::LaserScan& laser_scan);

  const SensorBridgeOptions options_;
  const TfBridge* const tf_bridge_;
  ::cartographer::mapping::TrajectoryBuilder* const trajectory_builder_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_SENSOR_BRIDGE_H_
