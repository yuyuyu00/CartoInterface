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

#include "sensor_bridge.h"

#include "cartographer/kalman_filter/pose_tracker.h"
#include "msg_conversion.h"
//#include "cartographer_ros/time_conversion.h"

namespace cartographer_ros {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

namespace {

const string& CheckNoLeadingSlash(const string& frame_id)
{
  if (frame_id.size() > 0) 
  {
    CHECK_NE(frame_id[0], '/');
  }
  return frame_id;
}

}  // namespace

SensorBridgeOptions CreateSensorBridgeOptions(  carto::common::LuaParameterDictionary* const lua_parameter_dictionary)
{
  SensorBridgeOptions options;
  options.constant_odometry_translational_variance =   lua_parameter_dictionary->GetDouble(  "constant_odometry_translational_variance");
  options.constant_odometry_rotational_variance =    lua_parameter_dictionary->GetDouble(   "constant_odometry_rotational_variance");
  return options;
}

SensorBridge::SensorBridge(
    const SensorBridgeOptions& options, const TfBridge* const tf_bridge,
    carto::mapping::TrajectoryBuilder* const trajectory_builder)
    : options_(options),
      tf_bridge_(tf_bridge),
      trajectory_builder_(trajectory_builder) {}
      
      
      
      void SensorBridge::HandleMultiEchoLaserScanData(const string& topic,MapPoint* dat)
	  {
		  
		  return;
	  }
      
/*
void SensorBridge::HandleOdometryMessage(
    const string& topic, const nav_msgs::Odometry::ConstPtr& msg) {
  const carto::common::Time time = FromRos(msg->header.stamp);
  const Eigen::Matrix3d translational =
      Eigen::Matrix3d::Identity() *
      options_.constant_odometry_translational_variance;
  const Eigen::Matrix3d rotational =
      Eigen::Matrix3d::Identity() *
      options_.constant_odometry_rotational_variance;
  // clang-format off
  carto::kalman_filter::PoseCovariance covariance;
  covariance <<
      translational, Eigen::Matrix3d::Zero(),
      Eigen::Matrix3d::Zero(), rotational;
  // clang-format on
  const auto sensor_to_tracking = tf_bridge_->LookupToTracking(
      time, CheckNoLeadingSlash(msg->child_frame_id));
  if (sensor_to_tracking != nullptr) {
    trajectory_builder_->AddOdometerPose(
        topic, time, ToRigid3d(msg->pose.pose) * sensor_to_tracking->inverse(),
        covariance);
  }
}

void SensorBridge::HandleImuMessage(const string& topic,
                                    const sensor_msgs::Imu::ConstPtr& msg) {
  CHECK_NE(msg->linear_acceleration_covariance[0], -1);
  CHECK_NE(msg->angular_velocity_covariance[0], -1);
  const carto::common::Time time = FromRos(msg->header.stamp);
  const auto sensor_to_tracking = tf_bridge_->LookupToTracking(
      time, CheckNoLeadingSlash(msg->header.frame_id));
  if (sensor_to_tracking != nullptr) {
    CHECK(sensor_to_tracking->translation().norm() < 1e-5)
        << "The IMU frame must be colocated with the tracking frame. "
           "Transforming linear acceleration into the tracking frame will "
           "otherwise be imprecise.";
    trajectory_builder_->AddImuData(
        topic, time,
        sensor_to_tracking->rotation() * ToEigen(msg->linear_acceleration),
        sensor_to_tracking->rotation() * ToEigen(msg->angular_velocity));
  }
}

void SensorBridge::HandleLaserScanMessage(
    const string& topic, const sensor_msgs::LaserScan::ConstPtr& msg) {
  HandleLaserScanProto(topic, FromRos(msg->header.stamp), msg->header.frame_id,
                       ToCartographer(*msg));
}

void SensorBridge::HandleMultiEchoLaserScanMessage(
    const string& topic, const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
  HandleLaserScanProto(topic, FromRos(msg->header.stamp), msg->header.frame_id,
                       ToCartographer(*msg));
}

void SensorBridge::HandlePointCloud2Message(    const string& topic, const sensor_msgs::PointCloud2::ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
  pcl::fromROSMsg(*msg, pcl_point_cloud);
  const carto::common::Time time = FromRos(msg->header.stamp);
  const auto sensor_to_tracking = tf_bridge_->LookupToTracking(
      time, CheckNoLeadingSlash(msg->header.frame_id));
  if (sensor_to_tracking != nullptr) {
    trajectory_builder_->AddLaserFan(
        topic, time,
        carto::sensor::TransformLaserFan(
            carto::sensor::FromProto(ToCartographer(pcl_point_cloud)),
            sensor_to_tracking->cast<float>()));
  }
}*/



void SensorBridge::HandleImuData(const string& topic,IMUData& p) 
{
	
	const carto::common::Time time = Rostime2CartoTime(p.m_tm);
	const auto sensor_to_tracking = tf_bridge_->LookupToTracking(	time, CheckNoLeadingSlash("imu_link"));
	if (sensor_to_tracking != nullptr)
	{
// 		CHECK(sensor_to_tracking->translation().norm() < 1e-5)
// 		<< "The IMU frame must be colocated with the tracking frame. "
// 		"Transforming linear acceleration into the tracking frame will "
// 		"otherwise be imprecise.";
		trajectory_builder_->AddImuData(	topic, time,	sensor_to_tracking->rotation() * ToEigen(p.m_accx,p.m_accy,p.m_accz),
											sensor_to_tracking->rotation() * ToEigen(p.m_gryx,p.m_gryy,p.m_gryz));
	}
}


::cartographer::common::Time SensorBridge::Rostime2CartoTime(double tm)
{
	// The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
	// exactly 719162 days before the Unix epoch.
	long long tsec = (long long) floor(tm);
	long long tnsec = (long long)(1000000000*(tm));
	
	return ::cartographer::common::FromUniversal((tsec + kUtsEpochOffsetFromUnixEpochInSeconds) * 10000000ll + 	(tnsec + 50) / 100);  // + 50 to get the rounding correct.
}

void SensorBridge::HandlePointCloudData(    const string& topic, const pcl::PointCloud<pcl::PointXYZ>& pcl_point_cloud,double tm) 
{
	const carto::common::Time time = Rostime2CartoTime(tm);
	std::cout<<time<<std::endl;
	//const auto sensor_to_tracking = tf_bridge_->LookupToTracking(	time, CheckNoLeadingSlash(topic));
	carto::transform::Rigid3f sensor_to_tracking ;
	//if (sensor_to_tracking != nullptr) {
		carto::sensor::PointCloud pointcloud;
		for (const auto& point : pcl_point_cloud)
		{
			pointcloud.emplace_back(point.x, point.y, point.z);
		}

		trajectory_builder_->AddRangefinderData(topic, time, sensor_to_tracking.translation(), carto::sensor::TransformPointCloud(pointcloud, sensor_to_tracking));
		
//	}
}

//void SensorBridge::HandleLaserScanProto( const string& topic, const carto::common::Time time, const string& frame_id, const carto::sensor::proto::LaserScan& laser_scan) 
//{
//  const carto::sensor::LaserFan laser_fan = {   Eigen::Vector3f::Zero(),   carto::sensor::ToPointCloud(laser_scan),   {},   {}};
//  const auto sensor_to_tracking =    tf_bridge_->LookupToTracking(time, CheckNoLeadingSlash(frame_id));
//  if (sensor_to_tracking != nullptr)
//  {
//    trajectory_builder_->AddRangefinderData( topic, time, sensor_to_tracking->cast<float>(), laser_fan);
//  }
//}

}  // namespace cartographer_ros
