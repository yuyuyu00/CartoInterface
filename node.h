#ifndef NODE_H
#define NODE_H



#include <chrono>
#include <map>
#include <queue>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/mutex.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/sparse_pose_graph.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

#include "node_options.h"
#include "sensor_bridge.h"

#include "MapPoint2D.h"
#include "MapPoint3D.h"
#include "IMUData.h"

using namespace std;
using namespace cartographer_ros;
using namespace My;
using namespace My::map2d;
using namespace My::map3d;

namespace carto = ::cartographer;

using carto::transform::Rigid3d;
using carto::kalman_filter::PoseCovariance;

constexpr int kInfiniteSubscriberQueueSize = 0;
constexpr int kLatestOnlyPublisherQueueSize = 1;

// Unique default topic names. Expected to be remapped as needed.
constexpr char kLaserScanTopic[] = "scan";
constexpr char kMultiEchoLaserScanTopic[] = "echoes";
constexpr char kPointCloud2Topic[] = "points2";
constexpr char kImuTopic[] = "imu";
constexpr char kOdometryTopic[] = "odom";
constexpr char kOccupancyGridTopic[] = "map";
constexpr char kScanMatchedPointCloudTopic[] = "scan_matched_points2";
constexpr char kSubmapListTopic[] = "submap_list";
constexpr char kSubmapQueryServiceName[] = "submap_query";
constexpr char kFinishTrajectoryServiceName[] = "finish_trajectory";

// Node that listens to all the sensor data that we are interested in and wires
// it up to the SLAM.
class Node
{
 public:
  Node(const NodeOptions& options);
  ~Node();

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

  void SpinForever();
  void Initialize();
  
  
  void HandleLaser(MapPoint3D& p);
  
  void HandleLaser(MapPoint& p);
  
  void HandleIMU(IMUData& p);
  
  private:
  
   void SpinOccupancyGridThreadForever(); 
  const NodeOptions options_;

  TfBridge tf_bridge_;
  
  
  // Set of all topics we subscribe to. We use the non-remapped default names  which are unique.
  std::unordered_set<string> expected_sensor_ids_; 
  
  carto::common::Mutex mutex_;
  
  std::deque<carto::mapping::TrajectoryNode::ConstantData> constant_data_   GUARDED_BY(mutex_);
  
  carto::mapping::MapBuilder map_builder_ GUARDED_BY(mutex_);
  
  std::unique_ptr<SensorBridge> sensor_bridge_ GUARDED_BY(mutex_);
  
  int trajectory_id_ = -1 GUARDED_BY(mutex_);
  
  carto::common::Time last_scan_matched_point_cloud_time_ =  carto::common::Time::min();
  
  std::thread occupancy_grid_thread_;
  
  bool terminating_ = false GUARDED_BY(mutex_);
  
};
#endif
