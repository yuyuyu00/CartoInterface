#include "node.h"


Node::Node(const NodeOptions& options)
    : options_(options),
     // tf_buffer_(::ros::Duration(1000)),
      //tf_(tf_buffer_),
      tf_bridge_(options_.tracking_frame, options_.lookup_transform_timeout_sec),
      map_builder_(options.map_builder_options, &constant_data_) {}

Node::~Node() {
  {
    carto::common::MutexLocker lock(&mutex_);
    terminating_ = true;
  }
  if (occupancy_grid_thread_.joinable()) {
    occupancy_grid_thread_.join();
  }
}



void Node::Initialize() 
{
  carto::common::MutexLocker lock(&mutex_);

  //激光给数据的接口
  // For 2D SLAM, subscribe to exactly one horizontal laser.
  if (options_.use_horizontal_laser)  	 
  {   
    std::cout<<"slam2d"<<std::endl;
    expected_sensor_ids_.insert(kLaserScanTopic);
  }
  if (options_.use_horizontal_multi_echo_laser)   
  {  
    std::cout<<"slam2d"<<std::endl;  
    expected_sensor_ids_.insert(kMultiEchoLaserScanTopic);
  }
  
  // For 3D SLAM, subscribe to all 3D lasers.
  if (options_.num_lasers_3d > 0) 
  {
    std::cout<<"slam3d"<<std::endl;
    //给数据的接口
    
    
    
  }

  //IMU给数据的接口
  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is required.
  if (options_.map_builder_options.use_trajectory_builder_3d() ||
      (options_.map_builder_options.use_trajectory_builder_2d() &&  options_.map_builder_options.trajectory_builder_2d_options().use_imu_data()) ) 
  {
    
  }

  //odom给数据的接口
  if (options_.use_odometry_data) {
    
  }

  trajectory_id_ = map_builder_.AddTrajectoryBuilder(expected_sensor_ids_);
  
  
  sensor_bridge_ = carto::common::make_unique<SensorBridge>(options_.sensor_bridge_options, &tf_bridge_, map_builder_.GetTrajectoryBuilder(trajectory_id_));

//   submap_list_publisher_ = 
//       node_handle_.advertise<::cartographer_ros_msgs::SubmapList>(
//           kSubmapListTopic, kLatestOnlyPublisherQueueSize);
//   submap_query_server_ = node_handle_.advertiseService(
//       kSubmapQueryServiceName, &Node::HandleSubmapQuery, this);

  if (options_.map_builder_options.use_trajectory_builder_2d()) {
//     occupancy_grid_publisher_ =
//         node_handle_.advertise<::nav_msgs::OccupancyGrid>(
//             kOccupancyGridTopic, kLatestOnlyPublisherQueueSize,
//             true /* latched */);
    occupancy_grid_thread_ =    std::thread(&Node::SpinOccupancyGridThreadForever, this);
  }

//   scan_matched_point_cloud_publisher_ =
//       node_handle_.advertise<sensor_msgs::PointCloud2>(
//           kScanMatchedPointCloudTopic, kLatestOnlyPublisherQueueSize);

//   finish_trajectory_server_ = node_handle_.advertiseService( kFinishTrajectoryServiceName, &Node::HandleFinishTrajectory, this);

//   wall_timers_.push_back(node_handle_.createWallTimer(
//       ::ros::WallDuration(options_.submap_publish_period_sec),
//       &Node::PublishSubmapList, this));
//   wall_timers_.push_back(node_handle_.createWallTimer(
//       ::ros::WallDuration(options_.pose_publish_period_sec),
//       &Node::PublishPoseAndScanMatchedPointCloud, this));
}




void Node::SpinOccupancyGridThreadForever() 
{
  for (;;)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    carto::common::MutexLocker lock(&mutex_);
    if (terminating_) 
    {
      return;
    }
    
    const auto trajectory_nodes =  map_builder_.sparse_pose_graph()->GetTrajectoryNodes();
    if (trajectory_nodes.empty()) 
    {
      continue;
    }
    //构建格网与显示
    
    //::nav_msgs::OccupancyGrid occupancy_grid;
    //BuildOccupancyGrid(trajectory_nodes, options_, &occupancy_grid);
   // occupancy_grid_publisher_.publish(occupancy_grid);
  }
}



void Node::SpinForever() { /*::ros::spin();*/ }

