#include "node.h"

using namespace cartographer_ros;

Node::Node(const NodeOptions& options)
    : options_(options),
     // tf_buffer_(::ros::Duration(1000)),
      //tf_(tf_buffer_),
      tf_bridge_(options_.tracking_frame, options_.lookup_transform_timeout_sec),
      map_builder_(options.map_builder_options, &constant_data_) 
	  {	}

Node::~Node() {
  {
    carto::common::MutexLocker lock(&mutex_);
    terminating_ = true;
  }
  if (occupancy_grid_thread_.joinable()) {
    occupancy_grid_thread_.join();
  }
}

  
bool Node::HandleSubmapQuery(int submap_id)
{
//	carto::common::MutexLocker lock(&mutex_);
	
	const carto::mapping::Submaps* submaps =	map_builder_.GetTrajectoryBuilder(trajectory_id_)->submaps();
	
	if (submap_id < 0 || submap_id >= submaps->size())  return false;
	
	carto::mapping::proto::SubmapQuery::Response response_proto;
	
	//response_proto.set_submap_id(submap_id);
	response_proto.set_submap_version(submaps->Get(submap_id)->end_laser_fan_index);
	const std::vector<carto::transform::Rigid3d> submap_transforms = map_builder_.sparse_pose_graph()->GetSubmapTransforms(*submaps);
	submaps->SubmapToProto(submap_id,  map_builder_.sparse_pose_graph()->GetTrajectoryNodes(),  submap_transforms[submap_id], &response_proto);
	cout<<response_proto.ByteSize()<<endl;
	
	string celldata = response_proto.cells();
	
	
	for(auto it = celldata.begin();it!=celldata.end();it++)
	{
		//const uint8 alpha = *it
		
	}
	
	for(int i=0;i<response_proto.height();i++)
		for(int j=0;j<response_proto.width();j++)
		{
			
		}
	
}

void Node::DrawTrajectory(vector<cartographer::mapping::TrajectoryNode> trajectory_nodes)
{
	
	
	
	vector<My::DrawPoint> pts;
	My::DrawPoint pt;
	for(int i=0;i<trajectory_nodes.size();i++)
	{
		carto::transform::Rigid3d rd = trajectory_nodes[i].pose;
		carto::transform::Rigid3d::Vector trans = rd.translation();
		pt.x = trans[0];
		pt.y =  trans[1];
		pt.z = 0;
	//	cout<<pt.x<<" "<<pt.y<<" "<<pt.z<<endl;
	//	cout<<rd<<endl;
		
		pts.push_back(pt);
	}
	dr.SetLinesRuningTime(pts);
	dr.SetWaitKey(1);
	cout<<trajectory_nodes.size()<<endl;
	
}


void Node::HandleLaser(MapPoint3D& p)
{
	
	pcl::PointCloud<pcl::PointXYZ>* pointbase = dynamic_cast< pcl::PointCloud<pcl::PointXYZ>*>(&p);
	
	sensor_bridge_->HandlePointCloudData(string("3d"),*pointbase,p.m_tm);
}

void Node::HandleLaser(MapPoint& p)
{
	double tm = (double)p.tm.tSecond+ (double)p.tm.tMilliSecond/1000.;
	
	
	pcl::PointCloud<pcl::PointXYZ> pointbase;
	for(int i=0;i<p.lt.LaserNums;i++)
	{
		pointbase.push_back(pcl::PointXYZ(p.lt.laserPoint[i].x,p.lt.laserPoint[i].y,0));
	}
	
	//cout<<tm<<endl;
	
	sensor_bridge_->HandlePointCloudData(string("2d"),pointbase,tm);
	
	//cout<<tm<<endl;
	
}

void Node::HandleIMU(IMUData& p)
{
	sensor_bridge_->HandleImuData(string("imu"),p);
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
	expected_sensor_ids_.insert("3d");
    
    
  }

  //IMU给数据的接口
  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is required.
  if (options_.map_builder_options.use_trajectory_builder_3d() ||
      (options_.map_builder_options.use_trajectory_builder_2d() &&  options_.map_builder_options.trajectory_builder_2d_options().use_imu_data()) ) 
  {
	  expected_sensor_ids_.insert("imu");
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
  }

  occupancy_grid_thread_ =    std::thread(&Node::SpinOccupancyGridThreadForever, this);
  
  
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
	cout<<trajectory_nodes.size()<<endl;
    if (trajectory_nodes.empty()) 
    {
      continue;
    }

    DrawTrajectory(trajectory_nodes);
    
	const carto::mapping::Submaps* submaps = map_builder_.GetTrajectoryBuilder(trajectory_id_)->submaps();
	if(submaps->size()>0)
	{
			HandleSubmapQuery(0);
	}

    //::nav_msgs::OccupancyGrid occupancy_grid;
    //BuildOccupancyGrid(trajectory_nodes, options_, &occupancy_grid);
   // occupancy_grid_publisher_.publish(occupancy_grid);
  }
}



void Node::SpinForever() { /*::ros::spin();*/ }

