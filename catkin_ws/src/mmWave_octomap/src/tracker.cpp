#include <algorithm>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/assert.h>


#include <gtrack.h>
#include <gtrack_int.h>
#include <mmWave_octomap/tracker.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

//#include <cmath.h>

using namespace boost::accumulators;
void Listener::callback(const Listener::PointCloud::ConstPtr &msg)
{
  ROS_DEBUG("\033[2J\033[1;1H");
  this->start_=ros::WallTime::now();
  //int i = 0;
  // int pointCloudSizeInBytes = msg->row_step * msg->height;

  // ROS_DEBUG("\tData size %d Bytes \n", pointCloudSizeInBytes);
  //printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  accumulator_set<double, features<tag::mean, tag::variance, tag::median>> acc;
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_radar_link_1";
  
  for (const pcl::PointXYZI &pt: msg->points)
  {

    float cart[4] = {pt.x, pt.y, 0, pt.intensity};
    float sph[3] = {0, 0, 0};

    gtrack_cartesian2spherical(GTRACK_STATE_VECTORS_2DA, cart, sph);


   //GTRACK_measurement_vector vector;
   //vector.range = sph[0];
   //vector.angle = sph[1];
   //vector.doppler = pt.z;
   if (abs(sph[1]) < 0.1 && sph[0] > 0.01){
    	acc(-1.0f * pt.z);
    }
    //ROS_DEBUG("\tRange: %f, Angle: %f, Doppler: %f, SNR: %f\n",vector.range, vector.angle, vector.doppler, cart[3]);
    
    //this->points[i].array[0] = sph[0];
    //this->points[i].array[1] = sph[1];
    //this->points[i].array[2] = pt.z;
    //this->points_[i].vector = vector;
    //this->points_[i].snr = cart[3];
    //i++;
    //ROS_DEBUG("\tC: (%f, %f, %f, %f)\n", pt.x, pt.y, pt.z, pt.intensity);
    //ROS_DEBUG("\tS: (%f, %f, %f, %f)\n", sph[0], sph[1], pt.z, pt.intensity);
  }
  if (std::isnan(mean(acc))){
    odom.twist.twist.linear.x = 0;
  }else{
  //for (const pcl::PointXYZI &pt: msg->points){
    //float cart[4] = {pt.x, pt.y, 0, pt.intensity};
    //float sph[3] = {0, 0, 0};

   // gtrack_cartesian2spherical(GTRACK_STATE_VECTORS_2DA, cart, sph);


   //GTRACK_measurement_vector vector;
   //vector.range = sph[0];
   //vector.angle = sph[1];
   //vector.doppler = pt.z;
   //if (abs(sph[1]) > 0.1 && sph[0] > 0.01){
	//acc(-1.0f * (mean(acc) / cos(sph[1]))); 
    //}
    //ROS_DEBUG("\tRange: %f, Angle: %f, Doppler: %f, SNR: %f\n",vector.range, vector.angle, vector.doppler, cart[3]);
    //}
    
    odom.twist.twist.linear.x = mean(acc);
  //}

  this->ego_velocity.publish(odom);  
  //number of input measurements
  //this->mNum_ = i;
  
  //ROS_DEBUG("\tRange: %f\n", this->points[0].vector.range);
  //gtrack_step(this->hTracker_, this->points_, NULL, this->mNum_, this->targetDescr_, &(this->tNum_), NULL, NULL);

  this->end_ = ros::WallTime::now();	
  double execution_time = (this->end_ - this->start_).toNSec() * 1e-6;
  auto n = count(acc);
  auto stdev = std::sqrt(variance(acc)*(n/(n-1.0)));
  ROS_DEBUG_STREAM("\tExecution time between steps: " << execution_time << " seconds");
  ROS_DEBUG_STREAM("\nMean: " << mean(acc) << "\n"<< "Variance:" << variance(acc)  << "\n" << "Median:" << median(acc) << "\n" << "MtM:" << (std::abs(mean(acc) - median(acc))/std::max(mean(acc),median(acc))) << "\n" << "StDev:" << stdev << "\n");

  //if (this->tNum_ > 0) {
   // ROS_DEBUG("\tTracks: %d for %d meausrements\n", this->tNum_, this->mNum_);
    
    //this->collectAndUpdateActiveTracks();
  //}else{
    // ROS_DEBUG("\tNo Tracks!!!\n");
  //}
    
}
void Listener::publishMarker(uint32_t shape,  std::string frame_id, uint32_t marker_id, uint32_t tid, float posX, float posY, float red, float blue, float green, float depth, float width){

  ros::Duration one_second(1, 0);
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.id = marker_id;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = posX;
  marker.pose.position.y = posY;

  marker.pose.position.z = 0;

  if (shape == visualization_msgs::Marker::TEXT_VIEW_FACING)
  {

    marker.ns = "track_label";
    std::string text = "tid: ";
    text += std::to_string(tid)+ '\n' + "X:" + std::to_string(posX) + '\n' + "Y:" + std::to_string(posY);
    marker.text = text;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.a = 1.0f;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

  }
  else if (shape == visualization_msgs::Marker::SPHERE)
  { 

    marker.ns = "gating";
    marker.scale.x = depth;
    marker.scale.y = width;
    marker.scale.z = 0.1;
    marker.color.a = 0.2f;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

  }
  else if (shape == visualization_msgs::Marker::ARROW)
  {
    marker.ns = "velocity";

    geometry_msgs::Point start;
    geometry_msgs::Point end;

    start.x = 0;
    start.y = 0;

    end.x = depth;
    end.y = width;

    marker.points.push_back(start);
    marker.points.push_back(end);
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.color.a = 1.0f;

  }
  
  marker.lifetime = one_second;

  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  

  this->position_pub.publish(marker);
}

void Listener::publishAxisMarkers(std::string frame_id, uint32_t marker_id, std::string marker_label, float posX, float posY, float posZ){
  
  ros::Duration one_second(100, 0);
  uint32_t shape =  visualization_msgs::Marker::TEXT_VIEW_FACING;
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.id = marker_id;
  marker.ns = "basic_shapes";
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = one_second;
  marker.pose.position.x = posX;
  marker.pose.position.y = posY;
  marker.pose.position.z = posZ;

  marker.text = "tid: ";

  marker.scale.x = 1.25;
  marker.scale.y = 1.25;
  marker.scale.z = 1.25;


  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  this->position_pub.publish(marker);
}

void Listener::drawTrack(GTRACK_targetDesc_cpp targetDescr_){

    float depth = targetDescr_.dim[0];
    float width = targetDescr_.dim[1];
    float velocity = targetDescr_.dim[2];
    ROS_DEBUG("\tdepth: %f, width:%f, doppler:%f,", depth, width, velocity);
    uint32_t shape = visualization_msgs::Marker::SPHERE;
    uint32_t shape1 = visualization_msgs::Marker::TEXT_VIEW_FACING;
    uint32_t shape2 = visualization_msgs::Marker::ARROW;
    std::string frameName = "/base_radar_link_" + this->instanceID;
    this->publishMarker(shape, frameName, targetDescr_.tid, targetDescr_.tid, targetDescr_.S[0], (targetDescr_.S[1]), 0.0, 0.0, 1.0, depth, width);
    this->publishMarker(shape1, frameName, targetDescr_.tid, targetDescr_.tid, targetDescr_.S[0] + 0.15 , (targetDescr_.S[1] - (targetDescr_.dim[1]/ 2)  - 0.45), 1.0, 1.0, 1.0, 0, 0);
    this->publishMarker(shape2, frameName, targetDescr_.tid, targetDescr_.tid, targetDescr_.S[0], (targetDescr_.S[1]), 1.0, 0.0, 0.0, targetDescr_.S[2], targetDescr_.S[3]);
}

void Listener::collectAndUpdateActiveTracks()
{
  std::vector<GTrack_trackHeartbeat> veryNewActiveTracks;

  for (uint8_t i = 0; i < this->tNum_; i++)
  { 
    //@Implementation: refactor the class for a constructor to handle this in one line Karl 13.03.19 :
    GTrack_trackHeartbeat heartBeat;
    heartBeat.begin = ros::Time::now();
    heartBeat.tid = this->targetDescr_[i].tid;
    heartBeat.track = this->targetDescr_[i];

    //@Implementation: END Karl 13.03.19

    veryNewActiveTracks.push_back(heartBeat);
 
    #if MMWAVE_OCTOMAP_DEBUG
    // ROS_DEBUG("\tResolved tid:%d\n", this->targetDescr_[i].tid);
    #endif



    this->drawTrack(this->targetDescr_[i]);

    #if MMWAVE_OCTOMAP_DEBUG
        ROS_DEBUG("\tTrack %d found (X:%f Y:%f vx:%f, vy:%f)\n", this->targetDescr_[i].tid, this->targetDescr_[i].S[0], this->targetDescr_[i].S[1], this->targetDescr_[i].S[2], this->targetDescr_[i].S[3]);
    #endif
  }
  //@Incomplete: updating tracks is not working.
  // if (this->activeTracksList_.size() != 0)
  // {
    
  //   //@Efficiency: IDK about this
  //   std::sort(this->activeTracksList_.begin(), this->activeTracksList_.end());
  //   //@Efficiency: END

  //   for (auto &it : veryNewActiveTracks)
  //   {
  //     std::vector<GTrack_trackHeartbeat>::iterator iter = std::find(this->activeTracksList_.begin(), this->activeTracksList_.end(), it);
  //     if (iter != this->activeTracksList_.end())
  //     //element found, update
  //     {

  //       GTrack_trackHeartbeat& heartBeat = this->activeTracksList_[distance(this->activeTracksList_.begin(), iter)];

  //       it.tid = heartBeat.tid;
  //       it.begin = heartBeat.begin;
  //       it.track = heartBeat.track;


  //     }
  //   }
  // }
  // this->activeTracksList_ = std::move(veryNewActiveTracks);
  //@Incomplete: updating tracks is not working. END
}

void Listener::setConfig(TRACKING_ADVANCED_PARAM_SET trackingParamSet, GTRACK_moduleConfig config)
{
#if MMWAVE_OCTOMAP_DEBUG
  ROS_DEBUG("\tConfiguring tracker\n");
#endif

  switch (trackingParamSet)
  {
  case TRACKING_DEFAULT_PARAM_SET:
    // Do not configure advanced parameters, use library default parameters
    config.advParams = 0;
    break;

  case TRACKING_TRAFFIC_MONITORING_PARAM_SET:
    /* Initialize CLI configuration: */
    // memset((void *)&(this->advParams_), 0, sizeof(GTRACK_advancedParameters));
    // this->advParams_.sceneryParams = &appSceneryParamTable[TRACKING_PARAM_SET_TM];
    // this->advParams_.allocationParams = &appAllocationParamTable[TRACKING_PARAM_SET_TM];
    // this->advParams_.gatingParams = &appGatingParamTable[TRACKING_PARAM_SET_TM];
    // this->advParams_.stateParams = &appStateParamTable[TRACKING_PARAM_SET_TM];
    // this->advParams_.variationParams = &appVariationParamTable[TRACKING_PARAM_SET_TM];

    // config.advParams = &(this->advParams_);
    // config.initialRadialVelocity = -8; // for TM, detected targets are approaching
    // config.maxAcceleration[0] = 2;     // for PC, maximum acceleration in lateral direction is set to 5m/s2
    // config.maxAcceleration[1] = 20;    // for PC, maximum acceleration is longitudinal direction set to 5m/s2
    // config.maxAcceleration[2] = 5;
    break;

  case TRACKING_PEOPLE_COUNTING_PARAM_SET:

    /* Initialize CLI configuration: */
    memset((void *)&this->advParams_, 0, sizeof(GTRACK_advancedParameters));
    this->advParams_.sceneryParams = &appSceneryParamTable[TRACKING_PARAM_SET_PC];
    this->advParams_.allocationParams = &appAllocationParamTable[TRACKING_PARAM_SET_PC];
    this->advParams_.gatingParams = &appGatingParamTable[TRACKING_PARAM_SET_PC];
    this->advParams_.stateParams = &appStateParamTable[TRACKING_PARAM_SET_PC];
    this->advParams_.variationParams = &appVariationParamTable[TRACKING_PARAM_SET_PC];

    config.advParams = &this->advParams_;
    config.initialRadialVelocity = 0; //For PC, detected target velocity is unknown
    config.maxAcceleration[0] = 2;    // for PC, maximum acceleration in lateral direction is set to 5m/s2
    config.maxAcceleration[1] = 2;   // for PC, maximum acceleration is longitudinal direction set to 5m/s2
    config.maxAcceleration[2] = 2;
    break;
  }
  this->config_ = config;
#if MMWAVE_OCTOMAP_DEBUG
  ROS_DEBUG("\tTracker Configuration:\n");
  ROS_DEBUG("\tstateVectorType: %d\n", this->config_.stateVectorType);
  ROS_DEBUG("\tmaxNumPoints: %d\n", this->config_.maxNumPoints);
  ROS_DEBUG("\tmaxNumActiveTracks: %d\n", this->config_.maxNumTracks);
  ROS_DEBUG("\tmaxRadialVelocity: %f\n", this->config_.maxRadialVelocity);
  ROS_DEBUG("\tradialVelocityResolution: %f\n", this->config_.radialVelocityResolution);
  ROS_DEBUG("\tdeltaT: %f\n", this->config_.deltaT);
  ROS_DEBUG("\tinitialRadialVelocity: %f\n", this->config_.initialRadialVelocity);
  ROS_DEBUG("\tmaxAcceleration: [%f, %f, %f]\n", this->config_.maxAcceleration[0], this->config_.maxAcceleration[1], this->config_.maxAcceleration[2]);
  ROS_DEBUG("\tGTRACK_NUM_TRACKS_MAX: %d\n", GTRACK_NUM_TRACKS_MAX);
  #endif
}

void Listener::create_tracker()
{
  
  this->hTracker_ = gtrack_create(&(this->config_), &(this->errCode_));
  ROS_DEBUG("\tTracker address %p, errcode %d\n", this->hTracker_, this->errCode_);

}

int main(int argc, char *argv[])
{
  if (argc<2)
  {
   return -1;
  }

  Listener listener;
  listener.instanceID = argv[1];
  std::string instanceName = "mmWave_octomap" + listener.instanceID;

  ros::init(argc, argv, instanceName);

  ROSCONSOLE_AUTOINIT;

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::NodeHandle hNodeHandle;

  listener.position_pub = hNodeHandle.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  listener.ego_velocity = hNodeHandle.advertise<nav_msgs::Odometry>("odom0",1 );
  std::string topicName = "/mmWaveDataHdl" + listener.instanceID;
  topicName = topicName  + "/RScan";
  ros::Subscriber sub = hNodeHandle.subscribe<Listener::PointCloud>(topicName, 1000, &Listener::callback, &listener);
  
  GTRACK_moduleConfig config;
  config.stateVectorType = GTRACK_STATE_VECTORS_2DA; //Track two dimensions with acceleration
  config.verbose = GTRACK_VERBOSE_MAXIMUM;            //currently hardcoded, get from cfg in the future            //GTRACK_VERBOSE_NONE;              
  config.maxNumPoints = (uint16_t)200;               //currently hardcoded, get from cfg in the future
  config.maxNumTracks = (uint16_t)10;                //currently hardcoded, get from cfg in the future
  config.maxRadialVelocity = (float)2.35f;            //currently hardcoded, get from cfg in the future
  config.radialVelocityResolution = (float)0.15;    //currently hardcoded, get from cfg in the future
  config.deltaT = (float)80 * 0.001f;                //currently hardcoded, get from cfg in the future
  listener.setConfig(TRACKING_PEOPLE_COUNTING_PARAM_SET, config);
  listener.create_tracker();

  ros::spin();

  return 0;
}
