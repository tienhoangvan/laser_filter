#include "laser_filters/box_filter.h"
#include <ros/ros.h>
#include <ros/node_handle.h>

laser_filters::LaserScanBoxFilter::LaserScanBoxFilter(){

}

bool laser_filters::LaserScanBoxFilter::configure(){

  ros::NodeHandle private_nh("~" + getName());
  dyn_server_.reset(new dynamic_reconfigure::Server<laser_filters::BoxFilterConfig>(own_mutex_, private_nh));
  dynamic_reconfigure::Server<laser_filters::BoxFilterConfig>::CallbackType f;
  f = boost::bind(&laser_filters::LaserScanBoxFilter::reconfigureCB, this, _1, _2);
  dyn_server_->setCallback(f);

  up_and_running_ = true;
  //double min_x = 0, min_y = 0, min_z = 0, max_x = 0, max_y = 0, max_z = 0;
  bool box_frame_set = getParam("box_frame", box_frame_);
  bool x_max_set = getParam("max_x", config_.max_x);
  bool y_max_set = getParam("max_y", config_.max_y);
  bool z_max_set = getParam("max_z", config_.max_z);
  bool x_min_set = getParam("min_x", config_.min_x);
  bool y_min_set = getParam("min_y", config_.min_y);
  bool z_min_set = getParam("min_z", config_.min_z);
  bool invert_set = getParam("invert", config_.invert);

  invert_filter = config_.invert;

  dyn_server_->updateConfig(config_);
  
  ROS_INFO("BOX filter started");

  max_.setX(config_.max_x);
  max_.setY(config_.max_y);
  max_.setZ(config_.max_z);
  min_.setX(config_.min_x);
  min_.setY(config_.min_y);
  min_.setZ(config_.min_z);
  
  if(!box_frame_set){
    ROS_ERROR("box_frame is not set!");
  }
  if(!x_max_set){
    ROS_ERROR("max_x is not set!");
  }
  if(!y_max_set){
    ROS_ERROR("max_y is not set!");
  }
  if(!z_max_set){
    ROS_ERROR("max_z is not set!");
  }
  if(!x_min_set){
    ROS_ERROR("min_x is not set!");
  }
  if(!y_min_set){
    ROS_ERROR("min_y is not set!");
  }
  if(!z_min_set){
    ROS_ERROR("min_z is not set!");
  }
  if(!invert_set){
    ROS_INFO("invert filter not set, assuming false");
    invert_filter=false;
  }


  return box_frame_set && x_max_set && y_max_set && z_max_set &&
    x_min_set && y_min_set && z_min_set;

}

bool laser_filters::LaserScanBoxFilter::update(
    const sensor_msgs::LaserScan& input_scan,
    sensor_msgs::LaserScan &output_scan)
{
  output_scan = input_scan;
  sensor_msgs::PointCloud2 laser_cloud;
  
  std::string error_msg;

  bool success = tf_.waitForTransform(
    box_frame_,
    input_scan.header.frame_id,
    input_scan.header.stamp + ros::Duration().fromSec(input_scan.ranges.size()*input_scan.time_increment),
    ros::Duration(1.0),
    ros::Duration(0.01),
    &error_msg
  );
  if(!success){
    ROS_WARN("Could not get transform, irgnoring laser scan! %s", error_msg.c_str());
    return false;
  }

  try{
    projector_.transformLaserScanToPointCloud(box_frame_, input_scan, laser_cloud, tf_);
  }
  catch(tf::TransformException& ex){
    if(up_and_running_){
      ROS_WARN_THROTTLE(1, "Dropping Scan: Tansform unavailable %s", ex.what());
      return true;
    }
    else
    {
      ROS_INFO_THROTTLE(.3, "Ignoring Scan: Waiting for TF");
    }
    return false;
  }
  const int i_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "index");
  const int x_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "x");
  const int y_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "y");
  const int z_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "z");

  if(i_idx_c == -1 || x_idx_c == -1 || y_idx_c == -1 || z_idx_c == -1){
      ROS_INFO_THROTTLE(.3, "x, y, z and index fields are required, skipping scan");

  }


  const int i_idx_offset = laser_cloud.fields[i_idx_c].offset;
  const int x_idx_offset = laser_cloud.fields[x_idx_c].offset;
  const int y_idx_offset = laser_cloud.fields[y_idx_c].offset;
  const int z_idx_offset = laser_cloud.fields[z_idx_c].offset;

  const int pstep = laser_cloud.point_step;
  const long int pcount = laser_cloud.width * laser_cloud.height;
  const long int limit = pstep * pcount;

  int i_idx, x_idx, y_idx, z_idx;  
  for(
    i_idx = i_idx_offset,
    x_idx = x_idx_offset,
    y_idx = y_idx_offset,
    z_idx = z_idx_offset;

    x_idx < limit;

    i_idx += pstep,
    x_idx += pstep,
    y_idx += pstep,
    z_idx += pstep)
  {

    // TODO works only for float data types and with an index field
    // I'm working on it, see https://github.com/ros/common_msgs/pull/78 
    float x = *((float*)(&laser_cloud.data[x_idx]));
    float y = *((float*)(&laser_cloud.data[y_idx]));
    float z = *((float*)(&laser_cloud.data[z_idx]));
    int index = *((int*)(&laser_cloud.data[i_idx]));

    tf::Point point(x, y, z);

    if(!invert_filter){
      if(inBox(point)){
        output_scan.ranges[index] = std::numeric_limits<float>::quiet_NaN();
      }
    }
    else{
      if(!inBox(point)){
        output_scan.ranges[index] = std::numeric_limits<float>::quiet_NaN();
      }
    }

  }
  up_and_running_ = true;
  return true;
}

bool laser_filters::LaserScanBoxFilter::inBox(tf::Point &point){
  return
    point.x() < max_.x() && point.x() > min_.x() && 
    point.y() < max_.y() && point.y() > min_.y() &&
    point.z() < max_.z() && point.z() > min_.z();
}

void laser_filters::LaserScanBoxFilter::reconfigureCB(laser_filters::BoxFilterConfig& config, uint32_t level)
{
  config_ = config;

}
