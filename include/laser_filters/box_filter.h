
#ifndef BOXFILTER_H
#define BOXFILTER_H

#include <dynamic_reconfigure/server.h>
#include <filters/filter_base.h>
#include <laser_filters/BoxFilterConfig.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>


namespace laser_filters
{
/**
 * @brief This is a filter that removes points in a laser scan inside of a cartesian box.
 */
class LaserScanBoxFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
  public:
    LaserScanBoxFilter();
    bool configure();

    bool update(
      const sensor_msgs::LaserScan& input_scan,
      sensor_msgs::LaserScan& filtered_scan);

  private:
    bool inBox(tf::Point &point);
    
    ////// TIEN ADD /////
    std::shared_ptr<dynamic_reconfigure::Server<laser_filters::BoxFilterConfig>> dyn_server_;
    void reconfigureCB(laser_filters::BoxFilterConfig& config, uint32_t level);

    boost::recursive_mutex own_mutex_;
    BoxFilterConfig config_ = BoxFilterConfig::__getDefault__();

    ///////////////////////////////////

    std::string box_frame_;
    laser_geometry::LaserProjection projector_;
    
    // tf listener to transform scans into the box_frame
    tf::TransformListener tf_; 
    
    // defines two opposite corners of the box
    tf::Point min_, max_; 
    bool invert_filter;
    bool up_and_running_;
};

}


#endif /* box_filter.h */
