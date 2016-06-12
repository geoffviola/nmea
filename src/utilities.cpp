#include "utilities.hpp"
#include <tuple>
#include <tf/transform_listener.h>
#include <angles/angles.h>
#include "enu/enu.h"
#include "sensor_msgs/NavSatFix.h"

using std::tuple;
using std::string;

tuple<double, double, double>
get_xyz(double origin_latitude_deg, double origin_longitude_deg,
        double origin_altitude_m, double new_latitude_deg,
        double new_longitude_deg, double new_altitude_m)
{
  sensor_msgs::NavSatFix fix;
  fix.latitude = new_latitude_deg;
  fix.longitude = new_longitude_deg;
  fix.altitude = new_altitude_m;
  sensor_msgs::NavSatFix datum;
  datum.latitude = origin_latitude_deg;
  datum.longitude = origin_longitude_deg;
  datum.altitude = origin_altitude_m;
  geometry_msgs::Point point;
  enu::fix_to_point(fix, datum, &point);
  return tuple<double, double, double>(point.x, point.y, point.z);
}

tf::Transform get_transform(double const x, double const y, double const z,
                            double const yaw_degrees)
{
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(x, y, z));
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, angles::from_degrees(yaw_degrees));
  transform.setRotation(q);
  return transform;
}

tf::StampedTransform get_transform(string const &from_frame_ID,
                                   string const &to_frame_ID)
{
  tf::StampedTransform output;
  bool tf_acquired = false;
  while (!tf_acquired && ros::ok())
  {
    try
    {
      tf::TransformListener listener;
      ros::Time now = ros::Time(0);
      bool found_tf = false;
      do
      {
        found_tf = listener.waitForTransform(from_frame_ID, to_frame_ID, now,
                                             ros::Duration(1.0));
        if (!found_tf)
        {
          ROS_INFO("Waiting for tf from %s to %s", from_frame_ID.c_str(),
                   to_frame_ID.c_str());
        }
      } while (!found_tf && ros::ok());
      listener.lookupTransform(from_frame_ID, to_frame_ID, now, output);
      tf_acquired = true;
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(0.1).sleep();
    }
  }
  return output;
}

double yaw_ned_to_enu(double const yaw_ned_degrees)
{
  return -1.0 * (yaw_ned_degrees - 90.0);
}

tuple<bool, bool, double, double, double> get_origin_params()
{
  double param_origin_lat_degrees, param_origin_long_degrees,
      param_origin_alt_meters;
  bool const param_origin_lat_degrees_present =
      ros::param::get("~origin_lat_degrees", param_origin_lat_degrees);
  bool const param_origin_long_degrees_present =
      ros::param::get("~origin_long_degrees", param_origin_long_degrees);
  bool const param_origin_alt_meters_present =
      ros::param::get("~origin_alt_meters", param_origin_alt_meters);
  bool const parameters_set = true == param_origin_lat_degrees_present &&
                              true == param_origin_long_degrees_present &&
                              true == param_origin_alt_meters_present;
  bool const parameters_set_properly =
      (parameters_set || (false == param_origin_lat_degrees_present &&
                          false == param_origin_long_degrees_present &&
                          false == param_origin_alt_meters_present));
  if (!parameters_set_properly)
  {
    ROS_FATAL("all parameters \"origin_lat_degrees\", "
              "\"origin_long_degrees\", "
              "and \"origin_alt_meters\" "
              "must be set or not set.\n"
              "origin_lat_degrees is %s\n"
              "origin_long_degrees is %s\n"
              "origin_alt_meters is %s\n",
              (param_origin_lat_degrees_present ? "set" : "unset"),
              (param_origin_long_degrees_present ? "set" : "unset"),
              (param_origin_alt_meters_present ? "set" : "unset"));
  }
  return tuple<bool, bool, double, double, double>(
      parameters_set_properly, parameters_set, param_origin_lat_degrees,
      param_origin_long_degrees, param_origin_alt_meters);
}

string get_GNSS_frame_ID_parameter()
{
  std::string GNSS_frame_ID;
  if (!ros::param::get("~GNSS_frame_ID", GNSS_frame_ID))
  {
    GNSS_frame_ID = "GNSS";
  }
  return GNSS_frame_ID;
}

string get_output_tf_frame_ID_parameter()
{
  std::string GNSS_frame_ID;
  if (!ros::param::get("~output_tf_frame_id", GNSS_frame_ID))
  {
    GNSS_frame_ID = "base_footprint";
  }
  return GNSS_frame_ID;
}
