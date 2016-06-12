#include <tuple>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <ros/callback_queue.h>
#include <tf/transform_listener.h>
#include "sensor_msgs/NavSatFix.h"
#include "utilities.hpp"
#include "lla.hpp"
#include "local_pose_2_5_d.hpp"
#include "origin_acquirer_navsatfix.hpp"
#include <angles/angles.h>

using std::tuple;
using std::get;
using std::atomic;
using std::string;

class GlobalToLocalSynchronizer
{
public:
  inline GlobalToLocalSynchronizer(Lla const &origin_in,
                                   ros::NodeHandle *const nh,
                                   std::string GNSS_frame_ID,
                                   std::string const &output_tf_frame_name)
      : origin(origin_in)
      , outputTFFrameName(output_tf_frame_name)
  {
    this->gnssToBaseLinkTransform =
        get_transform(GNSS_frame_ID, this->outputTFFrameName);
  }

  inline void NavSatFixCallback(sensor_msgs::NavSatFix const &message)
  {
    if (sensor_msgs::NavSatStatus::STATUS_NO_FIX == message.status.status)
    {
      ROS_WARN("NavSatFix.status: No fix");
    }
    else
    {
      tuple<double, double, double> xyz =
          get_xyz(this->origin.latitudeDeg, this->origin.longitudeDeg,
                  this->origin.altitudeM, message.latitude, message.longitude,
                  message.altitude);
      LocalPose2_5D const local_pose(this->GetState(xyz, this->prevState));
      this->prevState = local_pose;
      auto const world_to_gnss_transform = get_transform(
          local_pose.x, local_pose.y, local_pose.z, local_pose.yaw);
      auto const now = ros::Time::now();
      this->br.sendTransform(tf::StampedTransform(
          world_to_gnss_transform * this->gnssToBaseLinkTransform, now, "world",
          this->outputTFFrameName));
    }
  }

  static inline LocalPose2_5D GetState(tuple<double, double, double> const &xyz,
                                       LocalPose2_5D const &prev_state)
  {
    static double const SAME_LOCATION_TOLERANCE_M = 0.001;
    double const dx = get<0>(xyz) - prev_state.x;
    double const dy = get<1>(xyz) - prev_state.y;
    double yaw;
    if (sqrt(dx * dx + dy * dy) < SAME_LOCATION_TOLERANCE_M)
    {
      yaw = prev_state.yaw;
    }
    else
    {
      yaw = angles::to_degrees(atan2(dy, dx));
    }
    return LocalPose2_5D(get<0>(xyz), get<1>(xyz), get<2>(xyz), yaw);
  }

private:
  Lla origin;
  string outputTFFrameName;

  LocalPose2_5D prevState;
  tf::TransformBroadcaster br;
  tf::StampedTransform gnssToBaseLinkTransform;
};

int main(int argc, char *argv[])
{
  static char const *const NAVSATFIX_TOPIC_NAME = "fix";
  int exit_code;
  ros::init(argc, argv, "navsatfix_to_tf");

  ros::NodeHandle n;
  std::tuple<bool, bool, double, double, double> const origin_params(
      get_origin_params());
  if (!get<0>(origin_params))
  {
    exit_code = EXIT_FAILURE;
  }
  else
  {
    Lla origin;
    if (get<1>(origin_params))
    {
      origin = Lla(get<2>(origin_params), get<3>(origin_params),
                   get<4>(origin_params));
      ROS_INFO("using origin from parameter");
    }
    else
    {
      ROS_INFO("acquiring origin from first lat long");
      OriginAcquirerNavSatFix oa;
      ros::Subscriber sub = n.subscribe(
          NAVSATFIX_TOPIC_NAME, 1, &OriginAcquirerNavSatFix::Callback, &oa);
      while (!oa.IsOriginAcquired() && ros::ok())
      {
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
      }
      if (ros::ok())
      {
        origin = oa.GetLla();
      }
    }
    if (ros::ok())
    {
      static std::string const output_tf_frame_name =
          get_output_tf_frame_ID_parameter();
      ROS_INFO("origin = (%.10f deg, %.10f deg, %0.6f m)", origin.latitudeDeg,
               origin.longitudeDeg, origin.altitudeM);
      GlobalToLocalSynchronizer global_to_local_synchronizer(
          origin, &n, get_GNSS_frame_ID_parameter(), output_tf_frame_name);

      ros::Subscriber navsatfix_sub =
          n.subscribe(NAVSATFIX_TOPIC_NAME, 1,
                      &GlobalToLocalSynchronizer::NavSatFixCallback,
                      &global_to_local_synchronizer);

      while (ros::ok())
      {
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.05));
      }
    }

    exit_code = EXIT_SUCCESS;
  }

  return exit_code;
}
