#include <atomic>
#include <mutex>
#include <tuple>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <ros/callback_queue.h>
#include <tf/transform_listener.h>
#include "raw_nmea/gga.h"
#include "raw_nmea/avr.h"
#include "utilities.hpp"
#include "local_pose_2_5_d.hpp"
#include "origin_acquirer_gga.hpp"

using std::tuple;
using std::get;
using std::atomic;
using std::mutex;
using std::string;

class GlobalToLocalSynchronizer
{
public:
  inline GlobalToLocalSynchronizer(Lla const &origin_in,
                                   ros::NodeHandle *const nh,
                                   std::string GNSS_frame_ID,
                                   string const &output_tf_frame_name)
      : origin(origin_in)
      , outputTFFrameName(output_tf_frame_name)
  {
    this->gnssToBaseLinkTransform =
        get_transform(GNSS_frame_ID, this->outputTFFrameName);
  }

  inline void GgaCallback(raw_nmea::gga const &message)
  {
    auto const xyz =
        get_xyz(this->origin.latitudeDeg, this->origin.longitudeDeg,
                this->origin.altitudeM, message.latitude, message.longitude,
                message.altitude);
    {
      std::lock_guard<mutex> lock(this->stateMx);
      this->state.x = get<0>(xyz);
      this->state.y = get<1>(xyz);
      this->state.z = get<2>(xyz);
    }

    if (raw_nmea::gga::INVALID != message.fix_quality)
    {
      LocalPose2_5D const local_pose(this->GetState());
      auto const world_to_gnss_transform = get_transform(
          local_pose.x, local_pose.y, local_pose.z, local_pose.yaw);
      auto const now = ros::Time::now();
      this->br.sendTransform(tf::StampedTransform(
          world_to_gnss_transform * this->gnssToBaseLinkTransform, now, "world",
          this->outputTFFrameName));
    }
    else
    {
      ROS_WARN("Not sure how to handle GGA mode INVALID");
    }
  }

  inline void AvrCallback(raw_nmea::avr const &message)
  {
    if (raw_nmea::avr::INVALID == message.fix_quality)
    {
      ROS_WARN("Not sure how to handle AVR mode INVALID");
    }
    else
    {
      this->avrYaw = message.yaw;
    }
  }

  inline LocalPose2_5D GetState()
  {
    std::lock_guard<mutex> lock(this->stateMx);
    this->state.yaw = yaw_ned_to_enu(this->avrYaw);
    return this->state;
  }

  inline double GetAvrYaw() { return this->avrYaw; }

private:
  Lla origin;
  string outputTFFrameName;
  tf::TransformBroadcaster br;

  atomic<double> avrYaw;
  LocalPose2_5D state;
  mutex stateMx;
  tf::StampedTransform gnssToBaseLinkTransform;
};

int main(int argc, char *argv[])
{
  static char const *const GGA_TOPIC_NAME = "gga";
  int exit_code = EXIT_SUCCESS;
  ros::init(argc, argv, "gga_and_avr_tf");

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
      {
        OriginAcquirerGga oa;
        ros::Subscriber sub =
            n.subscribe(GGA_TOPIC_NAME, 1, &OriginAcquirerGga::Callback, &oa);
        while (!oa.IsOriginAcquired() && ros::ok())
        {
          ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
        }
        if (ros::ok())
        {
          origin = oa.GetLla();
        }
      }
    }
    if (ros::ok())
    {
      static string const output_tf_frame_name =
          get_output_tf_frame_ID_parameter();
      static string const GNSS_frame_ID = get_GNSS_frame_ID_parameter();
      ROS_INFO("origin = (%.10f deg, %.10f deg, %0.6f m)", origin.latitudeDeg,
               origin.longitudeDeg, origin.altitudeM);
      GlobalToLocalSynchronizer global_to_local_synchronizer(
          origin, &n, GNSS_frame_ID, output_tf_frame_name);

      ros::Subscriber avr_sub =
          n.subscribe("avr", 1, &GlobalToLocalSynchronizer::AvrCallback,
                      &global_to_local_synchronizer);
      ros::Subscriber gga_sub = n.subscribe(
          GGA_TOPIC_NAME, 1, &GlobalToLocalSynchronizer::GgaCallback,
          &global_to_local_synchronizer);

      while (ros::ok())
      {
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.05));
      }
    }
  }

  return exit_code;
}
