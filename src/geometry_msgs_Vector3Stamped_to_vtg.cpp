#include "nmea/vtg.h"
#include <ros/ros.h>
#include <boost/algorithm/string.hpp>
#include "geometry_msgs/Vector3Stamped.h"
#include "angles/angles.h"

using std::string;
using std::vector;
using std::tuple;
using std::get;

nmea::vtg
v3s_to_vtg_ros_msg(geometry_msgs::Vector3Stamped const &message);

nmea::vtg
v3s_to_vtg_ros_msg(geometry_msgs::Vector3Stamped const &message)
{
  double const speedMagMps = sqrt(message.vector.x * message.vector.x +
                                  message.vector.y * message.vector.y +
                                  message.vector.z * message.vector.z);

  double const trackFromEastRad = atan2(-message.vector.y, message.vector.x);
  double const trackFromNorthDeg = angles::to_degrees(trackFromEastRad);

  nmea::vtg ros_msg;
  ros_msg.true_track_made_good = trackFromNorthDeg;
  ros_msg.magnetic_track_made_good_valid = false;
  ros_msg.ground_speed_knots = speedMagMps * 1.94384;
  ros_msg.ground_speed_kph = speedMagMps * 3.6;
  return ros_msg;
}

class NmeaSubVelToNmeaPubVtg
{
public:
  inline NmeaSubVelToNmeaPubVtg(ros::NodeHandle *const nh)
      : vtgPub(nh->advertise<nmea::vtg>("vtg", 10))
  {
  }

  inline void Callback(geometry_msgs::Vector3Stamped const &message)
  {
    this->vtgPub.publish(v3s_to_vtg_ros_msg(message));
  }

private:
  ros::Publisher vtgPub;
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "vel_to_vtg");

  ros::NodeHandle n;
  NmeaSubVelToNmeaPubVtg nmea_sub_vel_to_nmea_pub_vtg(&n);
  ros::Subscriber sub =
      n.subscribe("gps/fix_velocity", 10, &NmeaSubVelToNmeaPubVtg::Callback,
                  &nmea_sub_vel_to_nmea_pub_vtg);

  ros::spin();

  return EXIT_SUCCESS;
}
