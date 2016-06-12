#include "nmea/gga.h"
#include "nmea_msgs/Sentence.h"
#include "sensor_msgs/NavSatFix.h"
#include <ros/ros.h>
#include <boost/algorithm/string.hpp>

using std::string;
using std::vector;
using std::tuple;
using std::get;

enum FixQuality
{
  INVALID = 0,
  GPS_FIX,
  DGPS_FIX,
  PPS_FIX,
  REAL_TIME_KINEMATIC,
  FLOAT_RTK,
  DEAD_RECKONING,
  MANUAL_INPUT,
  SIMULATION
};

tuple<bool, double> parse_angle(string const &message);

nmea::gga nsf_to_gga_ros_msg(sensor_msgs::NavSatFix const &message);

tuple<bool, double> parse_angle(string const &message)
{
  static int const EXPECTED_WORDS_AROUND_PERIOD = 2;
  bool is_valid = false;
  double angle_degrees;
  std::vector<string> words_around_period;
  boost::split(words_around_period, message, boost::is_any_of("."),
               boost::token_compress_on);
  if (EXPECTED_WORDS_AROUND_PERIOD == words_around_period.size())
  {
    double const whole_degrees = stod(
        words_around_period[0].substr(0, words_around_period[0].size() - 2));
    double const whole_minutes = stod(words_around_period[0].substr(
        words_around_period[0].size() - 2, words_around_period[0].size()));
    double const decimal_minutes = stod(words_around_period[1]) /
                                   std::pow(10, words_around_period[1].size());
    angle_degrees = whole_degrees + (whole_minutes + decimal_minutes) / 60.0;
    is_valid = true;
  }

  return tuple<bool, double>(is_valid, angle_degrees);
}

nmea::gga nsf_to_gga_ros_msg(sensor_msgs::NavSatFix const &message)
{
  nmea::gga ros_msg;
  ros_msg.stamp = message.header.stamp;
  ros_msg.latitude = message.latitude;
  ros_msg.longitude = message.longitude;
  ros_msg.fix_quality = REAL_TIME_KINEMATIC;
  ros_msg.num_satellites = 8;
  ros_msg.hdop = 1.0;
  ros_msg.altitude = message.altitude + 0; // raw_msg.geoidHeight;
  /*    ros_msg.time_since_last_dgps_valid = raw_msg.timeSinceLastDgpsValid;
      if (raw_msg.timeSinceLastDgpsValid)
      {
          ros_msg.time_since_last_dgps.fromSec(raw_msg.timeSinceLastDgps);
      }
      ros_msg.dgps_station_id_valid = raw_msg.dgpdStationIDValid;
      if (raw_msg.dgpdStationIDValid)
      {
          ros_msg.dgps_station_id = raw_msg.dgpdStationID;
      }*/
  return ros_msg;
}

class NmeaSubNsfToNmeaPubGga
{
public:
  inline NmeaSubNsfToNmeaPubGga(ros::NodeHandle *const nh)
      : ggaPub(nh->advertise<nmea::gga>("gga", 10))
  {
  }

  inline void Callback(sensor_msgs::NavSatFix const &message)
  {
    this->ggaPub.publish(nsf_to_gga_ros_msg(message));
  }

private:
  ros::Publisher ggaPub;
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "nav_to_gga");

  ros::NodeHandle n;
  NmeaSubNsfToNmeaPubGga nmea_sub_nsf_to_nmea_pub_gga(&n);
  ros::Subscriber sub =
      n.subscribe("gps/fix", 10, &NmeaSubNsfToNmeaPubGga::Callback,
                  &nmea_sub_nsf_to_nmea_pub_gga);

  ros::spin();

  return EXIT_SUCCESS;
}
