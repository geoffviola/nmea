#include "nmea/avr.h"
#include "nmea/gga.h"
#include "nmea/vtg.h"
#include "nmea_msgs/Sentence.h"
#include <ros/ros.h>
#include <boost/algorithm/string.hpp>
#include <nmea_lib/nmea_parser.hpp>

using std::string;
using std::vector;
using std::tuple;
using std::get;

nmea::avr to_ros_msg(AvrMessageData const &msg)
{
  nmea::avr ros_msg;
  ros_msg.stamp.fromSec(msg.timestamp);
  ros_msg.yaw = msg.yaw;
  ros_msg.tilt = msg.tilt;
  ros_msg.range = msg.range;
  ros_msg.fix_quality = static_cast<uint8_t>(msg.fixQuality);
  ros_msg.pdop = msg.pdop;
  ros_msg.num_satellites = msg.numSatellites;
  return ros_msg;
}

nmea::gga to_ros_msg(GgaMessageData const &msg)
{
  nmea::gga ros_msg;
  ros_msg.stamp.fromSec(msg.timestamp);
  ros_msg.latitude = msg.latitude;
  ros_msg.longitude = msg.longitude;
  ros_msg.fix_quality = msg.fixQuality;
  ros_msg.num_satellites = msg.numSatellites;
  ros_msg.hdop = msg.hdop;
  ros_msg.altitude = msg.altitude + msg.geoidHeight;
  ros_msg.time_since_last_dgps_valid = msg.timeSinceLastDgpsValid;
  if (msg.timeSinceLastDgpsValid)
  {
    ros_msg.time_since_last_dgps.fromSec(msg.timeSinceLastDgps);
  }
  ros_msg.dgps_station_id_valid = msg.dgpdStationIDValid;
  if (msg.dgpdStationIDValid)
  {
    ros_msg.dgps_station_id = msg.dgpdStationID;
  }
  return ros_msg;
}

nmea::vtg to_ros_msg(VtgMessageData const &msg)
{
  nmea::vtg ros_msg;
  ros_msg.true_track_made_good = msg.trueTrackMadeGood;
  ros_msg.magnetic_track_made_good = msg.magneticTrackMadeGood;
  ros_msg.ground_speed_knots = msg.groundSpeedKnots;
  ros_msg.ground_speed_kph = msg.groundSpeedKph;
  return ros_msg;
}

class NmeaSubToNmeaPub
{
public:
  inline NmeaSubToNmeaPub(ros::NodeHandle *const nh)
      : avrPub(nh->advertise<nmea::avr>("avr", 10))
      , ggaPub(nh->advertise<nmea::gga>("gga", 10))
      , vtgPub(nh->advertise<nmea::vtg>("vtg", 10))
  {
  }

  inline void Callback(nmea_msgs::Sentence const &message)
  {
    AvrMessageData avr = parse_avr(message.sentence);
    if (avr.valid)
    {
      this->avrPub.publish(to_ros_msg(avr));
    }

    GgaMessageData const gga = parse_gga(message.sentence);
    if (gga.valid)
    {
      this->ggaPub.publish(to_ros_msg(gga));
    }

    VtgMessageData vtg = parse_vtg(message.sentence);
    if (vtg.valid)
    {
      this->vtgPub.publish(to_ros_msg(vtg));
    }
  }

private:
  ros::Publisher avrPub;
  ros::Publisher ggaPub;
  ros::Publisher vtgPub;
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "nmea_parser");

  ros::NodeHandle n;
  NmeaSubToNmeaPub nmea_sub_to_nmea_pub(&n);
  ros::Subscriber sub =
      n.subscribe("navsat/nmea_sentence", 10, &NmeaSubToNmeaPub::Callback,
                  &nmea_sub_to_nmea_pub);

  ros::spin();

  return EXIT_SUCCESS;
}
