#include "nmea/avr.h"
#include "nmea/gga.h"
#include "nmea/vtg.h"
#include "nmea_msgs/Sentence.h"
#include <ros/ros.h>
#include <boost/algorithm/string.hpp>
#include "nmea_lib/nmea_builder.hpp"
#include "nmea_lib/avr_fix_quality.hpp"
#include <tuple>

using std::string;
using std::stringstream;
using std::tuple;
using std::get;

tuple<uint8_t, uint8_t, double> get_utc_time(ros::Time const& time)
{
  double const time_s = std::fmod(time.toSec(), 60.0 * 60.0 * 24.0);
  uint8_t const hours = static_cast<uint8_t>(time_s / 60.0 / 60.0);
  uint8_t const minutes = 
    static_cast<uint8_t>((time_s / 60.0) - (hours * 60.0));
  double const seconds = 
      time_s - (minutes * 60.0) - (hours * 60.0 * 60.0);
  return tuple<uint8_t, uint8_t, double>(hours, minutes, seconds);
}

class NmeaMsgSubToNmeaStrPub
{
public:
  inline NmeaMsgSubToNmeaStrPub(ros::NodeHandle *const nh)
      : pub(nh->advertise<nmea_msgs::Sentence>("navsat/nmea_sentence", 10))
  {
  }

  inline void GgaCallback(nmea::gga const &message)
  {
    tuple<uint8_t, uint8_t, double> const hours_minutes_seconds =
        get_utc_time(message.stamp);
    string strToPub = build_gga(get<0>(hours_minutes_seconds),
        get<1>(hours_minutes_seconds), get<2>(hours_minutes_seconds), 
        message.latitude, message.longitude, 
        static_cast<GgaFixQuality>(message.fix_quality),
        message.num_satellites, message.hdop, message.altitude,
        message.geoid_height);
    this->pubNmeaSentence(strToPub);
  }

  inline void VtgCallback(nmea::vtg const &message)
  {
    double constexpr KPH_TO_MPS = 0.277778;
    string strToPub = build_vtg(message.true_track_made_good, 
        message.ground_speed_kph * KPH_TO_MPS);
    this->pubNmeaSentence(strToPub);
  }

  inline void pubNmeaSentence(string strToPub)
  {
    nmea_msgs::Sentence nmeaSent;
    nmeaSent.header.stamp = ros::Time::now();
    nmeaSent.sentence = strToPub;
    this->pub.publish(nmeaSent);
  }

private:
  ros::Publisher pub;
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "nmea_builder");

  ros::NodeHandle n;
  NmeaMsgSubToNmeaStrPub nmea_msg_sub_to_nmea_str_pub(&n);
  ros::Subscriber ggaSub =
      n.subscribe("gga", 10, &NmeaMsgSubToNmeaStrPub::GgaCallback,
                  &nmea_msg_sub_to_nmea_str_pub);
  ros::Subscriber vtgSub =
      n.subscribe("vtg", 10, &NmeaMsgSubToNmeaStrPub::VtgCallback,
                  &nmea_msg_sub_to_nmea_str_pub);

  ros::spin();

  return EXIT_SUCCESS;
}
