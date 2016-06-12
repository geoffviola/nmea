#ifndef ORIGIN_ACQUIRER_NAVSATFIX_HPP
#define ORIGIN_ACQUIRER_NAVSATFIX_HPP

#include <atomic>
#include "sensor_msgs/NavSatFix.h"

class OriginAcquirerNavSatFix
{
public:
  inline OriginAcquirerNavSatFix()
      : originAcquired(false)
  {
  }

  inline void Callback(sensor_msgs::NavSatFix const &message)
  {
    this->lla = Lla(message.latitude, message.longitude, message.altitude);
    this->originAcquired = true;
  }

  inline bool IsOriginAcquired() { return this->originAcquired; }

  inline Lla GetLla() { return this->lla; }

private:
  std::atomic<bool> originAcquired;
  Lla lla;
};

#endif // ORIGIN_ACQUIRER_NAVSATFIX_HPP
