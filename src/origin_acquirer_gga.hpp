#ifndef ORIGIN_ACQUIRER_GGA_HPP
#define ORIGIN_ACQUIRER_GGA_HPP

#include <atomic>
#include "nmea/gga.h"
#include "lla.hpp"

class OriginAcquirerGga
{
public:
  inline OriginAcquirerGga()
      : originAcquired(false)
  {
  }

  inline void Callback(nmea::gga const &message)
  {
    this->lla = Lla(message.latitude, message.longitude, message.altitude);
    this->originAcquired = true;
  }

  inline bool IsOriginAcquired() { return this->originAcquired; }

  inline Lla GetLla() const { return this->lla; }

private:
  std::atomic<bool> originAcquired;
  Lla lla;
};

#endif // ORIGIN_ACQUIRER_GGA_HPP
