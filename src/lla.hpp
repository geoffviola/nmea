#ifndef LLA_HPP
#define LLA_HPP

struct Lla
{
  inline Lla() {}

  inline Lla(double const latitude_deg, double const longitude_deg,
             double const altitude_deg)
      : latitudeDeg(latitude_deg)
      , longitudeDeg(longitude_deg)
      , altitudeM(altitude_deg)
  {
  }

  double latitudeDeg;
  double longitudeDeg;
  double altitudeM;
};

#endif // LLA_HPP
