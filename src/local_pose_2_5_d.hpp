#ifndef LOCAL_POSE_2_5_D_HPP
#define LOCAL_POSE_2_5_D_HPP

struct LocalPose2_5D
{
  inline LocalPose2_5D()
      : x(0.0)
      , y(0.0)
      , z(0.0)
      , yaw(0.0)
  {
  }

  inline LocalPose2_5D(double const x_in, double const y_in, double const z_in,
                       double const yaw_in)
      : x(x_in)
      , y(y_in)
      , z(z_in)
      , yaw(yaw_in)
  {
  }

  double x;
  double y;
  double z;
  double yaw;
};

#endif // LOCAL_POSE_2_5_D_HPP
