#ifndef UTILITIES_HPP
#define UTILITIES_HPP

#include <tuple>
#include <tf/tf.h>

/// wrapper around geographic lib to follow functional style
std::tuple<double, double, double>
get_xyz(double origin_latitude_deg, double origin_longitude_deg,
        double origin_altitude_m, double new_latitude_deg,
        double new_longitude_deg, double new_altitude_m);
tf::Transform get_transform(double x, double y, double z, double yaw_degrees);
tf::StampedTransform get_transform(std::string const &from_frame_ID,
                                   std::string const &to_frame_ID);
double yaw_ned_to_enu(double const yaw_ned_degrees);
/**
    @return parameters set correct, parameters set, lat degrees, long degrees,
        altitude meters
*/
std::tuple<bool, bool, double, double, double> get_origin_params();
/// @return GNSS TF name or default value
std::string get_GNSS_frame_ID_parameter();
/// @return output TF name or default value
std::string get_output_tf_frame_ID_parameter();

#endif // UTILITIES_HPP
