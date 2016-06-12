cmake_minimum_required(VERSION 2.8.3)
project(nmea)

find_package(catkin REQUIRED COMPONENTS roscpp tf message_generation
    std_msgs sensor_msgs cmake_modules enu nmea_lib)

add_message_files(FILES avr.msg gga.msg hdt.msg vtg.msg)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs sensor_msgs tf
)

message("${nmea_lib_INCLUDE_DIRS};${nmealib_INCLUDE_DIRS}")
message("enu_INCLUDE_DIRS = ${enu_INCLUDE_DIRS}")
include_directories(${enu_INCLUDE_DIRS} ${nmea_lib_INCLUDE_DIRS})

add_definitions("-std=c++11 -Wall -Werror")

add_library(nmea_utilities
    src/utilities.cpp
    src/utilities.hpp
    src/lla.hpp
    src/local_pose_2_5_d.hpp
    src/origin_acquirer_gga.hpp
    src/origin_acquirer_navsatfix.hpp)
target_link_libraries(nmea_utilities ${catkin_LIBRARIES})
add_dependencies(nmea_utilities ${catkin_EXPORTED_TARGETS})
add_dependencies(nmea_utilities nmea_generate_messages_cpp)

add_executable(gga_and_avr_to_tf src/gga_and_avr_to_tf.cpp)
target_link_libraries(gga_and_avr_to_tf ${catkin_LIBRARIES} nmea_utilities)
add_dependencies(gga_and_avr_to_tf ${catkin_EXPORTED_TARGETS})
add_dependencies(gga_and_avr_to_tf nmea_generate_messages_cpp)

add_executable(gga_and_vtg_to_tf src/gga_and_vtg_to_tf.cpp)
target_link_libraries(gga_and_vtg_to_tf ${catkin_LIBRARIES} nmea_utilities)
add_dependencies(gga_and_vtg_to_tf ${catkin_EXPORTED_TARGETS})
add_dependencies(gga_and_vtg_to_tf nmea_generate_messages_cpp)

add_executable(navsatfix_to_tf src/navsatfix_to_tf.cpp)
target_link_libraries(navsatfix_to_tf ${catkin_LIBRARIES} nmea_utilities)
add_dependencies(navsatfix_to_tf ${catkin_EXPORTED_TARGETS})
add_dependencies(navsatfix_to_tf nmea_generate_messages_cpp)

add_executable(nmea_parser src/nmea_parser.cpp)
target_link_libraries(nmea_parser ${catkin_LIBRARIES})
add_dependencies(nmea_parser ${catkin_EXPORTED_TARGETS})
add_dependencies(nmea_parser nmea_generate_messages_cpp)

add_executable(sensor_msgs_navsatfix_to_gga src/sensor_msgs_navsatfix_to_gga.cpp)
target_link_libraries(sensor_msgs_navsatfix_to_gga ${catkin_LIBRARIES})
add_dependencies(sensor_msgs_navsatfix_to_gga ${catkin_EXPORTED_TARGETS})
add_dependencies(sensor_msgs_navsatfix_to_gga nmea_generate_messages_cpp)

add_executable(geometry_msgs_Vector3Stamped_to_vtg
    src/geometry_msgs_Vector3Stamped_to_vtg.cpp)
target_link_libraries(geometry_msgs_Vector3Stamped_to_vtg ${catkin_LIBRARIES})
add_dependencies(geometry_msgs_Vector3Stamped_to_vtg ${catkin_EXPORTED_TARGETS})
add_dependencies(geometry_msgs_Vector3Stamped_to_vtg nmea_generate_messages_cpp)

add_executable(nmea_builder src/nmea_builder.cpp)
target_link_libraries(nmea_builder ${catkin_LIBRARIES})
add_dependencies(nmea_builder ${catkin_EXPORTED_TARGETS})
add_dependencies(nmea_builder nmea_generate_messages_cpp)
