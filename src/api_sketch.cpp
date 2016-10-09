/* "Dynamic Dynamic" reconfigure in C++: ROS 1 parameters without code gen
 * */

#include <ros/ros.h>
#include <ddynamic_reconfigure_cpp/server.h>

namespace ddynrec = ddynamic_reconfigure_cpp;

void main(int argc, char** argv) {
  ros::init(argc, argv);
  ddynrec::Server server;

  auto int_callback = boost::bind();

  // add all parameters or use chaining syntax
  server.addParameter<int>(
    "int_test", "an integer value", 0, -5, 5, int_callback);
  server.addParameter<double>(
    "double_test", "a double value", 0, -1.0, 1.0, double_callback);
  server.addParameter<bool>(
    "bool_test", "a bool value", true, bool_callback);

  ros::spin();
  return 0;
}
