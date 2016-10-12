/* "Dynamic Dynamic" reconfigure in C++: ROS 1 parameters without code gen
 * */

#include <ros/ros.h>
#include <ddynamic_reconfigure_cpp/Server.hpp>

namespace ddynrec = ddynamic_reconfigure_cpp;

void callback(ddynamic_reconfigure_cpp::Config<int> &config, uint32_t level) {
  ROS_INFO("Got request to reconfigure");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ddynamic_reconfigure_test");
  ddynrec::Server server;

  // TODO function boilerplate to use lambdas which autoconvert to Config type
  auto int_callback = boost::bind(&callback, _1, _2);

  // add all parameters or use chaining syntax
  server.addParameter<int>(
    "int_test", "an integer value", 0, /*-5, 5,*/ int_callback);
  /*
  server.addParameter<double>(
    "double_test", "a double value", 0, -1.0, 1.0, double_callback);
  server.addParameter<bool>(
    "bool_test", "a bool value", true, bool_callback);
  */

  ros::spin();
  return 0;
}
