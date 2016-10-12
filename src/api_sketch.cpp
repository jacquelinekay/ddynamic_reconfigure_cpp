/* "Dynamic Dynamic" reconfigure in C++: ROS 1 parameters without code gen
 * */

#include <ros/ros.h>
#include <ddynamic_reconfigure_cpp/Server.hpp>

namespace ddynrec = ddynamic_reconfigure_cpp;

struct Foo {
  int a = 0;
  std::string b = "foo";
};

void config_int_callback(ddynamic_reconfigure_cpp::Config<int> &config, uint32_t level) {
  ROS_INFO("Got int request");
}

void config_double_callback(ddynamic_reconfigure_cpp::Config<double> &config, uint32_t level) {
  ROS_INFO("Got double request");
}

void config_bool_callback(ddynamic_reconfigure_cpp::Config<bool> &config, uint32_t level) {
  ROS_INFO("Got bool request");
}

void config_string_callback(ddynamic_reconfigure_cpp::Config<std::string> &config, uint32_t level) {
  ROS_INFO("Got string request");
}

void config_foo_callback(ddynamic_reconfigure_cpp::Config<Foo> &config, uint32_t level) {
  ROS_INFO("Got foo request");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ddynamic_reconfigure_test");
  ddynrec::Server server;

  // TODO function boilerplate to use lambdas which autoconvert to Config type
  auto int_callback = boost::bind(&config_int_callback, _1, _2);
  auto double_callback = boost::bind(&config_double_callback, _1, _2);
  auto bool_callback = boost::bind(&config_bool_callback, _1, _2);
  auto string_callback = boost::bind(&config_string_callback, _1, _2);
  auto foo_callback = boost::bind(&config_foo_callback, _1, _2);

  // add all parameters or use chaining syntax
  // TODO syntactic sugar with variadic templates to make ranges and default values optional
  server.addParameter<int>(
    "int_test", "an integer value", 0, /*-5, 5,*/ int_callback);
  server.addParameter<double>(
    "double_test", "a double value", 0, /*-1.0, 1.0,*/ double_callback);
  server.addParameter<bool>("bool_test", "a bool value", true, bool_callback);
  server.addParameter<std::string>("string_test", "a string value", "hello world", string_callback);
  server.addParameter<Foo>("foo_test", "a struct value", Foo{}, foo_callback);

  ros::spin();
  return 0;
}
