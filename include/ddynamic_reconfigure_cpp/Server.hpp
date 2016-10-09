/* A server that infers dynamic parameter from template arguments and serves
 * callbacks when those dynamic parameters are set.
 * */

#include <dynamic_reconfigure/server.h>
#include <ddynamic_reconfigure_cpp/MetaConfigType.h>
#include <boost/any.hpp>

namespace ddynamic_reconfigure_cpp {

template<typename T>
struct deduce_return_type {
  using type = ;
};

class Server {
public:
  // Convenience alias
  template<typename T>
  using CallbackT = boost::function<void(ConfigType&, uint32_t level)>;

  /* Returns true if adding a parameter to the server succeeded,
   * false otherwise.
   * */
  template<typename ParameterT>
  bool addParameter(
      std::string const& parameter_name,
      std::string const& parameter_description,
      const ParameterT& default_value,
      const CallbackT& callback) {
    // At compile time, deduce a valid ConfigType from ParameterT and instantiate
    // and store a dynamic_reconfigure server 
    using ConfigType = deduce_config_type<ParameterT>::type;

    // TODO Instantiate and store in an "any" map
    dynamic_reconfigure::Server<ddynamic_reconfigure_cpp::ConfigType> server;
    server.setCallback(callback);

    servers.push_back(server);
  }

  /* addParameter with ranges for comparable types. Should do range-checking
   * when parameters come in.
   * */

private:
  // Storage for 
  std::vector<boost::any> servers;
};

}  // namespace ddynamic_reconfigure
