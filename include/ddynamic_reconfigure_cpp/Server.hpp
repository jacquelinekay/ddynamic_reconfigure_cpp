/* A server that infers dynamic parameter from template arguments and serves
 * callbacks when those dynamic parameters are set.
 * */

#include <dynamic_reconfigure/server.h>
#include <ddynamic_reconfigure_cpp/MetaConfigType.hpp>
#include <boost/any.hpp>

namespace ddynamic_reconfigure_cpp {

class Server {
public:
  // Convenience alias
  template<typename T>
  using CallbackT = boost::function<void(Config<T>&, uint32_t level)>;

  /* Returns true if adding a parameter to the server succeeded,
   * false otherwise.
   * */
  template<typename ParameterT>
  bool addParameter(
      std::string const& parameter_name,
      std::string const& parameter_description,
      const ParameterT& default_value,
      const CallbackT<ParameterT>& callback) {
    using ServerT = dynamic_reconfigure::Server<Config<ParameterT>>;

    // TODO Instantiate and store in an "any" map
    std::shared_ptr<ServerT> server(new ServerT);
    server->setCallback(callback);
    servers.emplace_back(server);
  }

  /* addParameter with ranges for comparable types. Should do range-checking
   * when parameters come in.
   * */

private:
  std::vector<boost::any> servers;
};

}  // namespace ddynamic_reconfigure
