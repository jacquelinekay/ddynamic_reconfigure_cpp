// ${doline} ${linenum} "${filename}"
// *********************************************************
// Template classes for "dynamic dynamic reconfigure" C++ implementation,
// based on dynamic_reconfigure/templates/ConfigType.h.template
//
// ********************************************************/

#ifndef DDYNAMIC_RECONFIGURE_CPP__META_CONFIG_TYPE_H__
#define DDYNAMIC_RECONFIGURE_CPP__META_CONFIG_TYPE_H__

#include <boost/any.hpp>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/Group.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/config_init_mutex.h>
#include <dynamic_reconfigure/config_tools.h>
#include <limits>
#include <ros/node_handle.h>

namespace ddynamic_reconfigure_cpp {

template <typename ParameterT> class ConfigStatics;

template <typename ParameterT> class Config {
public:
  class AbstractParamDescription
      : public dynamic_reconfigure::ParamDescription {
  public:
    AbstractParamDescription(std::string n, std::string t, uint32_t l,
                             std::string d, std::string e) {
      name = n;
      type = t;
      level = l;
      description = d;
      edit_method = e;
    }

    // TODO Enable clamp only for comparable (numeric?) types
    virtual void clamp(Config &config, const Config &max,
                       const Config &min) const = 0;
    virtual void calcLevel(uint32_t &level, const Config &config1,
                           const Config &config2) const = 0;
    virtual void fromServer(const ros::NodeHandle &nh,
                            Config &config) const = 0;
    virtual void toServer(const ros::NodeHandle &nh,
                          const Config &config) const = 0;
    virtual bool fromMessage(const dynamic_reconfigure::Config &msg,
                             Config &config) const = 0;
    virtual void toMessage(dynamic_reconfigure::Config &msg,
                           const Config &config) const = 0;
    virtual void getValue(const Config &config, boost::any &val) const = 0;

  };

  typedef boost::shared_ptr<AbstractParamDescription>
      AbstractParamDescriptionPtr;
  typedef boost::shared_ptr<const AbstractParamDescription>
      AbstractParamDescriptionConstPtr;

  template <class T> class ParamDescription : public AbstractParamDescription {
  public:
    ParamDescription(std::string name, std::string type, uint32_t level,
                     std::string description, std::string edit_method,
                     T Config::*f)
        : AbstractParamDescription(name, type, level, description, edit_method),
          field(f) {}

    T(Config::*field);

    // TODO enable only for numeric types
    virtual void clamp(Config &config, const Config &max,
                       const Config &min) const {
      if (config.*field > max.*field)
        config.*field = max.*field;

      if (config.*field < min.*field)
        config.*field = min.*field;
    }

    virtual void calcLevel(uint32_t &comb_level, const Config &config1,
                           const Config &config2) const {
      if (config1.*field != config2.*field)
        comb_level |= AbstractParamDescription::level;
    }

    virtual void fromServer(const ros::NodeHandle &nh, Config &config) const {
      nh.getParam(AbstractParamDescription::name, config.*field);
    }

    virtual void toServer(const ros::NodeHandle &nh,
                          const Config &config) const {
      nh.setParam(AbstractParamDescription::name, config.*field);
    }

    virtual bool fromMessage(const dynamic_reconfigure::Config &msg,
                             Config &config) const {
      return dynamic_reconfigure::ConfigTools::getParameter(
          msg, AbstractParamDescription::name, config.*field);
    }

    virtual void toMessage(dynamic_reconfigure::Config &msg,
                           const Config &config) const {
      dynamic_reconfigure::ConfigTools::appendParameter(
          msg, AbstractParamDescription::name, config.*field);
    }

    virtual void getValue(const Config &config, boost::any &val) const {
      val = config.*field;
    }
  };

  class AbstractGroupDescription : public dynamic_reconfigure::Group {
  public:
    AbstractGroupDescription(std::string n, std::string t, int p, int i,
                             bool s) {
      name = n;
      type = t;
      parent = p;
      state = s;
      id = i;
    }

    std::vector<AbstractParamDescriptionConstPtr> abstract_parameters;
    bool state;

    virtual void toMessage(dynamic_reconfigure::Config &msg,
                           const boost::any &config) const = 0;
    virtual bool fromMessage(const dynamic_reconfigure::Config &msg,
                             boost::any &config) const = 0;
    virtual void updateParams(boost::any &cfg, Config &top) const = 0;
    virtual void setInitialState(boost::any &cfg) const = 0;

    void convertParams() {
      for (auto &p : abstract_parameters) {
        parameters.push_back(p);
      }
    }
  };

  typedef boost::shared_ptr<AbstractGroupDescription>
      AbstractGroupDescriptionPtr;
  typedef boost::shared_ptr<const AbstractGroupDescription>
      AbstractGroupDescriptionConstPtr;

  template <class T, class PT>
  class GroupDescription : public AbstractGroupDescription {
  public:
    GroupDescription(std::string name, std::string type, int parent, int id,
                     bool s, T PT::*f)
        : AbstractGroupDescription(name, type, parent, id, s), field(f) {}

    GroupDescription(const GroupDescription<T, PT> &g)
        : AbstractGroupDescription(g.name, g.type, g.parent, g.id, g.state),
          field(g.field), groups(g.groups) {
      AbstractGroupDescription::parameters = g.parameters;
      AbstractGroupDescription::abstract_parameters = g.abstract_parameters;
    }

    virtual bool fromMessage(const dynamic_reconfigure::Config &msg,
                             boost::any &cfg) const {
      PT *config = boost::any_cast<PT *>(cfg);
      if (!dynamic_reconfigure::ConfigTools::getGroupState(
              msg, AbstractGroupDescription::name, (*config).*field))
        return false;

      for (auto &g : groups) {
        boost::any n = &((*config).*field);
        if (!g.fromMessage(msg, n))
          return false;
      }

      return true;
    }

    virtual void setInitialState(boost::any &cfg) const {
      PT *config = boost::any_cast<PT *>(cfg);
      T *group = &((*config).*field);
      group->state = AbstractGroupDescription::state;

      for (auto &g : groups) {
        boost::any n = boost::any(&((*config).*field));
        g.setInitialState(n);
      }
    }

    virtual void updateParams(boost::any &cfg, Config &top) const {
      PT *config = boost::any_cast<PT *>(cfg);

      T *f = &((*config).*field);
      f->setParams(top, AbstractGroupDescription::abstract_parameters);

      for (auto &g : groups) {
        boost::any n = &((*config).*field);
        g.updateParams(n, top);
      }
    }

    virtual void toMessage(dynamic_reconfigure::Config &msg,
                           const boost::any &cfg) const {
      const PT config = boost::any_cast<PT>(cfg);
      dynamic_reconfigure::ConfigTools::appendGroup(
          msg, AbstractGroupDescription::name, AbstractGroupDescription::id,
          AbstractGroupDescription::parent, config.*field);

      for (auto &g : groups) {
        g.toMessage(msg, config.*field);
      }
    }

    T(PT::*field);
    std::vector<Config::AbstractGroupDescriptionConstPtr> groups;
  };
  // TODO groups, members, doline linenum filename
  // ${groups}

  // ${members}
  // ${doline} ${linenum} "${filename}"

  bool __fromMessage__(dynamic_reconfigure::Config &msg) {
    const std::vector<AbstractParamDescriptionConstPtr>
        &__param_descriptions__ = __getParamDescriptions__();
    const std::vector<AbstractGroupDescriptionConstPtr>
        &__group_descriptions__ = __getGroupDescriptions__();

    int count = 0;
    for (auto &p : __param_descriptions__) {
      if (p->fromMessage(msg, *this)) {
        count++;
      }
    }

    for (auto &g : __group_descriptions__) {
      if (g->id == 0) {
        boost::any n = boost::any(this);
        g->updateParams(n, *this);
        g->fromMessage(msg, n);
      }
    }

    if (count != dynamic_reconfigure::ConfigTools::size(msg)) {
      ROS_ERROR("${configname}Config::__fromMessage__ called with an "
                "unexpected parameter.");
      ROS_ERROR("Booleans:");
      for (unsigned int i = 0; i < msg.bools.size(); i++)
        ROS_ERROR("  %s", msg.bools[i].name.c_str());
      ROS_ERROR("Integers:");
      for (unsigned int i = 0; i < msg.ints.size(); i++)
        ROS_ERROR("  %s", msg.ints[i].name.c_str());
      ROS_ERROR("Doubles:");
      for (unsigned int i = 0; i < msg.doubles.size(); i++)
        ROS_ERROR("  %s", msg.doubles[i].name.c_str());
      ROS_ERROR("Strings:");
      for (unsigned int i = 0; i < msg.strs.size(); i++)
        ROS_ERROR("  %s", msg.strs[i].name.c_str());
      // @todo Check that there are no duplicates. Make this error more
      // explicit.
      return false;
    }
    return true;
  }

  // This version of __toMessage__ is used during initialization of
  // statics when __getParamDescriptions__ can't be called yet.
  void __toMessage__(dynamic_reconfigure::Config &msg,
                     const std::vector<Config::AbstractParamDescriptionConstPtr>
                         &__param_descriptions__,
                     const std::vector<Config::AbstractGroupDescriptionConstPtr>
                         &__group_descriptions__) const {
    dynamic_reconfigure::ConfigTools::clear(msg);
    for (auto &p : __param_descriptions__) {
      p->toMessage(msg, *this);
    }

    for (auto &g : __group_descriptions__) {
      {
        if (g->id == 0) {
          g->toMessage(msg, *this);
        }
      }
    }
  }

  void __toMessage__(dynamic_reconfigure::Config &msg) const {
    const std::vector<AbstractParamDescriptionConstPtr>
        &__param_descriptions__ = __getParamDescriptions__();
    const std::vector<AbstractGroupDescriptionConstPtr>
        &__group_descriptions__ = __getGroupDescriptions__();
    __toMessage__(msg, __param_descriptions__, __group_descriptions__);
  }

  void __toServer__(const ros::NodeHandle &nh) const {
    const std::vector<AbstractParamDescriptionConstPtr>
        &__param_descriptions__ = __getParamDescriptions__();
    for (auto &p : __param_descriptions__) {
      p->toServer(nh, *this);
    }
  }

  void __fromServer__(const ros::NodeHandle &nh) {
    static bool setup = false;

    const std::vector<AbstractParamDescriptionConstPtr>
        &__param_descriptions__ = __getParamDescriptions__();
    for (auto &p : __param_descriptions__) {
      p->fromServer(nh, *this);
    }

    const std::vector<AbstractGroupDescriptionConstPtr>
        &__group_descriptions__ = __getGroupDescriptions__();
    for (auto &g : __group_descriptions__) {
      if (!setup && g->id == 0) {
        setup = true;
        boost::any n = boost::any(this);
        g->setInitialState(n);
      }
    }
  }

  template<typename T>
  uint32_t __level__(const Config<T> &config) const {
    const std::vector<AbstractParamDescriptionConstPtr>
        &__param_descriptions__ = __getParamDescriptions__();
    uint32_t level = 0;
    for (auto &p : __param_descriptions__)
      p->calcLevel(level, config, *this);
    return level;
  }

  /*
  static const dynamic_reconfigure::ConfigDescription &
  __getDescriptionMessage__();
  static const Config &__getDefault__();
  static const Config &__getMax__();
  static const Config &__getMin__();
  static const std::vector<AbstractParamDescriptionConstPtr> &
  __getParamDescriptions__();
  static const std::vector<AbstractGroupDescriptionConstPtr> &
  __getGroupDescriptions__();

  */

  void __clamp__() {
    const std::vector<AbstractParamDescriptionConstPtr>
        &__param_descriptions__ = __getParamDescriptions__();
    const Config &__max__ = __getMax__();
    const Config &__min__ = __getMin__();
    for (auto &p : __param_descriptions__) {
      p->clamp(*this, __max__, __min__);
    }
  }

  static inline const dynamic_reconfigure::ConfigDescription
  __getDescriptionMessage__() {
    return ConfigStatics<ParameterT>::__get_statics__()->__description_message__;
  }

  static inline const Config &__getDefault__() {
    return ConfigStatics<ParameterT>::__get_statics__()->__default__;
  }

  static inline const Config &__getMax__() {
    return ConfigStatics<ParameterT>::__get_statics__()->__max__;
  }

  static inline const Config &__getMin__() {
    return ConfigStatics<ParameterT>::__get_statics__()->__min__;
  }

  static inline const std::vector<Config::AbstractParamDescriptionConstPtr> &
  __getParamDescriptions__() {
    return ConfigStatics<ParameterT>::__get_statics__()->__param_descriptions__;
  }

  static inline const std::vector<typename Config::AbstractGroupDescriptionConstPtr> &
  __getGroupDescriptions__() {
    return ConfigStatics<ParameterT>::__get_statics__()->__group_descriptions__;
  }



private:
  // static const ConfigStatics<ParameterT> *__get_statics__();
};  // class Config

/*
template<typename T>
inline void Config<T>::ParamDescription<std::string>::clamp(
Config<T> &config, const Config<T> &max, const Config<T> &min) const
{
// TODO wow holy shit this needs to be fixed
return;
}
*/

template <typename T>
class ConfigStatics {
  friend class Config<T>;

  ConfigStatics() {
    // TODO
    // ${paramdescr}
    // ${doline} ${linenum} "${filename}"

    for (auto &g : __group_descriptions__) {
      __description_message__.groups.push_back(*g);
    }
    __max__.__toMessage__(__description_message__.max, __param_descriptions__,
                          __group_descriptions__);
    __min__.__toMessage__(__description_message__.min, __param_descriptions__,
                          __group_descriptions__);
    __default__.__toMessage__(__description_message__.dflt,
                              __param_descriptions__, __group_descriptions__);
  }
  std::vector<typename Config<T>::AbstractParamDescriptionConstPtr> __param_descriptions__;
  std::vector<typename Config<T>::AbstractGroupDescriptionConstPtr> __group_descriptions__;
  Config<T> __max__;
  Config<T> __min__;
  Config<T> __default__;
  dynamic_reconfigure::ConfigDescription __description_message__;

  static const ConfigStatics *get_instance() {
    // Split this off in a separate function because I know that
    // instance will get initialized the first time get_instance is
    // called, and I am guaranteeing that get_instance gets called at
    // most once.
    static ConfigStatics instance;
    return &instance;
  }

  static inline const ConfigStatics *__get_statics__() {
    const static ConfigStatics *statics;

    if (statics) // Common case
      return statics;

    boost::mutex::scoped_lock lock(dynamic_reconfigure::__init_mutex__);

    if (statics) // In case we lost a race.
      return statics;

    statics = ConfigStatics<T>::get_instance();

    return statics;
  }
};

// ${constants}
}

#endif // DDYNAMIC_RECONFIGURE_CPP__META_CONFIG_TYPE_H__
