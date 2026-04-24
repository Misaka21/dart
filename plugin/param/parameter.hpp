#ifndef PLUGIN_PARAM_PARAMETER_COMPAT_HPP
#define PLUGIN_PARAM_PARAMETER_COMPAT_HPP

#include <memory>
#include <string>

#include "runtime_parameter.hpp"

namespace plugin {

using Param = runtime_param::Param;

using runtime_param::PARAM_VISITOR;
using runtime_param::Overloaded;

inline std::shared_ptr<Param> create_param(const std::string& name) {
    return runtime_param::create_param(name);
}

inline std::shared_ptr<Param> find_param(const std::string& name) {
    return runtime_param::find_param(name);
}

inline void wait_for_param(const std::string& name) {
    runtime_param::wait_for_param(name);
}

template<typename T>
T get_param(const std::string& name) {
    return runtime_param::get_param<T>(name);
}

void parameter_run(const std::string& param_file_path);

} // namespace plugin

#endif /* PLUGIN_PARAM_PARAMETER_COMPAT_HPP */
