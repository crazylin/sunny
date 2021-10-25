#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <vector>
#include <sys/stat.h>
#include <regex>
#include <vector>

using std::cout;
using std::endl;
using std::string;

string ns, cn, NS;

bool Exists(const string& file)
{
    std::ifstream f(file);
    return f.good();
}

void CreateConfig()
{
    if(Exists(ns + "/config/params.yaml"))
        return;
    auto dir = ns + "/config/";
    mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    std::ofstream ofile(ns + "/config/params.yaml");
    ofile << ns << "_node:\n";
    ofile << "  ros__parameters:\n";
    ofile << "    test: TODO";
}

void CreateLaunch()
{
    if(Exists(ns + "/launch/main.launch.py"))
        return;
    auto dir = ns + "/launch/";
    mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    std::ofstream ofile(dir + "main.launch.py");

    string content = R"----(# Copyright 2019 Zhushi Tech, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer


def generate_launch_description():
    """Generate launch description with a component."""
    configFile = os.path.join(
        get_package_share_directory('xxx_yyy_zzz'),
        'config',
        'params.yaml')

    with open(configFile, 'r') as file:
        configParams = yaml.safe_load(file)['xxx_yyy_zzz_node']['ros__parameters']

    node = ComposableNode(
        package='xxx_yyy_zzz',
        plugin='xxx_yyy_zzz::XxxYyyZzz',
        parameters=[configParams])

    container = ComposableNodeContainer(
        name='xxx_yyy_zzz_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[node],
        output='screen')

    return launch.LaunchDescription([container]))----";

    content = std::regex_replace(content, std::regex("xxx_yyy_zzz"), ns);
    content = std::regex_replace(content, std::regex("XxxYyyZzz"), cn);
    ofile << content << endl;
}

void CreateHeader()
{
    if(Exists(ns + "/include/" + ns + '/' + ns + ".hpp"))
        return;
    auto dir = ns + "/include/";
    mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    dir += ns + '/';
    mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    auto file = dir + ns + ".hpp";
    std::ofstream ofile(file);

    string content = R"----(// Copyright 2019 Zhushi Tech, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef XXX_YYY_ZZZ__XXX_YYY_ZZZ_HPP_
#define XXX_YYY_ZZZ__XXX_YYY_ZZZ_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace xxx_yyy_zzz
{

class XxxYyyZzz : public rclcpp::Node
{
public:
  explicit XxxYyyZzz(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~XxxYyyZzz();

private:
  void _Init();
  void _InitializeParameters();
  void _UpdateParameters();
  void _Sub(std_msgs::msg::String::UniquePtr ptr);  // TODO(imp)
  void _Srv(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);  // TODO(imp)

private:
  const char * _pubName = "~/pub";  // TODO(imp)
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub;

  class _Impl;
  std::unique_ptr<_Impl> _impl;

  const char * _subName = "~/sub";  // TODO(imp)
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub;

  const char * _srvName = "~/srv";  // TODO(imp)
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srv;

  std::thread _init;
};

}  // namespace xxx_yyy_zzz

#endif  // XXX_YYY_ZZZ__XXX_YYY_ZZZ_HPP_)----";

    content = std::regex_replace(content, std::regex("xxx_yyy_zzz"), ns);
    content = std::regex_replace(content, std::regex("XXX_YYY_ZZZ"), NS);
    content = std::regex_replace(content, std::regex("XxxYyyZzz"), cn);
    ofile << content << endl;
}

void CreateSource()
{
    if(Exists(ns + "/src/" + ns + ".cpp"))
        return;
    auto dir = ns + "/src/";
    mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    std::ofstream ofile(dir + ns + ".cpp");

    string content = R"----(// Copyright 2019 Zhushi Tech, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "xxx_yyy_zzz/xxx_yyy_zzz.hpp"

#include <memory>

namespace xxx_yyy_zzz
{

class XxxYyyZzz::_Impl
{
public:
  explicit _Impl(XxxYyyZzz * ptr)
  : _node(ptr)
  {
  }

  ~_Impl()
  {
  }

private:
  XxxYyyZzz * _node;
};

XxxYyyZzz::XxxYyyZzz(const rclcpp::NodeOptions & options)
: Node("xxx_yyy_zzz_node", options)
{
  _init = std::thread(&XxxYyyZzz::_Init, this);
}

XxxYyyZzz::~XxxYyyZzz()
{
  _init.join();

  _srv.reset();
  _sub.reset();
  _impl.reset();
  _pub.reset();

  RCLCPP_INFO(this->get_logger(), "Destroyed successfully");
}

void XxxYyyZzz::_Init()
{
  try {
    _InitializeParameters();

    _UpdateParameters();

    _pub = this->create_publisher<std_msgs::msg::String>(_pubName, 10);

    _impl = std::make_unique<_Impl>(this);

    _sub = this->create_subscription<std_msgs::msg::String>(
      _subName,
      10,
      std::bind(&XxxYyyZzz::_Sub, this, std::placeholders::_1));

    _srv = this->create_service<std_srvs::srv::Trigger>(
      _srvName,
      std::bind(&XxxYyyZzz::_Srv, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Initialized successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in initializer: %s", e.what());
    rclcpp::shutdown();
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Exception in initializer: unknown");
    rclcpp::shutdown();
  }
}

void XxxYyyZzz::_Sub(std_msgs::msg::String::UniquePtr /*ptr*/)
{
  try {
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in subscription: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Exception in subscription: unknown");
  }
}

void XxxYyyZzz::_Srv(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response>/*response*/)
{
  try {
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in service: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Exception in service: unknown");
  }
}

void XxxYyyZzz::_InitializeParameters()
{
  // this->declare_parameter("");
}

void XxxYyyZzz::_UpdateParameters()
{
  // this->get_parameter("", );
}

}  // namespace xxx_yyy_zzz

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(xxx_yyy_zzz::XxxYyyZzz))----";

    content = std::regex_replace(content, std::regex("xxx_yyy_zzz"), ns);
    content = std::regex_replace(content, std::regex("XxxYyyZzz"), cn);
    ofile << content << endl;
}

void CreateCMake()
{
    if(Exists(ns + "/CMakeLists.txt"))
        return;
    std::ofstream ofile(ns + "/CMakeLists.txt");

    string content = R"----(cmake_minimum_required(VERSION 3.5)
project(xxx_yyy_zzz)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(xxx_yyy_zzz SHARED src/xxx_yyy_zzz.cpp)

target_include_directories(xxx_yyy_zzz
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(xxx_yyy_zzz
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
)

rclcpp_components_register_node(xxx_yyy_zzz
  PLUGIN "xxx_yyy_zzz::XxxYyyZzz"
  EXECUTABLE xxx_yyy_zzz_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS xxx_yyy_zzz EXPORT export_xxx_yyy_zzz DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_xxx_yyy_zzz HAS_LIBRARY_TARGET)

ament_package()
)----";

    content = std::regex_replace(content, std::regex("xxx_yyy_zzz"), ns);
    content = std::regex_replace(content, std::regex("XxxYyyZzz"), cn);
    ofile << content << endl;
}

void CreatePackage()
{
    if(Exists(ns + "/package.xml"))
        return;
    std::ofstream ofile(ns + "/package.xml");

    string content = R"----(<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>xxx_yyy_zzz</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="zhuoqiw@hotmail.com">zhuoqiw</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_components</depend>
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
)----";

    content = std::regex_replace(content, std::regex("xxx_yyy_zzz"), ns);
    content = std::regex_replace(content, std::regex("XxxYyyZzz"), cn);
    ofile << content << endl;
}

int main(int argc, char * argv[])
{
    if(argc < 2)
    {
        cout<<"usage: template xxx yyy ..."<<endl;
        return 1;
    }

    for(int i = 1; i < argc; ++i)
    {
        string str(argv[i]);
        std::transform(str.begin(), str.end(), str.begin(), ::toupper);
        if(i != 1) NS += '_';
        NS += str;

        std::transform(str.begin(), str.end(), str.begin(), ::tolower);
        if(i != 1) ns += '_';
        ns += str;

        str[0] = toupper(str[0]);
        cn += str;
    }
    mkdir(ns.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    CreateConfig();
    CreateLaunch();
    CreateHeader();
    CreateSource();
    CreateCMake();
    CreatePackage();
    return 0;
}
