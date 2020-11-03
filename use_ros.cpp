// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <pybind11/chrono.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <map>
#include <string>
#include <utility>
#include <vector>

// Goal: rclcpp publisher and subscriber to see if bi-directional pub/sub is possible

namespace rccl
{
class ROS
{
public:
  ROS()
  {
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(0, nullptr);
    rclcpp::ExecutorOptions eo;
    eo.context = context_;
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(eo);
  }
  ~ROS()
  {
    context_->shutdown("~ROS()");
  }

  void spin()
  {
    pybind11::gil_scoped_release release;
    std::cout << "C++ spin start\n";
    executor_->spin();
    std::cout << "C++ spin stop\n";
  }

  void add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr)
  {
    executor_->add_node(node_ptr);
  }

  void remove_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr)
  {
    executor_->remove_node(node_ptr);
  }

  rclcpp::Context::SharedPtr context_;
private:
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
};


class PingPong
{
public:
  explicit PingPong(ROS & ros)
  {
    rclcpp::NodeOptions no;
    no.context(ros.context_);
    node_ = std::make_shared<rclcpp::Node>("PingPong", no);

    pub_ = rclcpp::create_publisher<std_msgs::msg::String>(
      node_, "cpp_to_py", 10);

    // std::function<void(std_msgs::msg::String::ConstSharedPtr &)> callback = 

    // sub_ = rclcpp::create_subscription<const std_msgs::msg::String &>(
    //   node_, "py_to_cpp", rclcpp::QoS(10), callback);
    sub_ = node_->create_subscription<std_msgs::msg::String>(
      "py_to_cpp", 10, std::bind(
      &PingPong::pong, this, std::placeholders::_1));

    ros.add_node(node_->get_node_base_interface());
  }

  ~PingPong()
  {
    // TODO(sloretz) don't leak this stuff
    // ros.remove_node(node_);
  }

  void ping(const std::string & msg)
  {
    auto message = std_msgs::msg::String();
    message.data = msg;
    pub_->publish(message);
  }

private:
  void pong(const std_msgs::msg::String::SharedPtr message)
  {
    std::cout << "c++:" << message->data << "\n";
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};
}  // namespace rccl

PYBIND11_MODULE(rccl, m) {
  m.doc() = "Python wrapper for code using rclcpp";

  pybind11::class_<rccl::ROS>(m, "ROS")
    .def(pybind11::init())
    .def("spin", &rccl::ROS::spin);

  pybind11::class_<rccl::PingPong>(m, "PingPong")
    .def(pybind11::init<rccl::ROS &>())
    .def("ping", &rccl::PingPong::ping,
        pybind11::arg("msg") = "C++ ping");
}
