// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <functional>
#include <memory>
#include <chrono>
#include <cstdlib>
#include "turtlesim/msg/pose.hpp" 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#define RED 0xFF0000
#define GREEN 0x00FF00
#define ORANGE 0xFFA500
#define BLUE 0x0000FF
using namespace std::chrono_literals;
void set_param(rcl_interfaces::msg::Parameter &p,int val,std::string name){
  p.name = name;
  p.value.type = 2; 
  p.value.integer_value = val;
}
//uses set_param
void set_col(std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically_Request> req,int r,int g,int b){
    auto rparam = rcl_interfaces::msg::Parameter(), gparam=rcl_interfaces::msg::Parameter(), bparam=rcl_interfaces::msg::Parameter();
    set_param(rparam,r,"background_r");
    set_param(gparam,g,"background_g");
    set_param(bparam,b,"background_b");
    req->parameters.push_back(rparam);
    req->parameters.push_back(gparam);
    req->parameters.push_back(bparam);
}
/*
Changes the background color with a call to the /turtlesim/set_parameters_atomically service (via the rcl_interfaces::srv::SetParametersAtomically interface)
*/
void change_background_request(int rgb){
  auto request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
  int b=rgb%256;
  rgb=rgb>>8;
  int g=rgb%256;
  rgb=rgb>>8;
  int r=rgb%256;
  set_col(request,r,g,b);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("node_created");
  auto client=node->create_client<rcl_interfaces::srv::SetParametersAtomically>("/turtlesim/set_parameters_atomically"); // E.g.: serviceName = "/turtlesim/set_parameters_atomically"

while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    return;
    }
    RCLCPP_INFO_STREAM(node->get_logger(), "service not available, waiting again..."); 
}
auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Success");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }
}
using std::placeholders::_1;
//Subscriber that uses the /turtle1/pose topic to monitor changes in the Position of the turtle
class CanvasColorChange : public rclcpp::Node
{
public:
  CanvasColorChange ()
  : Node("canvasColorChange")
  {
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      "turtle1/pose", 10, std::bind(&CanvasColorChange ::topic_callback, this, _1));
  }
  

private:


  void topic_callback(const turtlesim::msg::Pose & msg) const
  {
    //RCLCPP_INFO(this->get_logger(), "X: %f, Y: %f",msg.x,msg.y);
    if(msg.y>11.0){
      change_background_request(RED);//TOP Side
    }
    if(msg.x>11.0){
      change_background_request(GREEN);//Left Side
    }

    if(msg.y==0.0){
      change_background_request(ORANGE);//Right Side
    }
    if(msg.x==0.0){
      change_background_request(BLUE);//Bottom Side
    }
  };
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanvasColorChange>());
  rclcpp::shutdown();
  return 0;
};
