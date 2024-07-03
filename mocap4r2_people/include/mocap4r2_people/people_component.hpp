// Copyright 2022 Intelligent Robotics Lab
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

// Author: Jose Miguel Guerrero Hernandez <josemiguel.guerrero@urjc.es>

#ifndef MOCAP4R2_PEOPLE__PEOPLE_COMPONENT_HPP_
#define MOCAP4R2_PEOPLE__PEOPLE_COMPONENT_HPP_


#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <vector>

#include "mocap4r2_msgs/msg/rigid_bodies.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "people_msgs/msg/people.hpp"

#include "rclcpp/rclcpp.hpp"


namespace mocap4r2_people
{
class PeopleNode : public rclcpp::Node
{
public:
  explicit PeopleNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  void rigid_bodies_callback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg);
  
  geometry_msgs::msg::Pose get_pose_from_vector(const std::vector<double> & init_pos);

  void compute_odometry(
    const tf2::Transform & root2robot_tf,
    nav_msgs::msg::Odometry::UniquePtr & odom_msg);

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr rigid_body_sub_;
  rclcpp::Publisher<people_msgs::msg::People>::SharedPtr people_pub_;

  std::string root_frame_;
  std::string people_frame_prefix_;
  std::string mocap_frame_;

  std::string rigid_body_topic_;
  std::string rigid_body_prefix_;
  std::string people_topic_;

  tf2::Transform map2root_;
  

  //map of poses to keep track of the previous poses of the people
  std::map<std::string, geometry_msgs::msg::PoseStamped> prev_poses_;

  bool valid_root2map_{false};

  std::vector<double> pose_covariance_;
  std::vector<double> twist_covariance_;
};

}  // namespace mocap4r2_people

#endif  // MOCAP4R2_PEOPLE__PEOPLE_COMPONENT_HPP_
