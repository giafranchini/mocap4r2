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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <vector>
#include <ranges>

#include "mocap4r2_people/people_component.hpp"
#include "mocap4r2_msgs/msg/rigid_body.hpp"

#include "rclcpp/rclcpp.hpp"

namespace mocap4r2_people
{

using std::placeholders::_1;
using std::placeholders::_2;

PeopleNode::PeopleNode(const rclcpp::NodeOptions & options)
: Node("mocap4r2_people", options),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  declare_parameter<std::string>("root_frame", "mocap");
  declare_parameter<std::string>("map_frame", "map");
  declare_parameter<std::string>("people_frame_prefix", "person");
  declare_parameter<std::string>("rigid_body_topic", "rigid_bodies");
  declare_parameter<std::string>("people_topic", "people");
  declare_parameter<std::string>("rigid_body_prefix", "person");
  declare_parameter<std::vector<double>>("covariance.pose", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  declare_parameter<std::vector<double>>("covariance.twist", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  
  get_parameter("root_frame", root_frame_);
  get_parameter("people_frame_prefix", people_frame_prefix_);
  get_parameter("rigid_body_topic", rigid_body_topic_);
  get_parameter("rigid_body_name", rigid_body_name_);
  get_parameter("people_topic", people_topic_);
  get_parameter("covariance.pose", pose_covariance_);
  get_parameter("covariance.twist", twist_covariance_);


  rigid_body_sub_ = create_subscription<mocap4r2_msgs::msg::RigidBodies>(
    rigid_body_topic_, rclcpp::SensorDataQoS(), std::bind(&PeopleNode::rigid_bodies_callback, this, _1));
  
  people_pub_ = create_publisher<people_msgs::msg::People>(people_topic_, 10);
  
  valid_map2root_ = map_frame_ == root_frame_; 


}

void
PeopleNode::rigid_bodies_callback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg)
{
  if (!valid_map2root_) { // Find root2map (the mocap system origin in the map frame)
    try {
      auto map2root_msg = tf_buffer_.lookupTransform(
        map_frame_, root_frame_, tf2::TimePointZero);
      tf2::fromMsg(map2root_msg.transform, map2root_);
      valid_map2root_ = true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(
        get_logger(), "Transform %s->%s exception: [%s]",
        root_frame_.c_str(), map_frame_.c_str(), e.what());
    }
  } else {
    // filter out the non valid elements
    auto people = std::ranges::filter_view(msg->rigidbodies, [this](const mocap4r2_msgs::msg::RigidBody & rb) {
        return rb.rigid_body_name.find(rigid_body_prefix_) != std::string::npos;
      });

    if(people.empty()) {
      RCLCPP_WARN(get_logger(), "No people found in mocap system");
      return;
    }
    
    auto people_msg = std::make_unique<people_msgs::msg::People>();
    
    
    for(const auto & person : people) {
      // Check if the mocap is publishing the person pose and is not zero
      tf2::Quaternion q;
      tf2::fromMsg(person.pose.orientation, q);
      if (q < 1e-6) {
        RCLCPP_WARN(get_logger(), "Zero quaternion received from mocap system. Check that the person is being tracked");
        continue;
      }
      // obtain the root2person transform
      tf2::Transform root2person;
      root2person.setOrigin(tf2::Vector3(person.pose.position.x, person.pose.position.y, person.pose.position.z));
      root2person.setRotation(q);

      // obtain the map2person transform
      tf2::Transform map2person = map2root_ * root2person;

      auto person_pose = std::make_unique<geometry_msgs::msg::PoseStamped>();
      person_pose->header = msg->header;
      person_pose->header.frame_id = map_frame_;
      person_pose->child_frame_id = person.rigid_body_name;
      tf2::toMsg(map2person, person_pose->pose);
      geometry_msgs::msg::TransformStamped person_pose_msg;
      person_pose_msg.header = person_pose->header;
      person_pose_msg.transform = tf2::toMsg(map2person);




    }

    mocap2gtbody_.setOrigin(tf2::Vector3(robot_it->pose.position.x, robot_it->pose.position.y, robot_it->pose.position.z));
    mocap2gtbody_.setRotation(q);

    tf2::Transform root2robotgt;
    root2robotgt = offset_ * mocap2gtbody_ * gtbody2robot_;

    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
    odom_msg->header.frame_id = root_frame_;
    odom_msg->header.stamp = msg->header.stamp;
    odom_msg->child_frame_id = robot_frame_ + "_gt";
    PeopleNode::compute_odometry(root2robotgt, odom_msg);

    geometry_msgs::msg::TransformStamped root2robotgt_msg;
    root2robotgt_msg.header.frame_id = root_frame_;
    root2robotgt_msg.header.stamp = msg->header.stamp;
    root2robotgt_msg.child_frame_id = robot_frame_ + "_gt";
    root2robotgt_msg.transform = tf2::toMsg(root2robotgt);

    odometry_pub_->publish(std::move(odom_msg));
    tf_broadcaster_->sendTransform(root2robotgt_msg);
  }
}


geometry_msgs::msg::Pose
PeopleNode::get_pose_from_vector(const std::vector<double> & init_pos)
{
  geometry_msgs::msg::Pose ret;

  if (init_pos.size() == 6u) {
    tf2::Quaternion q;
    q.setEuler(init_pos[3], init_pos[4], init_pos[5]);

    ret.position.x = init_pos[0];
    ret.position.y = init_pos[1];
    ret.position.z = init_pos[2];
    ret.orientation.x = q.x();
    ret.orientation.y = q.y();
    ret.orientation.z = q.z();
    ret.orientation.w = q.w();

    return ret;
  } else {
    RCLCPP_WARN(get_logger(), "Trying to get Pose for a wrong vector");
    return ret;
  }
}

void PeopleNode::compute_odometry(
  const tf2::Transform & root2robot_tf,
  nav_msgs::msg::Odometry::UniquePtr & odom_msg)
{
  static geometry_msgs::msg::Twist smoothed_twist;
  tf2::Transform p1;
  tf2::fromMsg(prev_pose_.pose, p1);
  tf2::toMsg(root2robot_tf, odom_msg->pose.pose);

  const double q1_w = p1.getRotation().w();
  const double q1_x = p1.getRotation().x();
  const double q1_y = p1.getRotation().y();
  const double q1_z = p1.getRotation().z();

  const double q2_w = root2robot_tf.getRotation().w();
  const double q2_x = root2robot_tf.getRotation().x();
  const double q2_y = root2robot_tf.getRotation().y();
  const double q2_z = root2robot_tf.getRotation().z(); 
  
  rclcpp::Time t1 = prev_pose_.header.stamp;
  rclcpp::Time t2 = odom_msg->header.stamp;

  const double dt_inv = 1.0 / (t2 - t1).seconds();

  auto delta_trans = tf2::quatRotate(
    root2robot_tf.getRotation().inverse(), (root2robot_tf.getOrigin() - p1.getOrigin()));
    
  // Compute linear velocities
  
  // Smooth velocities with a simple low-pass filter
  smoothed_twist.linear.x = (1 - alpha_) * smoothed_twist.linear.x + alpha_ * (delta_trans.x() * dt_inv);
  smoothed_twist.linear.y = (1 - alpha_) * smoothed_twist.linear.y + alpha_ * (delta_trans.y() * dt_inv);
  smoothed_twist.linear.z = (1 - alpha_) * smoothed_twist.linear.z + alpha_ * (delta_trans.z() * dt_inv);

  odom_msg->twist.twist.linear.x = smoothed_twist.linear.x;
  odom_msg->twist.twist.linear.y = smoothed_twist.linear.y;
  odom_msg->twist.twist.linear.z = smoothed_twist.linear.z;
    
  // Compute angular velocities
  // smooth angular velocities with a simple low-pass filter
  smoothed_twist.angular.x = (1 - alpha_) * smoothed_twist.angular.x + alpha_ * (
    2.0 * dt_inv * (q1_w * q2_x - q1_x * q2_w - q1_y * q2_z + q1_z * q2_y));
  smoothed_twist.angular.y = (1 - alpha_) * smoothed_twist.angular.y + alpha_ * (
    2.0 * dt_inv * (q1_w * q2_y + q1_x * q2_z - q1_y * q2_w - q1_z * q2_x));
  smoothed_twist.angular.z = (1 - alpha_) * smoothed_twist.angular.z + alpha_ * (
    2.0 * dt_inv * (q1_w * q2_z - q1_x * q2_y + q1_y * q2_x - q1_z * q2_w));

  odom_msg->twist.twist.angular.x = smoothed_twist.angular.x;
  odom_msg->twist.twist.angular.y = smoothed_twist.angular.y;
  odom_msg->twist.twist.angular.z = smoothed_twist.angular.z;

  // Fill covariances
  for (size_t i = 0; i < 6; i++) 
  {
    odom_msg->pose.covariance[i * 6 + i] = pose_covariance_[i];
    odom_msg->twist.covariance[i * 6 + i] = twist_covariance_[i];
  }

  prev_pose_.header = odom_msg->header;
  prev_pose_.pose.position.x = odom_msg->pose.pose.position.x;
  prev_pose_.pose.position.y = odom_msg->pose.pose.position.y;
  prev_pose_.pose.position.z = odom_msg->pose.pose.position.z;
  prev_pose_.pose.orientation = odom_msg->pose.pose.orientation;
}

}  // namespace mocap4r2_people

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mocap4r2_robot_gt::PeopleNode)
