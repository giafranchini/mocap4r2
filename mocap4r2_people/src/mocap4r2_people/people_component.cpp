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
#include <tf2/utils.h>

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
  declare_parameter<std::string>("rigid_body_topic", "rigid_bodies");
  declare_parameter<std::string>("people_topic", "people");
  declare_parameter<std::string>("rigid_body_prefix", "person");
  declare_parameter<double>("alpha", 0.1);

  get_parameter("root_frame", root_frame_);
  get_parameter("map_frame", map_frame_);
  get_parameter("rigid_body_topic", rigid_body_topic_);
  get_parameter("rigid_body_prefix", rigid_body_prefix_);
  get_parameter("people_topic", people_topic_);
  get_parameter("alpha", alpha_);


  rigid_body_sub_ = create_subscription<mocap4r2_msgs::msg::RigidBodies>(
    rigid_body_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PeopleNode::rigid_bodies_callback, this, _1));

  people_pub_ = create_publisher<people_msgs::msg::People>(people_topic_, 10);
  pose_array_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("pose_array", 10);
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
    auto people = std::ranges::filter_view(
      msg->rigidbodies, [this](const mocap4r2_msgs::msg::RigidBody & rb) {
        return rb.rigid_body_name.find(rigid_body_prefix_) != std::string::npos;
      });

    if (people.empty()) {
      RCLCPP_WARN(get_logger(), "No people found in mocap system");
      return;
    }

    auto people_msg = std::make_unique<people_msgs::msg::People>();
    auto pose_array_msg = std::make_unique<geometry_msgs::msg::PoseArray>();

    for (const auto & person : people) {
      // Check if the mocap is publishing the person pose and is not zero
      tf2::Quaternion q;
      tf2::fromMsg(person.pose.orientation, q);
      if (q.length2() < 1e-6) {
        RCLCPP_WARN(
          get_logger(),
          "Zero quaternion received from mocap system. Check that the person is being tracked");
        continue;
      }
      // obtain the root2person transform
      tf2::Transform root2person;
      root2person.setOrigin(
        tf2::Vector3(
          person.pose.position.x, person.pose.position.y,
          person.pose.position.z));
      root2person.setRotation(q);

      // obtain the map2person transform
      tf2::Transform map2person = map2root_ * root2person;

      //consider markers instead of pose_array
      auto person_pose = std::make_unique<geometry_msgs::msg::Pose>();
      tf2::toMsg(map2person, *person_pose);
      geometry_msgs::msg::TransformStamped person_pose_msg;
      person_pose_msg.header = msg->header;
      person_pose_msg.header.frame_id = map_frame_;
      person_pose_msg.child_frame_id = person.rigid_body_name;
      person_pose_msg.transform = tf2::toMsg(map2person);

      tf_broadcaster_->sendTransform(person_pose_msg);

      auto person_msg = std::make_unique<people_msgs::msg::Person>();
      fill_person_msg(person.rigid_body_name, *person_pose, msg->header, person_msg);

      pose_array_msg->poses.push_back(std::move(*person_pose));
      people_msg->people.push_back(std::move(*person_msg));
    }
    pose_array_msg->header = msg->header;
    pose_array_msg->header.frame_id = map_frame_;
    people_msg->header = msg->header;
    people_msg->header.frame_id = map_frame_;

    people_pub_->publish(std::move(people_msg));
    pose_array_pub_->publish(std::move(pose_array_msg));
  }
}

void PeopleNode::fill_person_msg(
  const std::string & person_name,
  const geometry_msgs::msg::Pose & pose,
  const std_msgs::msg::Header & header,
  people_msgs::msg::Person::UniquePtr & person_msg)
{
  person_msg->name = person_name;
  person_msg->reliability = 1.0;

  // fill the pose of the person: x, y, yaw
  person_msg->position.x = pose.position.x;
  person_msg->position.y = pose.position.y;
  // get the yaw from the quaternion
  tf2::Quaternion q;
  tf2::fromMsg(pose.orientation, q);
  person_msg->position.z = tf2::getYaw(q);

  // fill the velocity: vx, vy, vtheta
  compute_velocity(pose, header, person_msg);
}

void PeopleNode::compute_velocity(
  const geometry_msgs::msg::Pose & person_pose,
  const std_msgs::msg::Header & header,
  people_msgs::msg::Person::UniquePtr & person_msg)
{
  static geometry_msgs::msg::Twist smoothed_twist;
  tf2::Transform p1, p2;
  // try to get the previous pose of the person from the map of poses
  // if it is not found, return
  rclcpp::Time t1;
  rclcpp::Time t2 = header.stamp;

  try {
    tf2::fromMsg(prev_poses_[person_msg->name].pose, p1);
    // update the previous pose of the person
    t1 = prev_poses_[person_msg->name].header.stamp;
    prev_poses_[person_msg->name].pose = person_pose;
    prev_poses_[person_msg->name].header = header;
  } catch (const std::out_of_range & e) {
    RCLCPP_WARN(get_logger(), "No previous pose found for person %s", person_msg->name.c_str());
    // add the person to the map of poses
    prev_poses_[person_msg->name].pose = person_pose;
    prev_poses_[person_msg->name].header = header;
    return;
  }

  tf2::fromMsg(person_pose, p2);

  const double q1_w = p1.getRotation().w();
  const double q1_x = p1.getRotation().x();
  const double q1_y = p1.getRotation().y();
  const double q1_z = p1.getRotation().z();

  const double q2_w = p2.getRotation().w();
  const double q2_x = p2.getRotation().x();
  const double q2_y = p2.getRotation().y();
  const double q2_z = p2.getRotation().z();

  const double dt_inv = 1.0 / (t2 - t1).seconds();

  auto delta_trans = p2.getOrigin() - p1.getOrigin();

  // Compute linear velocities

  // Smooth velocities with a simple low-pass filter
  smoothed_twist.linear.x = (1 - alpha_) * smoothed_twist.linear.x + alpha_ *
    (delta_trans.x() * dt_inv);
  smoothed_twist.linear.y = (1 - alpha_) * smoothed_twist.linear.y + alpha_ *
    (delta_trans.y() * dt_inv);

  person_msg->velocity.x = smoothed_twist.linear.x;
  person_msg->velocity.y = smoothed_twist.linear.y;


  // Compute angular velocities
  // smooth angular velocities with a simple low-pass filter
  smoothed_twist.angular.z = (1 - alpha_) * smoothed_twist.angular.z + alpha_ * (
    2.0 * dt_inv * (q1_w * q2_z - q1_x * q2_y + q1_y * q2_x - q1_z * q2_w));

  person_msg->velocity.z = smoothed_twist.angular.z;
}

}  // namespace mocap4r2_people

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mocap4r2_people::PeopleNode)
