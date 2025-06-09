#include <cnoid/SimpleController>
#include <cnoid/EigenUtil>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "choreonoid_tutorial/msg/robot_observation.hpp"

class RosChoreonoidBridgeController1 : public cnoid::SimpleController
{
public:
  void client2ServerCallback(const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr& msg)
  {
    const rclcpp::Time stamp = msg->header.stamp;
    if (stamp != stamp_sent_)
    {
      RCLCPP_INFO(node_->get_logger(), "Received duplicate or out-of-order data: %lf", stamp.seconds());
      return;
    }

    stamp_received_ = stamp;
    RCLCPP_INFO(node_->get_logger(), "Received new data: %lf", stamp_received_.seconds());
  }

  bool configure(cnoid::SimpleControllerConfig* config) override
  {
    node_ = std::make_shared<rclcpp::Node>(config->controllerName());

    server2client_pub_ = node_->create_publisher<choreonoid_tutorial::msg::RobotObservation>("server2client", 1);

    client2server_sub_ = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "client2server", 1,
        std::bind(&RosChoreonoidBridgeController1::client2ServerCallback, this, std::placeholders::_1));

    executor_ = std::make_unique<rclcpp::executors::StaticSingleThreadedExecutor>();
    executor_->add_node(node_);

    control_rate_ = std::make_unique<rclcpp::Rate>(200);

    return true;
  }

  bool initialize(cnoid::SimpleControllerIO* io) override
  {
    // initializes variables
    io_body_ = io->body();

    q_ref_.clear();
    q_ref_.reserve(io_body_->numJoints());
    dq_ref_.clear();
    dq_ref_.reserve(io_body_->numJoints());

    // enable joints
    for (const auto joint : io_body_->joints())
    {
      joint->setActuationMode(JointTorque);
      io->enableOutput(joint, JointTorque);
      io->enableInput(joint, JointAngle | JointVelocity);

      q_ref_.push_back(joint->q());
      dq_ref_.push_back(0);  // dq reference is 0 temporarily
    }

    // enable rootLink
    io->enableInput(io_body_->rootLink(), LinkPosition | LinkTwist);

    robot_observation_.joint_states.name.resize(io_body_->numJoints());
    robot_observation_.joint_states.position.resize(io_body_->numJoints());
    robot_observation_.joint_states.velocity.resize(io_body_->numJoints());

    for (size_t i = 0; i < io_body_->numJoints(); ++i)
    {
      const auto joint = io_body_->joint(i);
      robot_observation_.joint_states.name[i] = joint->name();
      robot_observation_.joint_states.position[i] = joint->q();
      robot_observation_.joint_states.velocity[i] = joint->dq();
    }

    stamp_received_ = stamp_sent_ = node_->now();

    return true;
  }

  bool control() override
  {
    while (rclcpp::ok() && stamp_sent_ > stamp_received_)
    {
      executor_->spin_some();

      robot_observation_.header.stamp = stamp_sent_;

      server2client_pub_->publish(robot_observation_);
      RCLCPP_INFO(node_->get_logger(), "Sent: %lf", stamp_sent_.seconds());

      control_rate_->sleep();
    }

    if (stamp_sent_ == stamp_received_)
    {
      stamp_sent_ = node_->now();

      for (size_t i = 0; i < io_body_->numJoints(); ++i)
      {
        const auto joint = io_body_->joint(i);
        robot_observation_.joint_states.name[i] = joint->name();
        robot_observation_.joint_states.position[i] = joint->q();
        robot_observation_.joint_states.velocity[i] = joint->dq();
      }

      // orientation
      cnoid::Quaternion quaternion(io_body_->rootLink()->T().rotation());
      robot_observation_.imu.orientation.x = quaternion.x();
      robot_observation_.imu.orientation.y = quaternion.y();
      robot_observation_.imu.orientation.z = quaternion.z();
      robot_observation_.imu.orientation.w = quaternion.w();

      // angular velocity
      const auto& w = io_body_->rootLink()->w();
      robot_observation_.imu.angular_velocity.x = w.x();
      robot_observation_.imu.angular_velocity.y = w.y();
      robot_observation_.imu.angular_velocity.z = w.z();
    }

    for (size_t i = 0; i < io_body_->numJoints(); ++i)
    {
      const auto joint = io_body_->joint(i);
      // PD control
      const auto u = (q_ref_[i] - joint->q()) * P_GAIN + (dq_ref_[i] - joint->dq()) * D_GAIN;
      io_body_->joint(i)->u() = u;
    }

    return true;
  }

private:
  // PD gains
  static constexpr auto P_GAIN = 200.0;
  static constexpr auto D_GAIN = 50.0;

  // ROS
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<choreonoid_tutorial::msg::RobotObservation>::SharedPtr server2client_pub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr client2server_sub_;
  rclcpp::executors::StaticSingleThreadedExecutor::UniquePtr executor_;
  rclcpp::Rate::UniquePtr control_rate_;

  choreonoid_tutorial::msg::RobotObservation robot_observation_;

  rclcpp::Time stamp_received_, stamp_sent_;

  // interfaces for a simulated body
  cnoid::BodyPtr io_body_;

  // data buffer
  std::vector<double> q_ref_;
  std::vector<double> dq_ref_;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(RosChoreonoidBridgeController1)
