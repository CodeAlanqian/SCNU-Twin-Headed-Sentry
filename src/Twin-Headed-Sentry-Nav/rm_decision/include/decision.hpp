#ifndef RM_DICISION__DECISION_HPP_
#define RM_DICISION__DECISION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

//nav2
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <thread>
//msg from sentry
#include "auto_aim_interfaces/msg/from_sentry.hpp"
namespace rm_decision
{

class RMDecision : public rclcpp::Node
{
public:
    explicit RMDecision(const rclcpp::NodeOptions & options);
    ~RMDecision();
private:
    rclcpp::Subscription<auto_aim_interfaces::msg::FromSentry>::SharedPtr from_sentry_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    void fromSentryCallback(const auto_aim_interfaces::msg::FromSentry::SharedPtr msg);
    void decision();
    void sendGoal();
    void getLocalposition(const nav_msgs::msg::Odometry::SharedPtr msg);
    void goalResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result);
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;

    nav2_msgs::action::NavigateToPose::Goal goal_msg_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions send_goal_options_;
    // nav_msgs::msg::Odometry localPosition_;
    float localPosition_x_;
    float localPosition_y_;
    uint16_t hp_;
    uint16_t time_;
    uint8_t mode_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult goal_result_;
    // float point_home_ = {0.0, 0.0};
    // float point_center_ = {3.0, 1.0};
    // float point_supplyArea_ = {-2.5, 4.07};
    // float point_enemySupplyArea_ = {5.3, -1.35};
    geometry_msgs::msg::Point point_home_;
    geometry_msgs::msg::Point point_center_;
    geometry_msgs::msg::Point point_supply_area_;
    geometry_msgs::msg::Point point_enemy_supply_area_;
    geometry_msgs::msg::Point point_wall_;

    geometry_msgs::msg::Point point_test1_;
    geometry_msgs::msg::Point point_test2_;

    std::thread receive_thread_;
    /* data */
};



}  // namespace rm_decision



#endif  // RM_DICISION__DECISION_HPP_