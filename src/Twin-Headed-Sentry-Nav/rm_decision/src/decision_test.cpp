#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

#include "decision.hpp"

using namespace std::chrono_literals;

namespace rm_decision
{
RMDecision::RMDecision(const rclcpp::NodeOptions & options)
: Node("rm_decision", options)
{
    RCLCPP_INFO(this->get_logger(), "RMDecision node has been started.");
    from_sentry_sub_ = this->create_subscription<auto_aim_interfaces::msg::FromSentry>(
        "/from_sentry", 10, std::bind(&RMDecision::fromSentryCallback, this, std::placeholders::_1));
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", 10, std::bind(&RMDecision::getLocalposition, this, std::placeholders::_1));

    goal_msg_.pose.header.frame_id = "map";
    goal_msg_.pose.pose.position.x = 0.0;
    goal_msg_.pose.pose.position.y = 0.0;
    goal_msg_.pose.pose.position.z = 0.0;
    goal_msg_.pose.pose.orientation.x = 0.0;
    goal_msg_.pose.pose.orientation.y = 0.0;
    goal_msg_.pose.pose.orientation.z = 0.0;
    goal_msg_.pose.pose.orientation.w = 1.0;

    
    point_home_.x = 0.0;
    point_home_.y = 0.0;

    point_test1_.x=1.46;
    point_test1_.y=3.5;

    point_test2_.x=0;
    point_test2_.y=0;

    hp_ = 0;
    time_ = 0;

    send_goal_options_.result_callback = std::bind(&RMDecision::goalResultCallback, this, std::placeholders::_1);
    // action_client_->async_send_goal(goal_msg, send_goal_options);
    receive_thread_ = std::thread(&RMDecision::decision, this);
    // decision();
}

void RMDecision::fromSentryCallback(const auto_aim_interfaces::msg::FromSentry::SharedPtr msg)
{
    hp_ = msg->hp;
    time_ = msg->time;
    RCLCPP_INFO(this->get_logger(), "Received message from Sentry. hp:%d, time:%d", hp_, time_);
}

void RMDecision::goalResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result)
{
    RCLCPP_INFO(this->get_logger(), "Goal has been reached.");
    goal_result_ = result;
    //goal_result_.code  =
    //rclcpp_action::ResultCode::SUCCEEDED
    //rclcpp_action::ResultCode::ABORTED
    //rclcpp_action::ResultCode::CANCELED
}


void RMDecision::sendGoal()
{
    RCLCPP_INFO(this->get_logger(), "Sending goal.");
    while (!action_client_->wait_for_action_server(10s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "Action server not available after waiting.");
            return;
        }
        RCLCPP_INFO(get_logger(), "Waiting for action server to be available...");
    }
    
    action_client_->async_send_goal(goal_msg_, send_goal_options_);
}


void RMDecision::getLocalposition(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received message from Odometry.");
    localPosition_x_ = msg->pose.pose.position.x;
    localPosition_y_ = msg->pose.pose.position.y;
    RCLCPP_INFO(this->get_logger(), "x:%f, y:%f", msg->pose.pose.position.x, msg->pose.pose.position.y);
}


void RMDecision::decision()
{
    rclcpp::WallRate loop_rate(10.0);
    while (rclcpp::ok())
    {
        RCLCPP_INFO(this->get_logger(), "Decision making.");
        loop_rate.sleep();
        if (time_ > 0)
        {
            if (hp_ == 600) {
                goal_msg_.pose.pose.position.x = point_enemy_supply_area_.x;
                goal_msg_.pose.pose.position.y = point_enemy_supply_area_.y;

            } else if (hp_ <= 300) {
                goal_msg_.pose.pose.position.x = point_supply_area_.x;
                goal_msg_.pose.pose.position.y = point_supply_area_.y;
            } else if (hp_ == 500 ) {
                goal_msg_.pose.pose.position.x = point_center_.x;
                goal_msg_.pose.pose.position.y = point_center_.y;
            }
            sendGoal();

        }
    }
    
    // RCLCPP_INFO(this->get_logger(), "Decision making.");
    // if (goal_result_.code == rclcpp_action::ResultCode::SUCCEEDED)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Goal has been reached.");
    // }
    // else if (goal_result_.code == rclcpp_action::ResultCode::ABORTED)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Goal has been aborted.");
    // }
    // else if (goal_result_.code == rclcpp_action::ResultCode::CANCELED)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Goal has been canceled.");
    // }
    // else
    // {
    //     RCLCPP_INFO(this->get_logger(), "Goal has not been reached.");
    // }
    
    
    // if (hp_ < 100)
    // {
    //     goal_msg_.pose.pose.position.x = point_home_[0];
    //     goal_msg_.pose.pose.position.y = point_home_[1];
    //     sendGoal();
    // }
    // else if (hp_ >= 100 && hp_ < 200)
    // {
    //     goal_msg_.pose.pose.position.x = point_center_[0];
    //     goal_msg_.pose.pose.position.y = point_center_[1];
    //     sendGoal();
    // }
    // else if (hp_ >= 200 && hp_ < 300)
    // {
    //     goal_msg_.pose.pose.position.x = point_supplyArea_[0];
    //     goal_msg_.pose.pose.position.y = point_supplyArea_[1];
    //     sendGoal();
    // }
    // else if (hp_ >= 300 && hp_ < 400)
    // {
    //     goal_msg_.pose.pose.position.x = point_enemySupplyArea_[0];
    //     goal_msg_.pose.pose.position.y = point_enemySupplyArea_[1];
    //     sendGoal();
    // }
    // else
    // {
    //     RCLCPP_INFO(this->get_logger(), "HP is too high.");
    // }
}

RMDecision::~RMDecision()
{
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
    RCLCPP_INFO(this->get_logger(), "RMDecision node has been destroyed.");
}




} // namespace rm_decision

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node_options = rclcpp::NodeOptions(); 
    auto node = std::make_shared<rm_decision::RMDecision>(node_options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}