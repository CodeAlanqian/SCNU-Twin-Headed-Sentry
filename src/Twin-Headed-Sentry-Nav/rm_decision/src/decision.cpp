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

    point_center_.x = 3.0;
    point_center_.y = 0.54;

    point_supply_area_.x = -2.5;
    point_supply_area_.y = 2.85;
    // -2.21, 2.85

    point_wall_center_.x = 0.45;
    point_wall_center_.y = 0.15;

    point_enemy_supply_area_.x = 5.3;
    point_enemy_supply_area_.y = -1.35;

    point_right_.x = 2.90;
    point_right_.y = -2.10;

    point_base_.x = 0.03;
    point_base_.y = -0.60;

    point_wall_left_.x = 0.23;
    point_wall_left_.y = 2.22;

    // point_test1_.x=1.46;
    // point_test1_.y=3.5;

    // point_test2_.x=0;
    // point_test2_.y=0;

    hp_ = 0;
    time_ = 0;
    last_hp_ = 0;
    added_hp_ = 0;
    last_added_hp_ = 0;
    return_state_ = false;
    return_time_ = 0;
    last_send_time_ = 0;
    last_state = 0;
    send_goal_options_.result_callback = std::bind(&RMDecision::goalResultCallback, this, std::placeholders::_1);
    // action_client_->async_send_goal(goal_msg, send_goal_options);
    receive_thread_ = std::thread(&RMDecision::decision, this);
    // decision();
}

void RMDecision::fromSentryCallback(const auto_aim_interfaces::msg::FromSentry::SharedPtr msg)
{
    last_hp_ = hp_;
    last_added_hp_ = added_hp_;
    hp_ = msg->hp;
    time_ = msg->time;
    mode_ = msg->mode;
    if (last_hp_ != 0 && hp_ - last_hp_ > 0 ) {
        added_hp_ += hp_ - last_hp_;
        RCLCPP_INFO(this->get_logger(), "HP has been added. Totally added_hp:%d", added_hp_);
    }

    if (last_added_hp_ != 0 && added_hp_ - last_added_hp_ > 0 ) {
        // returning blood
        return_state_ = true;
        if ( return_time_ == 0 ) {
            return_time_ = time_;
        }
        

        // RCLCPP_INFO(this->get_logger(), "HP has been added. Totally added_hp:%d", added_hp_);
    } else {
        return_state_ = false;
        return_time_ = 0;
    }

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
    rclcpp::WallRate loop_rate(5.0);
    while (rclcpp::ok())
    {
        // RCLCPP_INFO(this->get_logger(), "Decision making.");
        loop_rate.sleep();
        if (time_ > 0 && time_< 295)
        {
            // if (goal_result_.code == rclcpp_action::ResultCode::SUCCEEDED) {
            //     RCLCPP_WARN(this->get_logger(), "Goal has been reached.");
            // }

            
            // if (hp_ == 600) {
            //     goal_msg_.pose.pose.position.x = point_enemy_supply_area_.x;
            //     goal_msg_.pose.pose.position.y = point_enemy_supply_area_.y;

            // } else if (hp_ <= 300) {
            //     goal_msg_.pose.pose.position.x = point_supply_area_.x;
            //     goal_msg_.pose.pose.position.y = point_supply_area_.y;
            // } else if (hp_ == 500 ) {
            //     goal_msg_.pose.pose.position.x = point_center_.x;
            //     goal_msg_.pose.pose.position.y = point_center_.y;
            // }
            // sendGoal();

            // to return hp
            if ( hp_<340 && added_hp_ <= 580 && time_ <= 240 ) {
                goal_msg_.pose.pose.position.x = point_supply_area_.x;
                goal_msg_.pose.pose.position.y = point_supply_area_.y;
                
            } else {
                if ( time_ > 240 ) {
                    //protect base
                    goal_msg_.pose.pose.position.x = point_base_.x;
                    goal_msg_.pose.pose.position.y = point_base_.y;
                } else {
                    //not in returning state
                    if ( return_state_ == false ) {
                        if ( mode_ == 0 ) {
                            goal_msg_.pose.pose.position.x = point_home_.x;
                            goal_msg_.pose.pose.position.y = point_home_.y;
                        } else if ( mode_ == 1 ) {
                            
                            if ( time_ - last_send_time_ >= 6 && last_state == 0 ) {
                                last_send_time_ = time_;
                                last_state = 1;
                                goal_msg_.pose.pose.position.x = point_wall_left_.x;
                                goal_msg_.pose.pose.position.y = point_wall_left_.y;
                            } else if ( time_ - last_send_time_ >= 6 && last_state == 1 ){
                                last_send_time_ = time_;
                                last_state = 0;
                                goal_msg_.pose.pose.position.x = point_wall_center_.x;
                                goal_msg_.pose.pose.position.y = point_wall_center_.y;
                            }
                            //TODO 
                            // goal_msg_.pose.pose.position.x = point_wall_center_.x;
                            // goal_msg_.pose.pose.position.y = point_wall_center_.y;

                        } else if ( mode_ == 2 ) {

                            goal_msg_.pose.pose.position.x = point_right_.x;
                            goal_msg_.pose.pose.position.y = point_right_.y;

                        } else {
                            RCLCPP_INFO(this->get_logger(), "Mode is not correct.");
                        }
                    } else {
                        //returning state, remain 5s
                        if ( return_time_ > 0 && time_ - return_time_ > 5 ) {
                            goal_msg_.pose.pose.position.x = point_home_.x;
                            goal_msg_.pose.pose.position.y = point_home_.y;
                        }
                    }


                }
            
                


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

bool RMDecision:: isGoalReached(geometry_msgs::msg::Point target) {
    float error = 0.05;
    if ( (localPosition_x_ > target.x - error) && (localPosition_x_ < target.x + error) && \
        (localPosition_y_ > target.y - error) && (localPosition_y_ < target.y + error) ) {
        return true;
    }

    return false;

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