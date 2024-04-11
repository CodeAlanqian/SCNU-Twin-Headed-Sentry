#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include "auto_aim_interfaces/msg/from_sentry.hpp"

using namespace std::chrono_literals;
class sim_pub : public rclcpp::Node
{
private:
    /* data */
    rclcpp::Publisher<auto_aim_interfaces::msg::FromSentry>::SharedPtr from_sentry_pub_;
    uint16_t time_;
    uint16_t hp_;
    uint8_t mode_;
    rclcpp::Time time_begin, now;
public:
    sim_pub(/* args */);
    ~sim_pub();
    void update();
};

sim_pub::sim_pub(/* args */)
{
    time_begin = this->get_clock()->now();
    time = 0;
    hp_ = 600;
    mode = 0;
    from_sentry_pub_ = this->create_publisher<auto_aim_interfaces::msg::FromSentry>("/from_sentry", 10);
    auto msg = auto_aim_interfaces::msg::FromSentry();
    msg.hp = 100;
    msg.time = 100;
    msg.mode = 1;
    from_sentry_pub_->publish(msg);
}

sim_pub::update()
{
    now = this->get_clock()->now();
    time = now - time_begin;
    hp_ = 600 - time;
    if (hp_ < 0)
    {
        hp_ = 0;
    }
    auto msg = auto_aim_interfaces::msg::FromSentry();
    msg.hp = hp_;
    msg.time = time;
    msg.mode = 1;
    from_sentry_pub_->publish(msg);
}


sim_pub::~sim_pub()
{

}

