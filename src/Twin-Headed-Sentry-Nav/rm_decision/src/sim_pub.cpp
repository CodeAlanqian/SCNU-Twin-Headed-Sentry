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
    sim_pub();
    ~sim_pub();
};

sim_pub::sim_pub() : Node("sim_pub")
{
    time_ = 0;
    from_sentry_pub_ = this->create_publisher<auto_aim_interfaces::msg::FromSentry>("/from_sentry", 10);
    RCLCPP_INFO(this->get_logger(), "sim_pub has been started");
    auto msg = auto_aim_interfaces::msg::FromSentry();
    rclcpp::WallRate loop_rate(1.0);
    hp_ = 600;
    mode_ = 1;
    while (rclcpp::ok())
    {
        loop_rate.sleep();
        time_++;
        if (time_ == 10 || time_ == 20 || time_ == 30 )
        {
            hp_ -= 150;
        }
        if (time_ == 24 || time_ == 25)
        {
            hp_ += 100;
        }

        if(time_ == 100) {
            hp_ -= 100;
        }

        if (time_ == 110)
        {
            hp_ += 150;
        }
        msg.hp = hp_;
        msg.time = time_;
        msg.mode = mode_;
        from_sentry_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing: hp: %d, time: %d, mode: %d", msg.hp, msg.time, msg.mode);
        /* code */
    }
    
}




sim_pub::~sim_pub()
{

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sim_pub>());
    rclcpp::shutdown();
    return 0;
}