#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/leds.hpp" // Assuming the LED message type is defined in this header

using namespace std::chrono_literals; // For time literals

class LedStatePublisher : public rclcpp::Node {
public:
    LedStatePublisher() : Node("led_state_publisher"), message() {
        publisher_ = this->create_publisher<custom_interfaces::msg::Leds>("led_state", 10);
        timer_ = this->create_wall_timer(5s, std::bind(&LedStatePublisher::publish_led_state, this));
        message.state = 0; // Initialize state to 0
    }

private:
    void publish_led_state() {
        message.state = (message.state + 1) % 4; // Cycle between 0, 1, 2, and 3
        publisher_->publish(message);
    }

    custom_interfaces::msg::Leds message; // Member variable to hold the message
    rclcpp::Publisher<custom_interfaces::msg::Leds>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedStatePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
