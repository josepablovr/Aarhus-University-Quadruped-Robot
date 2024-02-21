#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "custom_interfaces/msg/leds.hpp"
#include "pigpiod_if2.h"

class LedControllerNode : public rclcpp::Node {
    private:
        int pi_handle;
        rclcpp::Subscription<custom_interfaces::msg::Leds>::SharedPtr subscription_;

    public:
        LedControllerNode() : Node("led_controller_node") {
            // Initialize pigpiod_if2
            pi_handle = pigpio_start(nullptr, nullptr);

            // Subscriber for "led_state" topic
           
            subscription_ = this->create_subscription<custom_interfaces::msg::Leds>(
          "led_state", 10, std::bind(&LedControllerNode::led_state_callback, this, std::placeholders::_1));
        }

        ~LedControllerNode() {
            // Cleanup pigpiod_if2 on program exit
            pigpio_stop(pi_handle);
        }

    private:
        void led_state_callback(const custom_interfaces::msg::Leds::SharedPtr msg) {
            // Turn off all LEDs
            gpio_write(pi_handle, 13, 0);
            gpio_write(pi_handle, 19, 0);
            gpio_write(pi_handle, 26, 0);

            // Turn on the LED based on the message
            int led_index = msg->state;
            switch (led_index) {
                case 1:
                    gpio_write(pi_handle, 13, 1);
                    break;
                case 2:
                    gpio_write(pi_handle, 19, 1);
                    break;
                case 3:
                    gpio_write(pi_handle, 26, 1);
                    break;
                default:
                    // Invalid LED index, do nothing
                    break;
        }
}
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LedControllerNode>());
    rclcpp::shutdown();
    return 0;
}
