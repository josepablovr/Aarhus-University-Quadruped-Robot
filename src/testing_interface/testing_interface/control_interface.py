#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/buttons.hpp"
#include <pigpio.h>

using namespace std::chrono_literals;
using custom_interfaces::msg::Buttons;

class ButtonReaderPublisherNode : public rclcpp::Node {
private:
    std::vector<int> button_pins;
    std::vector<int> last_states;
    rclcpp::Publisher<Buttons>::SharedPtr buttons_state_publisher;

public:
    ButtonReaderPublisherNode()
        : Node("button_reader"), last_states(std::vector<int>(5, PI_HIGH))
    {
        // GPIO pins for the buttons
        button_pins = {21, 20, 16, 1, 7};

        // Publisher for "buttons_state" topic
        buttons_state_publisher = this->create_publisher<Buttons>("buttons_state", 10);

        // Setup GPIO pins as inputs with pull-up resistors
        for (int pin : button_pins) {
            gpioSetMode(pin, PI_INPUT);
            gpioSetPullUpDown(pin, PI_PUD_UP);
        }

        // Timer to periodically publish button state
        auto timer_callback = [this]() -> void {
            publish_button_state();
        };
        this->create_wall_timer(100ms, timer_callback);

        // Setup GPIO interrupts
        for (int pin : button_pins) {
            gpioSetISRFunc(pin, FALLING_EDGE, 0, [](int gpio, int level, uint32_t tick, void* userdata) -> void {
                ButtonReaderPublisherNode* instance = static_cast<ButtonReaderPublisherNode*>(userdata);
                instance->handle_button_press(gpio);
            }, this);
        }
    }

    ~ButtonReaderPublisherNode() {
        // Cleanup GPIO on program exit
        gpioTerminate();
    }

private:
    void publish_button_state() {
        Buttons msg;
        msg.state = this->last_states;
        this->buttons_state_publisher->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing button state");
    }

    void handle_button_press(int gpio) {
        for (size_t i = 0; i < this->button_pins.size(); ++i) {
            if (gpio == this->button_pins[i]) {
                if (this->last_states[i] != 0) {
                    // Debounce: wait for stable state
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                    int level = gpioRead(gpio);
                    if (level != this->last_states[i]) {
                        this->last_states[i] = level;
                        RCLCPP_INFO(this->get_logger(), "Button %d pressed", gpio);
                    }
                }
                break;
            }
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ButtonReaderPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
