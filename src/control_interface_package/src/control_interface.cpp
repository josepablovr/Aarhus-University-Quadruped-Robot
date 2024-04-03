#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/buttons.hpp"
#include "pigpiod_if2.h"

using namespace std::chrono_literals;
using custom_interfaces::msg::Buttons;

class ButtonReaderNode : public rclcpp::Node {
private:
    std::vector<int> button_pins;
    int combined_state;
    bool initial_state_published;
    rclcpp::Publisher<Buttons>::SharedPtr buttons_state_publisher;
    int pi_handle;
    rclcpp::TimerBase::SharedPtr timer_;
    int stop_count;
    int start_count;
    int pause_count;
    int restart_count;
    int extra_count;
    int last_index;
    int index_count;

public:
    ButtonReaderNode() : Node("button_reader_node"), combined_state(0), initial_state_published(false) {
        // Initialize pigpiod_if2
        pi_handle = pigpio_start(nullptr, nullptr);

        // GPIO pins for the buttons
        button_pins = {21, 20, 16, 1, 7};

        // Publisher for "buttons_state" topic
        buttons_state_publisher = this->create_publisher<custom_interfaces::msg::Buttons>("buttons_state", 10);

        // Setup GPIO pins as inputs with pull-up resistors
        for (int pin : button_pins) {
            set_mode(pi_handle, pin, PI_INPUT);
            set_pull_up_down(pi_handle, pin, PI_PUD_UP);
        }

        // Setup GPIO interrupts
        for (int pin : button_pins) {
            callback_ex(pi_handle, pin, FALLING_EDGE, &ButtonReaderNode::button_interrupt_callback, this);
        }
        
        stop_count = 0;
        start_count = 0;
        pause_count = 0;
        restart_count = 0;
        extra_count = 0;
        last_index = 0;
        index_count = 0;
        
        timer_ = this->create_wall_timer(300ms, std::bind(&ButtonReaderNode::publish_button_state, this));
        
    }

    ~ButtonReaderNode() {
        // Cleanup pigpiod_if2 on program exit
        pigpio_stop(pi_handle);
    }

private:
    void publish_button_state() {
        custom_interfaces::msg::Buttons msg;
        msg.state = combined_state;
        buttons_state_publisher->publish(msg);
        //RCLCPP_INFO(get_logger(), "Publishing button state: %d", combined_state);
    }

    static void button_interrupt_callback(int pi, unsigned int gpio, unsigned int edge, uint32_t tick, void* userdata) {
        ButtonReaderNode* node = static_cast<ButtonReaderNode*>(userdata);
        
        // Determine the index of the button pin
        auto it = std::find(node->button_pins.begin(), node->button_pins.end(), gpio);
        
        if (it != node->button_pins.end()) {
            
            
            
            int index = std::distance(node->button_pins.begin(), it);
            if (node->last_index != index){
                node->index_count = 0;
            }
            node->index_count += 1;
                
            node->last_index = index;
            
            // Update combined state based on button activation
            // Use a bitmask to set specific bits corresponding to the buttons that are pressed
            
            //node->combined_state |= (1 << index);
           
            // Publish the button state if the initial state has been published
            if (node->index_count >= 7){
				node->combined_state = 1 + index;
                if (node->initial_state_published) {
                    node->publish_button_state();
                } else {
                    node->initial_state_published = true;
                }

                // Print the activation of the button
                RCLCPP_INFO(node->get_logger(), "Button %d activated", index);
                node->index_count = 0;
            }
        } else {
            //RCLCPP_WARN(node->get_logger(), "Unknown button pin %d", gpio);
        }
    }
  
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ButtonReaderNode>());
    rclcpp::shutdown();
    return 0;
}
