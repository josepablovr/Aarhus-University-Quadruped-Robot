#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include "custom_interfaces/msg/buttons.hpp"
#include <gpiod.h>

using namespace std::chrono_literals;
using custom_interfaces::msg::Buttons;

class ButtonReaderNode : public rclcpp::Node {
private:
    std::vector<unsigned int> button_pins;
    int combined_state;
    bool initial_state_published;
    rclcpp::Publisher<Buttons>::SharedPtr buttons_state_publisher;
    std::vector<gpiod_line*> line_objects;
    gpiod_chip* chip;

public:
    ButtonReaderNode() : Node("button_reader_node"), combined_state(0), initial_state_published(false) {
        // Initialize GPIO chip
        const char* chipname = "gpiochip4"; // Adjust based on gpiodetect output
        chip = gpiod_chip_open_by_name(chipname);
        if (!chip) {
            throw std::runtime_error("Failed to open GPIO chip: " + std::string(chipname));
        }

        // GPIO pins for the buttons
        button_pins = {21, 20, 16, 1, 7}; // Adjust based on gpioinfo output

        // Publisher for "buttons_state" topic
        buttons_state_publisher = this->create_publisher<custom_interfaces::msg::Buttons>("buttons_state", 10);

        // Setup GPIO pins as inputs with pull-up resistors and interrupts
        for (unsigned int pin : button_pins) {
            gpiod_line* line = gpiod_chip_get_line(chip, pin);
            if (!line) {
                gpiod_chip_close(chip);
                throw std::runtime_error("Failed to get GPIO line: " + std::to_string(pin));
            }

            if (gpiod_line_request_both_edges_events(line, "button_reader_node") < 0) {
                gpiod_chip_close(chip);
                throw std::runtime_error("Failed to request GPIO line as input: " + std::to_string(pin));
            }

            line_objects.push_back(line);
        }

        // Start a thread to poll for events
        std::thread([this]() { this->poll_events(); }).detach();
    }

    ~ButtonReaderNode() {
        // Cleanup GPIO on program exit
        for (gpiod_line* line : line_objects) {
            gpiod_line_release(line);
        }
        gpiod_chip_close(chip);
    }
private:
    void poll_events() {
        while (rclcpp::ok()) {
            for (size_t i = 0; i < line_objects.size(); ++i) {
                gpiod_line_event event;
                int ret = gpiod_line_event_wait(line_objects[i], nullptr);
                if (ret < 0) {
                    RCLCPP_WARN(this->get_logger(), "Error waiting for event on GPIO line %d", button_pins[i]);
                    continue;
                }

                if (ret == 0) {
                    // No event occurred, continue polling
                    continue;
                }

                ret = gpiod_line_event_read(line_objects[i], &event);
                if (ret < 0) {
                    RCLCPP_WARN(this->get_logger(), "Error reading event on GPIO line %d", button_pins[i]);
                    continue;
                }

                if (event.event_type == GPIOD_LINE_EVENT_FALLING_EDGE) {
                    button_interrupt_callback(button_pins[i]);
                }
            }
        }
    }

    void button_interrupt_callback(unsigned int gpio) {
        // Determine the index of the button pin
        auto it = std::find(button_pins.begin(), button_pins.end(), gpio);
        if (it != button_pins.end()) {
            int index = std::distance(button_pins.begin(), it);

            // Update combined state based on button activation
            combined_state = (1 << index);

            // Publish the button state if the initial state has been published
            if (initial_state_published) {
                publish_button_state();
            } else {
                initial_state_published = true;
            }

            // Print the activation of the button
            RCLCPP_INFO(this->get_logger(), "Button %d activated", index);
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown button pin %d", gpio);
        }
    }

    void publish_button_state() {
        custom_interfaces::msg::Buttons msg;
        msg.state = combined_state;
        buttons_state_publisher->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing button state: %d", combined_state);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ButtonReaderNode>());
    rclcpp::shutdown();
    return 0;
}
