#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include "custom_interfaces/msg/buttons.hpp"
#include <gpiod.h>

using namespace std::chrono_literals;
using custom_interfaces::msg::Buttons;

class ButtonReaderNode : public rclcpp::Node {
private:
    std::vector<unsigned int> button_pins;
    std::vector<gpiod_line*> line_objects;
    rclcpp::Publisher<Buttons>::SharedPtr buttons_state_publisher;
    int combined_state;

public:
    ButtonReaderNode() : Node("button_reader_node"), combined_state(0) {
        // Initialize GPIO
        const char* chipname = "gpiochip44";  // Adjust according to your setup
        gpiod_chip* chip = gpiod_chip_open_by_name(chipname);
        if (!chip) {
            throw std::runtime_error("Failed to open GPIO chip: " + std::string(chipname));
        }

        // GPIO pins for the buttons
        button_pins = {21, 20, 16, 1, 7};

        // Publisher for "buttons_state" topic
        buttons_state_publisher = this->create_publisher<custom_interfaces::msg::Buttons>("buttons_state", 10);

        // Setup GPIO pins as inputs with pull-up resistors and events
        for (unsigned int pin : button_pins) {
            gpiod_line* line = gpiod_chip_get_line(chip, pin);
            if (!line) {
                gpiod_chip_close(chip);
                throw std::runtime_error("Failed to get GPIO line: " + std::to_string(pin));
            }

            // Set the pull-up resistor
            if (gpiod_line_request_input_flags(line, "button_reader_node", GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP) < 0) {
                gpiod_chip_close(chip);
                throw std::runtime_error("Failed to set pull-up resistor for GPIO line: " + std::to_string(pin));
            }

            struct gpiod_line_request_config config = {
                .consumer = "button_reader_node",
                .request_type = GPIOD_LINE_REQUEST_EVENT_BOTH_EDGES,
                .flags = GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP
            };

            if (gpiod_line_request(line, &config, 0) < 0) {
                gpiod_chip_close(chip);
                throw std::runtime_error("Failed to request GPIO line as input: " + std::to_string(pin));
            }

            line_objects.push_back(line);
        }

        // Setup event handlers for each GPIO line
        for (size_t i = 0; i < line_objects.size(); ++i) {
            gpiod_line* line = line_objects[i];
            gpiod_line_request_falling_edge_events(line, "button_event_handler");
            gpiod_line_request_rising_edge_events(line, "button_event_handler");
        }
    }

    ~ButtonReaderNode() {
        // Cleanup gpiod on program exit
        for (gpiod_line* line : line_objects) {
            gpiod_line_release(line);
        }
    }
private:
    void button_event_handler(gpiod_line* line) {
        struct gpiod_line_event event;
        int fd = gpiod_line_event_get_fd(line);
        if (fd < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get file descriptor for GPIO event");
            return;
        }

        // Read the last event from the GPIO line
        int ret = gpiod_line_event_read_fd(fd, &event);
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read GPIO event");
            return;
        }

        // Extract GPIO pin from the event
        unsigned int pin = line->offset;
        // Extract the event value (0 for falling edge, 1 for rising edge)
        int value = event.event_type;

        // Print the value of the button
        RCLCPP_INFO(this->get_logger(), "Button %u value: %d", pin, value);

        // Update combined state based on button values (if needed)
        combined_state |= (value << pin);

        // Publish the button state
        Buttons msg;
        msg.state = combined_state;
        buttons_state_publisher->publish(msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ButtonReaderNode>());
    rclcpp::shutdown();
    return 0;
}
