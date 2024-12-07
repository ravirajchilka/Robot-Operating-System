#include <iostream>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <cstdint>
#include "../../udemy_ros2_pkg/include/udemy_ros2_pkg/led_blink_button_logic.hpp"

BlinkLEDbuttonClick::BlinkLEDbuttonClick()
	: gpiopathname("/dev/gpiochip0"),
	  led_offset(4),
	  button_offset(17),
	  consumer("led blink on btn click"),
	  chip(gpiopathname)
{
	try 
	{
		led_line = chip.get_line(led_offset);
		btn_line = chip.get_line(button_offset);
		led_line.request({consumer, gpiod::line_request::DIRECTION_OUTPUT, 0});
		btn_line.request({consumer, gpiod::line_request::DIRECTION_INPUT, 0});
	}
    catch(const std::exception& e) 
	{
        std::cerr << "Error in gpiod allocation" << e.what() << std::endl;
    }
}


BlinkLEDbuttonClick::~BlinkLEDbuttonClick()
{
    try {
        if(led_line.is_requested() || btn_line.is_requested()) {
            led_line.release();
            btn_line.release();
        }
    }
    catch(const std::exception& e) {
        std::cerr << "Error in gpiod release" << e.what() << std::endl;
    }
}


void BlinkLEDbuttonClick::blinkLEDonBtn(std::uint16_t onTime, std::uint16_t offTime)
{
    while(1) {
        if (btn_line.get_value() == 1) {
            led_line.set_value(1);
            std::this_thread::sleep_for(std::chrono::milliseconds(onTime));
            led_line.set_value(0);
            std::this_thread::sleep_for(std::chrono::milliseconds(offTime));
        }
        else {
            led_line.set_value(0);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
};

int main() {

    BlinkLEDbuttonClick bllinkld;
    bllinkld.blinkLEDonBtn(1000,2000);
    return 0;
}

