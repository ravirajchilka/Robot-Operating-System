#ifndef BLINK_LED_BUTTON_CLICK_HPP
#define BLINK_LED_BUTTON_CLICK_HPP

#include<cstdint>
#include<string>
#include<gpiod.hpp>

class BlinkLEDbuttonClick
{
    private:
        const char* gpiopathname;
        const std::uint8_t led_offset;
        const std::uint8_t button_offset;
        const std::string consumer;
        gpiod::chip chip;
        gpiod::line led_line;
        gpiod::line btn_line;

    public:
        BlinkLEDbuttonClick();
        ~BlinkLEDbuttonClick();
        void blinkLEDonBtn(std::uint16_t onTime, std::uint16_t offTime);
};

#endif
