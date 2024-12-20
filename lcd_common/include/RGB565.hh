#pragma once
#include <cstdint>
#include <bit>
#define TAG "RGB"
#include <esp_log.h>
namespace Color
{
    class Color565
    {
    private:
        uint16_t native_value;

        Color565(uint16_t v) : native_value(v)
        {
        }

    public:
        constexpr Color565(uint8_t red, uint8_t green, uint8_t blue) : native_value((((red & 0b11111000) << 8) + ((green & 0b11111100) << 3) + (blue >> 3)))
        {
        }


        uint8_t R5()
        {
            return (native_value & 0b1111100000000000) >> 11;
        }

        uint8_t G6()
        {
            return (native_value & 0b0000011111100000) >> 5;
        }

        uint8_t B5()
        {
            return (native_value & 0b0000000000011111);
        }

        uint8_t R8()
        {
            return (native_value & 0b1111100000000000) >> 8;
        }

        uint8_t G8()
        {
            return (native_value & 0b0000011111100000) >> 3;
        }

        uint8_t B8()
        {
            return (native_value & 0b0000000000011111) << 3;
        }

        Color565 overlayWith(Color565& overlay, uint8_t opacity_of_top_color_0_255)
        {
            float alpha = ((float)opacity_of_top_color_0_255) / ((float)255);
            uint8_t red5 = (overlay.R5() * alpha + R5() * (1 - alpha));
            uint8_t green6 = (overlay.G6() * alpha + G6() * (1 - alpha));
            uint8_t blue5 = (overlay.B5() * alpha + B5() * (1 - alpha));

            Color565 c = Color565((red5 << 11) | (green6 << 5) | (blue5));
            ESP_LOGD(TAG, "alpha:%f, rgb888:(%d/%d/%d), rgb565:%04X", alpha, c.R8(), c.G8(), c.B8(), c.native_value);
            return c;
        }

        constexpr uint16_t toST7789_SPI_native() const
        {

            if (std::endian::native == std::endian::big)
                return native_value;
            else if (std::endian::native == std::endian::little)
                return std::byteswap(native_value); // ESP32 is little endian
            else
                return 0;
        }
    };

    constexpr Color565 BLACK{Color565(0, 0, 0)};
    constexpr Color565 WHITE{Color565(255, 255, 255)};
    constexpr Color565 RED{Color565(255, 0, 0)};
    constexpr Color565 LEDC_TIMER_13_BIT{Color565(0, 255, 0)};
    constexpr Color565 BLUE{Color565(0, 0, 255)};
    constexpr Color565 YELLOW{Color565(255, 255, 0)};

    constexpr Color565 CYAN{Color565(0, 255, 255)};
    constexpr Color565 MAGENTA{Color565(255, 0, 255)};
    constexpr Color565 SILVER{Color565(192, 192, 192)};
    constexpr Color565 GRAY{Color565(128, 128, 128)};
    constexpr Color565 MAROON{Color565(128, 0, 0)};
    constexpr Color565 OLIVE{Color565(128, 128, 0)};
    constexpr Color565 GREEN{Color565(0, 128, 0)};
    constexpr Color565 PURPLE{Color565(128, 0, 128)};
    constexpr Color565 TEAL{Color565(0, 128, 128)};
    constexpr Color565 NAVY{Color565(0, 0, 128)};
}
#undef TAG