extern "C" {
    #include <gpiod.h>
    #include <stdlib.h>
    #include <stdio.h>
    #include <unistd.h>
}

class GpioManager {
    public:
        GpioManager();
        ~GpioManager();
        bool get_GPIO_PDU_C();
        void set_GPIO_camera(bool value);

    private:
        // GPIO_PDU_C (PIN11: GPIO3_C1)
        // for input
        const char *GPIO_PDU_C_CHIPNAME = "gpiochip3";
        const int GPIO_PDU_C_LINE_OFFSET = 17;
        struct gpiod_chip *chip_PDU_C;
        struct gpiod_line *line_PDU_C;

        // GPIO_camera (PIN13: GPIO3_B7)
        // for output
        const char *GPIO_camera_CHIPNAME = "gpiochip3";
        const int GPIO_camera_LINE_OFFSET = 15;
        struct gpiod_chip *chip_camera;
        struct gpiod_line *line_camera;

        bool return_val = false;
        bool last_PDU_C_val = false;
        int PDU_C_continuous_count = 0;
        int PDU_C_continuous_threshold = 10;
};
