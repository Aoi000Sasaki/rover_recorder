#include "gpio_manager.hpp"

// int main(int argc, char **argv) {
//     GpioManager gpioManager;
//     while (true) {
//         bool val = gpioManager.get_GPIO_PDU_C();
//         printf("GPIO_PDU_C: %d\n", val);
//         usleep(200 * 1000);
//     }
//     return 0;
// }

int main(int argc, char **argv) {
    GpioManager gpioManager;
    bool last = false;
    while (true) {
        bool val = !last;
        gpioManager.set_GPIO_camera(val);
        printf("GPIO_camera: %d\n", val);
        last = val;
        usleep(200 * 1000);
    }
    return 0;
}
