#include "gpio_manager.hpp"

GpioManager::GpioManager() {
    // Open GPIO_PDU_C
    this->chip_PDU_C = gpiod_chip_open_by_name(this->GPIO_PDU_C_CHIPNAME);
    if (!this->chip_PDU_C) {
        perror("Open chip failed");
        exit(1);
    }

    this->line_PDU_C = gpiod_chip_get_line(this->chip_PDU_C, this->GPIO_PDU_C_LINE_OFFSET);
    if (!this->line_PDU_C) {
        perror("Get line failed");
        gpiod_chip_close(this->chip_PDU_C);
        exit(1);
    }

    if (gpiod_line_request_input(this->line_PDU_C, "gpio_manager") < 0) {
        perror("Request line as input failed");
        gpiod_chip_close(this->chip_PDU_C);
        exit(1);
    }

    // Open GPIO_camera
    this->chip_camera = gpiod_chip_open_by_name(this->GPIO_camera_CHIPNAME);
    if (!this->chip_camera) {
        perror("Open chip failed");
        exit(1);
    }

    this->line_camera = gpiod_chip_get_line(this->chip_camera, this->GPIO_camera_LINE_OFFSET);
    if (!this->line_camera) {
        perror("Get line failed");
        gpiod_chip_close(this->chip_camera);
        exit(1);
    }

    if (gpiod_line_request_output(this->line_camera, "gpio_manager", 0) < 0) {
        perror("Request line as output failed");
        gpiod_chip_close(this->chip_camera);
        exit(1);
    }
}

GpioManager::~GpioManager() {
    gpiod_line_release(this->line_PDU_C);
    gpiod_chip_close(this->chip_PDU_C);

    gpiod_line_release(this->line_camera);
    gpiod_chip_close(this->chip_camera);
}

bool GpioManager::get_GPIO_PDU_C() {
    int val = gpiod_line_get_value(this->line_PDU_C);
    if (val < 0) {
        perror("Read line input failed");
        return false;
    }
    return val == 1;
}

void GpioManager::set_GPIO_camera(bool value) {
    if (gpiod_line_set_value(this->line_camera, value) < 0) {
        perror("Set line output failed");
    }
}
