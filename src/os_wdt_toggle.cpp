extern "C" {
    #include <gpiod.h>
    #include <stdlib.h>
    #include <stdio.h>
    #include <unistd.h>
}

#include <thread>
#include <chrono>

const int RETRY_INTERVAL_SECONDS = 5;
const int MAX_RETRIES = 12; // 1分間リトライ

int main(void) {
    // GPIO_OS_WDT (PIN15: GPIO3_C0)
    // for output
    const char *GPIO_OS_WDT_CHIPNAME = "gpiochip3";
    const int GPIO_OS_WDT_LINE_OFFSET = 16;
    struct gpiod_chip *chip_OS_WDT = nullptr;
    struct gpiod_line *line_OS_WDT = nullptr;

    int retries = 0;

    // Retry opening the GPIO chip
    while (retries < MAX_RETRIES) {
        chip_OS_WDT = gpiod_chip_open_by_name(GPIO_OS_WDT_CHIPNAME);
        if (chip_OS_WDT) {
            break;
        }
        perror("Open chip failed, retrying...");
        std::this_thread::sleep_for(std::chrono::seconds(RETRY_INTERVAL_SECONDS));
        retries++;
    }
    if (!chip_OS_WDT) {
        fprintf(stderr, "Failed to open chip after %d retries\n", MAX_RETRIES);
        exit(1);
    }

    retries = 0;
    // Retry getting the GPIO line
    while (retries < MAX_RETRIES) {
        line_OS_WDT = gpiod_chip_get_line(chip_OS_WDT, GPIO_OS_WDT_LINE_OFFSET);
        if (line_OS_WDT) {
            break;
        }
        perror("Get line failed, retrying...");
        std::this_thread::sleep_for(std::chrono::seconds(RETRY_INTERVAL_SECONDS));
        retries++;
    }
    if (!line_OS_WDT) {
        fprintf(stderr, "Failed to get line after %d retries\n", MAX_RETRIES);
        gpiod_chip_close(chip_OS_WDT);
        exit(1);
    }

    retries = 0;
    // Retry requesting the line as output
    while (retries < MAX_RETRIES) {
        if (gpiod_line_request_output(line_OS_WDT, "os_wdt_toggle", 0) == 0) {
            break;
        }
        perror("Request line as output failed, retrying...");
        std::this_thread::sleep_for(std::chrono::seconds(RETRY_INTERVAL_SECONDS));
        retries++;
    }
    if (retries == MAX_RETRIES) {
        fprintf(stderr, "Failed to request line as output after %d retries\n", MAX_RETRIES);
        gpiod_chip_close(chip_OS_WDT);
        exit(1);
    }

    bool current_OS_WDT_val = false;
    while (true) {
        current_OS_WDT_val = !current_OS_WDT_val;
        if (gpiod_line_set_value(line_OS_WDT, current_OS_WDT_val) < 0) {
            perror("Set line output failed");
            gpiod_chip_close(chip_OS_WDT);
            exit(1);
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    gpiod_line_release(line_OS_WDT);
    gpiod_chip_close(chip_OS_WDT);
}
