#include "data_recorder.hpp"

int main(int argc, char** argv) {
    int saveNum = 10;
    int interval = 50;
    if (argc == 3) {
        try {
            saveNum = std::stoi(argv[1]);
            interval = std::stoi(argv[2]);
        } catch (std::invalid_argument &e) {
            std::cout << "saveNum set to default value: " << saveNum << std::endl;
            std::cout << "interval set to default value: " << interval << std::endl;
        }
    } else if (argc == 2) {
        try {
            saveNum = std::stoi(argv[1]);
            std::cout << "interval set to default value: " << interval << std::endl;
        } catch (std::invalid_argument &e) {
            std::cout << "saveNum set to default value: " << saveNum << std::endl;
            std::cout << "interval set to default value: " << interval << std::endl;
        }
    } else {
        std::cout << "saveNum set to default value: " << saveNum << std::endl;
        std::cout << "interval set to default value: " << interval << std::endl;
    }

    DataRecorder dataRecorder(saveNum, interval);
    dataRecorder.startProcess();
    return 0;
}
