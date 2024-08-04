#include "data_recorder.hpp"

int main(int argc, char** argv) {
    double videoLength = 3; // seconds
    if (argc == 2) {
        try {
            videoLength = std::stod(argv[1]);
        } catch (std::invalid_argument &e) {
            std::cout << "videoLength set to default value: " << videoLength << std::endl;
        }
    } else {
        std::cout << "videoLength set to default value: " << videoLength << std::endl;
    }

    DataRecorder dataRecorder(videoLength);
    dataRecorder.startProcess();
    return 0;
}
