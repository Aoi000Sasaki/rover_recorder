#include "data_recorder.hpp"
#include "libobsensor/ObSensor.hpp"

// if gyro or accel, isSaveVideo and isSaveImage are ignored
std::vector<StreamInfo> openStreams{
    {OB_SENSOR_COLOR,    "color",    true, true, OB_PROFILE_DEFAULT},
    {OB_SENSOR_DEPTH,    "depth",    true, true,  OB_PROFILE_DEFAULT},
    {OB_SENSOR_IR_RIGHT, "ir_right", true, true, OB_PROFILE_DEFAULT},
    {OB_SENSOR_IR_LEFT,  "ir_left",  true, true, OB_PROFILE_DEFAULT},
    {OB_SENSOR_GYRO,     "gyro",     true, false, OB_PROFILE_DEFAULT},
    {OB_SENSOR_ACCEL,    "accel",    true, false, OB_PROFILE_DEFAULT}};
// seconds
float videoLength = 3.0;
// create a directory named "data" in this directory
std::string saveDir = "/home/amsl/orbbec-ws/src/rover_recorder/";

int main(int argc, char** argv) {
    if (argc == 2) {
        try {
            videoLength = std::stod(argv[1]);
        } catch (std::invalid_argument &e) {
            std::cout << "videoLength set to default value: " << videoLength << std::endl;
        }
    } else {
        std::cout << "videoLength set to default value: " << videoLength << std::endl;
    }

    DataRecorder dataRecorder(openStreams, videoLength, saveDir);
    dataRecorder.startProcess();
    return 0;
}
