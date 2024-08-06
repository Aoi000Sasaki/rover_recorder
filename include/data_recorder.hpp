#ifndef DATA_RECORDER_HPP
#define DATA_RECORDER_HPP

#include <iostream>
#include <filesystem>
#include <chrono>
#include <thread>
#include <sstream>
#include <fstream>
#include <filesystem>
#include <nlohmann/json.hpp>
#include "libobsensor/ObSensor.hpp"
#include "opencv2/opencv.hpp"
#include "stream_manager.hpp"

struct StreamInfo {
    OBSensorType sensorType;
    std::string streamName;
    bool isSaveVideo;
    bool isSaveImage;
    int profileIdx;
};

class DataRecorder {
    public:
        DataRecorder(std::vector<StreamInfo> info, float videoLength, const std::string& saveDir);
        void createSaveDir();
        void startProcess();
        void process();
        void saveMetadata();

    private:
        ob::Context context;
        std::shared_ptr<ob::Pipeline> pipe;
        std::shared_ptr<ob::Config> config;
        std::shared_ptr<ob::Device> device;

        std::vector<std::shared_ptr<StreamManager>> streamManagers;
        float videoLength;
        std::string saveDir;
        std::string crtDir;
        int frameCount = 0;
};

#endif
