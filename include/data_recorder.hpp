#ifndef DATA_RECORDER_HPP
#define DATA_RECORDER_HPP

#include <iostream>
#include <filesystem>
#include <chrono>
#include <thread>
#include <sstream>
#include <fstream>
#include <filesystem>
#include <atomic>
#include <nlohmann/json.hpp>
#include "libobsensor/ObSensor.hpp"
#include "opencv2/opencv.hpp"
#include "stream_manager.hpp"

struct Settings {
    std::vector<OBSensorType> sensorTypes;
    std::vector<std::string> streamNames;
    std::vector<bool> isSaveVideo;
    std::vector<bool> isSaveImage;
    std::vector<int> profileIdx;
    std::vector<std::string> containerFormats;
    std::vector<int> codecs;
    std::vector<std::string> imageFormats;
    std::vector<std::vector<int>> compressionParams;
    float videoLength;
    std::string saveDir;
};

class DataRecorder {
    public:
        DataRecorder(Settings settings);
        void createSaveDir();
        void startProcess();
        void process();
        void stopProcess();
        void saveMetadata();

    private:
        ob::Context context;
        std::shared_ptr<ob::Pipeline> pipe;
        std::shared_ptr<ob::Config> config;
        std::shared_ptr<ob::Device> device;

        std::atomic<bool> stopFlag{false};
        bool isUseFlag = false;
        std::vector<std::shared_ptr<StreamManager>> streamManagers;
        float videoLength;
        std::string saveDir;
        std::string crtDir;
        int frameCount = 0;
};

#endif
