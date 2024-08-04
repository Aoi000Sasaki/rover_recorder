#ifndef ROVER_RECORDER_HPP
#define ROVER_RECORDER_HPP

#include <iostream>
#include <filesystem>
#include <chrono>
#include <thread>
#include <sstream>
#include <fstream>
#include <nlohmann/json.hpp>
#include "libobsensor/ObSensor.hpp"
#include "opencv2/opencv.hpp"

struct sensorData{
    int sensorType = -1;
    std::string errorMsg = "";
    int fps = 0;
    int width = 0;
    int height = 0;
    float fx = 0;
    float fy = 0;
    float cx = 0;
    float cy = 0;
    float k1 = 0;
    float k2 = 0;
    float k3 = 0;
    float k4 = 0;
    float k5 = 0;
    float k6 = 0;
    float p1 = 0;
    float p2 = 0;
    std::vector<double> r;
    std::vector<double> t;
    int format = 0; // see: ../api/English/ObTypes_8h.html#a30904eab1a667b797e7ce1099ba7c36a
};

class DataRecorder {
    public:
        DataRecorder(double videoLength);
        ~DataRecorder();
        void createSaveDir();
        bool startStream(OBSensorType sensorType);
        void setCameraParams(std::shared_ptr<ob::VideoStreamProfile> profile, int sensorType);

        void startProcess();
        void process();
        void saveDepth(std::shared_ptr<ob::DepthFrame> depthFrame, int count);
        void saveColor(std::shared_ptr<ob::ColorFrame> colorFrame, int count);
        void saveIrRight(std::shared_ptr<ob::Frame> irFrame, int count);
        void saveIrLeft(std::shared_ptr<ob::Frame> irFrame, int count);
        void saveGyro(std::shared_ptr<ob::Frame> frame);
        void saveAccel(std::shared_ptr<ob::Frame> frame);
        void saveSensorData();
        nlohmann::json convertToJson(sensorData data);

    private:
        ob::Pipeline pipe;
        ob::FormatConvertFilter filter;
        std::shared_ptr<ob::Config> config;
        std::shared_ptr<ob::Device> device;
        std::map<int, sensorData> sensorDataMap;
        std::shared_ptr<ob::StreamProfile> colorProfile;
        std::vector<int> compressionParams = {cv::IMWRITE_PNG_COMPRESSION,0, // not use ?
                                              cv::IMWRITE_PNG_STRATEGY,
                                              cv::IMWRITE_PNG_STRATEGY_DEFAULT};
        std::ofstream colorFile;
        std::ofstream depthFile;
        std::ofstream irRightFile;
        std::ofstream irLeftFile;
        std::ofstream gyroFile;
        std::ofstream accelFile;

        double videoLength;
        int codec = cv::VideoWriter::fourcc('X','2','6','4');
        std::map<int, cv::VideoWriter> videoWriterMap;

        std::string crtDir;
        bool isRecColor = true;
        bool isRecDepth = true;
        bool isRecIrRight = true;
        bool isRecIrLeft = true;
        bool isRecGyro = true;
        bool isRecAccel = true;
        int frameCount = 0;
};

#endif
