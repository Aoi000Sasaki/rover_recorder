#ifndef STREAM_MANAGER_HPP
#define STREAM_MANAGER_HPP

#include <iostream>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include "libobsensor/ObSensor.hpp"
#include "opencv2/opencv.hpp"

class StreamManager {
    public:
        StreamManager(std::shared_ptr<ob::Pipeline> pipe,
                      std::shared_ptr<ob::Device> device,
                      std::shared_ptr<ob::Config> config,
                      OBSensorType sensorType,
                      const std::string& streamName,
                      const std::string& saveDir,
                      int profileIdx);
        virtual ~StreamManager();
        std::string getStreamName();
        int getSensorType();
        virtual nlohmann::json getMetadata();
        virtual void processFrameset(std::shared_ptr<ob::FrameSet> frameset);
        virtual void close();
    protected:
        bool isEnable = false;
        std::string errorMsg = "";
        int sensorType;
        std::string saveDir;
        std::string streamName;
        int profileIdx;
    private:
};

class ImageStreamManager : public StreamManager {
    public:
        ImageStreamManager(std::shared_ptr<ob::Pipeline> pipe,
                           std::shared_ptr<ob::Device> device,
                           std::shared_ptr<ob::Config> config,
                           OBSensorType sensorType,
                           const std::string& streamName,
                           const std::string& saveDir,
                           int profileIdx,
                           bool isSaveVideo,
                           bool isSaveImage);
        nlohmann::json getMetadata() override;
        void processFrameset(std::shared_ptr<ob::FrameSet> frameset) override;
        void close() override;
        void processColorFrame(std::shared_ptr<ob::FrameSet> frameset);
        void processDepthFrame(std::shared_ptr<ob::FrameSet> frameset);
        void processIrFrame(std::shared_ptr<ob::FrameSet> frameset);
        void setCameraParams(std::shared_ptr<ob::VideoStreamProfile> profile, bool isColor);
    private:
        bool isSaveVideo;
        bool isSaveImage;
        std::shared_ptr<ob::StreamProfile> colorProfile;
        ob::FormatConvertFilter filter;
        std::string containerFormat = ".avi";
        int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
        std::vector<int> compressionParams = {cv::IMWRITE_PNG_COMPRESSION,
                                              0, // not use ?
                                              cv::IMWRITE_PNG_STRATEGY,
                                              cv::IMWRITE_PNG_STRATEGY_DEFAULT};
        std::string videoName;
        std::string timecodeName;
        cv::VideoWriter videoWriter;
        std::ofstream timecodeWriter;
        int count = 0;

        float fps;
        int width;
        int height;
        float fx;
        float fy;
        float cx;
        float cy;
        float k1;
        float k2;
        float k3;
        float k4;
        float k5;
        float k6;
        float p1;
        float p2;
        std::vector<float> r;
        std::vector<float> t;
};

class ImuStreamManager : public StreamManager {
    public:
        ImuStreamManager(std::shared_ptr<ob::Pipeline> pipe,
                         std::shared_ptr<ob::Device> device,
                         std::shared_ptr<ob::Config> config,
                         OBSensorType sensorType,
                         const std::string& streamName,
                         const std::string& saveDir,
                         int profileIdx);
        nlohmann::json getMetadata() override;
        void processFrameset(std::shared_ptr<ob::FrameSet> frameset) override;
        void close() override;
        void imuCallback(std::shared_ptr<ob::Frame> frame);
    private:
        std::string imuName;
        std::ofstream imuWriter;
};

#endif
