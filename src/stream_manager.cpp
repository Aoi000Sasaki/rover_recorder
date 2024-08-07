#include "stream_manager.hpp"

StreamManager::StreamManager(std::shared_ptr<ob::Pipeline> pipe,
                             std::shared_ptr<ob::Device> device,
                             std::shared_ptr<ob::Config> config,
                             OBSensorType sensorType,
                             const std::string& streamName,
                             const std::string& saveDir,
                             int profileIdx) {
    try {
        // Check if sensor type is available
        auto sensor = device->getSensorList()->getSensor(sensorType);
        if (!sensor) {
            std::cerr << "Sensor type: " << sensorType << " not found" << std::endl;
            this->errorMsg += "Sensor type: " + std::to_string(sensorType) + " not found";
            this->isEnable = false;
            return;
        }
    } catch (ob::Error &e) {
        std::cerr << "Error: " << e.getMessage() << std::endl;
        this->errorMsg += e.getMessage();
        this->isEnable = false;
        return;
    } catch (std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        this->errorMsg += e.what();
        this->isEnable = false;
        return;
    }

    this->sensorType = sensorType;
    this->streamName = streamName;
    this->saveDir = saveDir;
    this->profileIdx = profileIdx;
    this->isEnable = true;
}

StreamManager::~StreamManager() {
    return;
}

std::string StreamManager::getStreamName() {
    return this->streamName;
}

int StreamManager::getSensorType() {
    return this->sensorType;
}

nlohmann::json StreamManager::getMetadata() {
    nlohmann::json metadata;
    metadata["sensorType"] = this->sensorType;
    metadata["profileIdx"] = this->profileIdx;

    if (!this->isEnable) {
        metadata["isEnable"] = false;
        metadata["errorMsg"] = this->errorMsg;
        return metadata;
    }
    else {
        metadata["isEnable"] = true;
        return metadata;
    }
}

inline void StreamManager::close() {
    return;
}

inline void StreamManager::processFrameset(std::shared_ptr<ob::FrameSet> frameset) {
    return;
}

ImageStreamManager::ImageStreamManager(std::shared_ptr<ob::Pipeline> pipe,
                                       std::shared_ptr<ob::Device> device,
                                       std::shared_ptr<ob::Config> config,
                                       OBSensorType sensorType,
                                       const std::string& streamName,
                                       const std::string& saveDir,
                                       int profileIdx,
                                       bool isSaveVideo,
                                       bool isSaveImage) :
    StreamManager(pipe, device, config, sensorType, streamName, saveDir, profileIdx) {
    if (!this->isEnable) {
        return;
    }
    this->isSaveVideo = isSaveVideo;
    this->isSaveImage = isSaveImage;
    // Check if sensor type is valid
    if (sensorType != OB_SENSOR_COLOR && sensorType != OB_SENSOR_DEPTH && sensorType != OB_SENSOR_IR_LEFT && sensorType != OB_SENSOR_IR_RIGHT) {
        std::cerr << "Invalid sensor type for ImageStreamManager" << std::endl;
        this->errorMsg += "Invalid sensor type for ImageStreamManager";
        this->isEnable = false;
        return;
    }

    try {
        // Enable stream
        auto profiles = pipe->getStreamProfileList(sensorType);
        auto profile = profiles->getProfile(profileIdx);
        auto videoProfile = profile->as<ob::VideoStreamProfile>();
        bool isColor = false;
        if (sensorType == OB_SENSOR_COLOR) {
            isColor = true;
        }
        config->enableStream(videoProfile);
        this->isEnable = true;

        // Store color profile
        try {
            colorProfile = pipe->getStreamProfileList(OB_SENSOR_COLOR)
                ->getProfile(OB_PROFILE_DEFAULT)
                ->as<ob::VideoStreamProfile>();
            this->colorProfile = colorProfile;
        } catch (ob::Error &e) {
            std::cerr << "Color profile not found" << std::endl;
            this->errorMsg += "Color profile not found";
        }

        // Set camera parameters
        setCameraParams(videoProfile, isColor);

        // Open timecode writer
        this->timecodeName = saveDir + "/" + streamName + "_timecode.txt";
        this->timecodeWriter.open(this->timecodeName);
        if (this->timecodeWriter.is_open()) {
            if (sensorType == OB_SENSOR_DEPTH) {
                this->timecodeWriter << "timestamp [ms],value scale" << std::endl;
            } else {
                this->timecodeWriter << "timestamp [ms]" << std::endl;
            }
        }

        // Open video writer
        if (this->isSaveVideo) {
            this->videoName = saveDir + "/" + streamName + this->containerFormat;
            float fps = videoProfile->fps();
            cv::Size frameSize(videoProfile->width(), videoProfile->height());
            this->videoWriter.open(this->videoName, this->codec, fps, frameSize, isColor);
        }

        // Create image directory
        if (this->isSaveImage) {
            namespace fs = std::filesystem;
            fs::path imageDir(saveDir + "/" + streamName);
            if (!fs::exists(imageDir)) {
                if (fs::create_directory(imageDir)) {
                    std::cout << "Directory created: " << imageDir << std::endl;
                } else {
                    std::cerr << "Failed to create directory: " << imageDir << std::endl;
                    this->errorMsg += "Failed to create directory: " + imageDir.string();
                    return;
                }
            } else {
                std::cout << "Directory already exists: " << imageDir << std::endl;
            }
        }
    } catch (ob::Error &e) {
        std::cerr << "Error: " << e.getMessage() << std::endl;
        this->errorMsg += e.getMessage();
        this->isEnable = false;
        return;
    } catch (std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        this->errorMsg += e.what();
        return;
    }
}

inline void ImageStreamManager::processFrameset(std::shared_ptr<ob::FrameSet> frameset) {
    if (!this->isEnable) {
        return;
    }

    switch (this->sensorType) {
        case OB_SENSOR_COLOR:
            processColorFrame(frameset);
            break;
        case OB_SENSOR_DEPTH:
            processDepthFrame(frameset);
            break;
        case OB_SENSOR_IR_LEFT:
            processIrFrame(frameset);
            break;
        case OB_SENSOR_IR_RIGHT:
            processIrFrame(frameset);
            break;
    }
}

inline void ImageStreamManager::processColorFrame(std::shared_ptr<ob::FrameSet> frameset) {
    auto colorFrame = frameset->colorFrame();
    if (colorFrame == nullptr) {
        return;
    }

    // Convert color frame to BGR format
    if (colorFrame->format() != OB_FORMAT_RGB) {
        if (colorFrame->format() == OB_FORMAT_MJPEG) {
            this->filter.setFormatConvertType(FORMAT_MJPEG_TO_RGB888);
        } else if (colorFrame->format() == OB_FORMAT_UYVY) {
            this->filter.setFormatConvertType(FORMAT_UYVY_TO_RGB888);
        } else if (colorFrame->format() == OB_FORMAT_YUYV) {
            this->filter.setFormatConvertType(FORMAT_YUYV_TO_RGB888);
        } else {
            std::cerr << "Color format is not supported!" << std::endl;
            return;
        }
        colorFrame = this->filter.process(colorFrame)->as<ob::ColorFrame>();
    }
    this->filter.setFormatConvertType(FORMAT_RGB888_TO_BGR);
    colorFrame = this->filter.process(colorFrame)->as<ob::ColorFrame>();
    cv::Mat colorMat(this->height, this->width, CV_8UC3, colorFrame->data());

    this->timecodeWriter << colorFrame->timeStamp() << std::endl;

    if (this->isSaveVideo && this->videoWriter.isOpened()) {
        this->videoWriter.write(colorMat);
        std::cout << "add frame to color video" << colorFrame->timeStamp() << std::endl;
    }

    if (this->isSaveImage) {
        std::string imageName = this->saveDir + "/" + this->streamName + "/" + std::to_string(this->count) + "_" + std::to_string(colorFrame->timeStamp()) + "ms" + this->imageFormat;
        cv::imwrite(imageName, colorMat, this->compressionParams);
        std::cout << "save color image: " << imageName << std::endl;
    }

    this->count++;
}

inline void ImageStreamManager::processDepthFrame(std::shared_ptr<ob::FrameSet> frameset) {
    auto depthFrame = frameset->depthFrame();
    if (depthFrame == nullptr) {
        return;
    }

    float valueScale = depthFrame->getValueScale();
    cv::Mat depthMat(this->height, this->width, CV_16UC1, depthFrame->data());
    cv::Mat depthMat8;
    double min, max;
    cv::minMaxLoc(depthMat, &min, &max);
    depthMat.convertTo(depthMat8, CV_8UC1, 255.0 / (max - min));

    this->timecodeWriter << depthFrame->timeStamp() << "," << valueScale << std::endl;

    if (this->isSaveVideo && this->videoWriter.isOpened()) {
        this->videoWriter.write(depthMat8);
        std::cout << "add frame to depth video" << depthFrame->timeStamp() << std::endl;
    }

    if (this->isSaveImage) {
        std::string imageName = this->saveDir + "/" + this->streamName + "/" + std::to_string(this->count) + "_" + std::to_string(depthFrame->timeStamp()) + "ms" + this->imageFormat;
        cv::imwrite(imageName, depthMat, this->compressionParams);
        // cv::imwrite(imageName, depthMat8, this->compressionParams);
        std::cout << "save depth image: " << imageName << std::endl;
    }

    this->count++;
}

inline void ImageStreamManager::processIrFrame(std::shared_ptr<ob::FrameSet> frameset) {
    std::shared_ptr<ob::Frame> irFrame;
    if (this->sensorType == OB_SENSOR_IR_LEFT) {
        irFrame = frameset->getFrame(OB_FRAME_IR_LEFT);
    } else if (this->sensorType == OB_SENSOR_IR_RIGHT) {
        irFrame = frameset->getFrame(OB_FRAME_IR_RIGHT);
    }

    cv::Mat irMat(this->height, this->width, CV_8UC1, irFrame->data());

    this->timecodeWriter << irFrame->timeStamp() << std::endl;
    
    if (this->isSaveVideo && this->videoWriter.isOpened()) {
        this->videoWriter.write(irMat);
        std::cout << "add frame to ir video" << irFrame->timeStamp() << std::endl;
    }

    if (this->isSaveImage) {
        std::string imageName = this->saveDir + "/" + this->streamName + "/" + std::to_string(this->count) + "_" + std::to_string(irFrame->timeStamp()) + "ms" + this->imageFormat;
        cv::imwrite(imageName, irMat, this->compressionParams);
        std::cout << "save ir image: " << imageName << std::endl;
    }

    this->count++;
}

void ImageStreamManager::close() {
    if (this->videoWriter.isOpened()) {
        this->videoWriter.release();
    }
    if (this->timecodeWriter.is_open()) {
        this->timecodeWriter.close();
    }
}

void ImageStreamManager::setCameraParams(std::shared_ptr<ob::VideoStreamProfile> profile, bool isColor) {
    auto intrinsics = profile->getIntrinsic();
    auto distortion = profile->getDistortion();
    std::vector<float> r;
    std::vector<float> t;
    if (isColor) {
        r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        t = {0, 0, 0};
    } else {
        try {
            auto extrinsics = profile->getExtrinsicTo(this->colorProfile);
            for (int i = 0; i < 9; i++) {
                r.push_back(extrinsics.rot[i]);
            }
            for (int i = 0; i < 3; i++) {
                t.push_back(extrinsics.trans[i]);
            }
        } catch (ob::Error &e) {
            std::cerr << "Error: " << e.getMessage() << std::endl;
            r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
            t = {0, 0, 0};
        }
    }

    this->fps = profile->fps();
    this->width = profile->width();
    this->height = profile->height();
    this->fx = intrinsics.fx;
    this->fy = intrinsics.fy;
    this->cx = intrinsics.cx;
    this->cy = intrinsics.cy;
    this->k1 = distortion.k1;
    this->k2 = distortion.k2;
    this->k3 = distortion.k3;
    this->k4 = distortion.k4;
    this->k5 = distortion.k5;
    this->k6 = distortion.k6;
    this->p1 = distortion.p1;
    this->p2 = distortion.p2;
    this->r = r;
    this->t = t;
}

nlohmann::json ImageStreamManager::getMetadata() {
    nlohmann::json metadata;
    metadata["streamName"] = this->streamName;
    metadata["sensorType"] = this->sensorType;
    metadata["profileIdx"] = this->profileIdx;
    metadata["isSaveVideo"] = this->isSaveVideo;
    metadata["isSaveImage"] = this->isSaveImage;
    metadata["containerFormat"] = this->containerFormat;
    metadata["codec"] = this->codec;
    metadata["imageFormat"] = this->imageFormat;
    metadata["compressionParams"] = this->compressionParams;

    if (!this->isEnable) {
        metadata["isEnable"] = false;
        metadata["errorMsg"] = this->errorMsg;
        return metadata;
    }
    else {
        metadata["isEnable"] = true;
        metadata["videoName"] = this->videoName;
        metadata["timecodeName"] = this->timecodeName;
        metadata["fps"] = this->fps;
        metadata["width"] = this->width;
        metadata["height"] = this->height;
        metadata["fx"] = this->fx;
        metadata["fy"] = this->fy;
        metadata["cx"] = this->cx;
        metadata["cy"] = this->cy;
        metadata["k1"] = this->k1;
        metadata["k2"] = this->k2;
        metadata["k3"] = this->k3;
        metadata["k4"] = this->k4;
        metadata["k5"] = this->k5;
        metadata["k6"] = this->k6;
        metadata["p1"] = this->p1;
        metadata["p2"] = this->p2;
        metadata["r"] = this->r;
        metadata["t"] = this->t;
        return metadata;
    }
}

ImuStreamManager::ImuStreamManager(std::shared_ptr<ob::Pipeline> pipe,
                                   std::shared_ptr<ob::Device> device,
                                   std::shared_ptr<ob::Config> config,
                                   OBSensorType sensorType,
                                   const std::string& streamName,
                                   const std::string& saveDir,
                                   int profileIdx) :
    StreamManager(pipe, device, config, sensorType, streamName, saveDir, profileIdx) {
    if (!this->isEnable) {
        return;
    }
    // Check if sensor type is valid
    if (sensorType != OB_SENSOR_ACCEL && sensorType != OB_SENSOR_GYRO) {
        std::cerr << "Invalid sensor type for ImuStreamManager" << std::endl;
        this->errorMsg += "Invalid sensor type for ImuStreamManager";
        this->isEnable = false;
        return;
    }

    try {
        // Open imu writer
        this->imuName = saveDir + "/" + streamName + ".csv";
        this->imuWriter.open(this->imuName);
        if (this->imuWriter.is_open()) {
            if (sensorType == OB_SENSOR_GYRO) {
                this->imuWriter << "timestamp [ms],temperature [C],gyro.x [rad/s],gyro.y [rad/s],gyro.z [rad/s]" << std::endl;
            } else if (sensorType == OB_SENSOR_ACCEL) {
                this->imuWriter << "timestamp [ms],temperature [C],accel.x [m/s^2],accel.y [m/s^2],accel.z [m/s^2]" << std::endl;
            }
        }

        // Set callback
        auto sensor = device->getSensorList()->getSensor(sensorType);
        auto profile = sensor->getStreamProfileList()->getProfile(profileIdx);
        auto callback = [this](std::shared_ptr<ob::Frame> frame) {
            imuCallback(frame);
        };
        sensor->start(profile, callback);
        this->isEnable = true;
    } catch (ob::Error &e) {
        std::cerr << "Error: " << e.getMessage() << std::endl;
        this->errorMsg += e.getMessage();
        this->isEnable = false;
        return;
    } catch (std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        this->errorMsg += e.what();
        return;
    }
}

inline void ImuStreamManager::processFrameset(std::shared_ptr<ob::FrameSet> frameset) {
    return;
}

void ImuStreamManager::close() {
    if (this->imuWriter.is_open()) {
        this->imuWriter.close();
    }
}

inline void ImuStreamManager::imuCallback(std::shared_ptr<ob::Frame> frame) {
    if (!this->isEnable) {
        return;
    }
    if (!this->imuWriter.is_open()) {
        return;
    }

    if (this->sensorType == OB_SENSOR_GYRO) {
        auto gyroFrame = frame->as<ob::GyroFrame>();
        if (gyroFrame != nullptr) {
            auto value = gyroFrame->value();
            this->imuWriter << frame->timeStamp() << "," << gyroFrame->temperature() << "," << value.x << "," << value.y << "," << value.z << std::endl;
        }
    } else if (this->sensorType == OB_SENSOR_ACCEL) {
        auto accelFrame = frame->as<ob::AccelFrame>();
        if (accelFrame != nullptr) {
            auto value = accelFrame->value();
            this->imuWriter << frame->timeStamp() << "," << accelFrame->temperature() << "," << value.x << "," << value.y << "," << value.z << std::endl;
        }
    }
}

nlohmann::json ImuStreamManager::getMetadata() {
    nlohmann::json metadata;
    metadata["sensorType"] = this->sensorType;
    metadata["profileIdx"] = this->profileIdx;
    metadata["streamName"] = this->streamName;

    if (!this->isEnable) {
        metadata["isEnable"] = false;
        metadata["errorMsg"] = this->errorMsg;
        return metadata;
    }
    else {
        metadata["isEnable"] = true;
        metadata["imuName"] = this->imuName;
        return metadata;
    }
}
