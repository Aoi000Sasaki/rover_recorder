#include "data_recorder.hpp"

DataRecorder::DataRecorder(int saveNum, int interval) {
    createSaveDir();
    this->saveNum = saveNum;
    this->interval = interval;
    this->config = std::make_shared<ob::Config>();
    ob::Context context;
    auto devList = context.queryDeviceList();
    if (devList->deviceCount() == 0) {
        std::cerr << "No device found!" << std::endl;
        exit(1);
    }
    this->device = devList->getDevice(0);

    this->isRecColor = startStream(OB_SENSOR_COLOR);
    this->isRecDepth = startStream(OB_SENSOR_DEPTH);
    this->isRecIrRight = startStream(OB_SENSOR_IR_RIGHT);
    this->isRecIrLeft = startStream(OB_SENSOR_IR_LEFT);
    this->isRecGyro = startStream(OB_SENSOR_GYRO);
    this->isRecAccel = startStream(OB_SENSOR_ACCEL);

    saveSensorData();
    this->pipe.start(this->config);
}

DataRecorder::~DataRecorder() {
}

void DataRecorder::startProcess() {
    while (true) {
        auto start = std::chrono::high_resolution_clock::now();
        process();

        if (!this->isRecColor && !this->isRecDepth && !this->isRecIrRight && !this->isRecIrLeft) {
            this->gyroFile.close();
            this->accelFile.close();
            this->pipe.stop();
            std::cout << "Record finished" << std::endl;
            break;
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        if (duration.count() < this->interval) {
            std::this_thread::sleep_for(std::chrono::milliseconds(this->interval - duration.count()));
        }
    }
}

void DataRecorder::process() {
    auto frameset = this->pipe.waitForFrames(100);
    if(frameset == nullptr) {
        std::cout << "The frameset is null!" << std::endl;
        return;
    }

    if(this->frameCount < 5) {
        this->frameCount++;
        return;
    }

    if (this->isRecColor) {
        auto colorFrame = frameset->colorFrame();
        if (colorFrame != nullptr && this->colorCount < this->saveNum) {
            if (colorFrame->format() != OB_FORMAT_RGB) {
                if (colorFrame->format() == OB_FORMAT_MJPEG) {
                    this->filter.setFormatConvertType(FORMAT_MJPEG_TO_RGB888);
                }
                else if (colorFrame->format() == OB_FORMAT_UYVY) {
                    this->filter.setFormatConvertType(FORMAT_UYVY_TO_RGB888);
                }
                else if (colorFrame->format() == OB_FORMAT_YUYV) {
                    this->filter.setFormatConvertType(FORMAT_YUYV_TO_RGB888);
                }
                else {
                    std::cerr << "Color format is not support!" << std::endl;
                    return;
                }
                colorFrame = this->filter.process(colorFrame)->as<ob::ColorFrame>();
            }
            this->filter.setFormatConvertType(FORMAT_RGB888_TO_BGR);
            colorFrame = this->filter.process(colorFrame)->as<ob::ColorFrame>();
            saveColor(colorFrame, this->colorCount);

            this->colorCount++;
            if (this->colorCount == this->saveNum) {
                this->isRecColor = false;
            }
        }
    }
    if (this->isRecDepth) {
        auto depthFrame = frameset->depthFrame();
        if (depthFrame != nullptr && this->depthCount < this->saveNum) {
            saveDepth(depthFrame, this->depthCount);

            this->depthCount++;
            if (this->depthCount == this->saveNum) {
                this->isRecDepth = false;
            }
        }
    }
    if (this->isRecIrRight) {
        auto irRightFrame = frameset->getFrame(OB_FRAME_IR_RIGHT);
        if (irRightFrame != nullptr && this->irRightCount < this->saveNum) {
            saveIrRight(irRightFrame, this->irRightCount);

            this->irRightCount++;
            if (this->irRightCount == this->saveNum) {
                this->isRecIrRight = false;
            }
        }
    }
    if (this->isRecIrLeft) {
        auto irLeftFrame = frameset->getFrame(OB_FRAME_IR_LEFT);
        if (irLeftFrame != nullptr && this->irLeftCount < this->saveNum) {
            saveIrLeft(irLeftFrame, this->irLeftCount);

            this->irLeftCount++;
            if (this->irLeftCount == this->saveNum) {
                this->isRecIrLeft = false;
            }
        }
    }
}

bool DataRecorder::startStream(OBSensorType sensorType) {
    try {
        auto sensor = this->device->getSensorList()->getSensor(sensorType);
        if (!sensor) {
            std::cerr << "SensorType: " << sensorType << " not found!" << std::endl;
            this->sensorDataMap[sensorType] = {
                .sensorType = sensorType,
                .errorMsg = "Sensor not found!"
            };
            return false;
        }

        if (sensorType == OB_SENSOR_COLOR ||
            sensorType == OB_SENSOR_DEPTH ||
            sensorType == OB_SENSOR_IR_RIGHT ||
            sensorType == OB_SENSOR_IR_LEFT) {
                auto profiles = this->pipe.getStreamProfileList(sensorType);
                auto profile = profiles->getProfile(OB_PROFILE_DEFAULT);
                auto videoProfile = profile->as<ob::VideoStreamProfile>();
                if (sensorType == OB_SENSOR_COLOR) {
                    this->colorProfile = profile;
                }

                setCameraParams(videoProfile, sensorType);
                this->config->enableStream(videoProfile);
        } else if (sensorType == OB_SENSOR_GYRO) {
            auto profile = sensor->getStreamProfileList()->getProfile(OB_PROFILE_DEFAULT);
            auto gyroCallback = [this](std::shared_ptr<ob::Frame> frame) {
                saveGyro(frame);
            };

            this->gyroFile.open(this->crtDir + "/gyro.csv");
            if (!this->gyroFile.is_open()) {
                std::cerr << "Failed to open file: " << this->crtDir + "/gyro.csv" << std::endl;
                this->sensorDataMap[sensorType] = {
                    .sensorType = sensorType,
                    .errorMsg = "Failed to open file: " + this->crtDir + "/gyro.csv"
                };
                return false;
            }
            this->gyroFile << "timestamp [ms],temperature [C],gyro.x [rad/s],gyro.y[rad/s],gyro.z [rad/s]" << std::endl;
            sensor->start(profile, gyroCallback);
        } else if (sensorType == OB_SENSOR_ACCEL) {
            auto profile = sensor->getStreamProfileList()->getProfile(OB_PROFILE_DEFAULT);
            auto accelCallback = [this](std::shared_ptr<ob::Frame> frame) {
                saveAccel(frame);
            };
            this->accelFile.open(this->crtDir + "/accel.csv");
            if (!this->accelFile.is_open()) {
                std::cerr << "Failed to open file: " << this->crtDir + "/accel.csv" << std::endl;
                this->sensorDataMap[sensorType] = {
                    .sensorType = sensorType,
                    .errorMsg = "Failed to open file: " + this->crtDir + "/accel.csv"
                };
                return false;
            }
            this->accelFile << "timestamp [ms],temperature [C],accel.x [m/s^2],accel.y[m/s^2],accel.z [m/s^2]" << std::endl;
            sensor->start(profile, accelCallback);
        } else {
            std::cerr << "SensorType: " << sensorType << " not support!" << std::endl;
            this->sensorDataMap[sensorType] = {
                .sensorType = sensorType,
                .errorMsg = "Sensor not support!"
            };
            return false;
        }

            return true;
    } catch(ob::Error &e) {
        std::cerr << "Failed to start sensorType: " << sensorType << std::endl;
        this->sensorDataMap[sensorType] = {
            .sensorType = sensorType,
            .errorMsg = e.getMessage()
        };
        return false;
    } catch (std::exception &e) {
        std::cerr << "Failed to start sensorType: " << sensorType << std::endl;
        this->sensorDataMap[sensorType] = {
            .sensorType = sensorType,
            .errorMsg = e.what()
        };
        return false;
    }
}

void DataRecorder::setCameraParams(std::shared_ptr<ob::VideoStreamProfile> videoProfile, int sensorType) {
    auto cameraIntrinsic = videoProfile->getIntrinsic();
    auto cameraDistortion = videoProfile->getDistortion();
    std::vector<double> r;
    std::vector<double> t;
    if (sensorType == OB_SENSOR_COLOR) {
        r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        t = {0, 0, 0};
    } else {
        try {
            auto cameraExtrinsic = videoProfile->getExtrinsicTo(this->colorProfile);
            for (int i = 0; i < 9; i++) {
                r.push_back(cameraExtrinsic.rot[i]);
            }
            for (int i = 0; i < 3; i++) {
                t.push_back(cameraExtrinsic.trans[i]);
            }
        } catch (ob::Error &e) {
            std::cerr << "Failed to get extrinsic: " << e.getMessage() << std::endl;
            r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
            t = {0, 0, 0};
        }
    }

    this->sensorDataMap[sensorType] = {
        .sensorType = sensorType,
        .fps = videoProfile->fps(),
        .width = videoProfile->width(),
        .height = videoProfile->height(),
        .fx = cameraIntrinsic.fx,
        .fy = cameraIntrinsic.fy,
        .cx = cameraIntrinsic.cx,
        .cy = cameraIntrinsic.cy,
        .k1 = cameraDistortion.k1,
        .k2 = cameraDistortion.k2,
        .k3 = cameraDistortion.k3,
        .k4 = cameraDistortion.k4,
        .k5 = cameraDistortion.k5,
        .k6 = cameraDistortion.k6,
        .p1 = cameraDistortion.p1,
        .p2 = cameraDistortion.p2,
        .r = r,
        .t = t,
        .format = videoProfile->format(),
        .saveNum = this->saveNum,
    };
}

void DataRecorder::saveColor(std::shared_ptr<ob::ColorFrame> colorFrame, int count) {
    sensorData data = this->sensorDataMap[OB_SENSOR_COLOR];
    std::string colorPath = this->crtDir + "/color/" + std::to_string(count) + "_" + std::to_string(colorFrame->timeStamp()) + "ms.png";
    cv::Mat colorMat(data.height, data.width, CV_8UC3, colorFrame->data());
    cv::imwrite(colorPath, colorMat, this->compressionParams);
    std::cout << "Save color image: " << colorPath << std::endl;
}

void DataRecorder::saveDepth(std::shared_ptr<ob::DepthFrame> depthFrame, int count) {
    sensorData data = this->sensorDataMap[OB_SENSOR_DEPTH];
    std::string depthPath = this->crtDir + "/depth/" + std::to_string(count) + "_" + std::to_string(depthFrame->timeStamp()) + "ms.png";
    cv::Mat depthMat(data.height, data.width, CV_16UC1, depthFrame->data());
    cv::imwrite(depthPath, depthMat, this->compressionParams);
    std::cout << "Save depth image: " << depthPath << std::endl;
}

void DataRecorder::saveIrRight(std::shared_ptr<ob::Frame> irFrame, int count) {
    sensorData data = this->sensorDataMap[OB_SENSOR_IR_RIGHT];
    std::string irRightPath = this->crtDir + "/ir_right/" + std::to_string(count) + "_" + std::to_string(irFrame->timeStamp()) + "ms.png";
    cv::Mat irRightMat(data.height, data.width, CV_8UC1, irFrame->data());
    cv::imwrite(irRightPath, irRightMat, this->compressionParams);
    std::cout << "Save IR_Right image: " << irRightPath << std::endl;
}

void DataRecorder::saveIrLeft(std::shared_ptr<ob::Frame> irFrame, int count) {
    sensorData data = this->sensorDataMap[OB_SENSOR_IR_LEFT];
    std::string irLeftPath = this->crtDir + "/ir_left/" + std::to_string(count) + "_" + std::to_string(irFrame->timeStamp()) + "ms.png";
    cv::Mat irLeftMat(data.height, data.width, CV_8UC1, irFrame->data());
    cv::imwrite(irLeftPath, irLeftMat, this->compressionParams);
    std::cout << "Save IR_Left image: " << irLeftPath << std::endl;
}

void DataRecorder::saveGyro(std::shared_ptr<ob::Frame> frame) {
    auto timestamp = frame->timeStamp();
    auto index = frame->index();
    auto gyroFrame = frame->as<ob::GyroFrame>();
    if (gyroFrame != nullptr) {
        auto value = gyroFrame->value();
        this->gyroFile << timestamp << "," << gyroFrame->temperature() << "," << value.x << "," << value.y << "," << value.z << std::endl;
    }
}

void DataRecorder::saveAccel(std::shared_ptr<ob::Frame> frame) {
    auto timestamp = frame->timeStamp();
    auto index = frame->index();
    auto accelFrame = frame->as<ob::AccelFrame>();
    if (accelFrame != nullptr) {
        auto value = accelFrame->value();
        this->accelFile << timestamp << "," << accelFrame->temperature() << "," << value.x << "," << value.y << "," << value.z << std::endl;
    }
}

void DataRecorder::createSaveDir() {
    namespace fs = std::filesystem;

    fs::path data_dir("data");
    if(!fs::exists(data_dir)) {
        if(fs::create_directory(data_dir)) {
            std::cout << "Directory created: " << data_dir << std::endl;
        } else {
            std::cerr << "Failed to create directory: " << data_dir << std::endl;
            exit(1);
        }
    } else {
        std::cout << "Directory already exists: " << data_dir << std::endl;
    }

    int dirNum = 0;
    try {
        for (auto &entry : fs::directory_iterator(data_dir)) {
            if (entry.is_directory()) {
                dirNum++;
            }
        }
    } catch (fs::filesystem_error &e) {
        dirNum = 0;
    } catch (std::exception &e) {
        dirNum = 0;
    }

    auto now = std::chrono::system_clock::now();
    auto now_time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_time), "%Y-%m-%d_%H-%M-%S");

    this->crtDir = "data/" + std::to_string(dirNum) + "_" + ss.str();
    fs::path pCrtDir(this->crtDir);
    if(!fs::exists(pCrtDir)) {
        if(fs::create_directory(pCrtDir)) {
            std::cout << "Directory created: " << pCrtDir << std::endl;
        } else {
            std::cerr << "Failed to create directory: " << pCrtDir << std::endl;
            exit(1);
        }
    } else {
        std::cout << "Directory already exists: " << pCrtDir << std::endl;
    }
    fs::path depthDir(this->crtDir + "/depth");
    if(!fs::exists(depthDir)) {
        if(fs::create_directory(depthDir)) {
            std::cout << "Directory created: " << depthDir << std::endl;
        } else {
            std::cerr << "Failed to create directory: " << depthDir << std::endl;
            exit(1);
        }
    } else {
        std::cout << "Directory already exists: " << depthDir << std::endl;
    }
    fs::path colorDir(this->crtDir + "/color");
    if(!fs::exists(colorDir)) {
        if(fs::create_directory(colorDir)) {
            std::cout << "Directory created: " << colorDir << std::endl;
        } else {
            std::cerr << "Failed to create directory: " << colorDir << std::endl;
            exit(1);
        }
    } else {
        std::cout << "Directory already exists: " << colorDir << std::endl;
    }
    fs::path irRightDir(this->crtDir + "/ir_right");
    if(!fs::exists(irRightDir)) {
        if(fs::create_directory(irRightDir)) {
            std::cout << "Directory created: " << irRightDir << std::endl;
        } else {
            std::cerr << "Failed to create directory: " << irRightDir << std::endl;
            exit(1);
        }
    } else {
        std::cout << "Directory already exists: " << irRightDir << std::endl;
    }
    fs::path irLeftDir(this->crtDir + "/ir_left");
    if(!fs::exists(irLeftDir)) {
        if(fs::create_directory(irLeftDir)) {
            std::cout << "Directory created: " << irLeftDir << std::endl;
        } else {
            std::cerr << "Failed to create directory: " << irLeftDir << std::endl;
            exit(1);
        }
    } else {
        std::cout << "Directory already exists: " << irLeftDir << std::endl;
    }
}

void DataRecorder::saveSensorData() {
    std::string sensorDataPath = this->crtDir + "/sensor_data.json";
    std::ofstream ofs(sensorDataPath);
    if (!ofs) {
        std::cerr << "Failed to open file: " << sensorDataPath << std::endl;
        return;
    }
    nlohmann::json j;
    j["saveNum"] = this->saveNum;
    j["interval"] = this->interval;
    j["currentDir"] = this->crtDir;
    for (auto &sensor : this->sensorDataMap) {
        if (sensor.first == OB_SENSOR_COLOR) {
            j["color"] = convertToJson(sensor.second);
        } else if (sensor.first == OB_SENSOR_DEPTH) {
            j["depth"] = convertToJson(sensor.second);
        } else if (sensor.first == OB_SENSOR_IR_RIGHT) {
            j["ir_right"] = convertToJson(sensor.second);
        } else if (sensor.first == OB_SENSOR_IR_LEFT) {
            j["ir_left"] = convertToJson(sensor.second);
        } else if (sensor.first == OB_SENSOR_GYRO) {
            j["gyro"] = convertToJson(sensor.second);
        } else if (sensor.first == OB_SENSOR_ACCEL) {
            j["accel"] = convertToJson(sensor.second);
        } else {
            j[sensor.first] = convertToJson(sensor.second);
        }
    }
    ofs << j.dump(4) << std::endl;
    ofs.close();
    std::cout << "Save sensor data: " << sensorDataPath << std::endl;
}

nlohmann::json DataRecorder::convertToJson(sensorData data) {
    nlohmann::json j;
    j["sensorType"] = data.sensorType;
    j["errorMsg"] = data.errorMsg;
    j["fps"] = data.fps;
    j["width"] = data.width;
    j["height"] = data.height;
    j["fx"] = data.fx;
    j["fy"] = data.fy;
    j["cx"] = data.cx;
    j["cy"] = data.cy;
    j["k1"] = data.k1;
    j["k2"] = data.k2;
    j["k3"] = data.k3;
    j["k4"] = data.k4;
    j["k5"] = data.k5;
    j["k6"] = data.k6;
    j["p1"] = data.p1;
    j["p2"] = data.p2;
    j["r"] = data.r;
    j["t"] = data.t;
    j["format"] = data.format;
    j["saveNum"] = data.saveNum;
    return j;
}
