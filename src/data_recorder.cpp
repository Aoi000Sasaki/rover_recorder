#include "data_recorder.hpp"

DataRecorder::DataRecorder(double videoLength) {
    createSaveDir();
    this->videoLength = videoLength;
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

    if (this->isRecColor) {
        this->colorFile.open(this->crtDir + "/color.csv");
        if (!this->colorFile.is_open()) {
            std::cerr << "Failed to open file: " << this->crtDir + "/color.csv" << std::endl;
        } else {
            this->colorFile << "timestamp [ms]" << std::endl;
        }
    }
    if (this->isRecDepth) {
        this->depthFile.open(this->crtDir + "/depth.csv");
        if (!this->depthFile.is_open()) {
            std::cerr << "Failed to open file: " << this->crtDir + "/depth.csv" << std::endl;
        } else {
            this->depthFile << "timestamp [ms]" << std::endl;
        }
    }
    if (this->isRecIrRight) {
        this->irRightFile.open(this->crtDir + "/ir_right.csv");
        if (!this->irRightFile.is_open()) {
            std::cerr << "Failed to open file: " << this->crtDir + "/ir_right.csv" << std::endl;
        } else {
            this->irRightFile << "timestamp [ms]" << std::endl;
        }
    }
    if (this->isRecIrLeft) {
        this->irLeftFile.open(this->crtDir + "/ir_left.csv");
        if (!this->irLeftFile.is_open()) {
            std::cerr << "Failed to open file: " << this->crtDir + "/ir_left.csv" << std::endl;
        } else {
            this->irLeftFile << "timestamp [ms]" << std::endl;
        }
    }

    saveSensorData();
    this->pipe.start(this->config);
}

DataRecorder::~DataRecorder() {
}

void DataRecorder::startProcess() {
    auto start = std::chrono::high_resolution_clock::now();
    while (true) {
        process();

        auto now = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - start);
        if (duration.count() > this->videoLength) {
            for (auto &writer : this->videoWriterMap) {
                writer.second.release();
            }
            this->gyroFile.close();
            this->accelFile.close();
            this->pipe.stop();
            std::cout << "Record finished" << std::endl;

            break;
        }
    }
}

void DataRecorder::process() {
    auto frameset = this->pipe.waitForFrames(100);
    if(frameset == nullptr) {
        std::cout << "The frameset is null!" << std::endl;
        return;
    }

    if (this->frameCount < 5) {
        this->frameCount++;
        return;
    }

    if (this->isRecColor) {
        auto colorFrame = frameset->colorFrame();
        if (colorFrame != nullptr) {
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

            sensorData data = this->sensorDataMap[OB_SENSOR_COLOR];
            cv::Mat colorMat(data.height, data.width, CV_8UC3, colorFrame->data());
            this->videoWriterMap[OB_SENSOR_COLOR].write(colorMat);
            std::cout << "add frame to color video" << colorFrame->timeStamp() << std::endl;

            if (this->colorFile.is_open()) {
                this->colorFile << colorFrame->timeStamp() << std::endl;
            }
        }
    }
    if (this->isRecDepth) {
        auto depthFrame = frameset->depthFrame();
        if (depthFrame != nullptr) {
            sensorData data = this->sensorDataMap[OB_SENSOR_DEPTH];
            cv::Mat depthMat(data.height, data.width, CV_16UC1, depthFrame->data());
            cv::Mat depthMat8;
            double min, max;
            cv::minMaxIdx(depthMat, &min, &max);
            depthMat.convertTo(depthMat8, CV_8UC1, 255.0 / (max - min));
            this->videoWriterMap[OB_SENSOR_DEPTH].write(depthMat8);
            std::cout << "add frame to depth video" << depthFrame->timeStamp() << std::endl;

            if (this->depthFile.is_open()) {
                this->depthFile << depthFrame->timeStamp() << std::endl;
            }
        }
    }
    if (this->isRecIrRight) {
        auto irRightFrame = frameset->getFrame(OB_FRAME_IR_RIGHT);
        if (irRightFrame != nullptr) {
            sensorData data = this->sensorDataMap[OB_SENSOR_IR_RIGHT];
            cv::Mat irRightMat(data.height, data.width, CV_8UC1, irRightFrame->data());
            this->videoWriterMap[OB_SENSOR_IR_RIGHT].write(irRightMat);
            std::cout << "add frame to ir_right video" << irRightFrame->timeStamp() << std::endl;

            if (this->irRightFile.is_open()) {
                this->irRightFile << irRightFrame->timeStamp() << std::endl;
            }
        }
    }
    if (this->isRecIrLeft) {
        auto irLeftFrame = frameset->getFrame(OB_FRAME_IR_LEFT);
        if (irLeftFrame != nullptr) {
            sensorData data = this->sensorDataMap[OB_SENSOR_IR_LEFT];
            cv::Mat irLeftMat(data.height, data.width, CV_8UC1, irLeftFrame->data());
            this->videoWriterMap[OB_SENSOR_IR_LEFT].write(irLeftMat);
            std::cout << "add frame to ir_left video" << irLeftFrame->timeStamp() << std::endl;

            if (this->irLeftFile.is_open()) {
                this->irLeftFile << irLeftFrame->timeStamp() << std::endl;
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
                bool isColor = false;
                if (sensorType == OB_SENSOR_COLOR) {
                    this->colorProfile = profile;
                    isColor = true;
                }

                setCameraParams(videoProfile, sensorType);
                this->config->enableStream(videoProfile);

                std::string fileName = this->crtDir + "/";
                switch (sensorType) {
                    case OB_SENSOR_COLOR:
                        fileName += "color"; break;
                    case OB_SENSOR_DEPTH:
                        fileName += "depth"; break;
                    case OB_SENSOR_IR_RIGHT:
                        fileName += "ir_right"; break;
                    case OB_SENSOR_IR_LEFT:
                        fileName += "ir_left"; break;
                    default:
                        fileName += "sensorType_" + std::to_string(sensorType);
                        break;
                }
                double fps = videoProfile->fps();
                cv::Size size(videoProfile->width(), videoProfile->height());
                cv::VideoWriter writer(fileName + ".mp4", this->codec, fps, size, isColor);
                this->videoWriterMap[sensorType] = writer;
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
    };
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
}

void DataRecorder::saveSensorData() {
    std::string sensorDataPath = this->crtDir + "/sensor_data.json";
    std::ofstream ofs(sensorDataPath);
    if (!ofs) {
        std::cerr << "Failed to open file: " << sensorDataPath << std::endl;
        return;
    }
    nlohmann::json j;
    j["videoLength"] = this->videoLength;
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
    return j;
}
