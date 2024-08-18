#include "data_recorder.hpp"

DataRecorder::DataRecorder(Settings settings) {
    this->videoLength = settings.videoLength;
    this->saveDir = settings.saveDir + "/data/";
    this->recordCount = settings.recordCount;
    this->pipe = std::make_shared<ob::Pipeline>();
    this->config = std::make_shared<ob::Config>();
    createSaveDir();

    // if videoLength is negative, record until stopProcess() is called
    if (this->videoLength < 0) {
        this->isUseFlag = true;
    }

    // Get connected device list
    // Assert only one device is connected
    auto devList = this->context.queryDeviceList();
    if (devList->deviceCount() == 0) {
        std::cerr << "No device found!" << std::endl;
        exit(1);
    }
    this->device = devList->getDevice(0);

    // Enable all streams
    for (int i = 0; i < settings.sensorTypes.size(); i++) {
        OBSensorType st = settings.sensorTypes[i];
        if (st == OB_SENSOR_COLOR || st == OB_SENSOR_DEPTH || st == OB_SENSOR_IR_RIGHT || st == OB_SENSOR_IR_LEFT) {
            auto sm = std::make_shared<ImageStreamManager>(this->pipe, this->device, this->config, st, settings.streamNames[i], this->crtDir, settings.profileIdx[i], settings.isSaveVideo[i], settings.isSaveImage[i], settings.containerFormats[i], settings.codecs[i], settings.imageFormats[i], settings.compressionParams[i]);
            this->streamManagers.push_back(sm);
        } else if (st == OB_SENSOR_GYRO || st == OB_SENSOR_ACCEL) {
            auto sm = std::make_shared<ImuStreamManager>(this->pipe, this->device, this->config, st, settings.streamNames[i], this->crtDir, settings.profileIdx[i]);
            this->streamManagers.push_back(sm);
        } else {
            std::cerr << "Invalid sensor type: " << st << std::endl;
            continue;
        }
    }

    saveMetadata();
    this->pipe->start(this->config);
}

void DataRecorder::startProcess() {
    auto start = std::chrono::high_resolution_clock::now();
    int timeCount = 0;
    int loopCount = 0;
    while (true) {
        process();

        auto now = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
        loopCount++;
        if (timeCount != duration.count() / 100) {
            timeCount = duration.count() / 100;
            std::cout << "[INFO][Record #" << this->recordCount << "] " << "Elapsed time: " << duration.count() << " ms (avg frequency: " << loopCount / (duration.count() / 1000.0) << " Hz)" << std::endl;
        }

        // if isUseFlag is true and stopFlag is true, stop recording
        if (this->isUseFlag && this->stopFlag.load()) {
            for (auto &manager : this->streamManagers) {
                manager->close();
            }
            this->pipe->stop();
            std::cout << "Record finished" << std::endl;
            break;
        }

        // if isUseFlag is false and duration is longer than videoLength, stop recording
        if (!this->isUseFlag && duration.count() > this->videoLength * 1000) {
            for (auto &manager : this->streamManagers) {
                manager->close();
            }
            this->pipe->stop();
            std::cout << "Record finished" << std::endl;
            break;
        }
    }
}

inline void DataRecorder::process() {
    auto frameset = this->pipe->waitForFrames(100);
    if(frameset == nullptr) {
        std::cout << "The frameset is null!" << std::endl;
        return;
    }

    if (this->frameCount < 10) {
        this->frameCount++;
        return;
    }

    for (auto &manager : this->streamManagers)
    {
        manager->processFrameset(frameset);
    }
}

void DataRecorder::stopProcess() {
    this->stopFlag.store(true);
}

void DataRecorder::createSaveDir() {
    namespace fs = std::filesystem;

    fs::path data_dir(this->saveDir);
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

    this->crtDir = this->saveDir + std::to_string(dirNum) + "_" + ss.str();
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

void DataRecorder::saveMetadata() {
    std::string metadataPath = this->crtDir + "/metadata.json";
    std::ofstream ofs(metadataPath);
    if (!ofs) {
        std::cerr << "Failed to open file: " << metadataPath << std::endl;
        return;
    }
    nlohmann::json j;
    j["videoLength"] = this->videoLength;
    j["currentDir"] = this->crtDir;
    for (auto &manager : this->streamManagers) {
        j[manager->getStreamName()] = manager->getMetadata();
    }
    ofs << j.dump(4) << std::endl;
    ofs.close();
    std::cout << "Save metadata: " << metadataPath << std::endl;
}
