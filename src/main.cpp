#include "data_recorder.hpp"
#include "gpio_manager.hpp"
#include "libobsensor/ObSensor.hpp"

Settings loadSettings(const std::string& settingsPath);

int main(int argc, char** argv) {
    Settings settings = loadSettings("/home/rock/camera_test/rover_recorder/settings.json");

    // continuous recording mode
    settings.videoLength = -1.0;

    DataRecorder dataRecorder(settings);
    GpioManager gpioManager;

    while (true) {
        if (gpioManager.get_GPIO_PDU_C()) {
            break;
        }
        std::cout << "Waiting for PDU_C signal..." << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    gpioManager.set_GPIO_camera(true);
    std::thread recorderThread(&DataRecorder::startProcess, &dataRecorder);

    while (true) {
        if (!gpioManager.get_GPIO_PDU_C()) {
            dataRecorder.stopProcess();
            std::cout << "Recording stopped." << std::endl;
            break;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    recorderThread.join();
    gpioManager.set_GPIO_camera(false);
    return 0;
}

Settings loadSettings(const std::string& settingsPath) {
    std::ifstream ifs(settingsPath);
    if (!ifs.is_open()) {
        Settings settings = {
            {OB_SENSOR_COLOR, OB_SENSOR_DEPTH, OB_SENSOR_IR_RIGHT, OB_SENSOR_IR_LEFT, OB_SENSOR_GYRO, OB_SENSOR_ACCEL},
            {"color", "depth", "ir_right", "ir_left", "gyro", "accel"},
            {true, true, true, true, true, true},
            {true, true, true, true, false, false},
            {OB_PROFILE_DEFAULT, OB_PROFILE_DEFAULT, OB_PROFILE_DEFAULT, OB_PROFILE_DEFAULT, OB_PROFILE_DEFAULT, OB_PROFILE_DEFAULT},
            {".mp4", ".mp4", ".mp4", ".mp4", "", ""},
            {cv::VideoWriter::fourcc('X', '2', '6', '4'), cv::VideoWriter::fourcc('X', '2', '6', '4'), cv::VideoWriter::fourcc('X', '2', '6', '4'), cv::VideoWriter::fourcc('X', '2', '6', '4'), 0, 0},
            {".jpg", ".jp2", ".jpg", ".jpg", "", ""},
            {{cv::IMWRITE_JPEG2000_COMPRESSION_X1000, 600}, {cv::IMWRITE_JPEG2000_COMPRESSION_X1000, 600}, {cv::IMWRITE_JPEG2000_COMPRESSION_X1000, 600}, {cv::IMWRITE_JPEG2000_COMPRESSION_X1000, 600}, {}, {}},
            3.0,
            "/home/rock/camera_test/rover_recorder",
        };
        return settings;
    } else {
        nlohmann::json j;
        ifs >> j;

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

        for (int i = 0; i < j["sensorTypes"].size(); i++) {
            OBSensorType sensorType = j["sensorTypes"][i];
            switch (sensorType) {
                case OB_SENSOR_COLOR:
                    sensorTypes.push_back(OB_SENSOR_COLOR);
                    streamNames.push_back("color");
                    break;
                case OB_SENSOR_DEPTH:
                    sensorTypes.push_back(OB_SENSOR_DEPTH);
                    streamNames.push_back("depth");
                    break;
                case OB_SENSOR_IR_RIGHT:
                    sensorTypes.push_back(OB_SENSOR_IR_RIGHT);
                    streamNames.push_back("ir_right");
                    break;
                case OB_SENSOR_IR_LEFT:
                    sensorTypes.push_back(OB_SENSOR_IR_LEFT);
                    streamNames.push_back("ir_left");
                    break;
                case OB_SENSOR_GYRO:
                    sensorTypes.push_back(OB_SENSOR_GYRO);
                    streamNames.push_back("gyro");
                    break;
                case OB_SENSOR_ACCEL:
                    sensorTypes.push_back(OB_SENSOR_ACCEL);
                    streamNames.push_back("accel");
                    break;
            }

            isSaveVideo.push_back(j["isSaveVideo"][i]);
            isSaveImage.push_back(j["isSaveImage"][i]);
            profileIdx.push_back(j["profileIdx"][i]);

            containerFormats.push_back(j["containerFormats"][i]);
            if (containerFormats[i] == ".mp4") {
                codecs.push_back(cv::VideoWriter::fourcc('X', '2', '6', '4'));
            } else if (containerFormats[i] == ".avi") {
                codecs.push_back(cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
            } else {
                codecs.push_back(0);
            }
            imageFormats.push_back(j["imageFormats"][i]);
            if (imageFormats[i] == ".jpg") {
                int quality = j["jpgQuality"];
                compressionParams.push_back({cv::IMWRITE_JPEG_QUALITY, quality});
            } else if (imageFormats[i] == ".jp2") {
                int quality = j["jp2Quality"];
                compressionParams.push_back({cv::IMWRITE_JPEG2000_COMPRESSION_X1000, quality});
            } else if (imageFormats[i] == ".png") {
                int quality = j["pngQuality"];
                compressionParams.push_back({cv::IMWRITE_PNG_COMPRESSION, quality});
            } else {
                compressionParams.push_back({});
            }
        }
        videoLength = j["videoLength"];
        saveDir = j["saveDir"];

        Settings settings = {
            sensorTypes,
            streamNames,
            isSaveVideo,
            isSaveImage,
            profileIdx,
            containerFormats,
            codecs,
            imageFormats,
            compressionParams,
            videoLength,
            saveDir,
        };

        return settings;
    }
};
