//
// Created by an on 2022/8/18.
//
#include "DSOSlamSystem.h"

DSOSlamSystem::DSOSlamSystem() : imuInt_(frameContainer_) {
}

int DSOSlamSystem::Init(int argc, char **argv) {
    setlocale(LC_ALL, "C");
    auto settingsUtil = std::make_shared<dmvio::SettingsUtil>();
    // Create Settings files.
    imuSettings.registerArgs(*settingsUtil);
    imuCalibration.registerArgs(*settingsUtil);
    mainSettings.registerArgs(*settingsUtil);
    frameSkippingSettings.registerArgs(*settingsUtil);
    settingsUtil->registerArg("start", start);
    auto normalizeCamSize = std::make_shared<double>(0.0);
    settingsUtil->registerArg("normalizeCamSize", *normalizeCamSize, 0.0, 5.0);
    // This call will parse all commandline arguments and potentially also read a settings yaml file if passed.
    mainSettings.parseArguments(argc, argv, *settingsUtil);

    if (mainSettings.calib == "")
        return -1;

    undistorter_ = Undistort::getUndistorterForFile(mainSettings.calib, mainSettings.gammaCalib,
                                                    mainSettings.vignette);
    setGlobalCalib(
            (int) undistorter_->getSize()[0],
            (int) undistorter_->getSize()[1],
            undistorter_->getK().cast<float>());

    if (mainSettings.imuCalibFile == "")
        return -1;
    imuCalibration.loadFromFile(mainSettings.imuCalibFile);
    fullSystem_ = new FullSystem(linearizeOperation_, imuCalibration, imuSettings);
    outputWrapper_ = new IOWrap::AndroidOutput3DWrapper(wG[0], hG[0], false);
    fullSystem_->outputWrapper.push_back(outputWrapper_);
    if (setting_photometricCalibration > 0 && undistorter_->photometricUndist == nullptr) {
        printf("ERROR: dont't have photometric calibation. Need to use commandline options mode=1 or mode=2 ");
        exit(1);
    }
    if (undistorter_->photometricUndist != nullptr) {
        fullSystem_->setGammaFunction(undistorter_->photometricUndist->getG());
    }
    return 0;
}

void DSOSlamSystem::process() {
    dmvio::FrameSkippingStrategy frameSkipping(frameSkippingSettings);
    // frameSkipping registers as an outputWrapper to get notified of changes of the system status.
    fullSystem_->outputWrapper.push_back(&frameSkipping);
    int ii = 0;
    int lastResetIndex = 0;
    while (true) {
        // Skip the first few frames if the start variable is set.
        if (start > 0 && ii < start) {
            auto pair = frameContainer_.getImageAndIMUData();
            ++ii;
            continue;
        }
        auto pair = frameContainer_.getImageAndIMUData(frameSkipping.getMaxSkipFrames(frameContainer_.getQueueSize()));
        fullSystem_->addActiveFrame(pair.first.get(), ii, &(pair.second), nullptr);
        if (fullSystem_->initFailed || setting_fullResetRequested) {
            if (ii - lastResetIndex < 250 || setting_fullResetRequested) {
                printf("RESETTING!\n");
                std::vector<IOWrap::Output3DWrapper *> wraps = fullSystem_->outputWrapper;
                delete fullSystem_;
                for (IOWrap::Output3DWrapper *ow: wraps) ow->reset();

                fullSystem_ = new FullSystem(linearizeOperation_, imuCalibration, imuSettings);
                if (undistorter_->photometricUndist != nullptr) {
                    fullSystem_->setGammaFunction(undistorter_->photometricUndist->getG());
                }
                fullSystem_->outputWrapper = wraps;

                setting_fullResetRequested = false;
                lastResetIndex = ii;
            }
        }
        if (fullSystem_->isLost) {
            printf("LOST!!\n");
            break;
        }
        ++ii;
    }
    fullSystem_->blockUntilMappingIsFinished();
    for (IOWrap::Output3DWrapper *ow: fullSystem_->outputWrapper) {
        ow->join();
    }
}


DSOSlamSystem::~DSOSlamSystem() {
    if (fullSystem_) {
        delete fullSystem_;
        fullSystem_ = NULL;
    }
    if (outputWrapper_) {
        delete outputWrapper_;
        outputWrapper_ = NULL;
    }
    if (undistorter_) {
        delete undistorter_;
        undistorter_ = NULL;
    }
}


float DSOSlamSystem::fx() {
    return fullSystem_->getCalibHessian().fxl();
}

float DSOSlamSystem::fy() {
    return fullSystem_->getCalibHessian().fyl();
}

float DSOSlamSystem::cx() {
    return fullSystem_->getCalibHessian().cxl();
}

float DSOSlamSystem::cy() {
    return fullSystem_->getCalibHessian().cyl();
}

int DSOSlamSystem::width() {
    return wG[0];
}

int DSOSlamSystem::height() {
    return hG[0];
}

SE3 DSOSlamSystem::currentCameraPose() {
    return outputWrapper_->currentCamPose();
}

int DSOSlamSystem::getKeyframeCount() {
    return outputWrapper_->getKeyframeCount();
}

MinimalImageB3 *DSOSlamSystem::cloneKeyframeImage() {
    return outputWrapper_->cloneKeyframeImage();
}

std::vector<std::pair<int, IOWrap::MyVertex *> > DSOSlamSystem::getVertices() {
    return outputWrapper_->getVertices();
}

void DSOSlamSystem::pushImu(vector<float> gyr, double gyrT, vector<float> acc, double accT) {
    imuInt_.addGyrData(std::move(gyr), gyrT);
    imuInt_.addAccData(std::move(acc), accT);
}

void DSOSlamSystem::pushImg(cv::Mat mat, double timestamp) {
    // We somehow seem to get each image twice.
    if (undistorter_ && std::abs(timestamp - lastImgTimestamp_) > 0.001) {
        assert(mat.type() == CV_8U);
        // Multiply exposure by 1000, as we want milliseconds.
        double exposure = exposureTimeUs_ * 1e-3;

        auto img = std::make_unique<dso::MinimalImageB>(mat.cols, mat.rows);
        memcpy(img->data, mat.data, mat.rows * mat.cols);
        // timestamp is in milliseconds, but shall be in seconds
        double finalTimestamp = timestamp;
        // gets float exposure and double timestamp
        std::unique_ptr<dso::ImageAndExposure> finalImage(undistorter_->undistort<unsigned char>(
                img.get(),
                static_cast<float>(exposure),
                finalTimestamp));
        img.reset();
        // Add image to the IMU interpolator, which will forward it to the FrameContainer, once the
        // corresponding IMU data is available.
        imuInt_.addImage(std::move(finalImage), finalTimestamp);
        lastImgTimestamp_ = timestamp;
    }
}
