//
// Created by an on 2022/8/18.
//
#ifndef SLAM_APP_DSOSLAMSYSTEM_H
#define SLAM_APP_DSOSLAMSYSTEM_H
#include <boost/thread.hpp>

#include "IOWrapper/Output3DWrapper.h"
#include "IOWrapper/ImageDisplay.h"
#include "IOWrapper/Android/AndroidOutput3DWrapper.h"
#include "IOWrapper/Android/KeyFrameDisplay.h"

#include "util/settings.h"
#include "util/globalFuncs.h"
#include "util/DatasetReader.h"
#include "util/globalCalib.h"
#include "util/logger.h"

#include "util/NumType.h"
#include "FullSystem/FullSystem.h"
#include "OptimizationBackend/MatrixAccumulators.h"
#include "FullSystem/PixelSelector2.h"
#include "util/MinimalImage.h"
#include <util/MainSettings.h>
#include "live/DatasetSaver.h"
#include "live/IMUInterpolator.h"
#include <live/FrameSkippingStrategy.h>
#include <utility>

using std::vector;

#define WIDTH 640
#define WIDTHTWICE WIDTH * 2
#define HEIGHT 480
#define IMGDATALEN WIDTHTWICE *HEIGHT


#define gravityG 9.80665f

#define ACCLSB 4096.0f
#define LSBTOMS2 ACCLSB *gravityG

#define GYRLSB 16.4f
#define PI 3.1415926535898f
#define LSBTORADS GYRLSB / 180 * PI

using namespace dso;
class DSOSlamSystem {
public:
    DSOSlamSystem();
    int Init(int argc, char** argv);
    ~DSOSlamSystem();
    float fx();
    float fy();
    float cx();
    float cy();
    int width();
    int height();
    SE3 currentCameraPose();
    int getKeyframeCount();
    MinimalImageB3 *cloneKeyframeImage();
    std::vector<std::pair<int, IOWrap::MyVertex *> > getVertices();
    void pushImu(vector<float> gyr, double gyrT, vector<float> acc, double accT);
    void pushImg(cv::Mat mat, double timestamp); 
    void process();
private:
    FullSystem *fullSystem_;
    Undistort *undistorter_;
    IOWrap::AndroidOutput3DWrapper *outputWrapper_;
    dmvio::FrameContainer frameContainer_;
    dmvio::IMUInterpolator imuInt_;
    bool linearizeOperation_ = false;
    int frameId_;
    uint32_t exposureTimeUs_;
    // IMU interpolator will take care of creating "fake measurements" to synchronize the sensors by interpolating IMU data.
    double lastImgTimestamp_ = -1.0;
    int start = 2;
    dmvio::MainSettings mainSettings;
    dmvio::IMUCalibration imuCalibration;
    dmvio::IMUSettings imuSettings;
    dmvio::FrameSkippingSettings frameSkippingSettings;
};
#endif //SLAM_APP_DSOSLAMSYSTEM_H
