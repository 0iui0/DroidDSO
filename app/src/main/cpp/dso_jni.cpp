#include <string.h>
#include <jni.h>
#include <boost/thread.hpp>
#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <util/MainSettings.h>

#include <iostream>
#include <istream>
#include <sstream>
#include <fstream>
#include "DSOSlamSystem.h"

static DSOSlamSystem *gSlamSystem = NULL;

extern "C" {
JavaVM *gJvm = NULL;

JNIEXPORT void JNICALL
Java_com_example_slamapp_TARNativeInterface_dsoInit(JNIEnv *env, jobject thiz, jobjectArray jargv) {
            printf("dsoInit\n");
    //jargv is a Java array of Java strings
    int argc = env->GetArrayLength(jargv);
    typedef char *pchar;
    pchar *argv = new pchar[argc];
    int i;
    for (i = 0; i < argc; i++) {
        jstring js = static_cast<jstring>(env->GetObjectArrayElement(jargv, i)); //A Java string
        const char *pjc = env->GetStringUTFChars(js, 0); //A pointer to a Java-managed char buffer
        size_t jslen = strlen(pjc);
        argv[i] = new char[jslen + 1]; //Extra char for the terminating null
        // Copy to *our* buffer. We could omit that, but IMHO this is cleaner. Also, const correctness.
        strcpy(argv[i], pjc);
        env->ReleaseStringUTFChars(js, pjc);
                printf("argv[%i]: %s\n", i, pjc);
    }
    env->GetJavaVM(&gJvm);
    gSlamSystem = new DSOSlamSystem();
    gSlamSystem->Init(argc, argv);
}

JNIEXPORT jint JNICALL
Java_com_example_slamapp_TARNativeInterface_dsoPushImu(JNIEnv *env, jobject clazz,
                                                       jbyteArray data) {
    jbyte *jdataP = env->GetByteArrayElements(data, nullptr);

    double gyroT;
    memcpy(&gyroT, jdataP + 44, 8);
    int16_t gyroX, gyroY, gyroZ;
    memcpy(&gyroX, jdataP + 60, 2);
    memcpy(&gyroY, jdataP + 64, 2);
    memcpy(&gyroZ, jdataP + 68, 2);
    double accT;
    memcpy(&accT, jdataP + 72, 8);
    int16_t accX, accY, accZ;
    memcpy(&accX, jdataP + 88, 2);
    memcpy(&accY, jdataP + 92, 2);
    memcpy(&accZ, jdataP + 96, 2);
    gSlamSystem->pushImu(
            {(float) gyroX / LSBTORADS, (float) gyroY / LSBTORADS, (float) gyroZ / LSBTORADS},
            gyroT,
            {(float) accX / LSBTOMS2, (float) accY / LSBTOMS2, (float) accZ / LSBTOMS2},
            accT);
    env->ReleaseByteArrayElements(data, jdataP, JNI_FALSE);
    return 0;
}
JNIEXPORT jint JNICALL
Java_com_example_slamapp_TARNativeInterface_dsoPushImage(JNIEnv *env, jobject clazz,
                                                         jbyteArray data) {
    jbyte *jdataP = env->GetByteArrayElements(data, nullptr);
    long long timestamp;
    memcpy(&timestamp, jdataP + IMGDATALEN, 8);

    if (timestamp == 0) {
        LOGE("lalala in this frame image timestamp is %lld", timestamp);
        return -1;
    }
    //convet image to show
    cv::Mat image(HEIGHT, WIDTHTWICE, CV_8UC1, jdataP);
    cv::Mat imgL = image(cv::Rect(0, 0, WIDTH, HEIGHT));
    cv::Mat imgR = image(cv::Rect(WIDTH, 0, WIDTH, HEIGHT));
    gSlamSystem->pushImg(imgL, timestamp);
    env->ReleaseByteArrayElements(data, jdataP, JNI_FALSE);
    return 0;
}

JNIEXPORT void JNICALL
Java_com_example_slamapp_TARNativeInterface_dsoRelease(JNIEnv *env, jobject thiz) {
            printf("dsoRelease\n");
    if (gSlamSystem) {
        delete gSlamSystem;
        gSlamSystem = NULL;
    }
}

JNIEXPORT void JNICALL
Java_com_example_slamapp_TARNativeInterface_dsoProcess(JNIEnv *env, jobject thiz) {
            printf("dsoProcess\n");
        gSlamSystem->process();
}

JNIEXPORT jfloatArray JNICALL
Java_com_example_slamapp_TARNativeInterface_dsoGetIntrinsics(JNIEnv *env, jobject thiz) {
    jfloatArray result;
    result = env->NewFloatArray(4);
    if (result == NULL) {
        return NULL; /* out of memory error thrown */
    }

    jfloat array1[4];
    array1[0] = gSlamSystem->cx();
    array1[1] = gSlamSystem->cy();
    array1[2] = gSlamSystem->fx();
    array1[3] = gSlamSystem->fy();

    env->SetFloatArrayRegion(result, 0, 4, array1);
    return result;
}

JNIEXPORT jfloatArray JNICALL
Java_com_example_slamapp_TARNativeInterface_dsoGetCurrentPose(JNIEnv *env, jobject thiz) {
    jfloatArray result;
    int length = 16;
    result = env->NewFloatArray(length);
    if (result == NULL) {
        return NULL; /* out of memory error thrown */
    }

    jfloat mat4[length];
    Sophus::Matrix4f m = gSlamSystem->currentCameraPose().matrix().cast<float>();
    float *pose = m.data();
    memcpy(mat4, pose, sizeof(jfloat) * length);

    env->SetFloatArrayRegion(result, 0, length, mat4);
    return result;
}

JNIEXPORT jobject JNICALL
Java_com_example_slamapp_TARNativeInterface_dsoGetPointCloud(JNIEnv *env, jobject thiz) {
    int pointNum = 0;
    std::vector<std::pair<int, IOWrap::MyVertex *> > vertices = gSlamSystem->getVertices();
    for (std::vector<std::pair<int, IOWrap::MyVertex *> >::iterator it = vertices.begin();
         it != vertices.end(); ++it) {
        pointNum += it->first;
    }

    jfloat *points = new jfloat[pointNum * 3];
    jint *colors = new jint[pointNum];
    int points_offset = 0;
    int colors_offset = 0;
    for (std::vector<std::pair<int, IOWrap::MyVertex *> >::iterator it = vertices.begin();
         it != vertices.end(); ++it) {
        for (int i = 0; i < it->first; ++i) {
            memcpy(points + points_offset, it->second[i].point, 3 * sizeof(float));
            colors[colors_offset] =
                    (it->second[i].color[3] << 24) + (it->second[i].color[0] << 16) +
                    (it->second[i].color[1] << 8) + it->second[i].color[2];
            points_offset += 3;
            colors_offset++;
        }
    }

    jclass classKeyFrame = env->FindClass("com/example/slamapp/DSOPointCloud");

    // new DSOPointCloud object
    jmethodID initMethodID = env->GetMethodID(classKeyFrame, "<init>", "()V");
    assert (initMethodID != NULL);
    jobject pointCloudObject = env->NewObject(classKeyFrame, initMethodID);
    assert (pointCloudObject != NULL);

    // set pointCount
    jint pointCount = pointNum;
    jfieldID pointCountFieldID = env->GetFieldID(classKeyFrame, "pointCount", "I");
    assert (pointCountFieldID != NULL);
    env->SetIntField(pointCloudObject, pointCountFieldID, pointCount);

    // set points
    jfloatArray pointsArray = env->NewFloatArray(pointNum * 3);
    env->SetFloatArrayRegion(pointsArray, 0, pointNum * 3, points);
    jfieldID pointsFieldID = env->GetFieldID(classKeyFrame, "worldPoints", "[F");
    assert (pointsFieldID != NULL);
    env->SetObjectField(pointCloudObject, pointsFieldID, pointsArray);

    // set colors
    jintArray colorsArray = env->NewIntArray(pointNum);
    env->SetIntArrayRegion(colorsArray, 0, pointNum, colors);
    jfieldID colorsFieldID = env->GetFieldID(classKeyFrame, "colors", "[I");
    assert (colorsFieldID != NULL);
    env->SetObjectField(pointCloudObject, colorsFieldID, colorsArray);

    // Release
    env->DeleteLocalRef(classKeyFrame);
    delete[] points;
    delete[] colors;

    return pointCloudObject;
}

JNIEXPORT jint JNICALL
Java_com_example_slamapp_TARNativeInterface_dsoGetKeyFrameCount(JNIEnv *env, jobject thiz) {
    return gSlamSystem->getKeyframeCount();
}

JNIEXPORT jbyteArray JNICALL
Java_com_example_slamapp_TARNativeInterface_dsoGetCurrentImage(JNIEnv *env, jobject thiz) {
    MinimalImageB3 *minImg = gSlamSystem->cloneKeyframeImage();
    int width = gSlamSystem->width();
    int height = gSlamSystem->height();
    int imgSize = width * height * 4;
    unsigned char *imgData = new unsigned char[imgSize];

    for (int i = 0; i < width * height; ++i) {
        imgData[i * 4] = minImg->data[i][0];
        imgData[i * 4 + 1] = minImg->data[i][1];
        imgData[i * 4 + 2] = minImg->data[i][2];
        imgData[i * 4 + 3] = (unsigned char) 0xff;
    }

    jbyteArray byteArray = env->NewByteArray(imgSize);
    env->SetByteArrayRegion(byteArray, 0, imgSize, (jbyte *) imgData);

    delete minImg;
    delete[] imgData;
    return byteArray;
}
}
