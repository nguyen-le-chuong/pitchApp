#include <jni.h>
#include <string>
#include <Eigen/Dense>
#include "Simulation.h"

static Simulation mSimulation;
static time_t prev_ts = 0;

extern "C"
JNIEXPORT void JNICALL
Java_com_example_myapp_MainActivity_initKalman(JNIEnv *env, jobject /* this */) {
    Eigen::VectorXd RotationState(3);
    Eigen::VectorXd SlopeState(3);
    Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(3, 3) * 0.01;

    RotationState << 0, 0, 1;
    SlopeState << 0, 0, 0;

    mSimulation.reset(loadSimulation4Parameters(), RotationState, SlopeState, cov);
}

extern "C"
JNIEXPORT jobject JNICALL
Java_com_example_myapp_MainActivity_updateKalmanState(
    JNIEnv *env,
    jobject /* this */,
    jfloatArray accData,
    jfloatArray gyroData,
    jlong timestamp) {

    jfloat *acc = env->GetFloatArrayElements(accData, nullptr);
    jfloat *gyro = env->GetFloatArrayElements(gyroData, nullptr);

    Eigen::VectorXd accVec(3);
    Eigen::VectorXd gyroVec(3);

    accVec << acc[0], acc[1], acc[2];
    gyroVec << gyro[0], gyro[1], gyro[2];

    env->ReleaseFloatArrayElements(accData, acc, 0);
    env->ReleaseFloatArrayElements(gyroData, gyro, 0);

    double dt = 0.0;
    time_t ts = static_cast<time_t>(timestamp);

    if (prev_ts != 0) {
        dt = static_cast<double>(ts - prev_ts) / 1000.0;
    } else {
        prev_ts = ts;
        return nullptr; 
    }

    double h_rear = 0.0, h_front = 0.0;
    double odo = 0.0;
    Eigen::Vector2d alpha;

    mSimulation.update(accVec, gyroVec, odo, h_rear, h_front, ts, dt, alpha);

    double pitch = mSimulation.returnPitch();
    double slope = mSimulation.returnSlope();

    jclass resultClass = env->FindClass("com/example/myapp/PitchSlopeResult");
    jmethodID constructor = env->GetMethodID(resultClass, "<init>", "(FFF)V");

    jobject resultObject = env->NewObject(resultClass, constructor, 
        static_cast<jfloat>(pitch), 
        static_cast<jfloat>(slope) // Trả về giá trị alpha
    );

    prev_ts = ts;

    return resultObject;
}
