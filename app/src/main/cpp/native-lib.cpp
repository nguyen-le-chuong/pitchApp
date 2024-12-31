
#include <jni.h>
#include <string>
#include <Eigen/Dense>
#include "simulation.h"

static Simulation mSimulation;
static time_t prev_ts = 0;
Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
extern "C"
JNIEXPORT void JNICALL
Java_com_example_pitchApp_1v2_MainActivity_initKalman(JNIEnv *env, jobject, jfloat cor) {
    Eigen::VectorXd RotationState(3);
    Eigen::VectorXd SlopeState(3);
    Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(3, 3) * cor;

    RotationState << 0, 0, 1;
    SlopeState << 0, 0, 0;

    // Reset the simulation with initial values
    mSimulation.reset(RotationState, SlopeState, cov);
}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_pitchApp_1v2_MainActivity_setKalmanParameters(
        JNIEnv *env,
        jobject,
        jfloatArray parameters) {

    jsize length = env->GetArrayLength(parameters);
    if (length != 9) return;  // Ensure the array has the correct number of parameters

    jfloat *params = env->GetFloatArrayElements(parameters, nullptr);

    // Set the parameters in the simulation
    mSimulation.setKalmanParameters(loadSimulation4Parameters(), 
            params[0], params[1], params[2], params[3], params[4],
            params[5], params[6], params[7], params[8]
    );

    env->ReleaseFloatArrayElements(parameters, params, 0);
}

extern "C"
JNIEXPORT jobject JNICALL
Java_com_example_pitchApp_1v2_MainActivity_updateKalmanState(
        JNIEnv *env,
        jobject,
        jfloatArray accData,
        jfloatArray gyroData,
        jlong timestamp) {

    jsize accLength = env->GetArrayLength(accData);
    jsize gyroLength = env->GetArrayLength(gyroData);
    if (accLength != 3 || gyroLength != 3) return nullptr;

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
        return nullptr; // No valid time difference yet
    }

    
    Eigen::Vector2d alpha;
    velocity = mSimulation.calculateVelocity(accVec, gyroVec, dt, velocity);
    mSimulation.update(accVec, gyroVec, velocity(0), ts, dt, alpha);
    mSimulation.updateRoadSlope(accVec, gyroVec, velocity(0), dt);
    double pitch = mSimulation.returnPitch();
    double slope = mSimulation.returnSlope();

    jclass resultClass = env->FindClass("com/example/pitchApp_v2/PitchSlopeResult");
    jmethodID constructor = env->GetMethodID(resultClass, "<init>", "(DDD)V");

    jobject resultObject = env->NewObject(resultClass, constructor,
                                          static_cast<jdouble>(pitch),
                                          static_cast<jdouble>(slope),
                                          static_cast<jdouble>(pitch - slope));

    prev_ts = ts;

    return resultObject;
}
