#include "uv_state.h"
#define THRUSTER_AMOUNT 8

UV_State::UV_State() {

    thruster = new Thruster[THRUSTER_AMOUNT];
    for (int i = 0; i < THRUSTER_AMOUNT; ++i) {
        thruster[i].id = i;
    }

    resetImu = false;
    thrusterPower = true;
}

UV_State::~UV_State() {
    if (thruster != nullptr){
        delete[] thruster;
    }
}

ControlData::ControlData() {
    yaw = 0;
    pitch = 0;
    roll = 0;
    march = 0;
    depth = 0;
    lag = 0;
}

ControlContoursFlags::ControlContoursFlags() {
    yaw = 0;
    pitch = 0;
    roll = 0;
    march = 0;
    depth = 0;
    lag = 0;
}

ImuData::ImuData() {
    ax = 0;
    ay = 0;
    az = 0;

    wx = 0;
    wy = 0;
    wz = 0;

    psi = 0;
    teta = 0;
    gamma = 0;

    q0 = 0;
    q1 = 0;
    q2 = 0;
    q3 = 0;
}

MoutionDataAfterAlgoritms::MoutionDataAfterAlgoritms() {
    ax = 0;
    ay = 0;
    az = 0;

    vx = 0;
    vy = 0;
    vz = 0;

    vxg = 0;
    vyg = 0;
    vzg = 0;

    x = 0;
    y = 0;
    z = 0;

    wx = 0;
    wy = 0;
    wz = 0;

    dPsi = 0;
    dTeta = 0;
    dGamma = 0;

    psi = 0;
    teta = 0;
    gamma = 0;

    q0 = 0;
    q1 = 0;
    q2 = 0;
    q3 = 0;
}

Thruster::Thruster() {
    id = 0;
    velocity = 0;
    current = 0;
}

ConnectionFlags::ConnectionFlags() {
    vectorNav = 0;
    joystick = 0;
    controlSystem = 0;
    thrusterController = 0;
}
