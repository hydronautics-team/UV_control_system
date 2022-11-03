#ifndef UV_STATE_H
#define UV_STATE_H

#include "stdint.h"

#include <QDebug>

enum class e_CSMode : unsigned char {
    MODE_HANDLE = 0,
    MODE_IDENTIFICATION_1,
    MODE_IDENTIFICATION_2,
    MODE_AUTO,
    MODE_THRUSTER
};

enum class e_StabilizationContours: unsigned char {
    CONTOUR_DEPTH = 0,
    CONTOUR_MARCH,
    CONTOUR_LAG,
    CONTOUR_YAW,
    CONTOUR_ROLL,
    CONTOUR_PITCH
};

#pragma pack(push,1)
//структура данных, которая передается из Северова в Пульт
//тут описаны данные, которые Пульт принимает от Северова

struct ControlData {
    ControlData();
    double yaw;
    double pitch;
    double roll;
    double march;
    double depth;
    double lag;
};

struct ControlContoursFlags {
    ControlContoursFlags();
    quint8 yaw;
    quint8 pitch;
    quint8 roll;
    quint8 march;
    quint8 depth;
    quint8 lag;
};

struct ImuData {
    ImuData();
    float ax;
    float ay;
    float az;

    float wx;
    float wy;
    float wz;

    float psi;
    float teta;
    float gamma;

    float q0;
    float q1;
    float q2;
    float q3;
};

struct MoutionDataAfterAlgoritms {
    MoutionDataAfterAlgoritms();
    float ax;
    float ay;
    float az;

    float vx;
    float vy;
    float vz;

    float vxg;
    float vyg;
    float vzg;

    float x;
    float y;
    float z;

    float wx;
    float wy;
    float wz;

    float dPsi;
    float dTeta;
    float dGamma;

    float psi;
    float teta;
    float gamma;

    float q0;
    float q1;
    float q2;
    float q3;
};

struct Thruster {
    Thruster();
    int id;
    float velocity;
    float current;
};

struct ConnectionFlags {
    ConnectionFlags();
    quint8 vectorNav;
    quint8 joystick;
    quint8 controlSystem;
    quint8 thrusterController;
};

struct SinTest {
    SinTest();
    double u0;
    double a;
    double w;
    double k;
    double h;
    e_StabilizationContours selectedContour;

};

struct ToPult {
    ImuData imuData;
    float depth;

    ConnectionFlags connectionFlags;
    uint checksum;
};

//структура данных, которая передается из планировщика в АНПА
struct FromPult {
    // эти структуры и енумы реализованы в uv_state
    ControlData controlData;
    ControlContoursFlags controlContoursFlags;
    e_CSMode cSMode;
    SinTest sinTest;

    quint8 resetImu;
    quint8 thrusterPower;
    quint8 experimentTypicalInput;
    quint8 sinSignal;
    uint checksum;
};

#pragma pack (pop)

class UV_State {
public:
    UV_State();
    ~UV_State();

    ControlData control;

    ControlContoursFlags controlContoursFlags;
    e_CSMode cSMode;

    ImuData imuData;
    MoutionDataAfterAlgoritms moutionDataAfterAlgoritms;
    float depth;

    Thruster *thruster;

    // Flags
    ConnectionFlags connectionFlags;
    bool resetImu;
    bool thrusterPower;
    bool experimentTypicalInput;
    bool sinSignal;
};

#endif // UV_STATE_H
