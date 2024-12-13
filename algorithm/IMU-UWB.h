#ifndef __IMU_UWB_H__
#define __IMU_UWB_H__

// 定义一个三维向量结构
typedef struct {
    double x;
    double y;
    double z;
} Vector3;

// 定义一个四元数结构
typedef struct {
    double w;
    double x;
    double y;
    double z;
} Quaternion;

// 定义惯性导航系统结构
typedef struct {
    Vector3 position;      // 位置 [x, y, z]
    Vector3 velocity;      // 速度 [vx, vy, vz]
    Quaternion orientation; // 姿态（四元数）
    double alpha;          // 互补滤波器参数
    Vector3 gravity;       // 重力加速度向量
} InertialNavigationSystem;


void process_imu(InertialNavigationSystem *ins, Vector3 accel, Vector3 gyro, double dt);

#endif