#include "IMU-UWB.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// 四元数归一化
Quaternion normalize_quaternion(Quaternion q) {
    double norm = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    Quaternion normalized = { q.w / norm, q.x / norm, q.y / norm, q.z / norm };
    return normalized;
}

// 四元数乘法
Quaternion multiply_quaternion(Quaternion q1, Quaternion q2) {
    Quaternion result;
    result.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
    result.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
    result.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
    result.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
    return result;
}

// 应用四元数旋转向量
Vector3 rotate_vector(Quaternion q, Vector3 v) {
    Quaternion q_v = {0, v.x, v.y, v.z};
    Quaternion q_conj = {q.w, -q.x, -q.y, -q.z};
    Quaternion q_result = multiply_quaternion(multiply_quaternion(q, q_v), q_conj);
    Vector3 rotated = { q_result.x, q_result.y, q_result.z };
    return rotated;
}

// 互补滤波器更新姿态
void update_orientation(InertialNavigationSystem *ins, Vector3 gyro, Vector3 accel, double dt) {
    // 1. 通过陀螺仪数据积分更新姿态
    // 将角速度转换为四元数旋转向量
    double gyro_mag = sqrt(gyro.x*gyro.x + gyro.y*gyro.y + gyro.z*gyro.z);
    Quaternion delta_q;
    if (gyro_mag * dt / 2.0 > 1e-6) {
        double angle = gyro_mag * dt;
        double sin_half_angle = sin(angle / 2.0) / gyro_mag;
        delta_q.w = cos(angle / 2.0);
        delta_q.x = gyro.x * sin_half_angle;
        delta_q.y = gyro.y * sin_half_angle;
        delta_q.z = gyro.z * sin_half_angle;
    } else {
        // 对于小角度，使用近似
        delta_q.w = 1.0;
        delta_q.x = gyro.x * dt / 2.0;
        delta_q.y = gyro.y * dt / 2.0;
        delta_q.z = gyro.z * dt / 2.0;
    }

    // 更新姿态四元数
    ins->orientation = multiply_quaternion(ins->orientation, delta_q);
    ins->orientation = normalize_quaternion(ins->orientation);

    // 2. 通过加速度计估计重力方向
    // 假设设备静止或匀速直线运动，加速度计测量的是重力加速度
    Vector3 accel_norm;
    double accel_mag = sqrt(accel.x*accel.x + accel.y*accel.y + accel.z*accel.z);
    if (accel_mag != 0) {
        accel_norm.x = accel.x / accel_mag;
        accel_norm.y = accel.y / accel_mag;
        accel_norm.z = accel.z / accel_mag;
    } else {
        accel_norm.x = 0;
        accel_norm.y = 0;
        accel_norm.z = 0;
    }

    // 计算当前重力方向
    Vector3 gravity_est = rotate_vector(ins->orientation, ins->gravity);
    // 计算误差（叉乘）
    Vector3 error;
    error.x = accel_norm.y * gravity_est.z - accel_norm.z * gravity_est.y;
    error.y = accel_norm.z * gravity_est.x - accel_norm.x * gravity_est.z;
    error.z = accel_norm.x * gravity_est.y - accel_norm.y * gravity_est.x;

    // 3. 计算误差旋转并应用互补滤波器
    Quaternion error_q;
    double error_mag = sqrt(error.x*error.x + error.y*error.y + error.z*error.z);
    if (error_mag > 1e-6) {
        double angle = error_mag * (1.0 - ins->alpha);
        double sin_half_angle = sin(angle / 2.0) / error_mag;
        error_q.w = cos(angle / 2.0);
        error_q.x = error.x * sin_half_angle;
        error_q.y = error.y * sin_half_angle;
        error_q.z = error.z * sin_half_angle;
        // 更新姿态
        ins->orientation = multiply_quaternion(error_q, ins->orientation);
        ins->orientation = normalize_quaternion(ins->orientation);
    }
}

// 更新速度和位置
void update_velocity_position(InertialNavigationSystem *ins, Vector3 accel_net, double dt) {
    // 更新速度
    ins->velocity.x += accel_net.x * dt;
    ins->velocity.y += accel_net.y * dt;
    ins->velocity.z += accel_net.z * dt;

    // 更新位置
    ins->position.x += ins->velocity.x * dt + 0.5 * accel_net.x * dt * dt;
    ins->position.y += ins->velocity.y * dt + 0.5 * accel_net.y * dt * dt;
    ins->position.z += ins->velocity.z * dt + 0.5 * accel_net.z * dt * dt;
}

// 处理IMU数据
void process_imu(InertialNavigationSystem *ins, Vector3 accel, Vector3 gyro, double dt) {
    // 更新姿态
    update_orientation(ins, gyro, accel, dt);

    // // 将加速度转换到导航坐标系
    Vector3 accel_nav = rotate_vector(ins->orientation, accel);

    // 去除重力
    Vector3 accel_net;
    accel_net.x = accel_nav.x - ins->gravity.x;
    accel_net.y = accel_nav.y - ins->gravity.y;
    accel_net.z = accel_nav.z - ins->gravity.z;

    accel_net.x = accel.x;
    accel_net.y = accel.y;
    accel_net.z = accel.z;

    // 更新速度和位置
    update_velocity_position(ins, accel_net, dt);
}

// 模拟IMU运动
void simulate_imu_motion(double t_total, double dt, Vector3 *accel_data, Vector3 *gyro_data, int n) {
    for(int i = 0; i < n; i++) {
        // 示例运动：在x方向以1 m/s²加速，旋转速度为0.1 rad/s绕z轴
        accel_data[i].x = 1.0; // 加速 1 m/s² 在x轴
        accel_data[i].y = 0.0;
        accel_data[i].z = 0.0;

        gyro_data[i].x = 0.0;
        gyro_data[i].y = 0.0;
        gyro_data[i].z = 0.1; // 以0.1 rad/s绕z轴旋转
    }
}

