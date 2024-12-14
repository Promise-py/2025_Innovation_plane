#ifndef __TRILATERATION_H__
#define __TRILATERATION_H__
 
#include "math.h"
#include "stdlib.h"
#include "time.h"
#include "stm32f1xx_hal.h"
#include <string.h>

#define   TRIL_3SPHERES    3
#define   TRIL_4SPHERES    4
 
typedef struct vec3d    vec3d;
struct vec3d {
    double  x;
    double  y;
    double  z;
};

typedef struct UWB_Data{
    double  x;
    double  y;
    double  z;
}UWB_Data;


//卡尔曼滤波参数结构体
typedef struct KalmanFilter
{
	float LastP;		//上一次协方差
	float NewP;		//最新的协方差
	float Out;			//卡尔曼输出
	float Kg;				//卡尔曼增益
	float Q;				//过程噪声的协方差
	float R;				//观测噪声的协方差
}KalmanFilter;


vec3d vdiff(const vec3d vector1, const vec3d vector2);  /* 返回两个向量的差值 （vector1 - vector2）。*/
vec3d vsum(const vec3d vector1, const vec3d vector2);   /* 返回两个向量之和。*/
vec3d vmul(const vec3d vector, const double n);         /* 将向量乘以一个数字。*/
vec3d vdiv(const vec3d vector, const double n);         /* 将向量除以一个数字。*/
double vdist(const vec3d v1, const vec3d v2);           /* 返回欧几里得范数。*/
double vnorm(const vec3d vector);                       /* 返回欧几里得范数。*/
double dot(const vec3d vector1, const vec3d vector2);   /* 返回两个向量的点积。*/
vec3d cross(const vec3d vector1, const vec3d vector2);  /* 将向量替换为其交叉积与另一个向量。*/
double gdoprate(const vec3d tag, const vec3d p1, const vec3d p2, const vec3d p3);  /* 返回介于 0-1 之间的 GDOP（精度的几何稀释）率。较低的 GDOP 率意味着更好的交叉精度 */
int sphereline(const vec3d p1, const vec3d p2, const vec3d sc, double r, double *const mu1, double *const mu2);/* 与半径为 r 的球体 sc 相交，与直线 p1-p2 相交。如果成功，则返回零，否则返回负错误。mu1 和 mu2 是常数，用于查找交点。*/
int GetLocation(vec3d *best_solution, int use4thAnchor, vec3d* anchorArray, int *distanceArray);
uint8_t Trilateration(int distance1, int distance2, int distance3, int distance4, float real_distance, int logal);
void KalmanFilter_Init(KalmanFilter*EKF);

extern KalmanFilter uwbkx;
extern KalmanFilter uwbky;
#endif

