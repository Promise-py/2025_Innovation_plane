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


//�������˲������ṹ��
typedef struct KalmanFilter
{
	float LastP;		//��һ��Э����
	float NewP;		//���µ�Э����
	float Out;			//���������
	float Kg;				//����������
	float Q;				//����������Э����
	float R;				//�۲�������Э����
}KalmanFilter;


vec3d vdiff(const vec3d vector1, const vec3d vector2);  /* �������������Ĳ�ֵ ��vector1 - vector2����*/
vec3d vsum(const vec3d vector1, const vec3d vector2);   /* ������������֮�͡�*/
vec3d vmul(const vec3d vector, const double n);         /* ����������һ�����֡�*/
vec3d vdiv(const vec3d vector, const double n);         /* ����������һ�����֡�*/
double vdist(const vec3d v1, const vec3d v2);           /* ����ŷ����÷�����*/
double vnorm(const vec3d vector);                       /* ����ŷ����÷�����*/
double dot(const vec3d vector1, const vec3d vector2);   /* �������������ĵ����*/
vec3d cross(const vec3d vector1, const vec3d vector2);  /* �������滻Ϊ�佻�������һ��������*/
double gdoprate(const vec3d tag, const vec3d p1, const vec3d p2, const vec3d p3);  /* ���ؽ��� 0-1 ֮��� GDOP�����ȵļ���ϡ�ͣ��ʡ��ϵ͵� GDOP ����ζ�Ÿ��õĽ��澫�� */
int sphereline(const vec3d p1, const vec3d p2, const vec3d sc, double r, double *const mu1, double *const mu2);/* ��뾶Ϊ r ������ sc �ཻ����ֱ�� p1-p2 �ཻ������ɹ����򷵻��㣬���򷵻ظ�����mu1 �� mu2 �ǳ��������ڲ��ҽ��㡣*/
int GetLocation(vec3d *best_solution, int use4thAnchor, vec3d* anchorArray, int *distanceArray);
uint8_t Trilateration(int distance1, int distance2, int distance3, int distance4, float real_distance, int logal);
void KalmanFilter_Init(KalmanFilter*EKF);

extern KalmanFilter uwbkx;
extern KalmanFilter uwbky;
#endif

