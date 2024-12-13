#include "math.h"
#include "stdlib.h"
#include "time.h"
#include "trilateration.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include "stdio.h"
#include "data_pool.h"

/* 最大非负数仍被视为零 */
#define   MAXZERO  0.001
 
#define     ERR_TRIL_CONCENTRIC                     -1
#define     ERR_TRIL_COLINEAR_2SOLUTIONS            -2
#define     ERR_TRIL_SQRTNEGNUMB                    -3
#define     ERR_TRIL_NOINTERSECTION_SPHERE4         -4
#define     ERR_TRIL_NEEDMORESPHERE                 -5
 
#define CM_ERR_ADDED (10) 

char str[40];
float xx;
float yy;
float zz;

UWB_Data uwb_data;

/* 返回两个向量的差值 （vector1 - vector2）。*/
vec3d vdiff(const vec3d vector1, const vec3d vector2)
{
    vec3d v;
    v.x = vector1.x - vector2.x;
    v.y = vector1.y - vector2.y;
    v.z = vector1.z - vector2.z;
    return v;
}
 
/* 返回两个向量之和。*/
vec3d vsum(const vec3d vector1, const vec3d vector2)
{
    vec3d v;
    v.x = vector1.x + vector2.x;
    v.y = vector1.y + vector2.y;
    v.z = vector1.z + vector2.z;
    return v;
}
 
/* 将向量乘以一个数字。*/
vec3d vmul(const vec3d vector, const double n)
{
    vec3d v;
    v.x = vector.x * n;
    v.y = vector.y * n;
    v.z = vector.z * n;
    return v;
}
 
/* 将向量除以一个数字。*/
vec3d vdiv(const vec3d vector, const double n)
{
    vec3d v;
    v.x = vector.x / n;
    v.y = vector.y / n;
    v.z = vector.z / n;
    return v;
}
 
/* 返回欧几里得范数。*/
double vdist(const vec3d v1, const vec3d v2)
{
    double xd = v1.x - v2.x;
    double yd = v1.y - v2.y;
    double zd = v1.z - v2.z;
    return sqrt(xd * xd + yd * yd + zd * zd);
}
 
/* 返回欧几里得范数。*/
double vnorm(const vec3d vector)
{
    return sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}
 
/* 返回两个向量的点积。*/
double dot(const vec3d vector1, const vec3d vector2)
{
    return vector1.x * vector2.x + vector1.y * vector2.y + vector1.z * vector2.z;
}
 
/* 将向量替换为其交叉积与另一个向量。*/
vec3d cross(const vec3d vector1, const vec3d vector2)
{
    vec3d v;
    v.x = vector1.y * vector2.z - vector1.z * vector2.y;
    v.y = vector1.z * vector2.x - vector1.x * vector2.z;
    v.z = vector1.x * vector2.y - vector1.y * vector2.x;
    return v;
}
 
/* 返回介于 0-1 之间的 GDOP（精度的几何稀释）率。
   较低的 GDOP 率意味着更好的交叉精度。
 */
double gdoprate(const vec3d tag, const vec3d p1, const vec3d p2, const vec3d p3)
{
    vec3d ex, t1, t2, t3;
    double h, gdop1, gdop2, gdop3, result;
 
    ex = vdiff(p1, tag);
    h = vnorm(ex);
    t1 = vdiv(ex, h);
 
    ex = vdiff(p2, tag);
    h = vnorm(ex);
    t2 = vdiv(ex, h);
 
    ex = vdiff(p3, tag);
    h = vnorm(ex);
    t3 = vdiv(ex, h);
 
    gdop1 = fabs(dot(t1, t2));
    gdop2 = fabs(dot(t2, t3));
    gdop3 = fabs(dot(t3, t1));
 
    if (gdop1 < gdop2) result = gdop2; else result = gdop1;
    if (result < gdop3) result = gdop3;
 
    return result;
}
 
/* 与半径为 r 的球体 sc 相交，与直线 p1-p2 相交。
 * 如果成功，则返回零，否则返回负错误。mu1 和 mu2 是常数，用于查找交点。
*/
int sphereline(const vec3d p1, const vec3d p2, const vec3d sc, double r, double *const mu1, double *const mu2)
{
   double a,b,c;
   double bb4ac;
   vec3d dp;
 
   dp.x = p2.x - p1.x;
   dp.y = p2.y - p1.y;
   dp.z = p2.z - p1.z;
 
   a = dp.x * dp.x + dp.y * dp.y + dp.z * dp.z;
 
   b = 2 * (dp.x * (p1.x - sc.x) + dp.y * (p1.y - sc.y) + dp.z * (p1.z - sc.z));
 
   c = sc.x * sc.x + sc.y * sc.y + sc.z * sc.z;
   c += p1.x * p1.x + p1.y * p1.y + p1.z * p1.z;
   c -= 2 * (sc.x * p1.x + sc.y * p1.y + sc.z * p1.z);
   c -= r * r;
 
   bb4ac = b * b - 4 * a * c;
 
   if (fabs(a) == 0 || bb4ac < 0) {
      *mu1 = 0;
      *mu2 = 0;
      return -1;
   }
 
   *mu1 = (-b + sqrt(bb4ac)) / (2 * a);
   *mu2 = (-b - sqrt(bb4ac)) / (2 * a);
 
   return 0;
}
 
/* 如果使用 3 个球体执行，则返回TRIL_3SPHERES，
 * 如果使用 4 个球体执行，则返回TRIL_4SPHERES
 * For TRIL_3SPHERES, there are two solutions: result1 and result2
 * 对于TRIL_3SPHERES，有两种解决方案：result1 和 result2
 * 对于TRIL_4SPHERES，只有一个解决方案：best_solution
 * 其他错误返回负数
 * 要强制函数仅使用 3 个球体，请提供p1、p2、p3 或 p4 中任何位置的任何球体。
 * 最后一个参数是被视为零的最大非负数; 它有点类似于机器epsilon（但包括）。
*/
int trilateration(vec3d *const result1,
                  vec3d *const result2,
                  vec3d *const best_solution,
                  const vec3d p1, const double r1,
                  const vec3d p2, const double r2,
                  const vec3d p3, const double r3,
                  const vec3d p4, const double r4,
                  const double maxzero)
{
    vec3d   ex, ey, ez, t1, t2, t3;
    double  h, i, j, x, y, z, t;
    double  mu1, mu2, mu;
    int result;
  
    //如果在前3个球体中至少有2个同心球，则计算可能不会继续，请将其错误地丢弃 -

    /* h = |p3 - p1|, ex = (p3 - p1) / |p3 - p1| */
    ex = vdiff(p3, p1); // vector p13
    h = vnorm(ex); // scalar p13
    if (h <= maxzero) {
        /* p1 and p3 are concentric, not good to obtain a precise intersection point */
        //printf("concentric13 return -1\n");
        return ERR_TRIL_CONCENTRIC;
    }
 
    /* h = |p3 - p2|, ex = (p3 - p2) / |p3 - p2| */
    ex = vdiff(p3, p2); // vector p23
    h = vnorm(ex); // scalar p23
    if (h <= maxzero) {
        /* p2 and p3 are concentric, not good to obtain a precise intersection point */
        //printf("concentric23 return -1\n");
        return ERR_TRIL_CONCENTRIC;
    }
 
    /* h = |p2 - p1|, ex = (p2 - p1) / |p2 - p1| */
    ex = vdiff(p2, p1); // vector p12
    h = vnorm(ex); // scalar p12
    if (h <= maxzero) {
        /* p1 and p2 are concentric, not good to obtain a precise intersection point */
        //printf("concentric12 return -1\n");
        return ERR_TRIL_CONCENTRIC;
    }
    ex = vdiv(ex, h); // unit vector ex with respect to p1 (new coordinate system)
 
    /* t1 = p3 - p1, t2 = ex (ex . (p3 - p1)) */
    t1 = vdiff(p3, p1); // vector p13
    i = dot(ex, t1); // the scalar of t1 on the ex direction
    t2 = vmul(ex, i); // colinear vector to p13 with the length of i
 
    /* ey = (t1 - t2), t = |t1 - t2| */
    ey = vdiff(t1, t2); // vector t21 perpendicular to t1
    t = vnorm(ey); // scalar t21
    if (t > maxzero) {
        /* ey = (t1 - t2) / |t1 - t2| */
        ey = vdiv(ey, t); // unit vector ey with respect to p1 (new coordinate system)
 
        /* j = ey . (p3 - p1) */
        j = dot(ey, t1); // scalar t1 on the ey direction
    } else
        j = 0.0;
 
    /* Note: t <= maxzero implies j = 0.0. */
    if (fabs(j) <= maxzero) {
 
        /* Is point p1 + (r1 along the axis) the intersection? */
        t2 = vsum(p1, vmul(ex, r1));
        if (fabs(vnorm(vdiff(p2, t2)) - r2) <= maxzero &&
            fabs(vnorm(vdiff(p3, t2)) - r3) <= maxzero) {
            /* Yes, t2 is the only intersection point. */
            if (result1)
                *result1 = t2;
            if (result2)
                *result2 = t2;
            return TRIL_3SPHERES;
        }
 
        /* Is point p1 - (r1 along the axis) the intersection? */
        t2 = vsum(p1, vmul(ex, -r1));
        if (fabs(vnorm(vdiff(p2, t2)) - r2) <= maxzero &&
            fabs(vnorm(vdiff(p3, t2)) - r3) <= maxzero) {
            /* Yes, t2 is the only intersection point. */
            if (result1)
                *result1 = t2;
            if (result2)
                *result2 = t2;
            return TRIL_3SPHERES;
        }
        /* p1, p2 and p3 are colinear with more than one solution */
        return ERR_TRIL_COLINEAR_2SOLUTIONS;
    }
 
    /* ez = ex x ey */
    ez = cross(ex, ey); // unit vector ez with respect to p1 (new coordinate system)
 
    x = (r1*r1 - r2*r2) / (2*h) + h / 2;
    y = (r1*r1 - r3*r3 + i*i) / (2*j) + j / 2 - x * i / j;
    z = r1*r1 - x*x - y*y;
    if (z < -maxzero) {
        /* The solution is invalid, square root of negative number */
        return ERR_TRIL_SQRTNEGNUMB;
    } else
    if (z > 0.0)
        z = sqrt(z);
    else
        z = 0.0;
 
    /* t2 = p1 + x ex + y ey */
    t2 = vsum(p1, vmul(ex, x));
    t2 = vsum(t2, vmul(ey, y));
 
    /* result1 = p1 + x ex + y ey + z ez */
    if (result1)
        *result1 = vsum(t2, vmul(ez, z));
 
    /* result1 = p1 + x ex + y ey - z ez */
    if (result2)
        *result2 = vsum(t2, vmul(ez, -z));

    //检查球体 4 到球体 1、2 和 3 的同心度（如果它与其中一个球体同心），则球体 4 不能用于确定最佳解并返回 -1

    /* h = |p4 - p1|, ex = (p4 - p1) / |p4 - p1| */
    ex = vdiff(p4, p1); // vector p14
    h = vnorm(ex); // scalar p14
    if (h <= maxzero) {
        /* p1 and p4 are concentric, not good to obtain a precise intersection point */
        //printf("concentric14 return 0\n");
        return TRIL_3SPHERES;
    }
    /* h = |p4 - p2|, ex = (p4 - p2) / |p4 - p2| */
    ex = vdiff(p4, p2); // vector p24
    h = vnorm(ex); // scalar p24
    if (h <= maxzero) {
        /* p2 and p4 are concentric, not good to obtain a precise intersection point */
        //printf("concentric24 return 0\n");
        return TRIL_3SPHERES;
    }
    /* h = |p4 - p3|, ex = (p4 - p3) / |p4 - p3| */
    ex = vdiff(p4, p3); // vector p34
    h = vnorm(ex); // scalar p34
    if (h <= maxzero) {
        /* p3 and p4 are concentric, not good to obtain a precise intersection point */
        //printf("concentric34 return 0\n");
        return TRIL_3SPHERES;
    }
 
    // if sphere 4 is not concentric to any sphere, then best solution can be obtained
    /* find i as the distance of result1 to p4 */
    t3 = vdiff(*result1, p4);
    i = vnorm(t3);
    /* find h as the distance of result2 to p4 */
    t3 = vdiff(*result2, p4);
    h = vnorm(t3);
 
    /* pick the result1 as the nearest point to the center of sphere 4 */
    if (i > h) {
        *best_solution = *result1;
        *result1 = *result2;
        *result2 = *best_solution;
    }
 
    int count4 = 0;
    double rr4 = r4;
    result = 1;
    /* intersect result1-result2 vector with sphere 4 */
    while(result && count4 < 10)
    {
        result=sphereline(*result1, *result2, p4, rr4, &mu1, &mu2);
        rr4+=0.1;
        count4++;
    }
 
    if (result) 
    {
        /* No intersection between sphere 4 and the line with the gradient of result1-result2! */
        *best_solution = *result1; // result1 is the closer solution to sphere 4
        //return ERR_TRIL_NOINTERSECTION_SPHERE4;
 
    } 
    else
    {
        if (mu1 < 0 && mu2 < 0) {
 
            /* if both mu1 and mu2 are less than 0 */
            /* result1-result2 line segment is outside sphere 4 with no intersection */
            if (fabs(mu1) <= fabs(mu2)) mu = mu1; else mu = mu2;
            /* h = |result2 - result1|, ex = (result2 - result1) / |result2 - result1| */
            ex = vdiff(*result2, *result1); // vector result1-result2
            h = vnorm(ex); // scalar result1-result2
            ex = vdiv(ex, h); // unit vector ex with respect to result1 (new coordinate system)
            /* 50-50 error correction for mu */
            mu = 0.5*mu;
            /* t2 points to the intersection */
            t2 = vmul(ex, mu*h);
            t2 = vsum(*result1, t2);
            /* the best solution = t2 */
            *best_solution = t2;
 
        } else if ((mu1 < 0 && mu2 > 1) || (mu2 < 0 && mu1 > 1)) {
 
            /* if mu1 is less than zero and mu2 is greater than 1, or the other way around */
            /* result1-result2 line segment is inside sphere 4 with no intersection */
            if (mu1 > mu2) mu = mu1; else mu = mu2;
            /* h = |result2 - result1|, ex = (result2 - result1) / |result2 - result1| */
            ex = vdiff(*result2, *result1); // vector result1-result2
            h = vnorm(ex); // scalar result1-result2
            ex = vdiv(ex, h); // unit vector ex with respect to result1 (new coordinate system)
            /* t2 points to the intersection */
            t2 = vmul(ex, mu*h);
            t2 = vsum(*result1, t2);
            /* vector t2-result2 with 50-50 error correction on the length of t3 */
            t3 = vmul(vdiff(*result2, t2),0.5);
            /* the best solution = t2 + t3 */
            *best_solution = vsum(t2, t3);
 
        } else if (((mu1 > 0 && mu1 < 1) && (mu2 < 0 || mu2 > 1))
                || ((mu2 > 0 && mu2 < 1) && (mu1 < 0 || mu1 > 1))) {
 
            /* if one mu is between 0 to 1 and the other is not */
            /* result1-result2 line segment intersects sphere 4 at one point */
            if (mu1 >= 0 && mu1 <= 1) mu = mu1; else mu = mu2;
            /* add or subtract with 0.5*mu to distribute error equally onto every sphere */
            if (mu <= 0.5) mu-=0.5*mu; else mu-=0.5*(1-mu);
            /* h = |result2 - result1|, ex = (result2 - result1) / |result2 - result1| */
            ex = vdiff(*result2, *result1); // vector result1-result2
            h = vnorm(ex); // scalar result1-result2
            ex = vdiv(ex, h); // unit vector ex with respect to result1 (new coordinate system)
            /* t2 points to the intersection */
            t2 = vmul(ex, mu*h);
            t2 = vsum(*result1, t2);
            /* the best solution = t2 */
            *best_solution = t2;
 
        } else if (mu1 == mu2) {
 
            /* if both mu1 and mu2 are between 0 and 1, and mu1 = mu2 */
            /* result1-result2 line segment is tangential to sphere 4 at one point */
            mu = mu1;
            /* add or subtract with 0.5*mu to distribute error equally onto every sphere */
            if (mu <= 0.25) mu-=0.5*mu;
            else if (mu <=0.5) mu-=0.5*(0.5-mu);
            else if (mu <=0.75) mu-=0.5*(mu-0.5);
            else mu-=0.5*(1-mu);
            /* h = |result2 - result1|, ex = (result2 - result1) / |result2 - result1| */
            ex = vdiff(*result2, *result1); // vector result1-result2
            h = vnorm(ex); // scalar result1-result2
            ex = vdiv(ex, h); // unit vector ex with respect to result1 (new coordinate system)
            /* t2 points to the intersection */
            t2 = vmul(ex, mu*h);
            t2 = vsum(*result1, t2);
            /* the best solution = t2 */
            *best_solution = t2;
 
        } else {
 
            /* if both mu1 and mu2 are between 0 and 1 */
            /* result1-result2 line segment intersects sphere 4 at two points */
 
            //return ERR_TRIL_NEEDMORESPHERE;
 
            mu = mu1 + mu2;
            /* h = |result2 - result1|, ex = (result2 - result1) / |result2 - result1| */
            ex = vdiff(*result2, *result1); // vector result1-result2
            h = vnorm(ex); // scalar result1-result2
            ex = vdiv(ex, h); // unit vector ex with respect to result1 (new coordinate system)
            /* 50-50 error correction for mu */
            mu = 0.5*mu;
            /* t2 points to the intersection */
            t2 = vmul(ex, mu*h);
            t2 = vsum(*result1, t2);
            /* the best solution = t2 */
            *best_solution = t2;
        }
    }
    return TRIL_4SPHERES;
}
 
/* 
 * 此函数调用三边化以获得最佳解决方案。
 * 如果任何三个球体不能产生有效的解，然后增加每个距离以确保交叉发生。
 * 返回TRIL_3SPHERES或TRIL_4SPHERES之间的所选三边模式
 * 对于TRIL_3SPHERES，有两种解决方案：解决方案1和解决方案2
 * 对于TRIL_4SPHERES，只有一个解决方案：best_solution
 * nosolution_count = 找到交叉点之前的失败尝试次数,通过增加球体直径。
*/
int deca_3dlocate ( vec3d   *const solution1,
                    vec3d   *const solution2,
                    vec3d   *const best_solution,
                    int     *const nosolution_count,
                    double  *const best_3derror,
                    double  *const best_gdoprate,
                    vec3d p1, double r1,
                    vec3d p2, double r2,
                    vec3d p3, double r3,
                    vec3d p4, double r4,
                    int *combination)
{
    vec3d   o1, o2, solution, ptemp;
    vec3d   solution_compare1, solution_compare2;
    double  /*error_3dcompare1, error_3dcompare2,*/ rtemp;
    double  gdoprate_compare1, gdoprate_compare2;
    double  ovr_r1, ovr_r2, ovr_r3, ovr_r4;
    int     overlook_count, combination_counter;
    int     trilateration_errcounter, trilateration_mode34;
    int     success, concentric, result;
 
    trilateration_errcounter = 0;
    trilateration_mode34 = 0;
    combination_counter = 1; /* four spheres combination */
    *best_gdoprate = 1; /* put the worst gdoprate init */
    gdoprate_compare1 = 1; gdoprate_compare2 = 1;
    solution_compare1.x = 0; solution_compare1.y = 0; solution_compare1.z = 0;
 
    do {
        success = 0;
        concentric = 0;
        overlook_count = 0;
        ovr_r1 = r1; ovr_r2 = r2; ovr_r3 = r3; ovr_r4 = r4;
 
        do {
 
            result = trilateration(&o1, &o2, &solution, p1, ovr_r1, p2, ovr_r2, p3, ovr_r3, p4, ovr_r4, MAXZERO);
 
            switch (result)
            {
                case TRIL_3SPHERES: // 3 spheres are used to get the result
                    trilateration_mode34 = TRIL_3SPHERES;
                    success = 1;
                    break;
 
                case TRIL_4SPHERES: // 4 spheres are used to get the result
                    trilateration_mode34 = TRIL_4SPHERES;
                    success = 1;
                    break;
 
                case ERR_TRIL_CONCENTRIC:
                    concentric = 1;
                    break;
 
                default: // any other return value goes here
                    ovr_r1 += 0.10;
                    ovr_r2 += 0.10;
                    ovr_r3 += 0.10;
                    ovr_r4 += 0.10;
                    overlook_count++;
                    break;
            }
  
        } while (!success && (overlook_count <= CM_ERR_ADDED) && !concentric);
 
        if (success)
        {
            switch (result)
            {
            case TRIL_3SPHERES:
                *solution1 = o1;
                *solution2 = o2;
                *nosolution_count = overlook_count;
 
                combination_counter = 0;
                break;
 
            case TRIL_4SPHERES:
 
                /* calculate the new gdop */
                gdoprate_compare1   = gdoprate(solution, p1, p2, p3);
 
                /* compare and swap with the better result */
                if (gdoprate_compare1 <= gdoprate_compare2)
                {
 
                    *solution1 = o1;
                    *solution2 = o2;
                    *best_solution  = solution;
                    *nosolution_count = overlook_count;
                    *best_3derror   = sqrt((vnorm(vdiff(solution, p1))-r1)*(vnorm(vdiff(solution, p1))-r1) +
                                        (vnorm(vdiff(solution, p2))-r2)*(vnorm(vdiff(solution, p2))-r2) +
                                        (vnorm(vdiff(solution, p3))-r3)*(vnorm(vdiff(solution, p3))-r3) +
                                        (vnorm(vdiff(solution, p4))-r4)*(vnorm(vdiff(solution, p4))-r4));
                    *best_gdoprate  = gdoprate_compare1;
 
                    /* save the previous result */
                    solution_compare2 = solution_compare1;
                    gdoprate_compare2 = gdoprate_compare1;
                    *combination = 5 - combination_counter;
 
                }
 
                ptemp = p1; p1 = p2; p2 = p3; p3 = p4; p4 = ptemp;
                rtemp = r1; r1 = r2; r2 = r3; r3 = r4; r4 = rtemp;
                combination_counter--;
                if(combination_counter<0)
                {
                    combination_counter=0;
                }
                break;
 
            default:
                break;
            }
        }
        else
        {
            trilateration_errcounter++;
            combination_counter--;
            if(combination_counter<0)
            {
               combination_counter=0;
            }
        }
 
    } while (combination_counter);
 
    //如果它给出了所有4个球体组合的错误，则没有给出有效结果，否则返回使用的三边形模式
    if (trilateration_errcounter >= 4)
        return -1;
    else
        return trilateration_mode34;
}

int GetLocation(vec3d *best_solution, int use4thAnchor, vec3d* anchorArray, int *distanceArray)
{
 
    vec3d   o1, o2, p1, p2, p3, p4;
    double  r1 = 0, r2 = 0, r3 = 0, r4 = 0, best_3derror, best_gdoprate;
    int     result;
    int     error, combination;
 
    if (use4thAnchor == 0)//3基站定位
    {
        if(distanceArray[3]==0)//A3无效 A0 A1 A2有效，执行3基站定位
        {
            /* Anchors coordinate */
            p1.x = anchorArray[0].x;        p1.y = anchorArray[0].y;    p1.z = anchorArray[0].z;
            p2.x = anchorArray[1].x;        p2.y = anchorArray[1].y;    p2.z = anchorArray[1].z;
            p3.x = anchorArray[2].x;        p3.y = anchorArray[2].y;    p3.z = anchorArray[2].z;
            p4.x = p1.x;                    p4.y = p1.y;                p4.z = p1.z;
 
            r1 = (double) distanceArray[0] / 1000.0;
            r2 = (double) distanceArray[1] / 1000.0;
            r3 = (double) distanceArray[2] / 1000.0;
            r4 = r1;
        }
        else if(distanceArray[0]==0)//A0无效 A1 A2 A3有效，执行3基站定位
        {
            /* Anchors coordinate */
            p1.x = anchorArray[1].x;        p1.y = anchorArray[1].y;    p1.z = anchorArray[1].z;
            p2.x = anchorArray[2].x;        p2.y = anchorArray[2].y;    p2.z = anchorArray[2].z;
            p3.x = anchorArray[3].x;        p3.y = anchorArray[3].y;    p3.z = anchorArray[3].z;
            p4.x = p1.x;                    p4.y = p1.y;                p4.z = p1.z;
 
            r1 = (double) distanceArray[1] / 1000.0;
            r2 = (double) distanceArray[2] / 1000.0;
            r3 = (double) distanceArray[3] / 1000.0;
            r4 = r1;
        }
        else if(distanceArray[1]==0)//A1无效 A0 A2 A3有效，执行3基站定位
        {
            /* Anchors coordinate */
            p1.x = anchorArray[0].x;        p1.y = anchorArray[0].y;    p1.z = anchorArray[0].z;
            p2.x = anchorArray[2].x;        p2.y = anchorArray[2].y;    p2.z = anchorArray[2].z;
            p3.x = anchorArray[3].x;        p3.y = anchorArray[3].y;    p3.z = anchorArray[3].z;
            p4.x = p1.x;                    p4.y = p1.y;                p4.z = p1.z;
 
            r1 = (double) distanceArray[0] / 1000.0;
            r2 = (double) distanceArray[2] / 1000.0;
            r3 = (double) distanceArray[3] / 1000.0;
            r4 = r1;
        }
        else if(distanceArray[2]==0)//A2无效 A0 A1 A3有效，执行3基站定位
        {
            /* Anchors coordinate */
            p1.x = anchorArray[0].x;        p1.y = anchorArray[0].y;    p1.z = anchorArray[0].z;
            p2.x = anchorArray[1].x;        p2.y = anchorArray[1].y;    p2.z = anchorArray[1].z;
            p3.x = anchorArray[3].x;        p3.y = anchorArray[3].y;    p3.z = anchorArray[3].z;
            p4.x = p1.x;                    p4.y = p1.y;                p4.z = p1.z;
 
            r1 = (double) distanceArray[0] / 1000.0;
            r2 = (double) distanceArray[1] / 1000.0;
            r3 = (double) distanceArray[3] / 1000.0;
            r4 = r1;
        }
    }
    else//4基站定位
    {
        /* Anchors coordinate */
        p1.x = anchorArray[0].x;        p1.y = anchorArray[0].y;    p1.z = anchorArray[0].z;
        p2.x = anchorArray[1].x;        p2.y = anchorArray[1].y;    p2.z = anchorArray[1].z;
        p3.x = anchorArray[2].x;        p3.y = anchorArray[2].y;    p3.z = anchorArray[2].z;
        p4.x = anchorArray[3].x;        p4.y = anchorArray[3].y;    p4.z = anchorArray[3].z;
 
 
        r1 = (double) distanceArray[0] / 1000.0;
        r2 = (double) distanceArray[1] / 1000.0;
        r3 = (double) distanceArray[2] / 1000.0;
        r4 = (double) distanceArray[3] / 1000.0;
 
    }
     
 
    result = deca_3dlocate (&o1, &o2, best_solution, &error, &best_3derror, &best_gdoprate,
                            p1, r1, p2, r2, p3, r3, p4, r4, &combination);
 
    if (result >= 0)
    {
        if(o1.z <= o2.z) best_solution->z = o1.z; else best_solution->z = o2.z;
        if (use4thAnchor == 0 || result == TRIL_3SPHERES)
        {
            if(o1.z < p1.z) *best_solution = o1; else *best_solution = o2; //assume tag is below the anchors (1, 2, and 3)
        }
 
        return result;
 
    }
 
    return -1;
}



void float2u8Arry(uint8_t *u8Arry, float *floatdata1, float *floatdata2, float *floatdata3, float *floatdata4)
{
  uint8_t farray1[4], farray2[4], farray3[4], farray4[4];
	
  *(float *)farray1 = *floatdata1;
  *(float *)farray2 = *floatdata2;
  *(float *)farray3 = *floatdata3;
  *(float *)farray4 = *floatdata4;
	
  u8Arry[17] = farray4[0];
  u8Arry[16] = farray4[1];
  u8Arry[15] = farray4[2];
  u8Arry[14] = farray4[3];	
  u8Arry[13] = farray3[0];
  u8Arry[12] = farray3[1];
  u8Arry[11] = farray3[2];
  u8Arry[10] = farray3[3];	
  u8Arry[9] = farray2[0];
  u8Arry[8] = farray2[1];
  u8Arry[7] = farray2[2];
  u8Arry[6] = farray2[3];	
  u8Arry[5] = farray1[0];
  u8Arry[4] = farray1[1];
  u8Arry[3] = farray1[2];
  u8Arry[2] = farray1[3];
  u8Arry[1] = 0x12;
  u8Arry[0] = 0x2C;

//   HAL_UART_Transmit(&huart1, u8Arry, 19, 0xffff);
}

uint8_t Trilateration(int distance1, int distance2, int distance3, int distance4, float real_distance, int logal)
{
    int result = 0; 
    vec3d anchorArray[4];
    vec3d report;
    int Range_deca[4];
	float X1 = 0.0, Y1 = 0.0, Z1 = 0.0;
	float distance;
	distance = real_distance;
	
	uint8_t test_buf[19];
	
    switch (logal){
			case 0:
				test_buf[18] = 0x0A; break;
			case 1:
				test_buf[18] = 0x0B; break;
			case 2:
				test_buf[18] = 0x0C; break;
			case 3:
				test_buf[18] = 0x0D; break;
			default:  break;
	}
	
	if(logal == 4)
	{
        float2u8Arry(test_buf, &X1, &Y1, &Z1, &distance);
        return 0;
	}
	
    Range_deca[0] = distance1;
    Range_deca[1] = distance2;
    Range_deca[2] = distance3;
    Range_deca[3] = distance4;
	
    //A0 
    anchorArray[0].x =  5.0; //anchor0.x uint:m
    anchorArray[0].y =  4.5; //anchor0.y uint:m
    anchorArray[0].z =  1.52; //anchor0.z uint:m

    //A1
    anchorArray[1].x =  0; //anchor2.x uint:m
    anchorArray[1].y =  4.5; //anchor2.y uint:m
    anchorArray[1].z =  1.52; //anchor2.z uint:m
 
    //A2
    anchorArray[2].x = 5.00; //anchor2.x uint:m
    anchorArray[2].y = 0.00; //anchor2.y uint:m
    anchorArray[2].z = 1.52; //anchor2.z uint:m
 
    //A3
    anchorArray[3].x =  0.00; //anchor2.x uint:m
    anchorArray[3].y =  0.00; //anchor2.y uint:m
    anchorArray[3].z =  1.52; //anchor2.z uint:m

//  result = GetLocation(&report, 1, &anchorArray[0], &Range_deca[0]);
    result = GetLocation(&report, 1, &anchorArray[0], &Range_deca[0]);
    
	uwb_data.x = (float)report.x;
	uwb_data.y = (float)report.y;
	uwb_data.z = (float)report.z;
	
    
	xx = uwb_data.x;//方便调试
	yy = uwb_data.y;
	zz = uwb_data.z;
    xQueueSendFromISR(UWB_RxPort, &uwb_data, 0);
    return 1;
}
