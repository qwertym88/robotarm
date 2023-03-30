#include "servo.h"
#include <math.h>
#include <stdlib.h>

#define pi 3.1415926

// 机械臂参数
#define a1 17.404 // 大臂到底盘圆心距离
#define a2 148.0  // 大臂长度
#define a3 160.0  // 小臂长度
#define a4 35.5   // 机械爪相对小臂末端伸出长度
#define a5 97.0   // 大臂转轴相对底座底部的高度

#define a6 20.0 // 面向结果的编程，详见报告

// 机械臂实际处于角(0,90,0)即大臂竖直小臂水平时各个舵机的角度值。装配时或许会歪了，导致有些许偏差。
#define dagl1 0
#define dagl2 90
#define dagl3 0

// 每个插补点之间的时间，单位ms。考虑舵机运行速度之后大概选取的一个数，虽然没太严谨的理论依据，但实际效果不错。想来至少得3*20ms吧。
const double dt = 60;

// 存储插补点的信息，事实上150就足够了，基本用不着开那么大的数组
uint16_t pwmAry[305][3];
int posQueTop, posQueBtm;

/**
 * @brief angle_t默认为角度，为方便使用，提供一个弧度转角度的函数
 * @param agl 起始角度
 * @param type 模式，type==1表示弧度转角度，==0表示角度转弧度
 * @retval 转换结果
 */
angle_t a2a(angle_t agl, uint8_t type)
{
    angle_t res;
    if (type == 1)
    {
        res.j1 = agl.j1 / pi * 180.0;
        res.j2 = agl.j2 / pi * 180.0;
        res.j3 = agl.j3 / pi * 180.0;
    }
    else
    {
        res.j1 = agl.j1 * pi / 180.0;
        res.j2 = agl.j2 * pi / 180.0;
        res.j3 = agl.j3 * pi / 180.0;
    }
    return res;
}

/**
 * @brief 检查机械臂能否到达该关节角度
 * @param agl 关节角度
 * @retval 关节是否能旋转到这组角度，能返回1
 */
uint8_t angleCheck(angle_t agl)
{
    double E = 1e-3;
    if (-90 - E <= agl.j1 && agl.j1 <= 90 + E)
        if (0 - E <= agl.j2 && agl.j2 <= 180 + E)
            if (0 - E <= agl.j3 && agl.j3 <= 180 + E)
                if (agl.j2 + agl.j3 <= 160 - E)
                    return 1;
    return 0;
}

/**
 * @brief 检查机械臂末端能否到达该坐标
 * @param pos 坐标
 * @retval 机械臂末端能否到达该坐标，能返回1
 */
uint8_t pointCheck(point_t pos)
{
    angle_t an = pointToAngle(pos);
    return angleCheck(an);
}

/**
 * @brief 通过坐标求关节角度
 * @param pos 坐标
 * @retval 该坐标点对应的关节角度
 */
angle_t pointToAngle(point_t pos)
{
    angle_t res;
    double x = pos.x, y = pos.y, z = pos.z;
    double alpha, beta, gamma;

    res.j1 = atan2(y, x);
    double b = sqrt(x * x + y * y) - a1 - a4; // base
    double sz = z - a5 + a6;                  // z short side
    double h = sqrt(b * b + sz * sz);         // hypotenuse

    beta = acos((a2 * a2 + h * h - a3 * a3) / (2 * a2 * h));
    gamma = acos((a2 * a2 - h * h + a3 * a3) / (2 * a2 * a3));
    alpha = atan(sz / b); // 这里alpha本来就可以是负数，不用专门变成正数
    res.j2 = alpha + beta;
    res.j3 = pi - alpha - beta - gamma;

    res = a2a(res, 1);
    return res;
}

/**
 * @brief 方便快捷地将旋转角度转换成坐标
 * @param agl 关节角度
 * @retval 该角度对应的坐标点
 */
point_t angleToPoint(angle_t agl)
{
    point_t res;
    angle_t an = a2a(agl, 0);
    double j1 = an.j1, j2 = an.j2, j3 = an.j3;
    double h, b;
    double gamma, beta, alpha;

    gamma = pi - j2 - j3;
    h = sqrt(a3 * a3 + a2 * a2 - 2 * a2 * a3 * cos(gamma));
    beta = acos((a2 * a2 + h * h - a3 * a3) / (2 * a2 * h));
    alpha = j2 - beta;
    b = h * cos(alpha);
    res.z = h * sin(alpha) + a5 - a6;
    res.x = (b + a1 + a4) * cos(j1);
    res.y = (b + a1 + a4) * sin(j1);
    return res;
}

/**
 * @brief 直接转动到指定关节角度
 * @param agl 需要转到的关节角度，舵机转动到对应的角度
 */
uint8_t movea(angle_t agl)
{
    // 每个坐标计算得到的理论角度加上一点点偏移才是实际应该转动的角度，这是由最开始装舵机时产生的，可能当时没有装正
    // 注意电源或许无法提供三个舵机同时转动所需功率，此时会抽搐着转动甚至于不动，增加电源功率自然能解决问题。考虑通用性这里加了延迟。
    // 该函数的本意是不做任何多余处理地、直接地到达某个角度，由于以上问题，实际表现并不好。无奈之下背离初衷，用下面的替代了。

    if (angleCheck(agl) == 1)
    {
        JOINT1_TURN2(agl.j1 + dagl1);
        HAL_Delay(dt / 3);
        JOINT2_TURN2((agl.j2 + dagl2 - 90));
        HAL_Delay(dt / 3);
        JOINT3_TURN2(agl.j3 + dagl3);
        HAL_Delay(dt / 3);
        return 1;
    }
    return 0;
}

/**
 * @brief 直接转动到指定关节角度
 * @param beg 出发点舵机角度
 * @param end 终点舵机角度
 */
uint8_t movea_s(angle_t beg, angle_t end)
{
    // 自动调整等待时间，确保功率足够

    if (angleCheck(end) == 1)
    {
        JOINT1_TURN2(end.j1 + dagl1);
        HAL_Delay(dt * abs((int)(end.j1 - beg.j1)) / 15);
        JOINT2_TURN2(end.j2 + dagl2 - 90);
        HAL_Delay(dt * abs((int)(end.j2 - beg.j2)) / 15);
        JOINT3_TURN2(end.j3 + dagl3);
        HAL_Delay(dt * abs((int)(end.j3 - beg.j3)) / 15);
        return 1;
    }
    return 0;
}

/**
 * @brief 正弦加减速规划,用了五次插值公式
 * @param p1 起始坐标
 * @param p2 终止坐标
 * @param V 运动速度，单位mm/s，一般取60就行
 */
uint8_t quintic_polynomial(point_t p1, point_t p2, double V)
{
    /*
        插补原理介绍：
        要在直线运动中实现慢-快-慢的过程，最简单的想法就是选定一组numda，让它在0-1的区间内大致分布在两端，例如[0.1,0.15,0.25,0.4,0.6,0.75,0.85,0.9]这样的一组
        已知路程S、速度V，可求得总时间T。相邻插值点之间运动时间设为dt，则共可以插入N=T/dt个点。
        f(x) = k0 + k1*x + k2*x^2 + k3*x^3 + k4*x^4 + k5*x^5，构造恰当的k，使其在0~T大致成s形，值域为[0,1]
        for i in range (0,N)，代入xi=i*dt，则可得符合条件的一组 numda={f(x1),...f(xi)}
        依次经过(x0 + numda_i*dx,y0 + numda_i*dy,z0 + numda_i*dz)，即沿直线慢-快-慢地运动。

        补充说明：
        其实插补有挺多种形式的，大体思路都跟这个差不多。舵机比较特殊，感觉是不需要考虑什么加速度和速度平滑，所以选择面很多。采用五次曲线只是通用而已，事实上梯形插补会更好，毕竟算的快。
        这个计算过程挺慢的，至少绝对比舵机在插补点间运动的时间长。因此需要先算完再动，不然会很慢，且绝大多数时间都是在等计算结果。
        别在这个for循环里或机械臂动的过程中printf什么东西，会严重拖累计算/运动，看着像bug一样。别测来测去发现是自己用来调试的printf语句出的问题（别问我怎么知道的）。
    */

    if (pointCheck(p2) == 0)
        return 0;

    p1.z += (sqrt(p1.x * p1.x + p1.y * p1.y) - 213) / 10; // 面向结果的编程，详见报告
    p2.z += (sqrt(p2.x * p2.x + p2.y * p2.y) - 213) / 10;
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dz = p2.z - p1.z;
    double S = sqrt(dx * dx + dy * dy + dz * dz); // 总路程
    double T = S / V;                             // 总用时
    int N = T * 1000 / dt;                        // 插值个数
    if (N > 300)                                  // 最多插入300个点
    {
        N = 300;
        T = N * dt;
    }
    double k0 = 0, k1 = 0, k2 = 0, k3 = 10 / pow(T, 3), k4 = -15 / pow(T, 4), k5 = 6 / pow(T, 5); // 五次插值公式
    for (int i = 0; i < N; i++)
    {
        double t = (double)i * dt / 1000.0;                                                             // 将一个连续的五次函数拆分成N个相隔dt的值
        double numda = k0 + k1 * t + k2 * pow(t, 2) + k3 * pow(t, 3) + k4 * pow(t, 4) + k5 * pow(t, 5); // 计算相对增量
        point_t nxtpos = {p1.x + numda * dx, p1.y + numda * dy, p1.z + numda * dz};                     // 计算每个插入点的坐标
        angle_t an = pointToAngle(nxtpos);                                                              // 将插入点坐标转成角度
        if (angleCheck(an) == 1)
        {
            pwmAry[i][0] = 500 + (an.j1 + 90 + dagl1) * 11.1111; // 提前转成pwm值，转的时候流畅些
            pwmAry[i][1] = 500 + (an.j2 + dagl2 - 90) * 11.1111;
            pwmAry[i][2] = 500 + (an.j3 + dagl3) * 11.1111;
        }
        else
            return 0;
    }
    angle_t an = pointToAngle(p2);
    pwmAry[N][0] = 500 + (an.j1 + 90 + dagl1) * 11.1111;
    pwmAry[N][1] = 500 + (an.j2 + dagl2 - 90) * 11.1111;
    pwmAry[N][2] = 500 + (an.j3 + dagl3) * 11.1111;
    posQueTop = 0, posQueBtm = N + 1;
    return 1;
}

/**
 * @brief 两点之间的直线运动，运用五次插值公式插补
 * @param p1 起始坐标
 * @param p2 终止坐标
 * @param V 运动速度，单位mm/s，一般取60就行
 */
uint8_t move(point_t ori, point_t dst, double V)
{
    uint8_t t = dt / 3;
    if (quintic_polynomial(ori, dst, V) == 1)
    {
        for (int i = posQueTop; i < posQueBtm; i++)
        {
            // 注意电源或许无法提供三个舵机同时转动所需功率，此时会抽搐着转动甚至于不动，增加电源功率自然能解决问题。考虑通用性这里加了延迟。
            // 注：其实这里加不加都无所谓，因为插补点特别密集，舵机基本不动，所以把延迟去了也不会影响。但绝不代表上述问题不存在。

            JOINT1_PWM(pwmAry[i][0]);
            HAL_Delay(t);
            JOINT2_PWM(pwmAry[i][1]);
            HAL_Delay(t);
            JOINT3_PWM(pwmAry[i][2]);
            HAL_Delay(t);
        }
        return 1;
    }
    return 0;
}

/**
 * @brief 两点间曲线运动，运用贝塞尔曲线插补
 * @param P0~P3 三次贝塞尔曲线的四个控制点
 * @param T 运动时间，单位ms
 */
uint8_t bezier(point_t P0, point_t P1, point_t P2, point_t P3, double T)
{
    int N = T / dt; // 插值个数
    if (N > 300)    // 最多插入300个点
        N = 300;
    double numda = 1 / (double)N;
    double t = 0;
    for (int i = 0; i < N; i++)
    {
        // B(t) = P0(1-t)^3 + 3*P1*t*(1-t)^2 + 3*P2*t^2*(1-t) + P3*t^3

        point_t nxtpos;
        nxtpos.x = P0.x * pow(1 - t, 3) + 3 * P1.x * t * pow(1 - t, 2) + 3 * P2.x * pow(t, 2) * (1 - t) + P3.x * pow(t, 3);
        nxtpos.y = P0.y * pow(1 - t, 3) + 3 * P1.y * t * pow(1 - t, 2) + 3 * P2.y * pow(t, 2) * (1 - t) + P3.y * pow(t, 3);
        nxtpos.z = P0.z * pow(1 - t, 3) + 3 * P1.z * t * pow(1 - t, 2) + 3 * P2.z * pow(t, 2) * (1 - t) + P3.z * pow(t, 3);
        angle_t an = pointToAngle(nxtpos); // 将插入点坐标转成角度
        if (angleCheck(an) == 1)
        {
            pwmAry[i][0] = 500 + (an.j1 + 90 + dagl1) * 11.1111; // 提前转成pwm值，转的时候流畅些
            pwmAry[i][1] = 500 + (an.j2 + dagl2 - 90) * 11.1111;
            pwmAry[i][2] = 500 + (an.j3 + dagl3) * 11.1111;
        }
        else
            return 0;
        t += numda;
    }
    angle_t an = pointToAngle(P3);
    pwmAry[N][0] = 500 + (an.j1 + 90 + dagl1) * 11.1111;
    pwmAry[N][1] = 500 + (an.j2 + dagl2 - 90) * 11.1111;
    pwmAry[N][2] = 500 + (an.j3 + dagl3) * 11.1111;
    posQueTop = 0, posQueBtm = N + 1;
    return 1;
}

/**
 * @brief 两点间曲线运动，运用贝塞尔曲线插补
 * @param ori 起始坐标
 * @param dst 终止坐标
 * @param op 控制点P1
 * @param dp 控制点P2
 * @param T 运动时间，单位ms
 */
uint8_t curve(point_t ori, point_t dst, point_t op, point_t dp, double T)
{
    uint8_t t = dt / 3;
    if (bezier(ori, op, dp, dst, T) == 1)
    {
        for (int i = posQueTop; i < posQueBtm; i++)
        {
            // 注意电源或许无法提供三个舵机同时转动所需功率，此时会抽搐着转动甚至于不动，增加电源功率自然能解决问题。考虑通用性这里加了延迟。
            // 注：其实这里加不加都无所谓，因为插补点特别密集，舵机基本不动，所以把延迟去了也不会影响。但绝不代表上述问题不存在。

            JOINT1_PWM(pwmAry[i][0]);
            HAL_Delay(t);
            JOINT2_PWM(pwmAry[i][1]);
            HAL_Delay(t);
            JOINT3_PWM(pwmAry[i][2]);
            HAL_Delay(t);
        }
        return 1;
    }
    return 0;
}
