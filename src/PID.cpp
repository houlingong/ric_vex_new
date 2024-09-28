#include "PID.h"

#include <math.h>

#include <iostream>

#include "calc.h"
#include "geometry.h"
#include "my-timer.h"
#include "vex.h"
using namespace std;

/**
 * 本文件定义PID控制器
 * 需完成以下内容并在注释中完整解释：
 * PID::update()
 * DirPID::update()
 * PosPID::update()
 * 
 * 样例使用方法：
 * PID pid;
 * pid.setTarget(100);
 * pid.setErrorTolerance(1);
 * pid.setCoefficient(1, 0.1, 0.1);
 * while(!pid.taragetArrived()){
 *      pid.update(curr_value);
 *      output = pid.getOutput();
 *      // do something with output
 * }
 * 
 */

PID::PID() : first_time(true), arrived(false), I_max(20), I_range(50), jump_time(50), D_tol(10){ my_timer.reset(); }

void PID::setFirstTime() { first_time = true; }

void PID::setCoefficient(double _kp, double _ki, double _kd) {
    kp = _kp;
    ki = _ki;
    kd = _kd;
}
void PID::setTarget(double _target) { target = _target; }
void PID::setIMax(double _IMax) { I_max = _IMax; }
void PID::setIRange(double _IRange) { I_range = _IRange; }
void PID::setErrorTolerance(double _errorTol) { error_tol = _errorTol; }
void PID::setDTolerance(double _DTol) { D_tol = _DTol; }
void PID::setJumpTime(double _jumpTime) { jump_time = _jumpTime; }
void PID::setArrived(bool _arrived) { arrived = _arrived; }
bool PID::targetArrived() { return arrived; }
double PID::getOutput() { return output; }

void PID::update(double input) {
    error_curt = target - input;  // calculate current error
    //TODO: calculate the contribution of P, I, D with kp, ki, kd
    
    //因为个人对于多线程下的时间操作没有充足把握，因此该程序中对于时间的部分可能不正确
    jump_time=my_timer.getTime();

    //对于P，计算方式为目标值与当前值之差乘以kp
    P = kp*error_curt;

    //对于I，按照个人对于代码的理解，应当使用PID的离散化形式，时间间隔为jump_time
    //离散化后用小矩形面积近似曲线与坐标轴围成的面积，公式为\sum error_curt*jump_time
    
    //文件PID.h中给出的条件为“I < abs(IMAX); I starts to increase when P < IRangef”
    //个人认为上述条件有误；第一个条件：IMAX自然是非负数（其他文件如auto_functions可以证明这一点）
    //第二个条件：应当改为当P的绝对值小于IRange时
    if (abs(P) < I_range)
        I += ki * error_curt * jump_time;
    if (abs(I) > I_max)
        I = sign(I)*I_max;
    
    //对于D，同上述I的分析，用曲线割线近似曲线切线，公式为error_curt/jump_time
    D = kd * (error_curt - error_prev)/ jump_time;
    

    //更新error_curt以及时间
    error_prev=error_curt;
    my_timer.reset();

    if (abs(error_curt) <= error_tol) {  // Exit when staying in tolerated region and
                                        // maintaining a low enough speed
        arrived = true;
    }
    output = P + I + D;
}

void PosPID::setTarget(Point p) { target_point = p; }

void PosPID::update(Point input) {
    Vector err = target_point - input;
    error_curt = err.mod();  // calculate current error
    //TODO: calculate the contribution of P, I, D with kp, ki, kd
    
    //PID的数学公式及实现思路见函数update，在此不再赘述
    P = kp*error_curt;
    if (abs(P) < I_range)
        I += ki * error_curt * jump_time;
    if (abs(I) > I_max)
        I = sign(I)*I_max;
    D = kd * (error_curt - error_prev)/ jump_time;

    //更新error_curt
    error_prev=error_curt;
    
    if (abs(error_curt) <= error_tol) {  // Exit when staying in tolerated region and
                                        // maintaining a low enough speed
        arrived = true;
    }
    output = P + I + D;
}

void DirPID::update(double input) {
    error_curt = degNormalize(target - input);  // calculate current error
    //TODO: calculate the contribution of P, I, D with kp, ki, kd

    //PID的数学公式及实现思路见函数update，在此不再赘述
    P = kp*error_curt;
    if (abs(P) < I_range)
        I += ki * error_curt * jump_time;
    if (abs(I) > I_max)
        I = sign(I)*I_max;
    D = kd * (error_curt - error_prev)/ jump_time;
    
    //更新error_curt
    error_prev=error_curt;

    if (abs(error_curt) <= error_tol) {  // Exit when staying in tolerated region and
                                        // maintaining a low enough speed
        arrived = true;
    }
    output = P + I + D;
}
