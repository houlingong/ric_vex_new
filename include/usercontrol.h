#ifndef USERCONTROL_H_
#define USERCONTROL_H_
#include "vex.h"

// basic functions
void baseControl();
void baseStopControl();
void intakerControl();

// 定义面试要求的控制电机正转/反转函数
// 电机对象定义见robot-config
void interviewtask();

// main function
void userControl();

#endif