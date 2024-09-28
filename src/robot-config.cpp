#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
controller Controller = controller(primary);
inertial Inertial = inertial(PORT10);

// BaseMotor
motor motor_base_l1 = motor(PORT15, ratio6_1, false); //+
motor motor_base_l2 = motor(PORT16, ratio6_1, true);  //-
motor motor_base_l3 = motor(PORT17, ratio6_1, true);  //-
motor motor_base_l4 = motor(PORT18, ratio6_1, false); //+
motor motor_base_r1 = motor(PORT11, ratio6_1, false); //+
motor motor_base_r2 = motor(PORT12, ratio6_1, false); //+
motor motor_base_r3 = motor(PORT1, ratio6_1, true);   //-
motor motor_base_r4 = motor(PORT20, ratio6_1, true);  //-

// IntakerMotor
motor motor_intaker = motor(PORT19, ratio6_1, false); //+

// AS5600
analog_in AS5600_L = analog_in(Brain.ThreeWirePort.A);
analog_in AS5600_R = analog_in(Brain.ThreeWirePort.H);

// dightal_out control
// digital_out name = digital_out(Brain.ThreeWirePort.E);

// 定义面试要求的电机，端口号为21，默认为正转
// 按键响应见usercontrol
motor motor_base_interview = motor(PORT21, ratio6_1, false);
