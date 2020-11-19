#include<ctime>
#include<iostream>
#include<math.h>
#include<Eigen/Core>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/InertialUnit.hpp>

using namespace webots;
using namespace Eigen;



class Motion_control
{
  public:
    //---构造函数---
    Motion_control();

    
    //---定义常量---
    const g = 9.8;
    const L1 = 70;
    const L2 = 50;
    const L3 = 24;
    const BL = 360;
    const BW = 80;
    const BODY_MASS = 1;

    //---定义变量---

      //---全局相关---
      double time_step;
      

      //---系数相关---
      double K_com_t_x,K_com_t_y,K_com_t_z;
      double D_com_t_x,D_com_t_y,D_com_t_z;
      double K_com_f_x,K_com_f_y,K_com_f_z;

      //---矩阵相关---
      MatrixXd transQ(6,6);
      MatrixXd transJ_rf(3,3);
      MatrixXd transJ_lh(3,3);

      //---期望相关(输入)---
      double roll_ang_d,pitch_ang_d,yaw_ang_d;
      double com_x_vel_d;


      //---机械结构相关---
      VectorXd rf_foot_pos(4),rh_foot_pos(4),lf_foot_pos(4),lh_foot_pos(4);
      VectorXd rf_shld_pos(4),rh_shld_pos(4),lf_shld_pos(4),lh_shld_pos(4);
      
      //---传感器相关---
      const double *imu_temp;
      VectorXd contect_flag(4);

      //---位置相关---
      double roll_ang,pitch_ang,yaw_ang;
      double pre_roll_ang;
      double pre_pitch_ang;
      double pre_yaw_ang;
      MatrixXd pre_joint_ang(4,3),joint_ang(4,3);

      double rf_pos_x,rf_pos_y,rf_pos_z;
      double rh_pos_x,rh_pos_y,rh_pos_z;
      double lf_pos_x,lf_pos_y,lf_pos_z;
      double lh_pos_x,lh_pos_y,lh_pos_z;

      //---速度相关---
      double roll_ang_vel,pitch_ang_vel,yaw_ang_vel;
      double pre_roll_ang_vel,pre_pitch_ang_vel,pre_yaw_ang_vel; 
      double roll_ang_vel_d,pitch_ang_vel_d,yaw_ang_vel_d;
      MatrixXd ang_vel(4,3),pre_ang_vel(4,3);

      double com_x_vel;

      //---加速度相关---

      //---质心力相关---
      double com_f_x,com_f_y,com_f_z;
      double com_t_x,com_t_y,com_t_z;
      double gvt_x,gvt_y,gvt_z;
      Vector6d com_F(6);

      //---足端力相关---
      double fr_foot_f_x,fr_foot_f_y,fr_foot_f_z,hd_foot_f_x,hd_foot_f_y,hd_foot_f_z;
      Vector6d foot_F_f_and_h(6);
      double rf_foot_f_x,rf_foot_f_y,rf_foot_f_z;
      double rh_foot_f_x,rh_foot_f_y,rh_foot_f_z;
      double lf_foot_f_x,lf_foot_f_y,lf_foot_f_z;
      double lh_foot_f_x,lh_foot_f_y,lh_foot_f_z;
      Vector3d rf_foot_F(3),rh_foot_F(3),lf_foot_F(3),lh_foot_F(3);
      
      //---关节力矩相关---
      Vector3d rf_jont_T(3),rh_jont_T(3),lf_jont_T(3),lh_jont_T(3);
      MatrixXd joint_T(4,3);

    

    
    //---机器人马达---
    const char* MOTOR_NAMES[4][3] = 
    {
      {"RF0", "RF1", "RF2"},
      {"RH0", "RH1", "RH2"},
      {"LF0", "LF1", "LF2"},
      {"LH0", "LH1", "LH2"}
    };

    //---机器人位置传感器---
    const char* SENSOR_NAMES[4][3] = 
    {
      {"RF0 sensor", "RF1 sensor", "RF2 sensor"},
      {"RH0 sensor", "RH1 sensor", "RH2 sensor"},
      {"LF0 sensor", "LF1 sensor", "LF2 sensor"},
      {"LH0 sensor", "LH1 sensor", "LH2 sensor"}
    };

    //---机器人触底传感器---
    const char* TOUCH_SENSOR_NAMES[4] = 
    {
      "RF touch sensor", "RH touch sensor", 
      "LF touch sensor", "LH touch sensor"
    };

    //---机器人质心传感器---
    const char* INERTIAL_UNIT = "inertial unit";
    const char* ACCELEROMETER = "accelerometer";

    //---webots初始化相关---
    webots_init();
  
    //---webots其他相关(set value)---
    webots_relate();
   
    //---姿态角速度计算---
    posture_vel_update();
    
    //---关节角速度计算---
    joint_vel_update();

    //---力矩赋值---
    joint_torque_update();

    //---四足位置计算---
    four_feet_position_update();
    
    //---四足触底平衡---
    four_feet_touch_balance();

  
  
    
    


};
