#include<ctime>
#include<iostream>
#include<math.h>
#include<Eigen/Core>
#include<Eigen/Dense>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/InertialUnit.hpp>

using namespace std;
using namespace webots;
using namespace Eigen;



class Motion_control
{
  public:
    //---构造函数---
    Motion_control();

    //---定义常量---
    double g;
    double body_mass;
    double L1;
    double L2;
    double L3;
    double BL;
    double BW;
    
    //---定义变量---
    
      //---webots---
      Motor *limb_motors[4][3];
      PositionSensor *limb_position_sensor[4][3];
      TouchSensor *limb_force_sensor[4];
      Accelerometer *acc;
      InertialUnit *imu;
      Robot *robot;
      int timeStep;
            
      //---全局相关---
      double time_step;
      double t;
      double time_sup;
      double time_fly;
      int sinario;

      //---状态机---
      enum {Init, A_fly, A_hold, B_fly, B_hold} state;

      //---系数相关---
      double K_com_t_x,K_com_t_y,K_com_t_z;
      double D_com_t_x,D_com_t_y,D_com_t_z;
      double K_com_f_x,K_com_f_y,K_com_f_z;
      double D_com_f_z;
        //摆动项
        double fly_f_Kx,fly_f_Ky,fly_f_Kz;
        double fly_f_Dx,fly_f_Dy,fly_f_Dz;
        double fly_h_Kx,fly_h_Ky,fly_h_Kz;
        double fly_h_Dx,fly_h_Dy,fly_h_Dz;

      //---矩阵相关---
      MatrixXd transQ = MatrixXd::Zero(6,6);
      MatrixXd transQA = MatrixXd::Zero(6,6);
      MatrixXd transQB = MatrixXd::Zero(6,6);

      MatrixXd transJ_rf = MatrixXd::Zero(3,3);
      MatrixXd transJ_rh = MatrixXd::Zero(3,3);
      MatrixXd transJ_lf = MatrixXd::Zero(3,3);
      MatrixXd transJ_lh = MatrixXd::Zero(3,3);

      //---期望相关(输入)---
      double roll_ang_d,pitch_ang_d,yaw_ang_d;
      double roll_ang_vel_d,pitch_ang_vel_d,yaw_ang_vel_d;
      double com_vel_x_d,com_vel_y_d;
      double com_z_d;

      //---机械结构相关---
      VectorXd rf_foot_pos = VectorXd::Zero(4),rh_foot_pos = VectorXd::Zero(4),
               lf_foot_pos = VectorXd::Zero(4),lh_foot_pos = VectorXd::Zero(4);
      VectorXd rf_shld_pos = VectorXd::Zero(4),rh_shld_pos = VectorXd::Zero(4),
               lf_shld_pos = VectorXd::Zero(4),lh_shld_pos = VectorXd::Zero(4);
      
      //---传感器相关---
      const double *imu_temp;
      VectorXd contect_flag = VectorXd::Zero(4);

      //---位置相关---
      double roll_ang,pitch_ang,yaw_ang;
      double pre_roll_ang,pre_pitch_ang,pre_yaw_ang;
      double com_z;

      double rf_pos_x,rf_pos_y,rf_pos_z;
      double rh_pos_x,rh_pos_y,rh_pos_z;
      double lf_pos_x,lf_pos_y,lf_pos_z;
      double lh_pos_x,lh_pos_y,lh_pos_z;
      MatrixXd four_feet_position = MatrixXd::Zero(4,3);
      MatrixXd pre_four_feet_position = MatrixXd::Zero(4,3);

      MatrixXd joint_ang = MatrixXd::Zero(4,3);
      MatrixXd pre_joint_ang = MatrixXd::Zero(4,3);
        
        //摆动项
        double x0f,x0h,y0f,y0h;
        double xtf,xth,ytf,yth;
        double xcf,xch,ycf,ych;
        double fly_hight;
        double zcf,zch;
        double z0f,z0h;
        double pre_xcf,pre_ycf,pre_zcf;
        double pre_xch,pre_ych,pre_zch;

      //---速度相关---
      double roll_ang_vel,pitch_ang_vel,yaw_ang_vel;
      double pre_roll_ang_vel,pre_pitch_ang_vel,pre_yaw_ang_vel; 
      
      double com_vel_x,com_vel_y;
      double com_vel_z;
      VectorXd com_vel = VectorXd::Zero(3);
      VectorXd pre_com_vel = VectorXd::Zero(3);

      MatrixXd ang_vel = MatrixXd::Zero(4,3);
      MatrixXd pre_ang_vel = MatrixXd::Zero(4,3);

      VectorXd rf_lnr_vel = VectorXd::Zero(3);
      VectorXd rh_lnr_vel = VectorXd::Zero(3);
      VectorXd lf_lnr_vel = VectorXd::Zero(3);
      VectorXd lh_lnr_vel = VectorXd::Zero(3);

        //摆动项
        double x0fdot,x0hdot,y0fdot,y0hdot;
        double xcfdot,ycfdot,zcfdot;
        double xchdot,ychdot,zchdot;

      //---加速度相关---

      //---质心力相关---
      double com_f_x,com_f_y,com_f_z;
      double com_t_x,com_t_y,com_t_z;
      double gvt_x,gvt_y,gvt_z;
      VectorXd com_F = VectorXd::Zero(6);

      //---足端力相关---
      VectorXd foot_F_f_and_h_sup = VectorXd::Zero(6);
      VectorXd foot_F_f_and_h_fly = VectorXd::Zero(6);
      double rf_foot_f_x,rf_foot_f_y,rf_foot_f_z;
      double rh_foot_f_x,rh_foot_f_y,rh_foot_f_z;
      double lf_foot_f_x,lf_foot_f_y,lf_foot_f_z;
      double lh_foot_f_x,lh_foot_f_y,lh_foot_f_z;
      Vector3d rf_foot_F = Vector3d::Zero(3);
      Vector3d rh_foot_F = Vector3d::Zero(3);
      Vector3d lf_foot_F = Vector3d::Zero(3);
      Vector3d lh_foot_F = Vector3d::Zero(3);
      MatrixXd foot_F = MatrixXd::Zero(4,3);
      
      //---关节力矩相关---
      Vector3d rf_jont_T = Vector3d::Zero(3);
      Vector3d rh_jont_T = Vector3d::Zero(3);
      Vector3d lf_jont_T = Vector3d::Zero(3);
      Vector3d lh_jont_T = Vector3d::Zero(3);
      MatrixXd joint_T = MatrixXd::Zero(4,3);
      MatrixXd torque_add = MatrixXd::Zero(4,3);
  


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
    void webots_init();
  
    //---webots其他相关(关节角度 触地检测 姿态角度)---
    void webots_relate();

    //---触地状态检测---
    void touching_check();

    //---姿态角速度计算---
    void posture_vel_update();

    //---质心速度计算---
    void com_vel_update();
    
    //---关节角速度计算---
    void joint_vel_update();

    //---足端移动速度---
    void feet_vel_update();

    //---力矩赋值---
    void joint_torque_update();

    //---关节力矩补偿---
    void joint_torque_compensation_update();

    //---四足位置计算---
    void four_feet_position_update();

    //---支撑相力矩---
    void sup_torque(int sup_leg1, int sup_leg2, int fly_leg1, int fly_leg2);

    //---飞行相力矩---
    void fly_torque(int fly_leg1, int fly_leg2, int sup_leg1, int sup_leg2);
    
    //---测试一：姿态平衡(对角触地)---
    void test_diag_feet_balance();

    //---测试二：状态机(对角步态)---
    void test_state_machine_diag();

    //---测试三：交替站立---
    void test_cross_move();

    //---测试四：飞行项测试---
    void fly_torque_test();




  
  
    
    


};
