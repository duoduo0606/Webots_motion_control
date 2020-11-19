    #include "motion_control.h"
    
    Motion_control::Motion_control()
    {
      //---初始化---
      pre_roll_ang = 0;
      pre_pitch_ang = 0;
      pre_yaw_ang = 0;
      pre_ang_vel.setZero();
      pre_roll_ang_vel = 0;
      pre_pitch_ang_vel = 0;
      pre_yaw_ang_vel = 0; 
    }

  
    Motion_control::webots_init()
    {
      Motor *limb_motors[4][3];
      PositionSensor *limb_position_sensor[4][3];
      TouchSensor *limb_force_sensor[4];
      Accelerometer *acc;
      InertialUnit *imu;

      //---创建机器人---
      Robot *robot = new Robot();

      //---时间设置---
      int timeStep = (int)robot->getBasicTimeStep();

      //---质心传感器设置---
      imu = robot->getInertialUnit(INERTIAL_UNIT);
      acc = robot->getAccelerometer(ACCELEROMETER);

      //---质心传感器使能---
      imu -> enable(timeStep);
      acc -> enable(timestep);

      //---传感器设置和使能(get value and enable)---
      for(int i=0;i<4;i++)
        {
          for(int j=0;j<3;j++)
            {
              limb_motors[i][j] = robot->getMotor(MOTION_NAMES[i][j]);
              limb_motors[i][j]->setPosition(0);
              limb_position_sensor[i][j] = robot->getPositionSensor(SENSOR_NAMES[i][j]);
              limb_position_sensor[i][j]->enable(timeStep);
            }
            limb_force_sensor[i] = robot->getTouchSensor(TOUCH_SENSOR_NAMES[i]);
            limb_force_sensor[i]->enable(timeStep);
        }
    }//---end webots_init---


    //---webots其他相关(set value)---
    Motion_control::webots_relate()
    {
      for (int i=0; i<4; i++)
        {
          for (int j=0; j<3; j++)
            {
             joint_ang(i,j) = limb_position_sensor[i][j]->getValue();
            }
             contect_flag(i) = limb_force_sensor[i]->getValue();
        }
      
      imu_temp = imu->getRollPitchYaw();
      roll_ang = *imu_temp;
      pitch_ang = *(imu_temp+1);
      yaw_ang = *(imu_temp+2); 
      
    }


    //---姿态角速度计算---
    Motion_control::posture_vel_update()
    {
      roll_ang_vel = (roll_ang - pre_roll_ang)/time_step;
      pitch_ang_vel = (pitch_ang - pre_pitch_ang)/time_step;
      yaw_ang_vel = (yaw_ang - pre_yaw_ang)/time_step;

      roll_ang_vel = roll_ang_vel * 0.3 + pre_roll_ang_vel * 0.7; 
      pitch_ang_vel = pitch_ang_vel * 0.3 + pre_pitch_ang_vel * 0.7;
      yaw_ang_vel = yaw_ang_vel * 0.3 + pre_yaw_ang_vel * 0.7;

      pre_roll_ang = roll_ang;
      pre_pitch_ang = pitch_ang;
      pre_yaw_ang = yaw_ang; 

      pre_roll_ang_vel = roll_ang_vel;
      pre_pitch_ang_vel = pitch_ang_vel;
      pre_yaw_ang_vel = yaw_ang_vel;
    }


    //---关节角速度计算---
    Motion_control::joint_vel_update()
    {
      for(int i=0; i<4; i++)
        {  
          for(int j=0; j<3; j++)
            {
              ang_vel(i,j) = (joint_ang(i,j) - pre_joint_ang(i,j))/time_step;
              ang_vel(i,j) = ang_vel(i,j) * 0.3 + pre_ang_vel(i,j) *0.7;
              pre_joint_ang(i,j) = joint_ang(i,j);
              pre_ang_vel(i,j) = ang_vel(i,j); 
            } 
        } 
    }

    //---力矩赋值---
    Motion_control::joint_torque_update()
    {
       //---力矩值整合---

       joint_T << rf_jont_T,rh_jont_T,lf_jont_T,lh_jont_T;

       //---力矩值输入--- 
       for(int i=0; i<4; i++)
        {  
          for(int j=0; j<3; j++)
            {
              limb_motors[i][j]->setTorque(joint_T(i,j)); 
            } 
        } 
    }

    //---四足位置计算---
    Motion_control::four_feet_position_update()
    {
        //RF
        rf_pos_x =  L2 * cos(joint_ang(0,1) + joint_ang(0,2)) 
                   +L1 * sin(joint_ang(0,1));
        rf_pos_y = -L1 * cos(joint_ang(0,0)) * cos(joint_ang(0,1)) 
                   +L3 * sin(joint_ang(0,0))
                   +L2 * cos(joint_ang(0,0)) * cos(joint_ang(0,1)) * sin(joint_ang(0,2))
                   +L2 * cos(joint_ang(0,0)) * sin(joint_ang(0,1)) * sin(joint_ang(0,2));
        rf_pos_z = -L1 * sin(joint_ang(0,0)) * cos(joint_ang(0,1)) 
                   -L3 * cosjoint_ang(0,0)) 
                   +L2 * sin(joint_ang(0,0)) * cos(joint_ang(0,1)) * sin(joint_ang(0,2))
                   +L2 * sin(joint_ang(0,0)) * sin(joint_ang(0,1)) * cos(joint_ang(0,2)); 
        //RH
        rh_pos_x = -L2 * cos(joint_ang(0,1) + joint_ang(0,2)) 
                   +L1 * sin(joint_ang(0,1));
        rh_pos_y = -L1 * cos(joint_ang(0,0)) * cos(joint_ang(0,1)) 
                   +L3 * sin(joint_ang(0,0)) 
                   -L2 * cos(joint_ang(0,0)) * cos(joint_ang(0,1)) * sin(joint_ang(0,2))
                   -L2 * cos(joint_ang(0,0)) * sin(joint_ang(0,1)) * sin(joint_ang(0,2));
        rh_pos_z = -L1 * sin(joint_ang(0,0)) * cos(joint_ang(0,1)) 
                   -L3 * cosjoint_ang(0,0)) 
                   -L2 * sin(joint_ang(0,0)) * cos(joint_ang(0,1)) * sin(joint_ang(0,2))
                   -L2 * sin(joint_ang(0,0)) * sin(joint_ang(0,1)) * cos(joint_ang(0,2));  
        //LF
        lf_pos_x =  L2 * cos(joint_ang(0,1) + joint_ang(0,2)) 
                   -L1 * sin(joint_ang(0,1));
        lf_pos_y =  L1 * cos(joint_ang(0,0)) * cos(joint_ang(0,1)) 
                   +L3 * sin(joint_ang(0,0)) 
                   +L2 * cos(joint_ang(0,0)) * cos(joint_ang(0,1)) * sin(joint_ang(0,2))
                   +L2 * cos(joint_ang(0,0)) * sin(joint_ang(0,1)) * sin(joint_ang(0,2));
        lf_pos_z =  L1 * sin(joint_ang(0,0)) * cos(joint_ang(0,1)) 
                   -L3 * cosjoint_ang(0,0)) 
                   +L2 * sin(joint_ang(0,0)) * cos(joint_ang(0,1)) * sin(joint_ang(0,2))
                   +L2 * sin(joint_ang(0,0)) * sin(joint_ang(0,1)) * cos(joint_ang(0,2));  
        //LH
        lh_pos_x = -L2 * cos(joint_ang(0,1) + joint_ang(0,2)) 
                   -L1 * sin(joint_ang(0,1));
        lh_pos_y =  L1 * cos(joint_ang(0,0)) * cos(joint_ang(0,1)) 
                   +L3 * sin(joint_ang(0,0)) 
                   -L2 * cos(joint_ang(0,0)) * cos(joint_ang(0,1)) * sin(joint_ang(0,2))
                   -L2 * cos(joint_ang(0,0)) * sin(joint_ang(0,1)) * sin(joint_ang(0,2));
        lh_pos_z =  L1 * sin(joint_ang(0,0)) * cos(joint_ang(0,1)) 
                   -L3 * cosjoint_ang(0,0)) 
                   -L2 * sin(joint_ang(0,0)) * cos(joint_ang(0,1)) * sin(joint_ang(0,2))
                   -L2 * sin(joint_ang(0,0)) * sin(joint_ang(0,1)) * cos(joint_ang(0,2)); }


    //---四足触地平衡---
    Motion_control::four_feet_touch_balance()
    {
        //---赋初值---

        //---计算质心广义力---
        com_t_x = K_com_t_x * (roll_ang_d - roll_ang) + D_com_t_y * (roll_ang_vel_d - roll_ang_vel); 
        com_t_y = K_com_t_y * (pitch_ang_d - pitch_ang) + D_com_t_y * (pitch_ang_vel_d - pitch_ang_vel);
        com_f_z = K_com_f_z * ()
        
        
        com_f_x = K_com_f_x * (com_x_vel_d - com_x_vel);
        com_t_z = K_com_t_z * (yaw_ang_vel_d - yaw_ang_vel);

        //---计算重力---
        gvt_x = body_mass * g * sin(pitch_ang);
        gvt_y = 0;
        gvt_z = body_mass * g * cos(pitch_ang);
      
        //---广义力向量---
        com_F(0) = com_f_x - gvt_x;
        com_F(1) = 0;
        com_F(2) = com_f_z - gvt_z;
        com_F(3) = com_t_x;
        com_F(4) = com_t_y;
        com_F(5) = com_t_z;

        //---Q矩阵---
        
        transQ << 1,0,0,1,0,0,
                  0,1,0,0,1,0,
                  0,0,1,0,0,1,
                  0,-rf_pos_z,rf_pos_y,0,-lh_pos_z,lh_pos_y,
                  rf_pos_z,0,-rf_pos_x,lh_pos_z,0,-lh_pos_x,
                  -rf_pos_y,rf_pos_x,0,-lh_pos_y,lh_pos_x,0;
        

        //---足端力求解---
        
        foot_F_f_and_h = transQ.llt().Solve(com_F);

        //---足端力关节力矩转换---

        rf_foot_F = foot_F_f_and_h.head(3);
        lh_foot_F = foot_F_f_and_h.tail(3);

        transJ_rf = , , 
                    , , 
                    , , ;

        transJ_lh = , ,
                    , , 
                    , , ;

        rf_jont_T = transJ_rf.transpose() * rf_foot_F;
        lh_jont_T = transJ_lh.transpose() * lh_foot_F;

    }

