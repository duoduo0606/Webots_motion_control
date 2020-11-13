    #include "motion_control.h"
    
    //---构造函数---
    Motion_control::Motion_control()
    {
      //---初始化---
      //pre值
      pre_roll_ang = 0;
      pre_pitch_ang = 0;
      pre_yaw_ang = 0;
      
      pre_roll_ang_vel = 0;
      pre_pitch_ang_vel = 0;
      pre_yaw_ang_vel = 0;

      pre_ang_vel.setZero();

      //期望值
      roll_ang_d = 0;
      pitch_ang_d = 0;
      yaw_ang_d = 0;

      roll_ang_vel_d = 0;
      pitch_ang_vel_d = 0;
      yaw_ang_vel_d = 0;
      
      com_vel_x_d = 0;
      com_vel_y_d = 0;
      com_z_d = -0.024;
      
      //全局变量
      g = 9.8;
      body_mass = 1;

      L1 = 0.07;
      L2 = 0.05;
      L3 = 0.024;
      BL = 0.26;
      BW = 0.1;
      
      //时间变量
      t = 0;
      
      //系数变量
      K_com_t_x = 0.0001;
      K_com_t_y = 0.0001;
      K_com_t_z = 0.0001;
      
      D_com_t_x = 0.00000001;
      D_com_t_y = 0.00000001;
      D_com_t_z = 0.00000001;
     
      K_com_f_x = 0.0001;
      K_com_f_y = 0.0001;
      K_com_f_z = 0.0001;
      D_com_f_z = 0.00000001;
    }

    //---webots初始化相关---
    void Motion_control::webots_init()
    {
      robot = new Robot();

      //时间设置
      timeStep = (int)robot->getBasicTimeStep();
      time_step = (float)timeStep/1000;

      //质心传感器设置
      imu = robot->getInertialUnit(INERTIAL_UNIT);
      acc = robot->getAccelerometer(ACCELEROMETER);

      //质心传感器使能
      imu -> enable(timeStep);
      acc -> enable(timeStep);

      //传感器设置和使能(get value and enable)
      for(int i=0;i<4;i++)
        {
          for(int j=0;j<3;j++)
            {
              limb_motors[i][j] = robot->getMotor(MOTOR_NAMES[i][j]);
              limb_motors[i][j]->setPosition(0);
              limb_position_sensor[i][j] = robot->getPositionSensor(SENSOR_NAMES[i][j]);
              limb_position_sensor[i][j]->enable(timeStep);
            }
            limb_force_sensor[i] = robot->getTouchSensor(TOUCH_SENSOR_NAMES[i]);
            limb_force_sensor[i]->enable(timeStep);
        }
    }//---end webots_init---

    //---webots其他相关(关节角度 触地检测 姿态角度)---
    void Motion_control::webots_relate()
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

    //---触地状态检测---
    void Motion_control::touching_check()
    {
        //暂时统计三种结果
        //sinario 0 RF LH
        if ((contect_flag(0) == 1) && (contect_flag(3) == 1) && (contect_flag.sum() == 2))
        {
            sinario = 0;
        }
        //sinario 1 RF LH + another leg
        if ((contect_flag(0) == 1) && (contect_flag(3) == 1) && (contect_flag.sum() == 3))
        {
            sinario = 1;
        }
        //sinario 2 four feet
        if (contect_flag.sum() == 4)
        {
            sinario = 2;
        }
        //sinario 3 RH LF
        if ((contect_flag(1) == 1) && (contect_flag(2) == 1) && (contect_flag.sum() == 2))
        {
            sinario = 3;
        }
        //sinario 4 RH LF + another leg
        if ((contect_flag(1) == 1) && (contect_flag(2) == 1) && (contect_flag.sum() == 3))
        {
            sinario = 4;
        }
    }

    //---姿态角速度计算---
    void Motion_control::posture_vel_update()
    {
      roll_ang_vel = (roll_ang - pre_roll_ang)/time_step;
      pitch_ang_vel = (pitch_ang - pre_pitch_ang)/time_step;
      yaw_ang_vel = (yaw_ang - pre_yaw_ang)/time_step;

      //低通滤波
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

    //---质心速度计算(need to fix)---
    void Motion_control::com_vel_update()
    {
     // RF LH TOUCH
     if (sinario == 0)
     {
       for (int i=0; i<3; i++)
       {
         com_vel(i) = ((four_feet_position(0,i) - pre_four_feet_position(0,i))/time_step
                      +(four_feet_position(3,i) - pre_four_feet_position(3,i))/time_step)/2;
         com_vel(i) = com_vel(i) * 0.3 + pre_com_vel(i) * 0.7;
         pre_com_vel(i) = com_vel(i);
       }
       com_vel_x = com_vel(0);
       com_vel_y = com_vel(1);
       com_vel_z = com_vel(2);

       com_z = (four_feet_position(0,2)
               +four_feet_position(3,2))/2;   
     }

     // RH LF TOUCH
     if (sinario == 3)
     {
       for (int i=0; i<3; i++)
       {
         com_vel(i) = ((four_feet_position(1,i) - pre_four_feet_position(1,i))/time_step
                      +(four_feet_position(2,i) - pre_four_feet_position(2,i))/time_step)/2;
         com_vel(i) = com_vel(i) * 0.3 + pre_com_vel(i) * 0.7;
         pre_com_vel(i) = com_vel(i);
       }
       com_vel_x = com_vel(0);
       com_vel_y = com_vel(1);
       com_vel_z = com_vel(2);

       com_z = (four_feet_position(1,2) 
               +four_feet_position(2,2))/2;
     }

     // FOUR FEET TOUCH
     if (sinario == 2)
     {
       for (int i=0; i<3; i++)
       {
         com_vel(i) = ((four_feet_position(0,i) - pre_four_feet_position(0,i))/time_step
                      +(four_feet_position(1,i) - pre_four_feet_position(1,i))/time_step
                      +(four_feet_position(2,i) - pre_four_feet_position(2,i))/time_step
                      +(four_feet_position(3,i) - pre_four_feet_position(3,i))/time_step)/4;
         com_vel(i) = com_vel(i) * 0.3 + pre_com_vel(i) * 0.7;
         pre_com_vel(i) = com_vel(i);
       }
       com_vel_x = com_vel(0);
       com_vel_y = com_vel(1);
       com_vel_z = com_vel(2);

       com_z = (four_feet_position(0,2)
               +four_feet_position(1,2)
               +four_feet_position(2,2)
               +four_feet_position(3,2))/4;
               
     }

    }

    //---关节角速度计算---
    void Motion_control::joint_vel_update()
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

    //---关节力矩补偿---
    void Motion_control::joint_torque_compensation_update()
    {
      double k = 1;
      double d = 0.002;
     
      for(int j=0; j<3; j++)
        {
          torque_add(0,j) = k * (0 - joint_ang(0,j)) + d * (0 - ang_vel(0,j));
          torque_add(1,j) = k * (0 - joint_ang(1,j)) + d * (0 - ang_vel(1,j));
          torque_add(2,j) = k * (0 - joint_ang(2,j)) + d * (0 - ang_vel(2,j));
          torque_add(3,j) = k * (0 - joint_ang(3,j)) + d * (0 - ang_vel(3,j));
        } 
    }

    //---力矩赋值---
    void Motion_control::joint_torque_update()
    {
       //力矩值整合
       //joint_T << rf_jont_T,rh_jont_T,lf_jont_T,lh_jont_T;

       //力矩值输入
       for(int i = 0; i < 4; i++)
        {  
          for(int j = 0; j < 3; j++)
            {
              //limb_motors[i][j]->setTorque(joint_T(i,j) + torque_add(i,j)); 
            } 
        }
    }

    //---四足位置计算(可改造成数组加函数将四种情况统一)---
    void Motion_control::four_feet_position_update()
    {
        //RF
        rf_pos_x =  L2 * cos(joint_ang(0,1) + joint_ang(0,2)) 
                   +L1 * sin(joint_ang(0,1));
        rf_pos_y = -L1 * cos(joint_ang(0,0)) * cos(joint_ang(0,1)) 
                   +L3 * sin(joint_ang(0,0))
                   +L2 * cos(joint_ang(0,0)) * cos(joint_ang(0,1)) * sin(joint_ang(0,2))
                   +L2 * cos(joint_ang(0,0)) * sin(joint_ang(0,1)) * sin(joint_ang(0,2));
        rf_pos_z = -L1 * sin(joint_ang(0,0)) * cos(joint_ang(0,1)) 
                   -L3 * cos(joint_ang(0,0)) 
                   +L2 * sin(joint_ang(0,0)) * cos(joint_ang(0,1)) * sin(joint_ang(0,2))
                   +L2 * sin(joint_ang(0,0)) * sin(joint_ang(0,1)) * cos(joint_ang(0,2)); 
        //RH
        rh_pos_x = -L2 * cos(joint_ang(1,1) + joint_ang(1,2)) 
                   +L1 * sin(joint_ang(1,1));
        rh_pos_y = -L1 * cos(joint_ang(1,0)) * cos(joint_ang(1,1)) 
                   +L3 * sin(joint_ang(1,0)) 
                   -L2 * cos(joint_ang(1,0)) * cos(joint_ang(1,1)) * sin(joint_ang(1,2))
                   -L2 * cos(joint_ang(1,0)) * sin(joint_ang(1,1)) * sin(joint_ang(1,2));
        rh_pos_z = -L1 * sin(joint_ang(1,0)) * cos(joint_ang(1,1)) 
                   -L3 * cos(joint_ang(1,0)) 
                   -L2 * sin(joint_ang(1,0)) * cos(joint_ang(1,1)) * sin(joint_ang(1,2))
                   -L2 * sin(joint_ang(1,0)) * sin(joint_ang(1,1)) * cos(joint_ang(1,2));  
        //LF
        lf_pos_x =  L2 * cos(joint_ang(2,1) + joint_ang(2,2)) 
                   -L1 * sin(joint_ang(2,1));
        lf_pos_y =  L1 * cos(joint_ang(2,0)) * cos(joint_ang(2,1)) 
                   +L3 * sin(joint_ang(2,0)) 
                   +L2 * cos(joint_ang(2,0)) * cos(joint_ang(2,1)) * sin(joint_ang(2,2))
                   +L2 * cos(joint_ang(2,0)) * sin(joint_ang(2,1)) * sin(joint_ang(2,2));
        lf_pos_z =  L1 * sin(joint_ang(2,0)) * cos(joint_ang(2,1)) 
                   -L3 * cos(joint_ang(2,0)) 
                   +L2 * sin(joint_ang(2,0)) * cos(joint_ang(2,1)) * sin(joint_ang(2,2))
                   +L2 * sin(joint_ang(2,0)) * sin(joint_ang(2,1)) * cos(joint_ang(2,2));  
        //LH
        lh_pos_x = -L2 * cos(joint_ang(3,1) + joint_ang(3,2)) 
                   -L1 * sin(joint_ang(3,1));
        lh_pos_y =  L1 * cos(joint_ang(3,0)) * cos(joint_ang(3,1)) 
                   +L3 * sin(joint_ang(3,0)) 
                   -L2 * cos(joint_ang(3,0)) * cos(joint_ang(3,1)) * sin(joint_ang(3,2))
                   -L2 * cos(joint_ang(3,0)) * sin(joint_ang(3,1)) * sin(joint_ang(3,2));
        lh_pos_z =  L1 * sin(joint_ang(3,0)) * cos(joint_ang(3,1)) 
                   -L3 * cos(joint_ang(3,0)) 
                   -L2 * sin(joint_ang(3,0)) * cos(joint_ang(3,1)) * sin(joint_ang(3,2))
                   -L2 * sin(joint_ang(3,0)) * sin(joint_ang(3,1)) * cos(joint_ang(3,2));

        rf_pos_x += BL/2;
        rf_pos_y -= BW/2;
        

        rh_pos_x -= BL/2;
        rh_pos_y -= BW/2;
        

        lf_pos_x += BL/2;
        lf_pos_y += BW/2;
        

        lh_pos_x -= BL/2;
        lh_pos_y += BW/2;
        

        four_feet_position(0,0) = rf_pos_x;
        four_feet_position(0,1) = rf_pos_y;
        four_feet_position(0,2) = rf_pos_z;

        four_feet_position(1,0) = rh_pos_x;
        four_feet_position(1,1) = rh_pos_y;
        four_feet_position(1,2) = rh_pos_z;

        four_feet_position(2,0) = lf_pos_x;
        four_feet_position(2,1) = lf_pos_y;
        four_feet_position(2,2) = lf_pos_z;

        four_feet_position(3,0) = lh_pos_x;
        four_feet_position(3,1) = lh_pos_y;
        four_feet_position(3,2) = lh_pos_z; 

        pre_four_feet_position = four_feet_position;
    }

    //---支撑相力矩---
    void Motion_control::sup_torque(int sup_leg1,int sup_leg2, int fly_leg1,int fly_leg2)
    {
      webots_relate();
      //touching_check();
      //sinario = 0;
      four_feet_position_update();
      posture_vel_update();
      com_vel_update();
      joint_torque_compensation_update();
      joint_vel_update();
      //joint_torque_update();
      
      
      //计算质心广义力
      com_t_x = K_com_t_x * (roll_ang_d - roll_ang) + D_com_t_y * (roll_ang_vel_d - roll_ang_vel); 
      com_t_y = K_com_t_y * (pitch_ang_d - pitch_ang) + D_com_t_y * (pitch_ang_vel_d - pitch_ang_vel);
      com_f_z = K_com_f_z * (com_z_d - com_z) + D_com_f_z * (com_vel_z);
    
      com_f_x = K_com_f_x * (com_vel_x_d - com_vel_x);
      com_f_y = K_com_f_y * (com_vel_y_d - com_vel_y);
      com_t_z = K_com_t_z * (yaw_ang_vel_d - yaw_ang_vel);
      
      //计算重力
      gvt_x = body_mass * g * sin(pitch_ang);
      gvt_y = body_mass * g * (-cos(pitch_ang)) * sin(yaw_ang);
      gvt_z = body_mass * g * (-cos(pitch_ang)) * cos(yaw_ang);
      
      //广义力向量
      com_F(0) = com_f_x - gvt_x;
      com_F(1) = 0;
      com_F(2) = com_f_z - gvt_z;
      com_F(3) = com_t_x;
      com_F(4) = com_t_y;
      com_F(5) = com_t_z;
      
      com_F(0) = -com_F(0);
      com_F(1) = -com_F(1);
      com_F(2) = -com_F(2);
      com_F(3) = -com_F(3);
      com_F(4) = -com_F(4);
      com_F(5) = -com_F(5);
      
      //Q矩阵(足端力和质心力关系)
      transQA << 1,0,0,1,0,0,
                0,1,0,0,-1,0,
                0,0,1,0,0,1,
                0,-rf_pos_z,rf_pos_y,0,-lh_pos_z,lh_pos_y,
                rf_pos_z,0,-rf_pos_x,lh_pos_z,0,-lh_pos_x,
                -rf_pos_y,rf_pos_x,0,-lh_pos_y,lh_pos_x,0;

      transQB << 1,0,0,1,0,0,
                0,1,0,0,-1,0,
                0,0,1,0,0,1,
                0,-lf_pos_z,-lf_pos_y,0,-rh_pos_z,-rh_pos_y,
                lf_pos_z,0,-lf_pos_x,rh_pos_z,0,-rh_pos_x,
                lf_pos_y,lf_pos_x,0,rh_pos_y,rh_pos_x,0;

      //足端力求解
      if (sinario < 3)
      {
        foot_F_f_and_h_sup = transQA.inverse() * com_F;
        //足端力关节力矩转换
        rf_foot_F = foot_F_f_and_h_sup.head(3);
        lh_foot_F = foot_F_f_and_h_sup.tail(3);
      }

      if (sinario > 1)
      {
        foot_F_f_and_h_sup = transQB.inverse() * com_F;
        //足端力关节力矩转换
        lf_foot_F = foot_F_f_and_h_sup.head(3);
        rh_foot_F = foot_F_f_and_h_sup.tail(3);
      }

      transJ_rf <<  0,
                    -L2 * sin(joint_ang(0,1)) * cos(joint_ang(0,2))
                    +L2 * cos(joint_ang(0,1)) * sin(joint_ang(0,2))
                    +L1 * cos(joint_ang(0,1)),
                    -L2 * cos(joint_ang(0,1)) * sin(joint_ang(0,2))
                    +L2 * sin(joint_ang(0,1)) * cos(joint_ang(0,2)),

                     L1 * sin(joint_ang(0,0)) * cos(joint_ang(0,1))
                    +L3 * cos(joint_ang(0,0))
                    -L2 * sin(joint_ang(0,0)) * cos(joint_ang(0,1)) * sin(joint_ang(0,2)) 
                    -L2 * sin(joint_ang(0,0)) * sin(joint_ang(0,1)) * sin(joint_ang(0,2)),
                    +L1 * cos(joint_ang(0,0)) * sin(joint_ang(0,1))
                    -L2 * cos(joint_ang(0,0)) * sin(joint_ang(0,1)) * sin(joint_ang(0,2)) 
                    +L2 * cos(joint_ang(0,0)) * cos(joint_ang(0,1)) * sin(joint_ang(0,2)),
                    +L2 * cos(joint_ang(0,0)) * cos(joint_ang(0,1)) * cos(joint_ang(0,2)) 
                    +L2 * cos(joint_ang(0,0)) * sin(joint_ang(0,1)) * cos(joint_ang(0,2)),

                    -L1 * cos(joint_ang(0,0)) * cos(joint_ang(0,1))
                    +L3 * sin(joint_ang(0,0))
                    +L2 * cos(joint_ang(0,0)) * cos(joint_ang(0,1)) * sin(joint_ang(0,2)) 
                    +L2 * cos(joint_ang(0,0)) * sin(joint_ang(0,1)) * cos(joint_ang(0,2)),
                    +L1 * sin(joint_ang(0,0)) * sin(joint_ang(0,1))
                    -L2 * sin(joint_ang(0,0)) * sin(joint_ang(0,1)) * sin(joint_ang(0,2)) 
                    +L2 * sin(joint_ang(0,0)) * cos(joint_ang(0,1)) * cos(joint_ang(0,2)),
                    +L2 * sin(joint_ang(0,0)) * cos(joint_ang(0,1)) * cos(joint_ang(0,2)) 
                    -L2 * sin(joint_ang(0,0)) * sin(joint_ang(0,1)) * sin(joint_ang(0,2));
         
      transJ_rh <<   0,
                     L2 * sin(joint_ang(1,1)) * cos(joint_ang(1,2))
                    -L2 * cos(joint_ang(1,1)) * sin(joint_ang(1,2))
                    +L1 * cos(joint_ang(1,1)),
                    +L2 * cos(joint_ang(1,1)) * sin(joint_ang(1,2))
                    -L2 * sin(joint_ang(1,1)) * cos(joint_ang(1,2)),

                     L1 * sin(joint_ang(1,0)) * cos(joint_ang(1,1))
                    +L3 * cos(joint_ang(1,0))
                    +L2 * sin(joint_ang(1,0)) * cos(joint_ang(1,1)) * sin(joint_ang(1,2)) 
                    +L2 * sin(joint_ang(1,0)) * sin(joint_ang(1,1)) * sin(joint_ang(1,2)),
                    +L1 * cos(joint_ang(1,0)) * sin(joint_ang(1,1))
                    +L2 * cos(joint_ang(1,0)) * sin(joint_ang(1,1)) * sin(joint_ang(1,2)) 
                    -L2 * cos(joint_ang(1,0)) * cos(joint_ang(1,1)) * sin(joint_ang(1,2)),
                    -L2 * cos(joint_ang(1,0)) * cos(joint_ang(1,1)) * cos(joint_ang(1,2)) 
                    -L2 * cos(joint_ang(1,0)) * sin(joint_ang(1,1)) * cos(joint_ang(1,2)),

                    -L1 * cos(joint_ang(1,0)) * cos(joint_ang(1,1))
                    +L3 * sin(joint_ang(1,0))
                    -L2 * cos(joint_ang(1,0)) * cos(joint_ang(1,1)) * sin(joint_ang(1,2)) 
                    -L2 * cos(joint_ang(1,0)) * sin(joint_ang(1,1)) * cos(joint_ang(1,2)),
                    +L1 * sin(joint_ang(1,0)) * sin(joint_ang(1,1))
                    +L2 * sin(joint_ang(1,0)) * sin(joint_ang(1,1)) * sin(joint_ang(1,2)) 
                    -L2 * sin(joint_ang(1,0)) * cos(joint_ang(1,1)) * cos(joint_ang(1,2)),
                    -L2 * sin(joint_ang(1,0)) * cos(joint_ang(1,1)) * cos(joint_ang(1,2)) 
                    +L2 * sin(joint_ang(1,0)) * sin(joint_ang(1,1)) * sin(joint_ang(1,2));

      transJ_lf <<   0,
                    -L2 * sin(joint_ang(2,1)) * cos(joint_ang(2,2))
                    +L2 * cos(joint_ang(2,1)) * sin(joint_ang(2,2))
                    -L1 * cos(joint_ang(2,1)),
                    -L2 * cos(joint_ang(2,1)) * sin(joint_ang(2,2))
                    +L2 * sin(joint_ang(2,1)) * cos(joint_ang(2,2)),

                    +L1 * sin(joint_ang(2,0)) * cos(joint_ang(2,1))
                    +L3 * cos(joint_ang(2,0))
                    -L2 * sin(joint_ang(2,0)) * cos(joint_ang(2,1)) * sin(joint_ang(2,2)) 
                    -L2 * sin(joint_ang(2,0)) * sin(joint_ang(2,1)) * sin(joint_ang(2,2)),
                    -L1 * cos(joint_ang(2,0)) * sin(joint_ang(2,1))
                    -L2 * cos(joint_ang(2,0)) * sin(joint_ang(2,1)) * sin(joint_ang(2,2)) 
                    +L2 * cos(joint_ang(2,0)) * cos(joint_ang(2,1)) * sin(joint_ang(2,2)),
                    +L2 * cos(joint_ang(2,0)) * cos(joint_ang(2,1)) * cos(joint_ang(2,2)) 
                    +L2 * cos(joint_ang(2,0)) * sin(joint_ang(2,1)) * cos(joint_ang(2,2)),

                    +L1 * cos(joint_ang(2,0)) * cos(joint_ang(2,1))
                    +L3 * sin(joint_ang(2,0))
                    +L2 * cos(joint_ang(2,0)) * cos(joint_ang(2,1)) * sin(joint_ang(2,2)) 
                    +L2 * cos(joint_ang(2,0)) * sin(joint_ang(2,1)) * cos(joint_ang(2,2)),
                    -L1 * sin(joint_ang(2,0)) * sin(joint_ang(2,1))
                    -L2 * sin(joint_ang(2,0)) * sin(joint_ang(2,1)) * sin(joint_ang(2,2)) 
                    +L2 * sin(joint_ang(2,0)) * cos(joint_ang(2,1)) * cos(joint_ang(2,2)),
                    +L2 * sin(joint_ang(2,0)) * cos(joint_ang(2,1)) * cos(joint_ang(2,2)) 
                    -L2 * sin(joint_ang(2,0)) * sin(joint_ang(2,1)) * sin(joint_ang(2,2));

      transJ_lh <<   0,
                     L2 * sin(joint_ang(3,1)) * cos(joint_ang(3,2))
                    -L2 * cos(joint_ang(3,1)) * sin(joint_ang(3,2))
                    +L1 * cos(joint_ang(3,1)),
                     L2 * cos(joint_ang(3,1)) * sin(joint_ang(3,2))
                    -L2 * sin(joint_ang(3,1)) * cos(joint_ang(3,2)),

                    +L1 * sin(joint_ang(3,0)) * cos(joint_ang(3,1))
                    +L3 * cos(joint_ang(3,0))
                    +L2 * sin(joint_ang(3,0)) * cos(joint_ang(3,1)) * sin(joint_ang(3,2)) 
                    +L2 * sin(joint_ang(3,0)) * sin(joint_ang(3,1)) * sin(joint_ang(3,2)),
                    +L1 * cos(joint_ang(3,0)) * sin(joint_ang(3,1))
                    +L2 * cos(joint_ang(3,0)) * sin(joint_ang(3,1)) * sin(joint_ang(3,2)) 
                    -L2 * cos(joint_ang(3,0)) * cos(joint_ang(3,1)) * sin(joint_ang(3,2)),
                    -L2 * cos(joint_ang(3,0)) * cos(joint_ang(3,1)) * cos(joint_ang(3,2)) 
                    -L2 * cos(joint_ang(3,0)) * sin(joint_ang(3,1)) * cos(joint_ang(3,2)),

                    -L1 * cos(joint_ang(3,0)) * cos(joint_ang(3,1))
                    +L3 * sin(joint_ang(3,0))
                    -L2 * cos(joint_ang(3,0)) * cos(joint_ang(3,1)) * sin(joint_ang(3,2)) 
                    -L2 * cos(joint_ang(3,0)) * sin(joint_ang(3,1)) * cos(joint_ang(3,2)),
                    +L1 * sin(joint_ang(3,0)) * sin(joint_ang(3,1))
                    +L2 * sin(joint_ang(3,0)) * sin(joint_ang(3,1)) * sin(joint_ang(3,2)) 
                    -L2 * sin(joint_ang(3,0)) * cos(joint_ang(3,1)) * cos(joint_ang(3,2)),
                    -L2 * sin(joint_ang(3,0)) * cos(joint_ang(3,1)) * cos(joint_ang(3,2)) 
                    +L2 * sin(joint_ang(3,0)) * sin(joint_ang(3,1)) * sin(joint_ang(3,2));

      //A
      rf_jont_T = transJ_rf.transpose() * rf_foot_F;
      lh_jont_T = transJ_lh.transpose() * lh_foot_F;
      lh_jont_T(0) = -lh_jont_T(0);

      //B
      rh_jont_T = transJ_rh.transpose() * rh_foot_F;
      lf_jont_T = transJ_lf.transpose() * lf_foot_F;
      lf_jont_T(0) = lf_jont_T(0);

      //joint_T << rf_jont_T,rh_jont_T,lf_jont_T,lh_jont_T;
      
      for(int i = 0; i < 3; i++)
        {
          joint_T(0,i) = rf_jont_T(i);
          joint_T(1,i) = rh_jont_T(i);
          joint_T(2,i) = lf_jont_T(i);
          joint_T(3,i) = lh_jont_T(i);
        } 

      for (int i=0; i<3; i++)
        { 
          limb_motors[sup_leg1][i]->setTorque(joint_T(sup_leg1,i) + torque_add(sup_leg1,i));
          limb_motors[sup_leg2][i]->setTorque(joint_T(sup_leg2,i) + torque_add(sup_leg2,i));
        }
      limb_motors[fly_leg1][0]->setPosition(-(-0.3*t*(t-2)-0));
      limb_motors[fly_leg2][0]->setPosition(-(0.3*t*(t-2)+0));
      //limb_motors[fly_leg1][0]->setPosition(-0.3);
      //limb_motors[fly_leg2][0]->setPosition(0.3);    
    }
    
    //---测试一：姿态平衡(对角触地)---
    void Motion_control::test_diag_feet_balance()
    {
      webots_relate();
      touching_check();
      sinario = 0;
      four_feet_position_update();
      posture_vel_update();
      com_vel_update();
      joint_torque_compensation_update();
      joint_vel_update();
      joint_torque_update();
      
      //计算质心广义力
      com_t_x = K_com_t_x * (roll_ang_d - roll_ang) + D_com_t_y * (roll_ang_vel_d - roll_ang_vel); 
      com_t_y = K_com_t_y * (pitch_ang_d - pitch_ang) + D_com_t_y * (pitch_ang_vel_d - pitch_ang_vel);
      com_f_z = K_com_f_z * (com_z_d - com_z) + D_com_f_z * (com_vel_z);
    
      com_f_x = K_com_f_x * (com_vel_x_d - com_vel_x);
      com_f_y = K_com_f_y * (com_vel_y_d - com_vel_y);
      com_t_z = K_com_t_z * (yaw_ang_vel_d - yaw_ang_vel);
      
      //计算重力
      gvt_x = body_mass * g * sin(pitch_ang);
      gvt_y = body_mass * g * (-cos(pitch_ang)) * sin(yaw_ang);
      gvt_z = body_mass * g * (-cos(pitch_ang)) * cos(yaw_ang);
      
      //广义力向量
      com_F(0) = com_f_x - gvt_x;
      com_F(1) = 0;
      com_F(2) = com_f_z - gvt_z;
      com_F(3) = com_t_x;
      com_F(4) = com_t_y;
      com_F(5) = com_t_z;
      
      com_F(0) = -com_F(0);
      com_F(1) = -com_F(1);
      com_F(2) = -com_F(2);
      com_F(3) = -com_F(3);
      com_F(4) = -com_F(4);
      com_F(5) = -com_F(5);
      
      //Q矩阵(足端力和质心力关系)
      transQ << 1,0,0,1,0,0,
                0,1,0,0,-1,0,
                0,0,1,0,0,1,
                0,-rf_pos_z,rf_pos_y,0,-lh_pos_z,lh_pos_y,
                rf_pos_z,0,-rf_pos_x,lh_pos_z,0,-lh_pos_x,
                -rf_pos_y,rf_pos_x,0,-lh_pos_y,lh_pos_x,0;
        
      //足端力求解
      foot_F_f_and_h_sup = transQ.inverse() * com_F;//保留对所有方向控制时使用最小二乘法
      
      //足端力关节力矩转换
      rf_foot_F = foot_F_f_and_h_sup.head(3);
      lh_foot_F = foot_F_f_and_h_sup.tail(3);

      transJ_rf <<  0,
                    -L2 * sin(joint_ang(0,1)) * cos(joint_ang(0,2))
                    +L2 * cos(joint_ang(0,1)) * sin(joint_ang(0,2))
                    +L1 * cos(joint_ang(0,1)),
                    -L2 * cos(joint_ang(0,1)) * sin(joint_ang(0,2))
                    +L2 * sin(joint_ang(0,1)) * cos(joint_ang(0,2)),

                     L1 * sin(joint_ang(0,0)) * cos(joint_ang(0,1))
                    +L3 * cos(joint_ang(0,0))
                    -L2 * sin(joint_ang(0,0)) * cos(joint_ang(0,1)) * sin(joint_ang(0,2)) 
                    -L2 * sin(joint_ang(0,0)) * sin(joint_ang(0,1)) * sin(joint_ang(0,2)),
                    +L1 * cos(joint_ang(0,0)) * sin(joint_ang(0,1))
                    -L2 * cos(joint_ang(0,0)) * sin(joint_ang(0,1)) * sin(joint_ang(0,2)) 
                    +L2 * cos(joint_ang(0,0)) * cos(joint_ang(0,1)) * sin(joint_ang(0,2)),
                    +L2 * cos(joint_ang(0,0)) * cos(joint_ang(0,1)) * cos(joint_ang(0,2)) 
                    +L2 * cos(joint_ang(0,0)) * sin(joint_ang(0,1)) * cos(joint_ang(0,2)),

                    -L1 * cos(joint_ang(0,0)) * cos(joint_ang(0,1))
                    +L3 * sin(joint_ang(0,0))
                    +L2 * cos(joint_ang(0,0)) * cos(joint_ang(0,1)) * sin(joint_ang(0,2)) 
                    +L2 * cos(joint_ang(0,0)) * sin(joint_ang(0,1)) * cos(joint_ang(0,2)),
                    +L1 * sin(joint_ang(0,0)) * sin(joint_ang(0,1))
                    -L2 * sin(joint_ang(0,0)) * sin(joint_ang(0,1)) * sin(joint_ang(0,2)) 
                    +L2 * sin(joint_ang(0,0)) * cos(joint_ang(0,1)) * cos(joint_ang(0,2)),
                    +L2 * sin(joint_ang(0,0)) * cos(joint_ang(0,1)) * cos(joint_ang(0,2)) 
                    -L2 * sin(joint_ang(0,0)) * sin(joint_ang(0,1)) * sin(joint_ang(0,2));
         
      transJ_rh <<   0,
                     L2 * sin(joint_ang(1,1)) * cos(joint_ang(1,2))
                    -L2 * cos(joint_ang(1,1)) * sin(joint_ang(1,2))
                    +L1 * cos(joint_ang(1,1)),
                    +L2 * cos(joint_ang(1,1)) * sin(joint_ang(1,2))
                    -L2 * sin(joint_ang(1,1)) * cos(joint_ang(1,2)),

                     L1 * sin(joint_ang(1,0)) * cos(joint_ang(1,1))
                    +L3 * cos(joint_ang(1,0))
                    +L2 * sin(joint_ang(1,0)) * cos(joint_ang(1,1)) * sin(joint_ang(1,2)) 
                    +L2 * sin(joint_ang(1,0)) * sin(joint_ang(1,1)) * sin(joint_ang(1,2)),
                    +L1 * cos(joint_ang(1,0)) * sin(joint_ang(1,1))
                    +L2 * cos(joint_ang(1,0)) * sin(joint_ang(1,1)) * sin(joint_ang(1,2)) 
                    -L2 * cos(joint_ang(1,0)) * cos(joint_ang(1,1)) * sin(joint_ang(1,2)),
                    -L2 * cos(joint_ang(1,0)) * cos(joint_ang(1,1)) * cos(joint_ang(1,2)) 
                    -L2 * cos(joint_ang(1,0)) * sin(joint_ang(1,1)) * cos(joint_ang(1,2)),

                    -L1 * cos(joint_ang(1,0)) * cos(joint_ang(1,1))
                    +L3 * sin(joint_ang(1,0))
                    -L2 * cos(joint_ang(1,0)) * cos(joint_ang(1,1)) * sin(joint_ang(1,2)) 
                    -L2 * cos(joint_ang(1,0)) * sin(joint_ang(1,1)) * cos(joint_ang(1,2)),
                    +L1 * sin(joint_ang(1,0)) * sin(joint_ang(1,1))
                    +L2 * sin(joint_ang(1,0)) * sin(joint_ang(1,1)) * sin(joint_ang(1,2)) 
                    -L2 * sin(joint_ang(1,0)) * cos(joint_ang(1,1)) * cos(joint_ang(1,2)),
                    -L2 * sin(joint_ang(1,0)) * cos(joint_ang(1,1)) * cos(joint_ang(1,2)) 
                    +L2 * sin(joint_ang(1,0)) * sin(joint_ang(1,1)) * sin(joint_ang(1,2));

      transJ_lf <<   0,
                    -L2 * sin(joint_ang(2,1)) * cos(joint_ang(2,2))
                    +L2 * cos(joint_ang(2,1)) * sin(joint_ang(2,2))
                    -L1 * cos(joint_ang(2,1)),
                    -L2 * cos(joint_ang(2,1)) * sin(joint_ang(2,2))
                    +L2 * sin(joint_ang(2,1)) * cos(joint_ang(2,2)),

                    +L1 * sin(joint_ang(2,0)) * cos(joint_ang(2,1))
                    +L3 * cos(joint_ang(2,0))
                    -L2 * sin(joint_ang(2,0)) * cos(joint_ang(2,1)) * sin(joint_ang(2,2)) 
                    -L2 * sin(joint_ang(2,0)) * sin(joint_ang(2,1)) * sin(joint_ang(2,2)),
                    -L1 * cos(joint_ang(2,0)) * sin(joint_ang(2,1))
                    -L2 * cos(joint_ang(2,0)) * sin(joint_ang(2,1)) * sin(joint_ang(2,2)) 
                    +L2 * cos(joint_ang(2,0)) * cos(joint_ang(2,1)) * sin(joint_ang(2,2)),
                    +L2 * cos(joint_ang(2,0)) * cos(joint_ang(2,1)) * cos(joint_ang(2,2)) 
                    +L2 * cos(joint_ang(2,0)) * sin(joint_ang(2,1)) * cos(joint_ang(2,2)),

                    +L1 * cos(joint_ang(2,0)) * cos(joint_ang(2,1))
                    +L3 * sin(joint_ang(2,0))
                    +L2 * cos(joint_ang(2,0)) * cos(joint_ang(2,1)) * sin(joint_ang(2,2)) 
                    +L2 * cos(joint_ang(2,0)) * sin(joint_ang(2,1)) * cos(joint_ang(2,2)),
                    -L1 * sin(joint_ang(2,0)) * sin(joint_ang(2,1))
                    -L2 * sin(joint_ang(2,0)) * sin(joint_ang(2,1)) * sin(joint_ang(2,2)) 
                    +L2 * sin(joint_ang(2,0)) * cos(joint_ang(2,1)) * cos(joint_ang(2,2)),
                    +L2 * sin(joint_ang(2,0)) * cos(joint_ang(2,1)) * cos(joint_ang(2,2)) 
                    -L2 * sin(joint_ang(2,0)) * sin(joint_ang(2,1)) * sin(joint_ang(2,2));

      transJ_lh <<   0,
                     L2 * sin(joint_ang(3,1)) * cos(joint_ang(3,2))
                    -L2 * cos(joint_ang(3,1)) * sin(joint_ang(3,2))
                    +L1 * cos(joint_ang(3,1)),
                     L2 * cos(joint_ang(3,1)) * sin(joint_ang(3,2))
                    -L2 * sin(joint_ang(3,1)) * cos(joint_ang(3,2)),

                    +L1 * sin(joint_ang(3,0)) * cos(joint_ang(3,1))
                    +L3 * cos(joint_ang(3,0))
                    +L2 * sin(joint_ang(3,0)) * cos(joint_ang(3,1)) * sin(joint_ang(3,2)) 
                    +L2 * sin(joint_ang(3,0)) * sin(joint_ang(3,1)) * sin(joint_ang(3,2)),
                    +L1 * cos(joint_ang(3,0)) * sin(joint_ang(3,1))
                    +L2 * cos(joint_ang(3,0)) * sin(joint_ang(3,1)) * sin(joint_ang(3,2)) 
                    -L2 * cos(joint_ang(3,0)) * cos(joint_ang(3,1)) * sin(joint_ang(3,2)),
                    -L2 * cos(joint_ang(3,0)) * cos(joint_ang(3,1)) * cos(joint_ang(3,2)) 
                    -L2 * cos(joint_ang(3,0)) * sin(joint_ang(3,1)) * cos(joint_ang(3,2)),

                    -L1 * cos(joint_ang(3,0)) * cos(joint_ang(3,1))
                    +L3 * sin(joint_ang(3,0))
                    -L2 * cos(joint_ang(3,0)) * cos(joint_ang(3,1)) * sin(joint_ang(3,2)) 
                    -L2 * cos(joint_ang(3,0)) * sin(joint_ang(3,1)) * cos(joint_ang(3,2)),
                    +L1 * sin(joint_ang(3,0)) * sin(joint_ang(3,1))
                    +L2 * sin(joint_ang(3,0)) * sin(joint_ang(3,1)) * sin(joint_ang(3,2)) 
                    -L2 * sin(joint_ang(3,0)) * cos(joint_ang(3,1)) * cos(joint_ang(3,2)),
                    -L2 * sin(joint_ang(3,0)) * cos(joint_ang(3,1)) * cos(joint_ang(3,2)) 
                    +L2 * sin(joint_ang(3,0)) * sin(joint_ang(3,1)) * sin(joint_ang(3,2));

      rf_jont_T = transJ_rf.transpose() * rf_foot_F;
      lh_jont_T = transJ_lh.transpose() * lh_foot_F;

      lh_jont_T(0) = -lh_jont_T(0);
      //lh_jont_T(1) = -lh_jont_T(1);
      //lh_jont_T(2) = -lh_jont_T(2);
      //rf_jont_T(1) = -rf_jont_T(1);
      //rf_jont_T(2) = -rf_jont_T(2);

      for (int i=0; i<3; i++)
        { 
          limb_motors[0][i]->setTorque( rf_jont_T(i) + torque_add(0,i));
          limb_motors[3][i]->setTorque( lh_jont_T(i) + torque_add(3,i));
        }
        limb_motors[1][0]->setPosition(-0.3);
        limb_motors[2][0]->setPosition(0.3);
        
        //limb_motors[1][0]->setPosition(-(-0.6*t*(t-2)-0.2));
        //limb_motors[2][0]->setPosition(-(0.6*t*(t-2)+0.2));
        //cout << sinario << endl;
    }
    
    //---测试二：状态机(对角步态)---
    void Motion_control::test_state_machine_diag()
    {
      /**********************************************
      *****A组为左前右后支撑相 左后右前飞行相 *****
      ***********************************************/

      //变量更新
      webots_relate();
      touching_check();
      four_feet_position_update();
      posture_vel_update();
      com_vel_update();
      joint_torque_compensation_update();
      joint_vel_update();

      //定义变量
      int AorB;

      enum {Init, A_fly, A_hold, A_switch, B_fly, B_hold, B_switch} state;

      //action part
      switch (state)
      {
        case Init:
          //...
          break;

        case A_fly: 
          //...
          
          break;
        

      }

      //transition part
      switch (state)
      {
        case Init:
          state = A_fly;
          break;

        case A_fly:
          if (sinario == 0)
          {
            state = A_fly;
          }
          if (sinario == 1)
          {
            state = A_hold;
          }
          break;

        case A_hold:
          if (sinario == 1)
          {
            state = A_hold;
          }
          if (sinario == 2)
          {
            state = A_switch;
            AorB = 0;
          }
          break;

        case A_switch:
          if (sinario == 2)
          {
            state = A_switch;
            AorB = 0;
          }
          if (sinario == 3)
          {
            state = B_fly;
          }
          break;
      
      case B_fly:
          if (sinario == 3)
          {
            state = B_fly;
          }
          if (sinario == 4)
          {
            state = B_hold;
          }
          break;

        case B_hold:
          if (sinario == 4)
          {
            state = B_hold;
          }
          if (sinario == 5)
          {
            state = B_switch;
            AorB = 1;
          }
          break;

        case B_switch:
          if (sinario == 5)
          {
            state = B_switch;
            AorB = 1;
          }
          if (sinario == 0)
          {
            state = A_fly;
          }
          break;
        
        
        
      }
    }

    //---测试三：交替站立---
    void Motion_control::test_cross_move()
    {
      //sinario = 0;
      //sup_torque(0,3,1,2);
      
      //sinario = 3;
      //sup_torque(1,2,0,3);
      
      
      static int num;
      cout << t << "\t" << num << endl;

      if (t >= 2)
      {
        t = 0;
        num = num + 1;
      }
      
       if (num == 4)
        {
          num = 0;
        }
        if (num == 0)
        {
          sinario = 0;
          sup_torque(0,3,1,2);
        }
        if (num == 1)
        {
          limb_motors[0][0]->setPosition(0);
          limb_motors[1][0]->setPosition(0);
          limb_motors[2][0]->setPosition(0);
          limb_motors[3][0]->setPosition(0);
          
          limb_motors[0][1]->setPosition(0);
          limb_motors[1][1]->setPosition(0);
          limb_motors[2][1]->setPosition(0);
          limb_motors[3][1]->setPosition(0);
          
          limb_motors[0][2]->setPosition(0);
          limb_motors[1][2]->setPosition(0);
          limb_motors[2][2]->setPosition(0);
          limb_motors[3][2]->setPosition(0);
         
            
        }
        if (num == 2)
        {
          sinario = 3;
          sup_torque(1,2,0,3);
        }
        if (num == 3)
        {
          limb_motors[0][0]->setPosition(0);
          limb_motors[1][0]->setPosition(0);
          limb_motors[2][0]->setPosition(0);
          limb_motors[3][0]->setPosition(0);
          
          limb_motors[0][1]->setPosition(0);
          limb_motors[1][1]->setPosition(0);
          limb_motors[2][1]->setPosition(0);
          limb_motors[3][1]->setPosition(0);
          
          limb_motors[0][2]->setPosition(0);
          limb_motors[1][2]->setPosition(0);
          limb_motors[2][2]->setPosition(0);
          limb_motors[3][2]->setPosition(0);
          
        }

      t = t + time_step;

      

      
   
    }
