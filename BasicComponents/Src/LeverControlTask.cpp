/**
  ******************************************************************************
  * @FileName			    LeverControlTask.cpp
  * @Description            Control the robot by the lever on the remote
  * @author                 Steve Young
  * @note
  ******************************************************************************
  *
  * Copyright (c) 2021 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
**/

#include "includes.h"




void Remote::LeverControl() {
		
    //Chassis::chassis.LockChassis();
	
    static WorkState_e lastWorkState = NORMAL_STATE;
    if (Car::car.GetWorkState() <= 0) return;

    //遥控器设定底盘和云台速度/角度
    #ifdef INFANTRY
        #ifdef USE_CHASSIS_FOLLOW
            Chassis::chassis.SetVelocity(channel.rcol, channel.rrow, 0);
            Gimbal::gimbal.SetPosition(-channel.lcol, channel.lrow, GIMBAL_SPEED_K, GIMBAL_SPEED_K);
        #else
            Chassis::chassis.SetVelocity(channel.rcol, channel.rrow, channel.lrow);
            Gimbal::gimbal.SetPosition(-channel.lcol, channel.lrow, GIMBAL_SPEED_K, GIMBAL_SPEED_K);
        #endif
        HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
        //遥控器设置挡位
        if (Car::car.GetWorkState() == NORMAL_STATE) {
            #ifdef DARTS_SHOOT
            Shoot::shoot.Darts(false,0);
            #else
                Shoot::shoot.Fric(false);
                Shoot::shoot.RefreshAngleGap();
                Chassis::chassis.twistState = 0;
                AutoAim::autoAim.autoaimMode = manual;
            #endif
//            Shoot::shoot.Fric(true);
//            Shoot::shoot.fricSpeed=4350;
        } else if (Car::car.GetWorkState() == ADDITIONAL_STATE_ONE) {
            #ifdef DARTS_SHOOT
                Shoot::shoot.Darts(true,0);
            #else
                Shoot::shoot.Fric(true);
                Chassis::chassis.twistState=0;
                AutoAim::autoAim.autoaimMode = predict;
                AutoAim::autoAim.energyMode=0;
            #endif
//            Shoot::shoot.Fric(true);
//            Shoot::shoot.fricSpeed=4450;
        } else if (Car::car.GetWorkState() == ADDITIONAL_STATE_TWO) {
            #ifdef DARTS_SHOOT
                Shoot::shoot.Darts(true,1);
            #else
               Shoot::shoot.Fric(true);
                AutoAim::autoAim.autoaimMode = predict;
                AutoAim::autoAim.energyMode=0;
                #ifdef KEEP_ON_SHOOT
                    #ifdef KEEP_ON_SHOOT_FIERCELY
                        Shoot::shoot.ShootCertainInSecond(0,3);
                    #else
                        Shoot::shoot.ShootNoStop(0);
                    #endif
                #else
                    if (lastWorkState != Car::car.GetWorkState()) {
                        Shoot::shoot.ShootOneBullet(0);
                    }
                #endif
            #endif
//            Shoot::shoot.Fric(true);
//            Shoot::shoot.fricSpeed=4550;
    }
    lastWorkState = Car::car.GetWorkState();

#endif

#ifdef HERO

		static uint8_t If_Shoot1 = 0;
		static uint8_t If_Shoot2 = 0;
		static uint8_t If_Shoot3 = 0;
		static uint8_t If_contrast=0;
    if (Car::car.GetWorkState() == NORMAL_STATE) {

			
			#ifdef USE_CHASSIS_FOLLOW
					Chassis::chassis.SetVelocity(channel.rcol, channel.rrow, 0);
					Gimbal::gimbal.SetPosition(channel.lcol, channel.lrow, GIMBAL_SPEED_K, GIMBAL_SPEED_K);
			#else
					Chassis::chassis.SetVelocity(channel.rcol, channel.rrow, channel.lrow);
					Gimbal::gimbal.SetPosition(channel.lcol);
			#endif
        Chassis::chassis.lock = 0;
        HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
        Shoot::shoot.Fric(false);
        AutoAim::autoAim.autoaimMode = manual;
        if(Remote::remote.GetInputMode() == KEY_MOUSE_INPUT)
        {


                rotate_timer++;
            if(rotate_timer>60)
            {
                Chassis::chassis.twistState = 3;

            }
        }
        else
        {
            Chassis::chassis.twistState =0;
            rotate_timer=0;

        }
    } else if (Car::car.GetWorkState() == ADDITIONAL_STATE_ONE) {
        Chassis::chassis.lock = 0;
        HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
        Shoot::shoot.Fric(true);
        AutoAim::autoAim.autoaimMode= manual;

        if (If_contrast==1){
            Shoot::shoot.Contrast();
            If_contrast=0;
        }
#ifdef USE_CHASSIS_FOLLOW
        Chassis::chassis.SetVelocity(channel.rcol, channel.rrow, 0);
        Gimbal::gimbal.SetPosition(channel.lcol, channel.lrow, GIMBAL_SPEED_K, GIMBAL_SPEED_K);
#else
        Chassis::chassis.SetVelocity(channel.rcol, channel.rrow, channel.lrow);
					Gimbal::gimbal.SetPosition(channel.lcol);
#endif

        if(Remote::remote.GetInputMode() == KEY_MOUSE_INPUT)
        {


            rotate_timer++;
            if(rotate_timer>60)
            {
                Chassis::chassis.twistState = 1;

            }
        }
        else
        {
            Chassis::chassis.twistState =0;
            rotate_timer=0;

        }
				

    } else if (Car::car.GetWorkState() == ADDITIONAL_STATE_TWO) {
        Chassis::chassis.lock = 1;
        HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
        Shoot::shoot.Fric(true);
        AutoAim::autoAim.autoaimMode = manual;
        Gimbal::gimbal.SetPosition(channel.lcol*0.1f, channel.lrow*0.1f, GIMBAL_SPEED_K, GIMBAL_SPEED_K);
        Chassis::chassis.twistState =0;
        if (If_contrast==0){
            Shoot::shoot.Extend();
            If_contrast=1;
        }
        if(channel.rcol>500)
        {
            if(If_Shoot1 == 0)
            {
                Shoot::shoot.ShootOneBullet(0);
                If_Shoot1 = 1;
            }
        }
        else
        {
            If_Shoot1 = 0;
        }




        if(channel.rcol<-400)
        {

            bullet_timer++;

            if(bullet_timer>=14)
            {
                Shoot::shoot.ShootOneBullet(0);

                bullet_timer=0;
            }
        }




    }
#endif
		
}

/* TIPS:
 * NORMAL_STATE 左上角金属摇杆第一档
 * ADDITIONAL_STATE_ONE 左上角金属摇杆第二档
 * ADDITIONAL_STATE_TWO 左上角金属摇杆第三档
 * 巧用遥控器调试 事半功倍
 */