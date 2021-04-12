/**
  ******************************************************************************
  * @FileName			    ClimbTask.cpp
  * @Description            Control Up And Down Stairs
  * @author                 zyt
  * @note
  ******************************************************************************
  *
  * Copyright (c) 2021 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
**/

#include "includes.h"

void Climb::Handle(void){
	CLIMB_F.Handle();
	CLIMB_B.Handle();
}
/*
void Climb::Reset(void){
	CLIMB_F.Reset(CAN_TYPE_2, 0x205, 19,
                30, 0, 0, 15000, 15000, 15000, 15000,
	              10, 0, 0, 2160, 2160, 2160, 2160);
	CLIMB_B.Reset(CAN_TYPE_2, 0x206, 19,
                30, 0, 0, 15000, 15000, 15000, 15000,
	              10, 0, 0, 1440, 1440, 1440, 1440);
}
*/
void Climb::autoClimb(void){

}

void Climb::frontUp(void){
	CLIMB_F.targetAngle=660;
}

void Climb::frontDown(void){
	CLIMB_F.targetAngle=20;
}

void Climb::backUp(void){
	CLIMB_B.targetAngle=-10;
}

void Climb::backDown(void){
	CLIMB_B.targetAngle=-520;
}

Climb Climb::climb;