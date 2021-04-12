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
#ifndef __CLIMB_H__
#define __CLIMB_H__

#include "includes.h"

class Climb {
private:
	NormalMotor CLIMB_F;
	NormalMotor CLIMB_B;

public:
	static Climb climb;

	void Reset();

	void Handle();

	void autoClimb();

	void frontUp();

	void frontDown();

	void backUp();

	void backDown();
};

#endif