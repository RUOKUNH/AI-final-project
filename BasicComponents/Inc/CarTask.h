/**
  ******************************************************************************
  * @FileName			    CarTask.h
  * @Description            This is a robot, not only a car
  * @author                 Chang Liu & Steve Young
  * @note
  ******************************************************************************
  *
  * Copyright (c) 2021 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
**/

#ifndef __CAR_H__
#define __CAR_H__

#include "includes.h"


#ifdef BOARD_SLAVE
#define DOUBLE_BOARD_CAN1
#endif

typedef enum {
    STOP_STATE = -1,
    PREPARE_STATE = 0,
    NORMAL_STATE = 1,
    ADDITIONAL_STATE_ONE = 2,
    ADDITIONAL_STATE_TWO = 3,
} WorkState_e;

class Car {
private:
    WorkState_e workState;
    uint8_t allMotorsRx = 0;
public:
    static Car car;
    uint32_t counterForStopMode;

    void WorkStateFSM();

    inline WorkState_e GetWorkState() const { return workState; }//left
    inline void SetWorkState(WorkState_e state) { workState = state; }

    void Reset();

};

void CarInit();

void MainControlLoop();

void Count();

#endif
