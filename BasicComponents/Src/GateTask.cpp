/**
  ******************************************************************************
  * @FileName			    GateTask.cpp
  * @Description            Open or close the bullets hatch(gate)
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

#ifdef INFANTRY
void Gate::Reset() {
    #ifdef _GATE_SERVO
    HAL_UART_Transmit_IT(&SERVO_UART, (uint8_t *) "#000P1000T0010!", sizeof("#000P1000T0010!"));
    #endif
}

void Gate::SetGateState(GateState_e _gateState) {
    #ifdef _GATE_SERVO
    gateState = _gateState;
    if (gateState == GATE_OPEN )
        HAL_UART_Transmit_IT(&SERVO_UART, (uint8_t *) "#000P0500T0010!", sizeof("#000P0500T0010!"));
    else if (gateState == GATE_CLOSE)
        HAL_UART_Transmit_IT(&SERVO_UART, (uint8_t *) "#000P1000T0010!", sizeof("#000P1000T0010!"));
    //lastGateState = gateState;
    #endif
}

void Gate::Handle() {
    #ifdef _GATE_SERVO
//    if(Remote::remote.Channel().lcol < -658) Gate::gate.SetGateState(open);
//    else Gate::gate.SetGateState(close);
    #endif
}

Gate Gate::gate;
#endif

