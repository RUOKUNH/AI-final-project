/**
  ******************************************************************************
  * @FileName			    RemoteTask.cpp
  * @Description            Receive the data from remote and switch the mode(FSM)
  * @author                 Steve Young
  * @note
  ******************************************************************************
  *
  * Copyright (c) 2021 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
**/

//#include <RemoteTask.h>

#include "includes.h"
#include "MotorTask.h"

void Remote::Reset() {
    inputMode = STOP;
    functionMode = UPPER_POS;
    rc_first_frame = 0;
    rc_cnt = 0;
    IWDG_counter = 0;

    channel.rrow = 0;
    channel.rcol = 0;
    channel.lrow = 0;
    channel.lcol = 0;
	
	frequency=0;

    if (HAL_UART_Receive_DMA(&RC_UART, rc_data, 50) != HAL_OK) {
        Error_Handler();
    }
}
void Remote::GetRemoteSwitchAction(RemoteSwitch_t *sw, uint8_t val) {
    static uint32_t switch_cnt = 0;

    sw->switch_value_raw = val;
    sw->switch_value_buf[sw->buf_index] = sw->switch_value_raw;

    //value1 value2ֵʵһ
    //value14λʼΪ0
    sw->switch_value1 = (sw->switch_value_buf[sw->buf_last_index] << 2) |
                        (sw->switch_value_buf[sw->buf_index]);

    sw->buf_end_index = (sw->buf_index + 1) % REMOTE_SWITCH_VALUE_BUF_DEEP;

    sw->switch_value2 = (sw->switch_value_buf[sw->buf_end_index] << 4) | sw->switch_value1;

    //һûиݣ˲
    if (sw->switch_value_buf[sw->buf_index] == sw->switch_value_buf[sw->buf_last_index]) {
        switch_cnt++;
    } else {
        switch_cnt = 0;
    }
    //άһʱ䣬40֡һݣѲдswitch_long_value
    if (switch_cnt >= 40) {
        sw->switch_long_value = sw->switch_value_buf[sw->buf_index];
    }
    //ָһ
    sw->buf_last_index = sw->buf_index;
    sw->buf_index++;
    if (sw->buf_index == REMOTE_SWITCH_VALUE_BUF_DEEP) {
        sw->buf_index = 0;
    }
}

void Remote::RemoteDataProcess(uint8_t *pData) {
    IWDG_counter = 0;

    if (pData == nullptr) return;

    //ң 11*4 + 2*2 = 48Ҫ 6 Bytes
    //16λֻ11λ
    RC_CtrlData.rc.ch0 = ((int16_t) pData[0] | ((int16_t) pData[1] << 8)) & 0x07FF;
    RC_CtrlData.rc.ch1 = (((int16_t) pData[1] >> 3) | ((int16_t) pData[2] << 5)) & 0x07FF;
    RC_CtrlData.rc.ch2 = (((int16_t) pData[2] >> 6) | ((int16_t) pData[3] << 2) |
                          ((int16_t) pData[4] << 10)) & 0x07FF;
    RC_CtrlData.rc.ch3 = (((int16_t) pData[4] >> 1) | ((int16_t) pData[5] << 7)) & 0x07FF;

    //16λֻλ
    RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
    RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);

    //Ҫ 8 Bytes
    RC_CtrlData.mouse.x = ((int16_t) pData[6]) | ((int16_t) pData[7] << 8);
    RC_CtrlData.mouse.y = ((int16_t) pData[8]) | ((int16_t) pData[9] << 8);
    RC_CtrlData.mouse.z = ((int16_t) pData[10]) | ((int16_t) pData[11] << 8);

    RC_CtrlData.mouse.press_l = pData[12];
    RC_CtrlData.mouse.press_r = pData[13];

    //Ҫ 2 Bytes = 16 bits ÿһλӦһ
    RC_CtrlData.key.v = ((int16_t) pData[14]) | ((int16_t) pData[15] << 8);

    //״̬
    if (RC_CtrlData.rc.s2 == 1) inputMode = REMOTE_INPUT;
    else if (RC_CtrlData.rc.s2 == 3) inputMode = KEY_MOUSE_INPUT;
    else inputMode = STOP;

    if (RC_CtrlData.rc.s1 == 1) functionMode = UPPER_POS;
    else if (RC_CtrlData.rc.s1 == 3) functionMode = MIDDLE_POS;
    else functionMode = LOWER_POS;

    GetRemoteSwitchAction(&g_switch1, RC_CtrlData.rc.s1);
   // RemoteKeyMouseControlLoop();
}
void Remote::RemoteDataRecv() {
    rc_cnt++;
	
    if (rx_free == 1) {
        RemoteDataProcess(rc_data);
        //看门狗，超过17ms没有收到就重新开始UART
        if ((rc_cnt <= 17) && (rc_first_frame == 1)) {
            rx_free = 0;
            rc_cnt = 0;
        } else {
            if (rc_first_frame == 0) {
                rc_cnt = 0;
                rc_first_frame = 1;
            }
        }
				
       
    }
  
}
//根据遥控器拨杆情况切换键鼠模式和遥控器模式
void Remote::RemoteKeyMouseControlLoop() {
    switch (Remote::remote.GetInputMode()) {
        case REMOTE_INPUT: {
            if (Car::car.GetWorkState() > 0) {
                Remote::remote.LeverControl();
            }
        }
            break;
        case KEY_MOUSE_INPUT: {
            if (Car::car.GetWorkState() > 0) {
                Remote::remote.KeyMouseControl();
            }
        }
            break;
        case STOP: {

        }
            break;
    }
}
void Remote::RemoteDataGet() {
    RemoteDataRecv();
    channel.rrow = (RC_CtrlData.rc.ch0 - (int16_t) REMOTE_CONTROLLER_STICK_OFFSET);
    channel.rcol = (RC_CtrlData.rc.ch1 - (int16_t) REMOTE_CONTROLLER_STICK_OFFSET);
    channel.lrow = (RC_CtrlData.rc.ch2 - (int16_t) REMOTE_CONTROLLER_STICK_OFFSET);
    channel.lcol = (RC_CtrlData.rc.ch3 - (int16_t) REMOTE_CONTROLLER_STICK_OFFSET);
    mouse = RC_CtrlData.mouse;
    key = RC_CtrlData.key;
    if (abs(channel.rrow) < IGNORE_RANGE) {
        channel.rrow = 0;
    }
    if (abs(channel.rcol) < IGNORE_RANGE) {
        channel.rcol = 0;
    }
    if (abs(channel.lrow) < IGNORE_RANGE) {
        channel.lrow = 0;
    }
    if (abs(channel.lcol) < IGNORE_RANGE) {
        channel.lcol = 0;
    }
}
//超过一定时间没有收到数据就直接reset程序
void Remote::IWDG_Handle() {
#ifdef BOARD_SLAVE
    HAL_IWDG_Refresh(&hiwdg);
#endif
    if (IWDG_counter < 1000) IWDG_counter++;
    if (IWDG_counter > 100) {
#ifndef _NCOMPLETE_TEST
        //遥控器离线导致持续stop模式
        inputMode = STOP;
#endif
    }
    if (IWDG_counter < 100) HAL_IWDG_Refresh(&hiwdg);
}

void Remote::Handle() {
    IWDG_Handle();
    RemoteDataGet();
    RemoteKeyMouseControlLoop();
}

void Remote::UartRxCpltCallback() {
    if(RECV_LEN(RC_UART) % 18 == 0) {
        rx_free = 1;
    }
    HAL_UART_Receive_DMA(&RC_UART, rc_data, 50);
}

Remote Remote::remote;
