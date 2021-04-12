/**
  ******************************************************************************
  * @FileName			    CANTask.h
  * @Description            CAN communication
  * @author                 Chang Liu
  * @note
  ******************************************************************************
  *
  * Copyright (c) 2021 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
**/

#ifndef __CANTASK_H__
#define __CANTASK_H__

#include "includes.h"

#define CanRxGetU16(canRxMsg, num) (((uint16_t)canRxMsg[num * 2] << 8) | (uint16_t)canRxMsg[num * 2 + 1])

typedef enum {
    CAN_TYPE_1 = 1,
    CAN_TYPE_2
} CanType_e;

class CAN {
public:
    static CAN can1, can2;

    /**
     * @brief reset CAN communication
     */
    void Reset(CanType_e type, uint8_t motorPart1, uint8_t motorPart2);
    /**
     * @brief add an CAN device
     */
     //由CANdevice调用
    void addDevice(uint32_t RxID, uint32_t TxID, void (*RxHandler)(uint8_t *),void (*TxHandler)(uint8_t *));

    /**
     * @brief process received data
     */
    void RxHandle();

    /**
     * @brief data transmission handler
     */
    void TxHandle();

    /**
     * @brief get the can type
     */
    inline CAN_HandleTypeDef *GetHcan() { return hcan; }

private:
    typedef struct{
        uint32_t TxID;
        uint32_t RxID;
        void (*TxHandler)(uint8_t *);
        void (*RxHandler)(uint8_t *);
    }CANDevice_t;
    /**
    * @brief transmit data to motors
    */
    void TxHandleMotor(uint8_t part);

    /**
    * @brief transmit data to another board
    */
    void TxHandleDevices(CANDevice_t device);

    /**
     * @brief define CAN object is used or not
     * @param [0] first 4 motors
     * @param [1] last 4 motors
     * @param [2] doubl board
     */
    uint8_t sendData[10];
    CANDevice_t CANDevice[10];
    uint8_t deviceNum;
    uint8_t currentSendData;
    CAN_HandleTypeDef *hcan;
    CAN_RxHeaderTypeDef RxMsg;
    CAN_TxHeaderTypeDef TxMsg;
    uint8_t RxData[8];
    uint8_t TxData[8];
};

#endif
