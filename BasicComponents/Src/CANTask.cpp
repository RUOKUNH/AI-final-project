/**
  ******************************************************************************
  * @FileName			    CANTask.cpp
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

#include "includes.h"
//从CAN接收数据并写入到相应位置
void CAN::RxHandle(void) {
    HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxMsg,RxData);
    uint8_t sort = (hcan == &hcan1) ? 0 : 1; //use hcan to determine whether it's CAN1 or CAN2
    if(RxMsg.StdId>=0x201 && RxMsg.StdId<=0x208) {
        if (Motor::motors[sort][RxMsg.StdId - 0x201] != nullptr) {
            Motor::motors[sort][RxMsg.StdId - 0x201]->RxFromCan(RxData);
            Motor::motors[sort][RxMsg.StdId - 0x201]->everRx = 1;
        }
    }else{
        for(uint8_t i=2;i<deviceNum;i++){
            if(CANDevice[i].RxID == RxMsg.StdId){
                CANDevice[i].RxHandler(RxData);
                break;
            }
        }
    }

}
//发送给电机
void CAN::TxHandleMotor(uint8_t part) {
    uint8_t sort = (hcan == &hcan2); //determine whether it's CAN1 or CAN2
    //<frame header>
    if (part == 0)
        TxMsg.StdId = 0x200;
    else
        TxMsg.StdId = 0x1ff;
    TxMsg.ExtId = 0;
    TxMsg.IDE = CAN_ID_STD;
    TxMsg.RTR = CAN_RTR_DATA;
    TxMsg.DLC = 0x08;
    //</frame header>

    //<encode data to be sent to motor>
    for (int i = 0; i < 4; ++i) {
        if (Motor::motors[sort][i + part * 4] == nullptr) {
            TxData[i * 2] = 0;
            TxData[i * 2 + 1] = 0;
        } else {
            TxData[i * 2] = (uint8_t) (Motor::motors[sort][i + part * 4]->GetIntensity() >> 8);
            TxData[i * 2 + 1] = (uint8_t) (Motor::motors[sort][i + part * 4]->GetIntensity());
        }
    }
    //</encode data to be sent to motor>

    //<transmit>
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0) {
        HAL_CAN_AddTxMessage(hcan, &TxMsg, TxData, (uint32_t *) CAN_TX_MAILBOX0);
    }
    //</transmit>
}
//发送给CAN设备
void CAN::TxHandleDevices(CANDevice_t device) {
    TxMsg.StdId = device.TxID;
    TxMsg.ExtId = 0;
    TxMsg.IDE = CAN_ID_STD;
    TxMsg.RTR = CAN_RTR_DATA;
    TxMsg.DLC = 0x08;

    device.TxHandler(TxData);
    //</transmit>
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan)>0) {
        HAL_CAN_AddTxMessage(hcan,&TxMsg,TxData,(uint32_t*)CAN_TX_MAILBOX0);
    }
    //</transmit>
}

void CAN::Reset(CanType_e type, uint8_t motorPart1, uint8_t motorPart2) {
    HAL_StatusTypeDef HAL_Status;

    currentSendData = 0;
    sendData[0] = motorPart1;
    sendData[1] = motorPart2;
    deviceNum = 2;
    if (type == CAN_TYPE_1)
        hcan = &hcan1;
    else
        hcan = &hcan2;
    //<CAN filter>
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.SlaveStartFilterBank = 0;

    HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
    //</CAN filter>
    HAL_CAN_Start(hcan);
    HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_Status = HAL_CAN_ActivateNotification(hcan,CAN_IT_TX_MAILBOX_EMPTY);
    if(HAL_Status != HAL_OK){
        Error_Handler();
    }
}
void CAN::addDevice(uint32_t RxID, uint32_t TxID, void (*RxHandler)(uint8_t *),void (*TxHandler)(uint8_t *)){
    CANDevice[deviceNum].RxHandler = RxHandler;
    CANDevice[deviceNum].TxHandler = TxHandler;
    CANDevice[deviceNum].RxID = RxID;
    CANDevice[deviceNum].TxID = TxID;
    sendData[deviceNum] = 1;
    deviceNum++;
}
//依次轮换发送信息给电机和其他CAN设备
void CAN::TxHandle(void) {
    uint8_t cnt = 0;
    //<switch object to send CAN data to>
    do {
        currentSendData = (currentSendData + 1) % deviceNum;
        cnt++;
    } while (sendData[currentSendData] == 0 && cnt <= deviceNum);
    //</switch object to send CAN data to>
    if(sendData[currentSendData]) {
        if (currentSendData == 0) TxHandleMotor(0);
        else if (currentSendData == 1) TxHandleMotor(1);
        else {
            TxHandleDevices(CANDevice[currentSendData]);
        }
    }
}

CAN CAN::can1;
CAN CAN::can2;
