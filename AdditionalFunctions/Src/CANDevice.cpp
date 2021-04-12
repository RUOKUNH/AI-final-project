#include "includes.h"

static void MASTER_BOARD_TxHandler(uint8_t *TxData){
    *(uint16_t *)(TxData) = (uint16_t)(Cap::cap.GetPowerVoltage() * 1000.0);
    *((uint16_t *)TxData+1) = (uint16_t)(Cap::cap.GetCapVoltage() * 1000.0);
    *((uint16_t *)TxData+2) = (uint16_t)(Cap::cap.GetOutVoltage() * 1000.0);
    switch (Cap::cap.GetCapState()) {
        case CAP_STATE_PREPARE:
            TxData[6] = 0x02;
            break;
        case CAP_STATE_EMERGENCY:
            TxData[6] = 0x03;
            break;
        case CAP_STATE_RELEASE:
            TxData[6] = 0x01;
            break;
        case CAP_STATE_STOP:
            TxData[6] = 0xff;
            break;
    }
    TxData[7] = 0;
}
static void MASTER_BOARD_RxHandler(uint8_t *RxData) {
    switch (RxData[0]) {
        case 0xff:
            Car::car.SetWorkState(STOP_STATE);
            break;
        case 0x00:
            Car::car.SetWorkState(PREPARE_STATE);
            break;
        case 0x01:
            Car::car.SetWorkState(NORMAL_STATE);
            break;
        case 0x02:
            Car::car.SetWorkState(ADDITIONAL_STATE_ONE);
            break;
        case 0x03:
            Car::car.SetWorkState(ADDITIONAL_STATE_TWO);
            break;
        default:
            Car::car.SetWorkState(STOP_STATE);
            break;
    }
    Cap::cap.RxAimedPower = RxData[1];
}
static void SLAVE_BOARD_TxHandler(uint8_t *TxData){
    switch (Car::car.GetWorkState()) {
        case STOP_STATE:
            TxData[0] = 0xff;
            break;
        case PREPARE_STATE:
            TxData[0] = 0x00;
            break;
        case NORMAL_STATE:
            TxData[0] = 0x01;
            break;
        case ADDITIONAL_STATE_ONE:
            TxData[0] = 0x02;
            break;
        case ADDITIONAL_STATE_TWO:
            TxData[0] = 0x03;
            break;
    }
    TxData[1] = (uint8_t)Cap::cap.GetAimedPower();
    TxData[2] = 0;
    TxData[3] = 0;
    TxData[4] = 0;
    TxData[5] = 0;
    TxData[6] = 0;
    TxData[7] = 0;
}
static void SLAVE_BOARD_RxHandler(uint8_t *RxData) {
    Cap::cap.SetRxPwrVoltage((float)(*(uint16_t *)RxData) / 1000.0f);
    Cap::cap.SetRxCapVoltage((float)(*((uint16_t *)RxData+1)) / 1000.0f);
    Cap::cap.SetRxOutVoltage((float)(*((uint16_t *)RxData+2)) / 1000.0f);
    Cap::cap.SetRxOutCurr((float)(*((uint16_t *)RxData+3)) / 100.0f);
//    switch (RxData[6]) {
//        case 0xff:
//            Cap::cap.SetRxCapState(CAP_STATE_STOP);
//            break;
//        case 0x01:
//            Cap::cap.SetRxCapState(CAP_STATE_RELEASE);
//            break;
//        case 0x02:
//            Cap::cap.SetRxCapState(CAP_STATE_PREPARE);
//            break;
//        case 0x03:
//            Cap::cap.SetRxCapState(CAP_STATE_EMERGENCY);
//            break;
//    }
    Cap::cap.SetRxCapState(CAP_STATE_RELEASE);
}

void CANDevice::Reset(){
    deviceNum = 0;
    //增加CAN设备（除了普通C6x0电调电机）：
    //1. 设置deviceNum
    //2. 直接给devices数组赋值以添加设备(示例如下)
#ifdef BOARD_SLAVE
    //**********************************示例*******************************
    deviceNum ++;
    devices[0] = {CAN_TYPE_1,0x301,0x300,MASTER_BOARD_TxHandler,MASTER_BOARD_RxHandler};
    //**********************************示例*******************************
#endif
#ifdef BOARD_MASTER
#ifdef _CAP
    //**********************************示例*******************************
    deviceNum ++;
    devices[0] = {CAN_TYPE_2,0x300,0x301,SLAVE_BOARD_TxHandler,SLAVE_BOARD_RxHandler};
    //**********************************示例*******************************
#endif
#endif
    //******************不用动*********************************************
    for(uint8_t i=0;i<deviceNum;i++){
        if(devices[i].type == CAN_TYPE_1){
            CAN::can1.addDevice(devices[i].RxID,devices[i].TxID,devices[i].RxHandler,devices[i].TxHandler);
        }else{
            CAN::can2.addDevice(devices[i].RxID,devices[i].TxID,devices[i].RxHandler,devices[i].TxHandler);
        }
    }
    //******************不用动*********************************************
}
CANDevice CANDevice::CANdevice;

