#ifndef __CANDEVICE__
#define __CANDEVICE__

#include "CANDevice.h"
#include "includes.h"

//init and handle CAN devices (excluding normal C6x0 motors)
class CANDevice{
    //structure storing information of a CANDevice.
    typedef struct{
        CanType_e type;
        uint32_t TxID;
        uint32_t RxID;
        void (*TxHandler)(uint8_t *);
        void (*RxHandler)(uint8_t *);
    }CANDevice_t;
    //a list of CAN devices
    CANDevice_t devices[10];
    //the number of all devices
    uint8_t deviceNum;
public:
    //static object of this class
    static CANDevice CANdevice;
    //init all CAN devices
    void Reset();
};

#endif