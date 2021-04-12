/**
  ******************************************************************************
  * @FileName			    CallbackAdministration.cpp
  * @Description            CAN & Uart & TIM callback
  * @author                 Chang Liu & Steve Young
  * @note
  ******************************************************************************
  *
  * Copyright (c) 2021 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
**/

#include "includes.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == htim6.Instance) {
        HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
        MainControlLoop();
        HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    }
    if (htim->Instance == htim7.Instance) {
        Count();
        Remote::remote.Handle();
    }
}

//void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *hcan) {
//    if (hcan == CAN::can1.GetHcan()) {
//        CAN::can1.RxHandle();
//	}
//    else {
//        CAN::can2.RxHandle();
//	}
//    HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
//}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan == CAN::can1.GetHcan()) {
        CAN::can1.RxHandle();
    } else {
        CAN::can2.RxHandle();
    }
}


//void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef *hcan) {
//    if (hcan == &hcan1)
//        CAN::can1.SetTxDone(1);
//    if (hcan == &hcan2)
//        CAN::can2.SetTxDone(1);
//}
//void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan){
//    if (hcan == &hcan1)
//        CAN::can1.SetTxDone(1);
//    if (hcan == &hcan2)
//        CAN::can2.SetTxDone(1);
//}
//void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan){
//    if(HAL_CAN_GetError(hcan) & HAL_CAN_ERROR_TX_ALST0 || HAL_CAN_GetError(hcan) & HAL_CAN_ERROR_TX_TERR0){
//        if (hcan == &hcan1)
//            CAN::can1.SetTxDone(1);
//        if (hcan == &hcan2)
//            CAN::can2.SetTxDone(1);
//    }
//}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
    if (UartHandle == &RC_UART) {
//        Remote::remote.UartRxCpltCallback();
    }
    if (UartHandle == &AUTOAIM_UART) {
//        AutoAim::autoAim.UartRxCpltCallback();
    }
    if (UartHandle == &JUDGE_UART) {
//	  		JudgeUartRxCpltCallback();
    }
}

uint16_t errorState;
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle) {
//    errorState = UartHandle->ErrorCode;                             // 即rx_free = 1
//    uint32_t isrflags   = READ_REG(UartHandle->Instance->SR);// 手册上有讲 清错误都要先读SR [应该是状态寄存器]
    __HAL_UART_CLEAR_OREFLAG(UartHandle);

//    if((__HAL_UART_GET_FLAG(UartHandle, UART_FLAG_PE))!=RESET)    //  读PE[奇偶校验错误 Parity Error] + 做判断
//    {
//        READ_REG(UartHandle->Instance->DR);                  //  读DR[数据寄存器]
//        __HAL_UART_CLEAR_FLAG(UartHandle, UART_FLAG_PE);          //  清除 PE flag
//        UartHandle->gState = HAL_UART_STATE_READY;                //  gState 与全局句柄管理相关 也与发送操作相关的UART状态
//        UartHandle->RxState = HAL_UART_STATE_READY;               //  和串口接收相关的状态
//    }                                                             //  重复事项不再赘述
//
//    if((__HAL_UART_GET_FLAG(UartHandle, UART_FLAG_FE))!=RESET)    //  读FE[Frame Error] + 做判断
//    {
//        READ_REG(UartHandle->Instance->DR);                  //  读DR
//        __HAL_UART_CLEAR_FLAG(UartHandle, UART_FLAG_FE);          //  清FR标志
//        UartHandle->gState = HAL_UART_STATE_READY;
//        UartHandle->RxState = HAL_UART_STATE_READY;
//    }
//    if((__HAL_UART_GET_FLAG(UartHandle, UART_FLAG_NE))!=RESET)    // 读NE标志[Noise Error] + 做判断
//    {
//        READ_REG(UartHandle->Instance->DR);                  // 读DR
//        __HAL_UART_CLEAR_FLAG(UartHandle, UART_FLAG_NE);          // 清NE标志
//        UartHandle->gState = HAL_UART_STATE_READY;
//        UartHandle->RxState = HAL_UART_STATE_READY;
//    }
//
//    if((__HAL_UART_GET_FLAG(UartHandle, UART_FLAG_ORE))!=RESET)   //  读ORE[Overrun Error] + 做判断
//    {
//        READ_REG(UartHandle->Instance->CR1);                 //  读CR1 [USART控制寄存器]
//        __HAL_UART_CLEAR_FLAG(UartHandle, UART_FLAG_ORE);         //  清ORE标志
//        UartHandle->gState = HAL_UART_STATE_READY;
//        UartHandle->RxState = HAL_UART_STATE_READY;
//    }
    //  重新开卡死的串口
    if (UartHandle == &RC_UART) {
        Remote::remote.Reset();
    } else if (UartHandle == &JUDGE_UART) {
        InitJudgeUart();
    } else if (UartHandle == &AUTOAIM_UART) {
#ifdef _AUTOAIM
        AutoAim::autoAim.Reset();
#endif /*USE_AUTOAIM*/
    }
}
void HAL_UART_AbortReceiveCpltCallback (UART_HandleTypeDef *huart) {
    if (huart == &JUDGE_UART) {
        JudgeUartRxCpltCallback();
    } else if (huart == &RC_UART) {
        Remote::remote.UartRxCpltCallback();
    } else if (huart == &AUTOAIM_UART) {
        AutoAim::autoAim.UartRxCpltCallback();
    }
}
