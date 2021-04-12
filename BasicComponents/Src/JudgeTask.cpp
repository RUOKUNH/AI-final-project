/**
  ******************************************************************************
  * @FileName			    JudgeTask.h
  * @Description            Receive the data from the Judge and transmmit
  * @author                 Yunhao Wang
  * @note
  ******************************************************************************
  *
  * Copyright (c) 2021 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
**/

#include "includes.h"
#include "offical_Judge_Handler.h"
#include <stdlib.h>
#include <string.h>

static void Referee_Update_GameStatus(uint8_t * head);

static void Referee_Update_RobotHP(uint8_t * head);

static void Referee_Update_EventData(uint8_t * head);

static void Referee_Update_DartStatus(uint8_t * head);

static void Referee_Update_DartRemaining(uint8_t * head);

static void Referee_Update_Warning(uint8_t * head);

static void Referee_Update_RobotState(uint8_t * head);

static void Referee_Update_PowerHeatData(uint8_t * head);

static void Referee_Update_RobotPos(uint8_t * head);

static void Referee_Update_Buff(uint8_t * head);

static void Referee_Update_AerialEnergy(uint8_t * head);

static void Referee_Update_RobotHurt(uint8_t * head);

static void Referee_Update_ShootData(uint8_t * head);

static void Referee_Update_BulletRemaining(uint8_t * head);

static void Referee_Update_RfidStatus(uint8_t * head);

static void Referee_Update_DartClient(uint8_t * head);

static void Referee_Update_RobotWarps(uint8_t * head);

static void Referee_Update_GameResult(uint8_t * head);

static void Referee_Update_Supply(uint8_t * head);

static void Refresh_Client_Data(void);

static void Referee_Transmit_UserData(void);

static void Client_Graph_Process(void);

static RefereeDataInfo_t RefereeDataInfo[] =
        {{0x0001,11,Referee_Update_GameStatus},
         {0x0002,1,Referee_Update_GameResult},
         {0x0003,32,Referee_Update_RobotHP},
         {0x0004,3,Referee_Update_DartStatus},
         {0x0101,4,Referee_Update_EventData},
         {0x0102,3,Referee_Update_Supply},
         {0x0104,2,Referee_Update_Warning},
         {0x0105,1,Referee_Update_DartRemaining},
         {0x0201,27,Referee_Update_RobotState},
         {0x0202,16,Referee_Update_PowerHeatData},
         {0x0203,16,Referee_Update_RobotPos},
         {0x0204,1,Referee_Update_Buff},
         {0x0205,3,Referee_Update_AerialEnergy},
         {0x0206,1,Referee_Update_RobotHurt},
         {0x0207,7,Referee_Update_ShootData},
         {0x0208,2,Referee_Update_BulletRemaining},
         {0x0209,4,Referee_Update_RfidStatus},
         {0x020A,12,Referee_Update_DartClient}
//         {0x0301,12,Referee_Update_DartClient},
//         {0x0301,12,Referee_Update_DartClient}
        };
uint8_t RefereeDataInfoNum = 18;

uint8_t JUDGE_Received = 0;
JudgeState_e JUDGE_State = OFFLINE;
referee_data_t RefereeData;
client_custom_data_t custom_data;
graphic_data_struct_t client_graph;
Pack *pack;
Robot_Warps_data_t robot_warps;

uint8_t buf0[90];
				
				


uint32_t judge_cnt = 0;
uint32_t enemy_buff_remaining = 20;
robot_status_t cur_robot_status;

int8_t client_graphic_steps = -9;
uint8_t client_graphic_busy = 0;
ext_client_custom_graphic_single_t Client_Custom_Graphic_Single;

uint8_t tmp_judge;
				 
uint8_t receiving = 0;
uint8_t received = 0;
uint8_t buffer[200] = {0};
uint8_t buffercnt = 0;
uint16_t cmdID;
uint16_t dataLen;

void InitJudgeUart(void) {
    Referee_Transmit();
    //srand((unsigned)rc_cnt);
    if (HAL_UART_Receive_DMA(&JUDGE_UART, buffer, 200) != HAL_OK) {
        //Error_Handler();
    }
    //tx_free = 0;
}

void DivideBit_int(int32_t a) {
    client_graph.radius = a & 0x3ff;      //取前十位
    client_graph.end_x = (a >> 10) & 0x7ff; //取中间11位
    client_graph.end_y = (a >> 21) & 0x7ff;    //取最后十一位
}

void DivideBit_float(float a) {
    pack = (Pack *) (&a);
    client_graph.radius = pack->radius;
    client_graph.end_x = pack->end_x;
    client_graph.end_y = pack->end_y;
}

void JudgeUartRxCpltCallback(void) {
    static uint32_t RecvSize;
    static uint8_t found;
    buffercnt = 0;
    RecvSize = RECV_LEN(JUDGE_UART);
    while (buffercnt < RecvSize) {
        if (buffer[buffercnt] == 0xA5 && myVerify_CRC8_Check_Sum(buffer + buffercnt, 5)) {
            dataLen = *(uint16_t *) (buffer + buffercnt + 1);
            if (myVerify_CRC16_Check_Sum(buffer + buffercnt, 7 + 2 + dataLen)) {
                cmdID = *(uint16_t *) (buffer + 5 + buffercnt);
                found = 0;
                //**************************裁判系统收包****************
                for (uint8_t i = 0; i < RefereeDataInfoNum; i++) {
							
                    if ( cmdID == RefereeDataInfo[i].cmdID /*&& dataLen == RefereeDataInfo[i].len*/) {
                        found = 1;
                        JUDGE_Received = 1;
                        RefereeDataInfo[i].decodingFunction(buffer + buffercnt);
                        buffercnt += dataLen + 9;
                        RefereeData.len[i]=dataLen;
                        break;
                    }
                }
                //**************************裁判系统收包****************
                if (!found) {
                    buffercnt++;
                }
            } else {
                buffercnt++;
            }
        } else {
            buffercnt++;
        }
    }
    HAL_UART_Receive_DMA(&JUDGE_UART, buffer, 200);
		 //Referee_Transmit();
}

void refreshJudgeState(void) {
    static int s_count_judge = 0;
    if (JUDGE_Received == 1) {
        s_count_judge = 0;
        JUDGE_State = ONLINE;
        JUDGE_Received = 0;
    } else {
        s_count_judge++;
        if (s_count_judge > 300)//300ms
        {
            JUDGE_State = OFFLINE;
        }
    }
    judge_cnt++;
    if (judge_cnt == 1000 && enemy_buff_remaining > 0) {
        judge_cnt = 0;
        enemy_buff_remaining--;
    } else if (judge_cnt == 1000 && enemy_buff_remaining == 0) {
        judge_cnt = 0;
    }
}

void Referee_Update_GameStatus(uint8_t * head) {
    RefereeData.GameStatus = *(ext_game_status_t*)(head+7);
}

void Referee_Update_RobotHP(uint8_t * head) {
    RefereeData.RobotHP = *(ext_game_robot_HP_t*)(head+7);
}

void Referee_Update_EventData(uint8_t * head) {

}

void Referee_Update_DartStatus(uint8_t * head) {

}

void Referee_Update_DartRemaining(uint8_t * head) {

}

void Referee_Update_GameResult(uint8_t * head){

}

void Referee_Update_Supply(uint8_t * head){

}

void Referee_Update_Warning(uint8_t * head) {
    RefereeData.refereeWarning = *(ext_referee_warning_t*)(head+7);
}

void Referee_Update_RobotState(uint8_t * head) {
    RefereeData.GameRobotState = *(ext_game_robot_status_t*)(head+7);
}

void Referee_Update_PowerHeatData(uint8_t * head) {
    RefereeData.PowerHeat = *(ext_power_heat_data_t *)(head+7);
}

void Referee_Update_RobotPos(uint8_t * head) {
    RefereeData.RobotPos = *(ext_game_robot_pos_t*)(head+7);
}

void Referee_Update_Buff(uint8_t * head) {
    RefereeData.Buff.power_rune_buff = *(head+7);
}

void Referee_Update_AerialEnergy(uint8_t * head) {
    RefereeData.AreialRobotEnergy = *(ext_aerial_robot_energy_t*)(head+7);
}

void Referee_Update_RobotHurt(uint8_t * head) {
    RefereeData.RobotHurt = *(ext_robot_hurt_t*)(head+7);
}

void Referee_Update_ShootData(uint8_t * head) {
    RefereeData.ShootData = *(ext_shoot_data_t*)(head+7);
}

void Referee_Update_BulletRemaining(uint8_t * head) {
    //RefereeData.bulletRemaining.bullet_remaining_num = jointbyte(*(head+8), *(head+7));
    RefereeData.bulletRemaining = *(ext_bullet_remaining_t*)(head+7);
}

void Referee_Update_RfidStatus(uint8_t * head) {
    RefereeData.RfidStatus.rfid_status = *(uint32_t *) (head + 7);
}

void Referee_Update_DartClient(uint8_t * head) {

}

void Referee_Update_RobotWarps(uint8_t * head) {
    robot_warps = *(Robot_Warps_data_t*)(head+7);
//    robot_warps.x1 = jointbyte(buffer[14], buffer[13]);
//    robot_warps.y1 = jointbyte(buffer[16], buffer[15]);
//    robot_warps.x2 = jointbyte(buffer[18], buffer[17]);
//    robot_warps.y2 = jointbyte(buffer[20], buffer[19]);
//    robot_warps.x3 = jointbyte(buffer[22], buffer[21]);
//    robot_warps.y3 = jointbyte(buffer[24], buffer[23]);
//    robot_warps.x4 = jointbyte(buffer[26], buffer[25]);
//    robot_warps.y4 = jointbyte(buffer[28], buffer[27]);
//    robot_warps.x5 = jointbyte(buffer[30], buffer[29]);
//    robot_warps.y5 = jointbyte(buffer[32], buffer[31]);
}


uint8_t check_buffer[60] = {0};
/*void Refresh_Client_Data()
{
}
*/
/*
void Referee_Transmit_UserData()
{
	Refresh_Client_Data();

	uint8_t buffer[28] = {0};
	uint8_t * ud1 = (uint8_t*)&custom_data.data1;
	uint8_t * ud2 = (uint8_t*)&custom_data.data2;
	uint8_t * ud3 = (uint8_t*)&custom_data.data3;

	//帧头
	buffer[0] = 0xA5;//数据帧起始字节，固定值为 0xA5
	buffer[1] = 19;//数据帧中 data 的长度,占两个字节
	buffer[2] = 0;
	buffer[3] = 1;//包序号
	buffer[4] = myGet_CRC8_Check_Sum(&buffer[0], 5-1, myCRC8_INIT);//帧头 CRC8 校验
	//cmd ID: 0x0301
	buffer[5] = 0x01;
	buffer[6] = 0x03;
	//数据的内容 ID:0xD180  ,占两个字节
	buffer[7] = 0x80;
	buffer[8] = 0xD1;
	//发送者的 ID, 占两个字节
	buffer[9] = RefereeData.GameRobotState.robot_id;
	buffer[10] = 0;
	//客户端的 ID, 只能为发送者机器人对应的客户端,  占两个字节
	buffer[11] = RefereeData.GameRobotState.robot_id;
	if(buffer[11]>9&&buffer[11]<16)buffer[11]+=6;
	buffer[12] = 0x01;
	//自定义数据
	for(int i=13;i<17;i++) buffer[i] = ud1[i-13];
	for(int i=17;i<21;i++) buffer[i] = ud2[i-17];
	for(int i=21;i<25;i++) buffer[i] = ud3[i-21];
	buffer[25] = custom_data.masks;
	//CRC16校验位，两个字节
	Append_CRC16_Check_Sum(buffer,sizeof(buffer));

	for(int i = 0; i < 28; i++)
	{
		check_buffer[i] = buffer[i];
	}

	HAL_UART_Transmit(&JUDGE_UART,(uint8_t *)&buffer,sizeof(buffer),0xff);
}*/

uint8_t debug_buffer[30] = {0};
uint8_t gragh_buffer[30]={0};
uint8_t char_buffer[60]={0};
void Referee_Transmit_ClientGraph() {
//    Client_Graph_Process();
    static uint16_t quanti;

    static uint8_t *buf=buf0;
    static uint8_t option;
    static uint8_t colors;

    quanti=(Cap::cap.GetCapVoltage()*Cap::cap.GetCapVoltage()-121)/(23*23-121)*200;
    INRANGE(quanti,0,200);

    option = (option+1)%10;
    uint8_t op = (option == 0)?1:2;
    colors = (quanti < 60)?4:8;
    memset(buf,0,90);
    buf[0]=0xA5;
    *(uint16_t*)(buf+1)=81;
//    buf[1]=0x33;
//    buf[2]=0x00;

    buf[3]=1;
    buf[4] = myGet_CRC8_Check_Sum(&buf[0], 5 - 1, myCRC8_INIT);//帧头 CRC8 校验

    *(uint16_t*)(buf+5)=0x0301;
//    buf[5]=0x01;
//    buf[6]=0x03;
    *(uint16_t*)(buf+7)=0x0103;
//    buf[7]=0x10;
//    buf[8]=0x01;
    *(uint16_t*)(buf+9)=RefereeData.GameRobotState.robot_id;
//    buf[9]=0x03;
//    buf[10]=0x00;
//    *(uint16_t*)(buf+11)=0x0103;
    buf[11]=RefereeData.GameRobotState.robot_id;
    buf[12]=0x01;

    client_graph.graphic_name[0]=0;
    client_graph.graphic_name[1]=0;
    client_graph.graphic_name[2]=0;
    client_graph.operate_type=op;
    client_graph.graphic_type=1;
    client_graph.layer=0;
    client_graph.color=colors;
    client_graph.start_angle=40;
    client_graph.end_angle=0;
    client_graph.width=4;
    client_graph.start_x=0;
    client_graph.start_y=700;
    client_graph.radius=0;
    client_graph.end_x=200;
    client_graph.end_y=750;

    *(graphic_data_struct_t*)(buf+13)=client_graph;

    client_graph.graphic_name[0]=0;
    client_graph.graphic_name[1]=0;
    client_graph.graphic_name[2]=1;
    client_graph.operate_type=op;
    client_graph.graphic_type=0;
    client_graph.layer=0;
    client_graph.color=colors;
    client_graph.start_angle=40;
    client_graph.end_angle=0;
    client_graph.width=4;
    client_graph.start_x=quanti;
    client_graph.start_y=700;
    client_graph.radius=0;
    client_graph.end_x=quanti;
    client_graph.end_y=750;

    *(graphic_data_struct_t*)(buf+13+15)=client_graph;

    client_graph.graphic_name[0]=0;
    client_graph.graphic_name[1]=0;
    client_graph.graphic_name[2]=2;
    client_graph.operate_type=op;
    client_graph.graphic_type=1;
    client_graph.layer=0;
    client_graph.color=colors;
    client_graph.start_angle=40;
    client_graph.end_angle=0;
    client_graph.width=4;
    client_graph.start_x=200;
    client_graph.start_y=715;
    client_graph.radius=0;
    client_graph.end_x=230;
    client_graph.end_y=735;

    *(graphic_data_struct_t*)(buf+13+30)=client_graph;

    client_graph.graphic_name[0]=0;
    client_graph.graphic_name[1]=0;
    client_graph.graphic_name[2]=3;
    client_graph.operate_type=op;
    client_graph.graphic_type=0;
    client_graph.layer=0;
    client_graph.color=colors;
    client_graph.start_angle=40;
    client_graph.end_angle=0;
    client_graph.width=4;
    client_graph.start_x=0;
    client_graph.start_y=716;
    client_graph.radius=0;
    client_graph.end_x=quanti;
    client_graph.end_y=716;

    *(graphic_data_struct_t*)(buf+13+45)=client_graph;

    client_graph.graphic_name[0]=0;
    client_graph.graphic_name[1]=0;
    client_graph.graphic_name[2]=0;
    client_graph.operate_type=op;
    client_graph.graphic_type=0;
    client_graph.layer=1;
    client_graph.color=colors;
    client_graph.start_angle=40;
    client_graph.end_angle=0;
    client_graph.width=4;
    client_graph.start_x=0;
    client_graph.start_y=734;
    client_graph.radius=0;
    client_graph.end_x=quanti;
    client_graph.end_y=734;

    *(graphic_data_struct_t*)(buf+13+60)=client_graph;
//    quanti--;
//    if(quanti == 0) quanti = 200;


//    buf[0]=0xA5;
//    *(uint16_t*)(buf+1)=36;
////    buf[1]=0x33;
////    buf[2]=0x00;
//
//    buf[3]=1;
//    buf[4] = myGet_CRC8_Check_Sum(&buf[0], 5 - 1, myCRC8_INIT);//帧头 CRC8 校验
//
//    *(uint16_t*)(buf+5)=0x0301;
////    buf[5]=0x01;
////    buf[6]=0x03;
//    *(uint16_t*)(buf+7)=0x0102;
////    buf[7]=0x10;
////    buf[8]=0x01;
//    *(uint16_t*)(buf+9)=0x0003;
////    buf[9]=0x03;
////    buf[10]=0x00;
//    *(uint16_t*)(buf+11)=0x0103;
//    buf[11]=0x03;
//    buf[12]=0x01;
//
//    client_graph.graphic_name[0]=0;
//    client_graph.graphic_name[1]=0;
//    client_graph.graphic_name[2]=0;
//    client_graph.operate_type=1;
//    client_graph.graphic_type=1;
//    client_graph.layer=0;
//    client_graph.color=4;
//    client_graph.start_angle=40;
//    client_graph.end_angle=0;
//    client_graph.width=4;
//    client_graph.start_x=0;
//    client_graph.start_y=750;
//    client_graph.radius=0;
//    client_graph.end_x=200;
//    client_graph.end_y=800;
//
//    *(graphic_data_struct_t*)(buf+13)=client_graph;
//
//    client_graph.graphic_name[0]=0;
//    client_graph.graphic_name[1]=0;
//    client_graph.graphic_name[2]=1;
//    client_graph.operate_type=2;
//    client_graph.graphic_type=0;
//    client_graph.layer=0;
//    client_graph.color=5;
//    client_graph.start_angle=40;
//    client_graph.end_angle=0;
//    client_graph.width=4;
//    client_graph.start_x=quanti;
//    client_graph.start_y=750;
//    client_graph.radius=0;
//    client_graph.end_x=quanti;
//    client_graph.end_y=800;
//
//    *(graphic_data_struct_t*)(buf+13+15)=client_graph;
//    quanti--;
//    if(quanti == 0) quanti = 200;
//    //帧头
//    buf[0] = 0xA5;//数据帧起始字节，固定值为 0xA5
//    buf[1] = 21;//数据帧中 data 的长度,占两个字节
//    buf[2] = 0;
//    buf[3] = 1;//包序号
//    buf[4] = myGet_CRC8_Check_Sum(&buffer[0], 5 - 1, myCRC8_INIT);//帧头 CRC8 校验
//    //cmd ID: 0x0301
//    buf[5] = 0x01;
//    buf[6] = 0x03;
//    //数据的内容 ID:0x0101  ,占两个字节
//    buf[7] = 0x01;
//    buf[8] = 0x01;
//    //发送者的 ID, 占两个字节
//    buf[9] = 0x03;
//    buf[10] = 0;
//    //客户端的 ID, 只能为发送者机器人对应的客户端,  占两个字节
//    buf[11] = 0x03;
//    buf[12] = 0x01;
//    //自定义图形数据
//    buf[13] = client_graph.graphic_name[0];
//    buf[14] = client_graph.graphic_name[1];
//    buf[15] = client_graph.graphic_name[2];
//
//
//    buf[16] = ((uint8_t) (client_graph.operate_type) | (uint8_t) (client_graph.graphic_type << 3) |
//                  (uint8_t) (client_graph.layer << 6));
////	client_graph.operate_type = 0;
//    buf[17] = ((uint8_t) (client_graph.layer >> 2) | (uint8_t) (client_graph.color << 2) |
//                  (uint16_t) (client_graph.start_angle << 6));
//    buf[18] = ((uint16_t) (client_graph.start_angle >> 2) | (uint16_t) (client_graph.end_angle << 7));
//    buf[19] = ((uint16_t) (client_graph.end_angle >> 1));
//
//    buf[20] = ((uint16_t) (client_graph.width));
//    buf[21] = ((uint16_t) (client_graph.width >> 8) | (uint16_t) (client_graph.start_x << 2));
//    buf[22] = ((uint16_t) (client_graph.start_x >> 6) | (uint16_t) (client_graph.start_y << 5));
//    buf[23] = ((uint16_t) (client_graph.start_y >> 3));
//
//    buf[24] = ((uint32_t) (client_graph.radius));
//    buf[25] = ((uint32_t) (client_graph.radius >> 8) | (uint32_t) (client_graph.end_x << 2));
//    buf[26] = ((uint32_t) (client_graph.end_x >> 6) | (uint32_t) (client_graph.end_y << 5));
//    buf[27] = ((uint32_t) (client_graph.end_y >> 3));
//    for (int i = 0; i < 30 - 2; i++) {
//        debug_buffer[i] = buf[i];
//    }
//
    Append_CRC16_Check_Sum(buf, 90);

    for (int i = 0; i < 30; i++) {
        check_buffer[i] = buf[i];
    }

    HAL_UART_Transmit_IT(&JUDGE_UART, (uint8_t *) buf, 90);

    client_graphic_busy = 0;
}

char Referee_Transmit_Char[30] = {0};

void Referee_Transmit_ClientChar() {
    Client_Graph_Process();

    uint8_t *buffer=char_buffer;

    //帧头
    buffer[0] = 0xA5;//数据帧起始字节，固定值为 0xA5
    buffer[1] = 51;//数据帧中 data 的长度,占两个字节
    buffer[2] = 0;
    buffer[3] = 1;//包序号
    buffer[4] = myGet_CRC8_Check_Sum(&buffer[0], 5 - 1, myCRC8_INIT);//帧头 CRC8 校验
    //cmd ID: 0x0301
    buffer[5] = 0x01;
    buffer[6] = 0x03;
    //数据的内容 ID:0x0110  ,占两个字节
    buffer[7] = 0x10;
    buffer[8] = 0x01;
    //发送者的 ID, 占两个字节
    buffer[9] = RefereeData.GameRobotState.robot_id;
    buffer[10] = 0;
    //客户端的 ID, 只能为发送者机器人对应的客户端,  占两个字节
    buffer[11] = (uint16_t) RefereeData.GameRobotState.robot_id;
    buffer[12] = 0x01;
    //自定义图形数据
    buffer[13] = client_graph.graphic_name[0];
    buffer[14] = client_graph.graphic_name[1];
    buffer[15] = client_graph.graphic_name[2];
    char Referee_Transmit_Cap[30] = {0};
    //sprintf(Referee_Transmit_Cap,"%d%c",75,'%');//超级电容的电量
    strcpy(Referee_Transmit_Char, Referee_Transmit_Cap);//测试用字符串
    client_graph.end_angle = sizeof(Referee_Transmit_Char);//字符串长度
    buffer[16] = ((uint8_t) (client_graph.operate_type) | (uint8_t) (client_graph.graphic_type << 3) |
                  (uint8_t) (client_graph.layer << 6));
    buffer[17] = ((uint8_t) (client_graph.layer >> 2) | (uint8_t) (client_graph.color << 2) |
                  (uint16_t) (client_graph.start_angle << 6));
    buffer[18] = ((uint16_t) (client_graph.start_angle >> 2) | (uint16_t) (client_graph.end_angle << 7));
    buffer[19] = ((uint16_t) (client_graph.end_angle >> 1));

    buffer[20] = ((uint16_t) (client_graph.width));
    buffer[21] = ((uint16_t) (client_graph.width >> 8) | (uint16_t) (client_graph.start_x << 2));
    buffer[22] = ((uint16_t) (client_graph.start_x >> 6) | (uint16_t) (client_graph.start_y << 5));
    buffer[23] = ((uint16_t) (client_graph.start_y >> 3));

    buffer[24] = ((uint8_t) (client_graph.radius));
    buffer[25] = ((uint16_t) (client_graph.radius >> 8) | (uint16_t) (client_graph.end_x << 2));
    buffer[26] = ((uint16_t) (client_graph.end_x >> 6) | (uint16_t) (client_graph.end_y << 5));
    buffer[27] = ((uint16_t) (client_graph.end_y >> 3));


    for (int i = 0; i < 30; i++) {
        buffer[i + 28] = Referee_Transmit_Char[i];
    }

    Append_CRC16_Check_Sum(buffer, sizeof(buffer));

    for (int i = 0; i < sizeof(buffer); i++) {
        check_buffer[i] = buffer[i];
    }

    HAL_UART_Transmit(&JUDGE_UART, (uint8_t *) &buffer, sizeof(buffer), 0xff);
    client_graphic_busy = 0;

}

void Referee_Transmit(void) {
    static uint8_t cnt = 0;
    cnt++;
    if(cnt == 150) {
//        if (client_graphic_steps == 10 || client_graphic_steps == -9)
//            Referee_Transmit_ClientChar();
//        else
            Referee_Transmit_ClientGraph();
        cnt = 0;
    }
}

void Client_Graph_Process(void) {
    if (client_graphic_busy)
        return;
    if (client_graphic_steps <= 0) {
        client_graph.operate_type = 1;
        client_graph.graphic_type = 0;
        client_graph.color = 0;
        client_graph.width = 0;
        client_graph.start_x = 0;
        client_graph.start_y = 0;
        client_graph.radius = 0;
        client_graph.end_x = 0;
        client_graph.end_y = 0;
        client_graph.start_angle = 0;
        client_graph.end_angle = 0;
    }
    switch (client_graphic_steps) {
        case -9:                         //全部初始化至空图层，此后只需要使用修改指令（operate_type=2）而非新增指令
        {
            client_graph.layer = 9;//case -9 对应图层9 显示超级电容剩余电量
            client_graph.operate_type = 1;
            client_graph.graphic_type = 7;
            client_graph.color = 3;
            client_graph.width = 5;
            client_graph.start_x = 960;
            client_graph.start_y = 600;
            client_graph.radius = 0;
            client_graph.end_x = 0;
            client_graph.end_y = 0;
            client_graph.start_angle = 40;
            client_graph.end_angle = 0;
            client_graph.graphic_name[0] = 'n';
            client_graph.graphic_name[1] = 'i';
            client_graph.graphic_name[2] = 'n';
            client_graphic_steps++;
        }
            break;
        case -8: {
            client_graph.layer = 8;
            client_graph.operate_type = 1;
            client_graph.graphic_type = 6;
            client_graph.color = 8;
            client_graph.width = 5;
            client_graph.start_x = 920;
            client_graph.start_y = 900;
            DivideBit_int(enemy_buff_remaining);
            //client_graph.radius = 8;
            //client_graph.end_x = 0;
            //client_graph.end_y = 0;
            client_graph.start_angle = 40;
            client_graph.end_angle = 0;
            client_graph.graphic_name[0] = 'e';
            client_graph.graphic_name[1] = 'i';
            client_graph.graphic_name[2] = 'g';
            client_graphic_steps++;
        }
            break;
        case -7: {
            client_graph.layer = 7;
            client_graphic_steps++;
            client_graph.graphic_name[0] = 's';
            client_graph.graphic_name[1] = 'e';
            client_graph.graphic_name[2] = 'v';
        }
            break;
        case -6: {
            client_graph.layer = 6;
            client_graphic_steps++;
            client_graph.graphic_name[0] = 's';
            client_graph.graphic_name[1] = 'i';
            client_graph.graphic_name[2] = 'x';
        }
            break;
        case -5: {
            client_graph.layer = 5;
            client_graphic_steps++;
            client_graph.graphic_name[0] = 'f';
            client_graph.graphic_name[1] = 'i';
            client_graph.graphic_name[2] = 'v';
        }
            break;
        case -4: {
            client_graph.layer = 4;
            client_graphic_steps++;
            client_graph.graphic_name[0] = 'f';
            client_graph.graphic_name[1] = 'o';
            client_graph.graphic_name[2] = 'u';
        }
            break;
        case -3: {
            client_graph.layer = 3;
            client_graphic_steps++;
            client_graph.graphic_name[0] = 't';
            client_graph.graphic_name[1] = 'h';
            client_graph.graphic_name[2] = 'r';
        }
            break;
        case -2: {
            client_graph.layer = 2;
            client_graphic_steps++;
            client_graph.graphic_name[0] = 't';
            client_graph.graphic_name[1] = 'w';
            client_graph.graphic_name[2] = 'o';
        }
            break;
        case -1: {
            client_graph.layer = 1;
            client_graphic_steps++;
            client_graph.graphic_name[0] = 'o';
            client_graph.graphic_name[1] = 'n';
            client_graph.graphic_name[2] = 'e';
        }
            break;
        case 0: {
            client_graph.layer = 0;
            client_graphic_steps++;
            client_graph.graphic_name[0] = 'z';
            client_graph.graphic_name[1] = 'e';
            client_graph.graphic_name[2] = 'r';
        }
            break;
        case 1:                  //目前是测试用图层（绘制小地图）               416*242 矩形
        {
            client_graph.layer = 0;//case 1 对应图层0 后面依次类推
            client_graph.operate_type = 2;
            client_graph.graphic_type = 1;
            client_graph.color = 0;
            client_graph.width = 5;
            client_graph.start_x = 1450;
            client_graph.start_y = 380;
            client_graph.radius = 0;
            client_graph.end_x = 1860;
            client_graph.end_y = 620;
            client_graph.start_angle = 0;
            client_graph.end_angle = 0;
            client_graph.graphic_name[0] = 'z';
            client_graph.graphic_name[1] = 'e';
            client_graph.graphic_name[2] = 'r';
            client_graphic_steps++;
        }
            break;
        case 2:                  //目前是测试用图层（绘制小地图）
        {
            client_graph.layer = 1;//case 2 对应图层1
            client_graph.operate_type = 2;
            client_graph.graphic_type = 0;
            client_graph.color = 0;
            client_graph.width = 3;
            client_graph.start_x = 1640;
            client_graph.start_y = 620;
            client_graph.radius = 0;
            client_graph.end_x = 1580;
            client_graph.end_y = 490;
            client_graph.start_angle = 0;
            client_graph.end_angle = 0;
            client_graph.graphic_name[0] = 'o';
            client_graph.graphic_name[1] = 'n';
            client_graph.graphic_name[2] = 'e';
            client_graphic_steps++;

        }
            break;
        case 3:                  //目前是测试用图层（绘制小地图）
        {
            client_graph.layer = 2;//case 3 对应图层2
            client_graph.operate_type = 2;
            client_graph.graphic_type = 0;
            client_graph.color = 0;
            client_graph.width = 3;
            client_graph.start_x = 1670;
            client_graph.start_y = 380;
            client_graph.radius = 0;
            client_graph.end_x = 1730;
            client_graph.end_y = 510;
            client_graph.start_angle = 0;
            client_graph.end_angle = 0;
            client_graph.graphic_name[0] = 't';
            client_graph.graphic_name[1] = 'w';
            client_graph.graphic_name[2] = 'o';
            client_graphic_steps++;
        }
            break;
        case 4:                  //目前是测试用图层（敌方车辆）
        {
            client_graph.layer = 3;//case 4 对应图层3
            client_graph.operate_type = 2;
            client_graph.graphic_type = 2;
            client_graph.color = 1;
            client_graph.width = 6;
            client_graph.start_x = robot_warps.x1;
            client_graph.start_y = robot_warps.y1;
            client_graph.radius = 4;
            client_graph.end_x = 0;
            client_graph.end_y = 0;
            client_graph.start_angle = 0;
            client_graph.end_angle = 0;
            client_graph.graphic_name[0] = 't';
            client_graph.graphic_name[1] = 'h';
            client_graph.graphic_name[2] = 'r';
            client_graphic_steps++;
        }
            break;
        case 5:                  //目前是测试用图层（敌方车辆）
        {
            client_graph.layer = 4;//case 5 对应图层4
            client_graph.operate_type = 2;
            client_graph.graphic_type = 2;
            client_graph.color = 1;
            client_graph.width = 6;
            client_graph.start_x = robot_warps.x2;
            client_graph.start_y = robot_warps.y2;
            client_graph.radius = 4;
            client_graph.end_x = 0;
            client_graph.end_y = 0;
            client_graph.start_angle = 0;
            client_graph.end_angle = 0;
            client_graph.graphic_name[0] = 'f';
            client_graph.graphic_name[1] = 'o';
            client_graph.graphic_name[2] = 'u';
            client_graphic_steps++;
        }
            break;
        case 6:                  //目前是测试用图层（敌方车辆）
        {
            client_graph.layer = 5;//case 6 对应图层5
            client_graph.operate_type = 2;
            client_graph.graphic_type = 2;
            client_graph.color = 1;
            client_graph.width = 6;
            client_graph.start_x = robot_warps.x3;
            client_graph.start_y = robot_warps.y3;
            client_graph.radius = 4;
            client_graph.end_x = 0;
            client_graph.end_y = 0;
            client_graph.start_angle = 0;
            client_graph.end_angle = 0;
            client_graph.graphic_name[0] = 'f';
            client_graph.graphic_name[1] = 'i';
            client_graph.graphic_name[2] = 'v';
            client_graphic_steps++;
        }
            break;
        case 7:                  //目前是测试用图层（敌方车辆）
        {
            client_graph.layer = 6;//case 7 对应图层6
            client_graph.operate_type = 2;
            client_graph.graphic_type = 2;
            client_graph.color = 1;
            client_graph.width = 6;
            client_graph.start_x = robot_warps.x4;
            client_graph.start_y = robot_warps.y4;
            client_graph.radius = 4;
            client_graph.end_x = 0;
            client_graph.end_y = 0;
            client_graph.start_angle = 0;
            client_graph.end_angle = 0;
            client_graph.graphic_name[0] = 's';
            client_graph.graphic_name[1] = 'i';
            client_graph.graphic_name[2] = 'x';
            client_graphic_steps++;
        }
            break;
        case 8:                  //目前是测试用图层（敌方车辆）
        {
            client_graph.layer = 7;//case 8 对应图层7
            client_graph.operate_type = 2;
            client_graph.graphic_type = 2;
            client_graph.color = 1;
            client_graph.width = 6;
            client_graph.start_x = robot_warps.x5;
            client_graph.start_y = robot_warps.y5;
            client_graph.radius = 4;
            client_graph.end_x = 0;
            client_graph.end_y = 0;
            client_graph.start_angle = 0;
            client_graph.end_angle = 0;
            client_graph.graphic_name[0] = 's';
            client_graph.graphic_name[1] = 'e';
            client_graph.graphic_name[2] = 'v';
            client_graphic_steps++;
        }
            break;
        case 9:                  //目前是测试用图层（倒计时）
        {
            client_graph.layer = 8;//case 9 对应图层8
            client_graph.operate_type = 2;
            client_graph.graphic_type = 6;
            client_graph.color = 8;
            client_graph.width = 5;
            client_graph.start_x = 920;
            client_graph.start_y = 900;
            DivideBit_int(enemy_buff_remaining);
            //client_graph.radius = 8;
            //client_graph.end_x = 0;
            //client_graph.end_y = 0;
            client_graph.start_angle = 40;
            client_graph.end_angle = 0;
            client_graph.graphic_name[0] = 'e';
            client_graph.graphic_name[1] = 'i';
            client_graph.graphic_name[2] = 'g';
            client_graphic_steps++;
        }
            break;
        case 10:                  //目前是测试用图层（超级电容）
        {
            client_graph.layer = 9;//case 10 对应图层9 显示超级电容剩余电量
            client_graph.operate_type = 1;
            client_graph.graphic_type = 7;
            client_graph.color = 3;
            client_graph.width = 5;
            client_graph.start_x = 900;
            client_graph.start_y = 700;
            client_graph.radius = 0;
            client_graph.end_x = 0;
            client_graph.end_y = 0;
            client_graph.start_angle = 40;
            client_graph.end_angle = 0;
            client_graph.graphic_name[0] = 'n';
            client_graph.graphic_name[1] = 'i';
            client_graph.graphic_name[2] = 'n';
            client_graphic_steps = 4;
        }
            break;
        default:
            break;
    }


    client_graphic_busy = 1;
}

void Client_Graph_Start(void) {
    if (client_graphic_steps == 0) {
        client_graphic_steps = 1;
    }
}

void Client_Graph_Clear(void) {
    client_graphic_steps = 1;
    client_graphic_busy = 1;

    client_graph.operate_type = 5;
    client_graph.graphic_type = 0;
    client_graph.layer = 0;
    for (int i = 0; i < 5; i++) {
        client_graph.graphic_name[i] = client_graphic_steps;
    }
    client_graph.start_x = 0;
    client_graph.start_y = 0;
    client_graph.radius = 0;
    client_graph.end_x = 0;
    client_graph.end_y = 0;
    client_graph.start_angle = 0;
    client_graph.end_angle = 0;
}
