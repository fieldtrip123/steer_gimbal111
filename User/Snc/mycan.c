//
// Created by Administrator on 2024/12/4.
//

#include "mycan.h"
#include "gimbal_control.h"
motor_t  shoot_m3508_info[2];
motor_t  shoot_m2006_info;
motor_t  yaw_6020_info;
motor_t  pitch_6020_info;
boardA_info boardA_info1;

void my_can1_init ()
{
    CAN_FilterTypeDef  can_filter;
    for(uint8_t i=0;i<14;i++) {
        can_filter.FilterActivation = CAN_FILTER_ENABLE;
        can_filter.FilterBank = i;
        can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
        can_filter.FilterIdHigh = 0;
        can_filter.FilterIdLow = 0 ;
        can_filter.FilterMaskIdHigh = 0 ;
        can_filter.FilterMaskIdLow = 0;
        can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
        can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
        can_filter.SlaveStartFilterBank=14;

        HAL_CAN_ConfigFilter(&hcan1, &can_filter);
    }
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
}

void my_can2_init ()
{
    CAN_FilterTypeDef  can_filter2;

    can_filter2.FilterActivation=  CAN_FILTER_ENABLE;
    can_filter2.FilterBank=              14;
    can_filter2.FilterFIFOAssignment=CAN_FILTER_FIFO1;
    can_filter2.FilterIdHigh=          0x205<<5;
    can_filter2.FilterIdLow=           0x206<<5;
    can_filter2.FilterMaskIdHigh=      0x207<<5;
    can_filter2.FilterMaskIdLow=       0x208<<5;
    can_filter2.FilterMode=       CAN_FILTERMODE_IDLIST;
    can_filter2.FilterScale=      CAN_FILTERSCALE_16BIT;

    HAL_CAN_ConfigFilter(&hcan2,&can_filter2);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO1_MSG_PENDING);
}


void set_m3508_v(int16_t v1,int16_t v2)
{
    CAN_TxHeaderTypeDef  tx_header1;
    uint8_t              tx_data[4];

    tx_header1.StdId= 0x1ff;  //  0x200(1~4) or 0x1ff(5~8)
    tx_header1.DLC =8;
    tx_header1.IDE=CAN_ID_STD;
    tx_header1.RTR=CAN_RTR_DATA;

    tx_data[0]=   (v1)>>8 &  0xff;
    tx_data[1]=    v1     &  0xff;
    tx_data[2]=   (v2)>>8 &  0xff;
    tx_data[3]=    v2     &  0xff;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
    {
        HAL_CAN_AddTxMessage(&hcan1, &tx_header1, tx_data, (uint32_t *)CAN_TX_MAILBOX0);
    }
}

void set_m2006_v(int16_t v1)
{
    CAN_TxHeaderTypeDef  tx_header1;
    uint8_t              tx_data[6];

    tx_header1.StdId= 0x200;  //  0x200(1~4) or 0x1ff(5~8)
    tx_header1.DLC =8;
    tx_header1.IDE=CAN_ID_STD;
    tx_header1.RTR=CAN_RTR_DATA;

    tx_data[4]=   (v1)>>8 &  0xff;
    tx_data[5]=    v1     &  0xff;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
    {
        HAL_CAN_AddTxMessage(&hcan1, &tx_header1, tx_data, (uint32_t *)CAN_TX_MAILBOX0);
    }
}
void set_m6020_v(int16_t v1,int16_t v2)
{
    CAN_TxHeaderTypeDef  tx_header2;
    uint8_t              tx_data2[4];

    tx_header2.StdId= 0x1ff;
    tx_header2.DLC =8;
    tx_header2.IDE=CAN_ID_STD;
    tx_header2.RTR=CAN_RTR_DATA;

    tx_data2[0]=   (v1)>>8 &  0xff;
    tx_data2[1]=    v1     &  0xff;
    tx_data2[2]=   (v2)>>8 &  0xff;
    tx_data2[3]=    v2     &  0xff;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0)
    {
        HAL_CAN_AddTxMessage(&hcan2, &tx_header2, tx_data2, (uint32_t *)CAN_TX_MAILBOX1);
    }
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header1;
    uint8_t rx_data[8];

    if (hcan->Instance == CAN1) {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header1, rx_data);
        //receive can data

        switch (rx_header1.StdId) {
            case 0x211: {
                boardA_info1.UART_yaw = ((rx_data[0] << 8) | rx_data[1]);
                boardA_info1.UART_pitch = ((rx_data[2] << 8) | rx_data[3]);
                boardA_info1.UART_flag1 = ((rx_data[4] << 8) | rx_data[5]);
                boardA_info1.UART_flag2 = ((rx_data[6] << 8) | rx_data[7]);

                led();
                break;
            }
            case 0x206: {
                shoot_m3508_info[0].rotor_angle = ((rx_data[0] << 8) | rx_data[1]);
                shoot_m3508_info[0].rotor_speed = ((rx_data[2] << 8) | rx_data[3]);
                shoot_m3508_info[0].torque_current = ((rx_data[4] << 8) | rx_data[5]);
                shoot_m3508_info[0].temp = rx_data[6];
                break;
            }
            case 0x208:
                //0x200+ID;
            {

                shoot_m3508_info[1].rotor_angle = ((rx_data[0] << 8) | rx_data[1]);
                shoot_m3508_info[1].rotor_speed = ((rx_data[2] << 8) | rx_data[3]);
                shoot_m3508_info[1].torque_current = ((rx_data[4] << 8) | rx_data[5]);
                shoot_m3508_info[1].temp = rx_data[6];

                break;
            }
            case 0x207: {
                shoot_m2006_info.rotor_angle = ((rx_data[0] << 8) | rx_data[1]);
                shoot_m2006_info.rotor_speed = ((rx_data[2] << 8) | rx_data[3]);
                shoot_m2006_info.torque_current = ((rx_data[4] << 8) | rx_data[5]);
                shoot_m2006_info.temp = rx_data[6];
                break;
            }

            case 0x205: {
                boardA_info1.UART_yaw = ((rx_data[0] << 8) | rx_data[1]);
                boardA_info1.UART_pitch = ((rx_data[2] << 8) | rx_data[3]);
                boardA_info1.UART_flag1 = ((rx_data[4] << 8) | rx_data[5]);
                boardA_info1.UART_flag2 = ((rx_data[6] << 8) | rx_data[7]);
                default:
                    led();
                break;
            }
        }

     }
    }

    void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
    {
        if (hcan->Instance == CAN2) {
            CAN_RxHeaderTypeDef rx_header2;
            uint8_t rx_data2[8];

            HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header2, rx_data2);
            //receive can data

            switch (rx_header2.StdId) {
                case 0x205:
                    //id=1,yaw
                {
                    // get motor index by can_id
                    yaw_6020_info.rotor_angle = ((rx_data2[0] << 8) | rx_data2[1]);
                    yaw_6020_info.rotor_speed = ((rx_data2[2] << 8) | rx_data2[3]);
                    yaw_6020_info.torque_current = ((rx_data2[4] << 8) | rx_data2[5]);
                    yaw_6020_info.temp = rx_data2[6];

                    if ((yaw_6020_info.last_encode - yaw_6020_info.rotor_angle) > 4096) {
                        yaw_6020_info.circle_count++;
                    } else if ((yaw_6020_info.last_encode - yaw_6020_info.rotor_angle) < -4096) {
                        yaw_6020_info.circle_count--;
                    }
                    yaw_6020_info.last_encode = yaw_6020_info.rotor_angle;
                    yaw_6020_info.add_encode = (yaw_6020_info.circle_count) * 8192 + yaw_6020_info.rotor_angle;
                    break;
                }
                case 0x206:
                case 0x207: {
                    pitch_6020_info.rotor_angle = ((rx_data2[0] << 8) | rx_data2[1]);
                    pitch_6020_info.rotor_speed = ((rx_data2[2] << 8) | rx_data2[3]);
                    pitch_6020_info.torque_current = ((rx_data2[4] << 8) | rx_data2[5]);
                    pitch_6020_info.temp = rx_data2[6];
                    break;
                }
            }
        }

    }

    void led(void) {
        HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
    }
