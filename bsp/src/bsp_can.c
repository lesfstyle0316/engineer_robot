#include "bsp_io.h"
#include "can.h"
#include "stm32f4xx_hal.h"
#include <string.h>
static volatile DjiFeedback s_dji[DJI_COUNT];
static volatile DmFeedback s_dm[DM_COUNT];
static volatile PowerTelemetry s_power;
static volatile uint8_t s_fault_events;
static uint8_t s_started;
static uint32_t s_tx_stamp[2][3];
static CAN_HandleTypeDef *handle(uint8_t bus){return bus==1?&hcan1:&hcan2;}
extern bool BSP_Dbus_Init(void);
extern void BSP_Dbus_Copy(RcState *out);
bool BSP_IO_Init(void) {
    for(unsigned bus=1;bus<=2;bus++) {
        CAN_FilterTypeDef f={0};
        f.FilterBank=bus==1?0:14;f.FilterMode=CAN_FILTERMODE_IDMASK;
        f.FilterScale=CAN_FILTERSCALE_32BIT;f.FilterFIFOAssignment=CAN_RX_FIFO0;
        f.FilterActivation=ENABLE;f.SlaveStartFilterBank=14;
        CAN_HandleTypeDef *h=handle((uint8_t)bus);
        if(HAL_CAN_ConfigFilter(h,&f)!=HAL_OK||HAL_CAN_Start(h)!=HAL_OK)return false;
        s_started|=(uint8_t)(1u<<(bus-1));
        if(HAL_CAN_ActivateNotification(h,CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_RX_FIFO0_OVERRUN|
            CAN_IT_BUSOFF|CAN_IT_ERROR|CAN_IT_LAST_ERROR_CODE|CAN_IT_ERROR_PASSIVE)!=HAL_OK)return false;
    }
    return BSP_Dbus_Init();
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h) {
    uint8_t bus=h->Instance==CAN1?1:2;
    while(HAL_CAN_GetRxFifoFillLevel(h,CAN_RX_FIFO0)>0) {
        CAN_RxHeaderTypeDef header;uint8_t data[8];
        if(HAL_CAN_GetRxMessage(h,CAN_RX_FIFO0,&header,data)!=HAL_OK){s_fault_events|=1u<<(bus-1);break;}
        if(header.IDE!=CAN_ID_STD||header.RTR!=CAN_RTR_DATA)continue;
        uint32_t now=HAL_GetTick();
        /* 必须先处理DLC4裁判帧，再筛选DLC8电机帧。 */
        if(bus==ROBOT_POWER_CAN_BUS && header.StdId==ROBOT_POWER_CAN_ID) {
            PowerTelemetry v=s_power;
            (void)PowerTelemetry_Decode(&v,data,(uint8_t)header.DLC,ROBOT_POWER_BYTE_ORDER,now);
            s_power=v;continue;
        }
        if(header.DLC!=8u)continue;
        for(unsigned i=0;i<DJI_COUNT;i++)if(g_dji_config[i].bus==bus&&header.StdId==g_dji_config[i].feedback_id) {
            DjiFeedback v=s_dji[i];
            if(Dji_Decode(&v,data,8,now))s_dji[i]=v;
        }
        if(bus==2)for(unsigned i=0;i<DM_COUNT;i++)if(header.StdId==g_dm_config[i].master_id) {
            DmFeedback v=s_dm[i];
            if(Dm_Decode(&v,&g_dm_config[i],data,8,now))s_dm[i]=v;
        }
    }
}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *h){s_fault_events|=h->Instance==CAN1?1u:2u;}
void BSP_IO_Snapshot(BspSnapshot *out) {
    uint32_t saved=__get_PRIMASK();__disable_irq();
    BSP_Dbus_Copy(&out->rc);
    out->power=s_power;
    for(unsigned i=0;i<DJI_COUNT;i++)out->dji[i]=s_dji[i];
    for(unsigned i=0;i<DM_COUNT;i++)out->dm[i]=s_dm[i];
    out->bus_fault_mask=s_fault_events;s_fault_events=0;
    __set_PRIMASK(saved);
    for(unsigned b=0;b<2;b++) {
        CAN_HandleTypeDef *h=handle((uint8_t)(b+1));
        if(h->Instance->ESR&(CAN_ESR_BOFF|CAN_ESR_EPVF))out->bus_fault_mask|=(uint8_t)(1u<<b);
        for(unsigned m=0;m<3;m++)if(HAL_CAN_IsTxMessagePending(h,1u<<m) &&
            (uint32_t)(HAL_GetTick()-s_tx_stamp[b][m])>5u) {
            out->bus_fault_mask|=(uint8_t)(1u<<b);
            (void)HAL_CAN_AbortTxRequest(h,1u<<m);
        }
        (void)HAL_CAN_ResetError(h);
    }
}
bool BSP_CAN_Send(const MotorFrame *f) {
    if(f->bus<1||f->bus>2||f->dlc>8||f->id>0x7ffu)return false;
    CAN_HandleTypeDef *h=handle(f->bus);
    if(!(s_started&(1u<<(f->bus-1)))||HAL_CAN_GetTxMailboxesFreeLevel(h)==0u) {
        s_fault_events|=(uint8_t)(1u<<(f->bus-1));return false;
    }
    CAN_TxHeaderTypeDef hdr={0};hdr.StdId=f->id;hdr.IDE=CAN_ID_STD;hdr.RTR=CAN_RTR_DATA;hdr.DLC=f->dlc;
    uint32_t mailbox;
    if(HAL_CAN_AddTxMessage(h,&hdr,(uint8_t*)f->data,&mailbox)!=HAL_OK) {
        s_fault_events|=(uint8_t)(1u<<(f->bus-1));return false;
    }
    for(unsigned m=0;m<3;m++)if(mailbox==(1u<<m))s_tx_stamp[f->bus-1][m]=HAL_GetTick();
    return true;
}
void BSP_CAN_AbortBus(uint8_t bus) {
    if(bus>=1u && bus<=2u && (s_started&(1u<<(bus-1))))
        (void)HAL_CAN_AbortTxRequest(handle(bus),CAN_TX_MAILBOX0|CAN_TX_MAILBOX1|CAN_TX_MAILBOX2);
}
void BSP_CAN_AbortAll(void) {
    if(s_started&1u)(void)HAL_CAN_AbortTxRequest(&hcan1,CAN_TX_MAILBOX0|CAN_TX_MAILBOX1|CAN_TX_MAILBOX2);
    if(s_started&2u)(void)HAL_CAN_AbortTxRequest(&hcan2,CAN_TX_MAILBOX0|CAN_TX_MAILBOX1|CAN_TX_MAILBOX2);
}
/* 异常处理不依赖调度器、HAL 时钟或动态内存。总线故障时只能尽力发送。 */
static void emergency_frame(const MotorFrame *f) {
    if(!(s_started&(1u<<(f->bus-1))))return;
    for(volatile unsigned i=0;i<200000u;i++) {
        if(HAL_CAN_GetTxMailboxesFreeLevel(handle(f->bus))) {(void)BSP_CAN_Send(f);return;}
    }
}
void BSP_FatalStop(void) {
    __disable_irq();BSP_CAN_AbortAll();
    for(volatile unsigned i=0;i<2000u;i++)__NOP();
    MotorFrame f;const int16_t zero[4]={0};
    Dji_GroupFrame(&f,1,0x200,zero);emergency_frame(&f);
    Dji_GroupFrame(&f,2,0x200,zero);emergency_frame(&f);
    Dji_GroupFrame(&f,2,0x1ff,zero);emergency_frame(&f);
    for(unsigned i=0;i<DM_COUNT;i++){Dm_SpecialFrame(&f,&g_dm_config[i],false);emergency_frame(&f);}
    for(;;)__NOP(); /* 不再喂独立看门狗 */
}
