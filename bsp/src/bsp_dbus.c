#include "bsp_io.h"
#include "usart.h"
static uint8_t s_dma[36];
static volatile RcState s_rc;
static volatile bool s_restart;
static bool start_rx(void) {
    if(HAL_UARTEx_ReceiveToIdle_DMA(&huart3,s_dma,sizeof(s_dma))!=HAL_OK)return false;
    __HAL_DMA_DISABLE_IT(huart3.hdmarx,DMA_IT_HT);return true;
}
bool BSP_Dbus_Init(void){s_restart=!start_rx();return !s_restart;}
void BSP_Dbus_Copy(RcState *out){*out=s_rc;}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *h,uint16_t size) {
    if(h->Instance!=USART3)return;
    RcState r=s_rc;
    if(size==18u) (void)Rc_Decode(&r,s_dma,18,HAL_GetTick());
    else if(size==36u) {
        (void)Rc_Decode(&r,s_dma,18,HAL_GetTick());
        (void)Rc_Decode(&r,s_dma+18,18,HAL_GetTick());
    } else Rc_Invalidate(&r); /* 无帧头的 DBUS 不滑窗猜测同步，等下一次 IDLE */
    s_rc=r;
    if(!start_rx())s_restart=true;
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *h) {
    if(h->Instance!=USART3)return;
    RcState r=s_rc;Rc_Invalidate(&r);s_rc=r;s_restart=true;
}
void BSP_Dbus_Service(void) {
    if(!s_restart)return;
    uint32_t saved=__get_PRIMASK();__disable_irq();
    (void)HAL_UART_AbortReceive(&huart3);
    __HAL_UART_CLEAR_PEFLAG(&huart3);
    s_restart=!start_rx();__set_PRIMASK(saved);
}
