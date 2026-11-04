#include "bsp_io.h"
#include "can_device.h"
#include "motor_system.h"
#include "can.h"
#include "cmsis_os2.h"
/* CAN设备/电机对象参考模板；HAL和RTOS只在这一平台适配文件出现。 */
namespace {
volatile uint8_t started = 0, fault_events = 0;
volatile uint32_t service_heartbeat[2]{};
osThreadId_t send_threads[2]{};
uint32_t tx_stamp[2][3]{};
volatile uint32_t hardware_errors[2]{}, mailbox_timeouts[2]{};
constexpr uint32_t tx_wakeup = 1u;
uint32_t enter_critical() {
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    return saved;
}
void leave_critical(uint32_t saved) {
    __set_PRIMASK(saved);
}
CAN_HandleTypeDef *handle(uint8_t bus) {
    return bus == 1 ? &hcan1 : &hcan2;
}
uint8_t bus_number(CAN_HandleTypeDef *h) {
    return h->Instance == CAN1 ? 1u : 2u;
}
void notify(uint8_t bus) {
    if (__get_PRIMASK() != 0u)
        return; /* fatal path does not call RTOS */
    if (bus >= 1 && bus <= 2 && send_threads[bus - 1])
        (void)osThreadFlagsSet(send_threads[bus - 1], tx_wakeup);
}
void set_fault(uint8_t bus) {
    uint32_t saved = enter_critical();
    fault_events |= 1u << (bus - 1);
    leave_critical(saved);
}
bool filters(uint8_t bus) {
    auto &net = robot_can_network();
    unsigned count = net.rx_count(bus);
    if (count == 0 || count > 56)
        return false;
    for (unsigned first = 0; first < count; first += 4) {
        CAN_FilterTypeDef f{};
        uint16_t ids[4];
        for (unsigned i = 0; i < 4; i++)
            ids[i] = net.rx_id(bus, first + i < count ? first + i : count - 1);
        f.FilterBank = (bus == 1 ? 0 : 14) + first / 4;
        f.FilterMode = CAN_FILTERMODE_IDLIST;
        f.FilterScale = CAN_FILTERSCALE_16BIT;
        f.FilterFIFOAssignment = CAN_RX_FIFO0;
        f.FilterIdLow = ids[0] << 5;
        f.FilterMaskIdLow = ids[1] << 5;
        f.FilterIdHigh = ids[2] << 5;
        f.FilterMaskIdHigh = ids[3] << 5;
        f.FilterActivation = ENABLE;
        f.SlaveStartFilterBank = 14;
        if (HAL_CAN_ConfigFilter(handle(bus), &f) != HAL_OK)
            return false;
    }
    return true;
}
void receive(CAN_HandleTypeDef *h, uint32_t fifo) {
    while (HAL_CAN_GetRxFifoFillLevel(h, fifo)) {
        CAN_RxHeaderTypeDef header{};
        MotorFrame f{};
        if (HAL_CAN_GetRxMessage(h, fifo, &header, f.data) != HAL_OK) {
            set_fault(bus_number(h));
            return;
        }
        if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.DLC > 8)
            continue;
        f.bus = bus_number(h);
        f.id = static_cast<uint16_t>(header.StdId);
        f.dlc = static_cast<uint8_t>(header.DLC);
        (void)robot_can_network().dispatch(f, HAL_GetTick());
    }
}
void sender(void *arg) {
    uint8_t bus = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(arg));
    auto &net = robot_can_network();
    CAN_HandleTypeDef *h = handle(bus);
    for (;;) {
        service_heartbeat[bus - 1] = HAL_GetTick();
        bool sent = false;
        if (started & (1u << (bus - 1))) {
            /* pop、世代校验、HAL入邮箱在同一短临界区；关控不能在其间插入旧帧。 */
            uint32_t saved = enter_critical();
            can_device_transmit_member msg{};
            if (HAL_CAN_GetTxMailboxesFreeLevel(h) && net.pop(bus, HAL_GetTick(), msg) &&
                net.is_current(msg, HAL_GetTick())) {
                CAN_TxHeaderTypeDef hdr{};
                hdr.StdId = msg.frame.id;
                hdr.IDE = CAN_ID_STD;
                hdr.RTR = CAN_RTR_DATA;
                hdr.DLC = msg.frame.dlc;
                uint32_t mailbox = 0;
                if (HAL_CAN_AddTxMessage(h, &hdr, msg.frame.data, &mailbox) == HAL_OK) {
                    for (unsigned i = 0; i < 3; i++)
                        if (mailbox == (1u << i))
                            tx_stamp[bus - 1][i] = HAL_GetTick();
                    sent = true;
                } else {
                    fault_events |= 1u << (bus - 1);
                    ++hardware_errors[bus - 1];
                    net.cancel(bus);
                    (void)HAL_CAN_AbortTxRequest(h, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
                }
            }
            leave_critical(saved);
        }
        if (!sent)
            (void)osThreadFlagsWait(tx_wakeup, osFlagsWaitAny, 1u);
    }
}
/* 故障路径绕开发送任务，但仍有限时轮询，不依赖已停住的RTOS或HAL tick。 */
void emergency_frame(const MotorFrame &f) {
    if (!(started & (1u << (f.bus - 1))))
        return;
    CAN_HandleTypeDef *h = handle(f.bus);
    for (volatile unsigned i = 0; i < 200000u; i++)
        if (HAL_CAN_GetTxMailboxesFreeLevel(h)) {
            CAN_TxHeaderTypeDef hdr{};
            hdr.StdId = f.id;
            hdr.IDE = CAN_ID_STD;
            hdr.RTR = CAN_RTR_DATA;
            hdr.DLC = f.dlc;
            uint32_t mailbox;
            (void)HAL_CAN_AddTxMessage(h, &hdr, const_cast<uint8_t *>(f.data), &mailbox);
            return;
        }
}
} // namespace
extern "C" bool BSP_IO_Init(void) {
    robot_can_network().bind_platform(enter_critical, leave_critical, notify);
    if (!MotorSystem_Init())
        return false;
    for (uint8_t bus = 1; bus <= 2; bus++) {
        auto *h = handle(bus);
        if (!filters(bus) || HAL_CAN_Start(h) != HAL_OK)
            return false;
        started |= 1u << (bus - 1);
        IRQn_Type irq = bus == 1 ? CAN1_TX_IRQn : CAN2_TX_IRQn;
        HAL_NVIC_SetPriority(irq, 5, 0);
        HAL_NVIC_EnableIRQ(irq);
        if (HAL_CAN_ActivateNotification(h, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN |
                                                CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_BUSOFF | CAN_IT_ERROR |
                                                CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR_PASSIVE) != HAL_OK)
            return false;
        notify(bus);
    }
    return BSP_Dbus_Init();
}
extern "C" void BSP_CAN_CreateTasks(void) {
    const osThreadAttr_t first = {.name = "can1_send",
                                  .attr_bits = 0,
                                  .cb_mem = nullptr,
                                  .cb_size = 0,
                                  .stack_mem = nullptr,
                                  .stack_size = 1536,
                                  .priority = osPriorityHigh,
                                  .tz_module = 0,
                                  .reserved = 0};
    osThreadAttr_t second = first;
    second.name = "can2_send";
    send_threads[0] = osThreadNew(sender, reinterpret_cast<void *>(uintptr_t(1)), &first);
    send_threads[1] = osThreadNew(sender, reinterpret_cast<void *>(uintptr_t(2)), &second);
    if (!send_threads[0] || !send_threads[1])
        BSP_FatalStop();
}
extern "C" bool BSP_CAN_ServicesHealthy(uint32_t now) {
    return started == 3u && (uint32_t)(now - service_heartbeat[0]) < 30u &&
           (uint32_t)(now - service_heartbeat[1]) < 30u;
}
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h) {
    receive(h, CAN_RX_FIFO0);
}
extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *h) {
    receive(h, CAN_RX_FIFO1);
}
extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *h) {
    ++hardware_errors[bus_number(h) - 1];
    set_fault(bus_number(h));
    notify(bus_number(h));
}
extern "C" void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *h) {
    robot_can_network().mark_sent(bus_number(h));
    notify(bus_number(h));
}
extern "C" void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *h) {
    HAL_CAN_TxMailbox0CompleteCallback(h);
}
extern "C" void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *h) {
    HAL_CAN_TxMailbox0CompleteCallback(h);
}
extern "C" void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *h) {
    notify(bus_number(h));
}
extern "C" void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *h) {
    notify(bus_number(h));
}
extern "C" void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *h) {
    notify(bus_number(h));
}
extern "C" void CAN1_TX_IRQHandler(void) {
    HAL_CAN_IRQHandler(&hcan1);
}
extern "C" void CAN2_TX_IRQHandler(void) {
    HAL_CAN_IRQHandler(&hcan2);
}
extern "C" void BSP_IO_Snapshot(BspSnapshot *out) {
    uint32_t saved = enter_critical();
    BSP_Dbus_Copy(&out->rc);
    MotorSystem_CopyFeedback(out->dji, out->dm, &out->power);
    out->bus_fault_mask = fault_events | robot_can_network().take_faults();
    fault_events = 0;
    leave_critical(saved);
    for (uint8_t bus = 1; bus <= 2; bus++) {
        auto *h = handle(bus);
        bool stuck = false;
        if (h->Instance->ESR & (CAN_ESR_BOFF | CAN_ESR_EPVF))
            out->bus_fault_mask |= 1u << (bus - 1);
        for (unsigned m = 0; m < 3; m++)
            if (HAL_CAN_IsTxMessagePending(h, 1u << m) &&
                (uint32_t)(HAL_GetTick() - tx_stamp[bus - 1][m]) > ROBOT_CAN_MAX_TX_AGE_MS)
                stuck = true;
        if (stuck) {
            ++mailbox_timeouts[bus - 1];
            out->bus_fault_mask |= 1u << (bus - 1);
            BSP_CAN_AbortBus(bus);
        }
        (void)HAL_CAN_ResetError(h);
    }
}
extern "C" bool BSP_CAN_Send(const MotorFrame *f) {
    return MotorSystem_SendFrame(f, HAL_GetTick());
}
extern "C" void BSP_CAN_AbortBus(uint8_t bus) {
    if (bus < 1 || bus > 2)
        return;
    uint32_t saved = enter_critical();
    robot_can_network().cancel(bus);
    if (started & (1u << (bus - 1)))
        (void)HAL_CAN_AbortTxRequest(handle(bus), CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    leave_critical(saved);
    notify(bus);
}
extern "C" void BSP_CAN_AbortAll(void) {
    uint32_t saved = enter_critical();
    BSP_CAN_AbortBus(1);
    BSP_CAN_AbortBus(2);
    leave_critical(saved);
    notify(1);
    notify(2);
}
extern "C" void BSP_CAN_GetStatistics(uint8_t bus, BspCanStatistics *out) {
    if (!out)
        return;
    auto s = robot_can_network().statistics(bus);
    out->enqueued = s.enqueued;
    out->coalesced = s.coalesced;
    out->completed = s.sent;
    out->expired = s.expired;
    out->rejected = s.rejected;
    out->cancelled = s.cancelled;
    out->pending = robot_can_network().pending(bus);
    out->hardware_errors = (bus >= 1 && bus <= 2) ? hardware_errors[bus - 1] : 0;
    out->mailbox_timeouts = (bus >= 1 && bus <= 2) ? mailbox_timeouts[bus - 1] : 0;
}
extern "C" void BSP_FatalStop(void) {
    __disable_irq();
    BSP_CAN_AbortAll();
    for (volatile unsigned i = 0; i < 2000u; i++)
        __NOP();
    MotorFrame f{};
    const int16_t zero[4] = {0};
    Dji_GroupFrame(&f, 1, 0x200, zero);
    emergency_frame(f);
    Dji_GroupFrame(&f, 2, 0x200, zero);
    emergency_frame(f);
    Dji_GroupFrame(&f, 2, 0x1ff, zero);
    emergency_frame(f);
    for (unsigned i = 0; i < DM_COUNT; i++) {
        Dm_SpecialFrame(&f, &g_dm_config[i], false);
        emergency_frame(f);
    }
    for (;;)
        __NOP();
}
extern "C" void __cxa_pure_virtual(void) {
    BSP_FatalStop();
}
