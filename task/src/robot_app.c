#include "robot_app.h"
#include "bsp_io.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "iwdg.h"
#include <string.h>
volatile uint32_t g_robot_emergency_stop;
volatile RobotDebug g_robot_debug;
static RobotCore s_core;
static volatile uint32_t s_heartbeat;
static volatile bool s_running;
static void ControlTask(void *arg) {
    (void)arg;
    RobotCore_Init(&s_core);
    if (!BSP_IO_Init())
        BSP_FatalStop();
    TickType_t wake = xTaskGetTickCount();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_running = true;
    uint8_t tx_fault_mask = 0;
    for (;;) {
        uint32_t cycles = DWT->CYCCNT, now = HAL_GetTick();
        BspSnapshot snapshot;
        RobotInput in = {0};
        RobotOutput out;
        BSP_Dbus_Service();
        BSP_IO_Snapshot(&snapshot);
        in.rc = snapshot.rc;
        in.power = snapshot.power;
        memcpy(in.dji, snapshot.dji, sizeof(in.dji));
        memcpy(in.dm, snapshot.dm, sizeof(in.dm));
        in.bus_fault_mask = snapshot.bus_fault_mask;
        in.bus_fault_mask |= tx_fault_mask;
        in.emergency_stop = g_robot_emergency_stop != 0 || BSP_UserEmergencyStop();
        in.hardware_limit_mask = BSP_UserLimitMask();
        RobotCore_Step(&s_core, &in, now, &out);
        if (out.abort_pending)
            BSP_CAN_AbortAll();
        else if (out.abort_wheel_pending)
            BSP_CAN_AbortBus(1u);
        tx_fault_mask = 0;
        for (unsigned i = 0; i < out.count; i++)
            if (!BSP_CAN_Send(&out.frames[i])) {
                ++g_robot_debug.can_send_failures;
                tx_fault_mask = (uint8_t)(1u << (out.frames[i].bus - 1u));
                BSP_CAN_AbortAll();
                break;
            }
        s_heartbeat = now;
        g_robot_debug.heartbeat_ms = now;
        ++g_robot_debug.iterations;
        /* 10ms 发布一份一致的调试快照；只有该任务写 controller/input。 */
        if (s_core.step_count % 10u == 0u) {
            g_robot_debug.input = in;
            g_robot_debug.rc_age_ms = (uint32_t)(now - in.rc.stamp_ms);
            g_robot_debug.rc_online = in.rc.valid && g_robot_debug.rc_age_ms < ROBOT_RC_TIMEOUT_MS;
            g_robot_debug.controller = s_core;
            BspCanStatistics stats;
            BSP_CAN_GetStatistics(1, &stats);
            g_robot_debug.can[0] = stats;
            BSP_CAN_GetStatistics(2, &stats);
            g_robot_debug.can[1] = stats;
        }
        uint32_t elapsed = (DWT->CYCCNT - cycles) / (SystemCoreClock / 1000000u);
        if (elapsed > g_robot_debug.max_cycle_us)
            g_robot_debug.max_cycle_us = elapsed;
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
    }
}
static void ServiceTask(void *arg) {
    (void)arg;
    for (;;) {
        uint32_t now = HAL_GetTick();
        if (s_running && (uint32_t)(now - s_heartbeat) < 30u && BSP_CAN_ServicesHealthy(now)) {
            (void)HAL_IWDG_Refresh(&hiwdg);
            /* 读取控制器标志的临界区极短，不读取中途更新的大结构。 */
            taskENTER_CRITICAL();
            bool armed = s_core.armed;
            taskEXIT_CRITICAL();
            BSP_StatusLed(armed, now);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
void RobotApp_CreateTasks(void) {
    if (configTICK_RATE_HZ != 1000)
        BSP_FatalStop();
    BSP_CAN_CreateTasks();
    const osThreadAttr_t control = {.name = "robot_control", .stack_size = 4096, .priority = osPriorityHigh2};
    const osThreadAttr_t service = {
        .name = "robot_service", .stack_size = 1536, .priority = osPriorityNormal};
    if (!osThreadNew(ControlTask, NULL, &control) || !osThreadNew(ServiceTask, NULL, &service))
        BSP_FatalStop();
}
void vApplicationStackOverflowHook(TaskHandle_t task, char *name) {
    (void)task;
    (void)name;
    BSP_FatalStop();
}
void vApplicationMallocFailedHook(void) {
    BSP_FatalStop();
}
