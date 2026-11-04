#include "bsp_io.h"
#include "gpio.h"
/* C 板板载驱动三极管：GPIO 高电平点亮。 */
void BSP_StatusLed(bool armed, bool rc_ok, uint8_t profile, uint32_t now) {
    bool blink = (now / 250u) % 2u;
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10,
                      (rc_ok && profile == ROBOT_PROFILE_RC_ONLY) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, armed ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, (!armed && blink) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

__weak bool BSP_UserEmergencyStop(void) {
    return false;
}
__weak uint32_t BSP_UserLimitMask(void) {
    return 0u;
}
