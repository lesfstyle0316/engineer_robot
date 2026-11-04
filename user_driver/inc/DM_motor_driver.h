#ifndef DM_MOTOR_DRIVER_H
#define DM_MOTOR_DRIVER_H
#include "motor_driver.h"
#include "can_device.h"
class DM_motor : public motor_dev {
  public:
    DM_motor(can_network &net, const DmConfig &cfg);
    bool valid() const { return valid_; }
    bool motor_ready(uint32_t now) const override;
    void motor_stop() override;
    bool motor_enable(bool enable);
    bool motor_set_position(float native_rad, float max_velocity_rad_s);
    bool apply_frame(const MotorFrame &frame, uint32_t now);
    bool can_send_msg(uint32_t now) { return valid_ && can_tx.can_send_msg(now); }
    uint16_t command_id() const { return can_tx.member.id; }
    const DmFeedback &feedback() const { return motor_data; }
    static void dm_motor_data_update_callback(can_device_receive *rx, const MotorFrame &frame, uint32_t now);

  protected:
    float motor_get_speed_forward() const override { return motor_data.velocity * 60.0f / (2.0f * ROBOT_PI); }
    float motor_get_rounds_forward() const override { return motor_data.position / (2.0f * ROBOT_PI); }

  private:
    const DmConfig &cfg_;
    DmFeedback motor_data{};
    bool valid_;
    can_device_receive can_rx;
    can_device_transmit can_tx;
};
#endif
