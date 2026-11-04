#ifndef DJI_MOTOR_DRIVER_H
#define DJI_MOTOR_DRIVER_H
#include "motor_driver.h"
#include "can_device.h"
enum DJI_MOTOR_TYPE { DJI_M3508, DJI_M2006, DJI_GM6020 };
class DJI_motor;
class DJI_motor_group {
  public:
    DJI_motor_group(can_network &net, uint8_t bus, uint16_t id);
    bool claim(unsigned slot, DJI_motor *owner);
    void release(unsigned slot, DJI_motor *owner);
    bool write(unsigned slot, int16_t value);
    bool apply_frame(const MotorFrame &frame, uint32_t now);
    bool can_send_msg(uint32_t now) { return can_tx.can_send_msg(now); }
    uint8_t bus() const { return can_tx.member.bus; }
    uint16_t id() const { return can_tx.member.id; }
    const uint8_t *data() const { return can_tx.member.data; }

  private:
    DJI_motor *owners[4]{};
    can_device_transmit can_tx;
};
class DJI_motor : public motor_dev {
  public:
    DJI_motor(can_network &net, DJI_motor_group &group, const DjiConfig &cfg, uint8_t id,
              DJI_MOTOR_TYPE type);
    ~DJI_motor();
    bool valid() const { return valid_; }
    bool motor_ready(uint32_t now) const override;
    void motor_stop() override;
    bool motor_set_current(float normalized);        /* M3508/C620、M2006/C610；逻辑方向，-1..1 */
    bool motor_set_voltage(float normalized);        /* GM6020；逻辑方向，-1..1 */
    bool motor_set_raw_output(int16_t native_value); /* 最终CAN原始值，已做方向和功率约束 */
    bool accepts_raw(int16_t native_value) const;
    const DjiFeedback &feedback() const { return motor_data; }
    int16_t command() const { return current_to_send; }
    static void dji_motor_data_update_callback(can_device_receive *rx, const MotorFrame &frame, uint32_t now);

  protected:
    float motor_get_speed_forward() const override { return static_cast<float>(motor_data.rpm); }
    float motor_get_rounds_forward() const override {
        return static_cast<float>(motor_data.total_counts) / 8192.0f;
    }

  private:
    const DjiConfig &cfg_;
    DJI_motor_group &group_;
    DJI_MOTOR_TYPE type_;
    uint8_t slot_;
    DjiFeedback motor_data{};
    int16_t current_to_send{};
    bool valid_;
    can_device_receive can_rx;
    float protocol_limit() const;
    bool normalized_output(float value);
};
#endif
