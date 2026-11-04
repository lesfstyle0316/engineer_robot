#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H
#include <cstdint>
/* 统一获取“协议轴”速度rpm和累计圈数。大疆为转子轴，达妙为减速输出轴。
 * 角度/外部减速比映射和PI继续由robot_core管理，以保留功率约束与抗饱和。 */
class motor_dev {
  public:
    float motor_get_speed() const { return direction_ * motor_get_speed_forward(); }
    float motor_get_rounds() const { return direction_ * motor_get_rounds_forward(); }
    virtual bool motor_ready(uint32_t now) const = 0;
    virtual void motor_stop() = 0;

  protected:
    explicit motor_dev(float direction) : direction_(direction) {}
    ~motor_dev() = default; /* 固定生命周期，禁止通过基类delete */
    virtual float motor_get_speed_forward() const = 0;
    virtual float motor_get_rounds_forward() const = 0;
    float direction_;
};
#endif
