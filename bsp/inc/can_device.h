#ifndef CAN_DEVICE_H
#define CAN_DEVICE_H
/* 参考 dx_assignment5_freertos 的 can_device_receive/transmit 组织方式。
 * 本层不依赖HAL/RTOS；平台在初始化时绑定临界区和发送唤醒函数。 */
extern "C" {
#include "drv_motor.h"
}
class can_device_receive;
using can_rx_callback = void (*)(can_device_receive *, const MotorFrame &, uint32_t);
struct can_device_transmit_member {
    MotorFrame frame;
    uint32_t stamp_ms, generation; /* 按值复制全部payload */
};
struct can_bus_statistics {
    uint32_t enqueued, coalesced, sent, expired, rejected, cancelled;
};
class can_network {
  public:
    static constexpr unsigned max_rx_per_bus = ROBOT_CAN_MAX_RX_PER_BUS;
    static constexpr unsigned tx_capacity = ROBOT_CAN_TX_QUEUE_CAPACITY;
    static_assert(max_rx_per_bus > 0 && max_rx_per_bus <= 56 && tx_capacity > 0, "invalid CAN capacity");
    using enter_fn = uint32_t (*)();
    using leave_fn = void (*)(uint32_t);
    using notify_fn = void (*)(uint8_t);
    can_network();
    void bind_platform(enter_fn enter, leave_fn leave, notify_fn notify);
    bool register_rx(can_device_receive *device);
    void unregister_rx(can_device_receive *device);
    bool dispatch(const MotorFrame &frame, uint32_t now);
    unsigned rx_count(uint8_t bus) const;
    uint16_t rx_id(uint8_t bus, unsigned index) const;
    bool enqueue(const MotorFrame &frame, uint32_t now);
    bool pop(uint8_t bus, uint32_t now, can_device_transmit_member &out);
    bool is_current(const can_device_transmit_member &msg, uint32_t now) const;
    void cancel(uint8_t bus);
    unsigned pending(uint8_t bus) const;
    uint8_t take_faults();
    void mark_sent(uint8_t bus);
    can_bus_statistics statistics(uint8_t bus) const;
    can_network(const can_network &) = delete;
    can_network &operator=(const can_network &) = delete;

  private:
    struct lane {
        can_device_receive *rx[max_rx_per_bus];
        unsigned rx_size, tx_size;
        can_device_transmit_member tx[tx_capacity];
        uint32_t generation;
        can_bus_statistics stats;
    } lanes[2];
    class guard {
      public:
        explicit guard(const can_network &n);
        ~guard();

      private:
        const can_network &net;
        uint32_t saved;
    };
    enter_fn enter_;
    leave_fn leave_;
    notify_fn notify_;
    uint8_t faults_;
};
class can_device_receive {
  public:
    can_device_receive(can_network &net, uint8_t bus, uint16_t id, can_rx_callback callback,
                       void *context = nullptr, uint8_t data_length = 8,
                       can_rx_callback bad_length = nullptr);
    ~can_device_receive();
    bool accepts_frame(const MotorFrame &frame) const;
    bool can_set_callback(can_rx_callback callback);
    bool valid() const { return registered_; }
    void *context() const { return context_; }
    uint8_t bus() const { return bus_; }
    uint16_t id() const { return id_; }
    uint8_t rx_data[8]{};
    can_device_receive(const can_device_receive &) = delete;
    can_device_receive &operator=(const can_device_receive &) = delete;

  private:
    friend class can_network;
    can_network &net_;
    uint8_t bus_, length_;
    uint16_t id_;
    can_rx_callback callback_, bad_length_;
    void *context_;
    bool registered_;
};
class can_device_transmit {
  public:
    can_device_transmit(can_network &net, uint8_t bus, uint16_t id, uint8_t len = 8);
    bool can_send_msg(uint32_t now);
    bool set_buf_data(const uint8_t *data, uint8_t len); /* 设置内容，不保留外部指针 */
    bool set_id(uint16_t id);
    MotorFrame member{};

  private:
    can_network &net_;
};
/* 所有实际设备共用此网络；对象构造阶段只注册，不调用HAL或等待反馈。 */
can_network &robot_can_network();
#endif
