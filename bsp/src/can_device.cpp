#include "can_device.h"
#include <cstring>
namespace {
bool good_bus(uint8_t bus) {
    return bus >= 1u && bus <= 2u;
}
constexpr uint32_t max_age_ms = ROBOT_CAN_MAX_TX_AGE_MS;
static_assert(max_age_ms > 0 && max_age_ms < 0x80000000u, "invalid CAN deadline");
} // namespace
can_network::can_network() : lanes{}, enter_(nullptr), leave_(nullptr), notify_(nullptr), faults_(0) {}
can_network::guard::guard(const can_network &n) : net(n), saved(n.enter_ ? n.enter_() : 0) {}
can_network::guard::~guard() {
    if (net.leave_)
        net.leave_(saved);
}
void can_network::bind_platform(enter_fn e, leave_fn l, notify_fn n) {
    enter_ = e;
    leave_ = l;
    notify_ = n;
}
bool can_network::register_rx(can_device_receive *d) {
    guard lock(*this);
    if (!d || !good_bus(d->bus_) || d->id_ > 0x7ffu || d->length_ > 8)
        return false;
    lane &l = lanes[d->bus_ - 1];
    if (l.rx_size == max_rx_per_bus)
        return false;
    for (unsigned i = 0; i < l.rx_size; i++)
        if (l.rx[i]->id_ == d->id_)
            return false;
    l.rx[l.rx_size++] = d;
    return true;
}
void can_network::unregister_rx(can_device_receive *d) {
    guard lock(*this);
    if (!d || !good_bus(d->bus_))
        return;
    lane &l = lanes[d->bus_ - 1];
    for (unsigned i = 0; i < l.rx_size; i++)
        if (l.rx[i] == d) {
            for (unsigned j = i + 1; j < l.rx_size; j++)
                l.rx[j - 1] = l.rx[j];
            l.rx[--l.rx_size] = nullptr;
            break;
        }
}
bool can_network::dispatch(const MotorFrame &f, uint32_t now) {
    guard lock(*this);
    if (!good_bus(f.bus) || f.id > 0x7ffu || f.dlc > 8)
        return false;
    lane &l = lanes[f.bus - 1];
    for (unsigned i = 0; i < l.rx_size; i++) {
        auto *d = l.rx[i];
        if (d->id_ != f.id)
            continue;
        if (!d->accepts_frame(f)) {
            ++l.stats.rejected;
            if (d->bad_length_)
                d->bad_length_(d, f, now);
            return false;
        }
        std::memset(d->rx_data, 0, 8);
        std::memcpy(d->rx_data, f.data, f.dlc);
        if (d->callback_)
            d->callback_(d, f, now);
        return true;
    }
    ++l.stats.rejected;
    return false;
}
unsigned can_network::rx_count(uint8_t bus) const {
    guard lock(*this);
    return good_bus(bus) ? lanes[bus - 1].rx_size : 0;
}
uint16_t can_network::rx_id(uint8_t bus, unsigned index) const {
    guard lock(*this);
    return good_bus(bus) && index < lanes[bus - 1].rx_size ? lanes[bus - 1].rx[index]->id_ : 0xffffu;
}
bool can_network::enqueue(const MotorFrame &f, uint32_t now) {
    if (!good_bus(f.bus) || f.id > 0x7ffu || f.dlc > 8)
        return false;
    {
        guard lock(*this);
        lane &l = lanes[f.bus - 1];
        bool replaced = false;
        for (unsigned i = 0; i < l.tx_size; i++)
            if (l.tx[i].frame.id == f.id) {
                l.tx[i] = {f, now, l.generation};
                ++l.stats.coalesced;
                replaced = true;
                break;
            }
        if (!replaced) {
            if (l.tx_size == tx_capacity) {
                ++l.stats.rejected;
                faults_ |= 1u << (f.bus - 1);
                return false;
            }
            l.tx[l.tx_size++] = {f, now, l.generation};
            ++l.stats.enqueued;
        }
    }
    if (notify_)
        notify_(f.bus);
    return true;
}
bool can_network::pop(uint8_t bus, uint32_t now, can_device_transmit_member &out) {
    guard lock(*this);
    if (!good_bus(bus))
        return false;
    lane &l = lanes[bus - 1];
    while (l.tx_size) {
        can_device_transmit_member candidate = l.tx[0];
        for (unsigned j = 1; j < l.tx_size; j++)
            l.tx[j - 1] = l.tx[j];
        --l.tx_size;
        if (candidate.generation != l.generation || (uint32_t)(now - candidate.stamp_ms) > max_age_ms) {
            ++l.stats.expired;
            faults_ |= 1u << (bus - 1);
            continue;
        }
        out = candidate;
        return true;
    }
    return false;
}
bool can_network::is_current(const can_device_transmit_member &msg, uint32_t now) const {
    guard lock(*this);
    return good_bus(msg.frame.bus) && msg.generation == lanes[msg.frame.bus - 1].generation &&
           (uint32_t)(now - msg.stamp_ms) <= max_age_ms;
}
void can_network::cancel(uint8_t bus) {
    guard lock(*this);
    if (!good_bus(bus))
        return;
    lane &l = lanes[bus - 1];
    ++l.generation;
    l.stats.cancelled += l.tx_size;
    l.tx_size = 0;
}
unsigned can_network::pending(uint8_t bus) const {
    guard lock(*this);
    return good_bus(bus) ? lanes[bus - 1].tx_size : 0;
}
uint8_t can_network::take_faults() {
    guard lock(*this);
    uint8_t f = faults_;
    faults_ = 0;
    return f;
}
void can_network::mark_sent(uint8_t bus) {
    guard lock(*this);
    if (good_bus(bus))
        ++lanes[bus - 1].stats.sent;
}
can_bus_statistics can_network::statistics(uint8_t bus) const {
    guard lock(*this);
    return good_bus(bus) ? lanes[bus - 1].stats : can_bus_statistics{};
}
can_device_receive::can_device_receive(can_network &n, uint8_t bus, uint16_t id, can_rx_callback cb,
                                       void *ctx, uint8_t len, can_rx_callback bad)
    : net_(n), bus_(bus), length_(len), id_(id), callback_(cb), bad_length_(bad), context_(ctx),
      registered_(false) {
    registered_ = net_.register_rx(this);
}
can_device_receive::~can_device_receive() {
    if (registered_)
        net_.unregister_rx(this);
}
bool can_device_receive::accepts_frame(const MotorFrame &f) const {
    return f.bus == bus_ && f.id == id_ && f.dlc == length_;
}
bool can_device_receive::can_set_callback(can_rx_callback cb) {
    if (!cb)
        return false;
    callback_ = cb;
    return true;
}
can_device_transmit::can_device_transmit(can_network &net, uint8_t bus, uint16_t id, uint8_t len)
    : net_(net) {
    member.bus = bus;
    member.id = id;
    member.dlc = len;
}
bool can_device_transmit::can_send_msg(uint32_t now) {
    return net_.enqueue(member, now);
}
bool can_device_transmit::set_buf_data(const uint8_t *data, uint8_t len) {
    if (!data || len > 8)
        return false;
    std::memset(member.data, 0, 8);
    std::memcpy(member.data, data, len);
    member.dlc = len;
    return true;
}
bool can_device_transmit::set_id(uint16_t id) {
    if (id > 0x7ff)
        return false;
    member.id = id;
    return true;
}
can_network &robot_can_network() {
    static can_network net;
    return net;
}
