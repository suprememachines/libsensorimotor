#ifndef SUPREME_COMMUNICATION_INTERFACE_HPP
#define SUPREME_COMMUNICATION_INTERFACE_HPP

#include <cstdint>

class communication_interface {
public:

    virtual ~communication_interface() {};

    virtual void read_msg(void) = 0;
    virtual bool send_msg(void) = 0;
    virtual bool wait_us(unsigned usec) const = 0;
    virtual void sleep_s(unsigned  sec) const = 0;


    virtual void enqueue_sync_bytes(uint8_t sync) = 0;
    virtual void enqueue_byte(uint8_t byte) = 0;
    virtual void enqueue_word(uint16_t word) = 0;
    virtual void enqueue_checksum() = 0;

    /* queue-like interface */
    virtual bool empty(void) const = 0;
    virtual uint8_t front() const = 0;
    virtual void pop(void) = 0;
    virtual std::size_t size() const = 0;

    virtual bool is_checksum_ok(void) const = 0;
    virtual void reset_checksum(void) = 0;

    virtual uint8_t  get_byte(void) = 0;
    virtual uint16_t get_word(void) = 0;

};

#endif /* SUPREME_COMMUNICATION_INTERFACE_HPP */
