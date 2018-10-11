#ifndef SUPREME_COMMUNICATION_CTRL_HPP
#define SUPREME_COMMUNICATION_CTRL_HPP

#include <unistd.h>
#include <mutex>
#include <queue>
#include "serial/rs232.h"
#include "communication_interface.hpp"

namespace supreme {

template <unsigned BaudRate = 1000000>
class communication_controller : public communication_interface {

    typedef std::mutex               mutex_t;
    typedef std::lock_guard<mutex_t> lock_t;

    const int def_device = 17; /* /dev/ttyUSB1 */
    const int alt_device = 16; /* /dev/ttyUSB0 */

    /**TODO try checking all ttyUSBx until first device responds, IDs 16..21 are allowed by RS232 lib*/

    int device = def_device;

    const int baudrate = BaudRate;  /* baud */
    unsigned char buf[4096];
    const char mode[4] = {'8','N','1',0};

    bool connected; //TODO test regularly
    mutable mutex_t send_mtx;
    mutable mutex_t recv_mtx;

    std::queue<uint8_t> send_queue;
    std::queue<uint8_t> recv_queue;
    uint8_t send_checksum = 0;
    uint8_t recv_checksum = 0;

public:
    communication_controller()
    : connected(0 == RS232_OpenComport(device, baudrate, mode))
    , send_mtx()
    , recv_mtx()
    , send_queue()
    , recv_queue()
    {
        if (connected)
            sts_msg("Connected to device: %d", device);
        else {
            wrn_msg("Could not connect to default device, trying alternative device...");
            device = alt_device;
            connected = (0 == RS232_OpenComport(device, baudrate, mode));
            if (!connected)
                err_msg(__FILE__,__LINE__, "Can not connect to device %d\n", device);
        }
        sts_msg("Connected to serial device.");
    }

    bool wait_us(unsigned usec) const { return usleep(usec) == 0; }
    void sleep_s(unsigned  sec) const { sleep(sec); }

    void read_msg() {
        lock_t lock(recv_mtx);
        if (not connected) return;

        int n = RS232_PollComport(device, buf, 4095);

        if (n > 0) // copy to queue
            for (int i = 0; i < n; ++i)
                recv_queue.push(buf[i]);
    }

    void enqueue_sync_bytes(uint8_t sync) {
        lock_t lock(send_mtx);
        send_queue.push(sync);
        send_queue.push(sync);
        send_checksum += sync+sync;
    }

    void enqueue_byte(uint8_t byte) {
        lock_t lock(send_mtx);
        send_queue.push(byte);
        send_checksum += byte;
    }

    void enqueue_word(uint16_t word) {
        enqueue_byte( (uint8_t) (word >> 8) ); // high byte
        enqueue_byte( (uint8_t) (0x00ff & word) ); //  low byte
    }

    void enqueue_checksum(void) {
        lock_t lock(send_mtx);
        send_queue.push(~send_checksum + 1);
        send_checksum = 0;
    }

    bool       empty() const { lock_t lock(recv_mtx); return recv_queue.empty(); }
    void         pop()       { lock_t lock(recv_mtx); recv_queue.pop();          }
    uint8_t    front() const { lock_t lock(recv_mtx); return recv_queue.front(); }
    std::size_t size() const { lock_t lock(recv_mtx); return recv_queue.size();  }

    uint8_t get_byte() {
        lock_t lock(recv_mtx);
        assert(recv_queue.size() > 0);
        uint8_t tmp = recv_queue.front();
        recv_queue.pop();
        recv_checksum += tmp;
        return tmp;
    }

    uint16_t get_word() {
        lock_t lock(recv_mtx);
        assert(recv_queue.size() > 1);
        uint8_t tmp1 = recv_queue.front(); recv_queue.pop();
        uint8_t tmp0 = recv_queue.front(); recv_queue.pop();
        recv_checksum += tmp1;
        recv_checksum += tmp0;
        return (tmp1 << 8) + tmp0;
    }

    bool is_checksum_ok() const { return (recv_checksum == 0); }
    void reset_checksum() { recv_checksum = 0; }



    /**
        returns true if buffer was sent successfully
    */
    bool send_msg() {
        lock_t lock(send_mtx);
        if (not connected) return 0;
        std::size_t ptr = 0;
        while(not send_queue.empty()) {
            buf[ptr++] = send_queue.front();
            send_queue.pop();
            if (ptr >= sizeof(buf) - 1 )
                err_msg(__FILE__,__LINE__,"Buffer overflow.");
        }
        assert(send_checksum == 0);
        /* send buffer all at once */
        return ((int)ptr == RS232_SendBuf(device, buf, ptr));
    }
};


} /* namespace supreme */

#endif /* SUPREME_COMMUNICATION_CTRL_HPP */
