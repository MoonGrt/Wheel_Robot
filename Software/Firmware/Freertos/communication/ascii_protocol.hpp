#ifndef __ASCII_PROTOCOL_HPP
#define __ASCII_PROTOCOL_HPP

#include <fibre/async_stream.hpp>
#include <fibre/../../stream_utils.hpp>

#define MAX_LINE_LENGTH ((size_t)256)

class AsciiProtocol {
public:
    AsciiProtocol(fibre::AsyncStreamSource* rx_channel, fibre::AsyncStreamSink* tx_channel)
        : rx_channel_(rx_channel), sink_(*tx_channel) {}

    void start();
    template<typename ... TArgs> void respond(bool include_checksum, const char * fmt, TArgs&& ... args);

private:
    void cmd_set_position(char * pStr, bool use_checksum);
    void cmd_set_position_wl(char * pStr, bool use_checksum);
    void cmd_set_velocity(char * pStr, bool use_checksum);
    void cmd_set_torque(char * pStr, bool use_checksum);
    void cmd_set_trapezoid_trajectory(char * pStr, bool use_checksum);
    void cmd_set_servo_motor(char * pStr, bool use_checksum);
    void cmd_get_feedback(char * pStr, bool use_checksum);
    void cmd_help(char * pStr, bool use_checksum);
    void cmd_info_dump(char * pStr, bool use_checksum);
    void cmd_system_ctrl(char * pStr, bool use_checksum);
    void cmd_read_property(char * pStr, bool use_checksum);
    void cmd_write_property(char * pStr, bool use_checksum);
    void cmd_update_axis_wdg(char * pStr, bool use_checksum);
    void cmd_unknown(char * pStr, bool use_checksum);
    void cmd_encoder(char * pStr, bool use_checksum);

    void process_line(fibre::cbufptr_t buffer);
    void on_write_finished(fibre::WriteResult result);
    void on_read_finished(fibre::ReadResult result);

    fibre::AsyncStreamSource* rx_channel_ = nullptr;
    uint8_t* rx_end_ = nullptr; // non-zero if an RX operation has finished but wasn't handled yet because the TX channel was busy

    uint8_t rx_buf_[MAX_LINE_LENGTH];
    bool read_active_ = true;

    fibre::BufferedStreamSink<512> sink_;
};


// @brief Sends a line on the specified output.
template<typename ... TArgs>
void AsciiProtocol::respond(bool include_checksum, const char * fmt, TArgs&& ... args) {
    char tx_buf[64];

    size_t len = snprintf(tx_buf, sizeof(tx_buf), fmt, std::forward<TArgs>(args)...);

    // Silently truncate the output if it's too long for the buffer.
    len = std::min(len, sizeof(tx_buf));

    if (include_checksum) {
        uint8_t checksum = 0;
        for (size_t i = 0; i < len; ++i)
            checksum ^= tx_buf[i];
        len += snprintf(tx_buf + len, sizeof(tx_buf) - len, "*%u\r\n", checksum);
    } else {
        len += snprintf(tx_buf + len, sizeof(tx_buf) - len, "\r\n");
    }

    // Silently truncate the output if it's too long for the buffer.
    len = std::min(len, sizeof(tx_buf));

    sink_.write({(const uint8_t*)tx_buf, len});
    sink_.maybe_start_async_write();
}

#endif // __ASCII_PROTOCOL_HPP
