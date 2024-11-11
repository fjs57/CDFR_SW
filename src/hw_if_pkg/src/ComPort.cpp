#include "ComPort.hpp"

#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

ComPort::ComPort(void)
{
    
}

ComPort::~ComPort(void)
{
    stop_receiving();
}

bool ComPort::open_port(string port_name)
{
    struct termios tty;

    m_port_name = port_name;

    m_fd = open(m_port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

    if ( m_fd < 0 )
    {
        LOG_ERROR("Immpossible to open port %s, error : %s", m_port_name.c_str(), strerror(errno));
        return false;
    }

    if ( tcgetattr(m_fd, &tty) )
    {
        LOG_ERROR("Immpossible to get port %s parameters, error : %s", m_port_name.c_str(), strerror(errno));
        return false;
    }

    cfsetospeed(&tty, DEFAULT_BAUDRATE);
    cfsetispeed(&tty, DEFAULT_BAUDRATE);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
    tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_iflag &= ~IGNBRK; // disable break processing
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_lflag = 0; // no signaling chars, no echo, no canonical processing

    tty.c_oflag = 0; // no remapping, no delays

    tty.c_cc[VMIN] = 0; // read doesn't block
    tty.c_cc[VTIME] = 1; // 0.5 seconds read timeout

    if ( tcsetattr(m_fd, TCSANOW, &tty) != 0 )
    {
        LOG_ERROR("Immpossible to set port %s parameters, error : %s", m_port_name.c_str(), strerror(errno));
        return false;
    }

    LOG_INFO("port %s opened successfully", m_port_name.c_str());

    return true;
}

void ComPort::receive_process(void)
{
    int count;
    uint8_t buffer[DEFAULT_RECEIVE_BUFFER_SIZE];

    while ( m_continue_receiving )
    {
        count = read(m_fd, buffer, DEFAULT_RECEIVE_BUFFER_SIZE);

        if ( count <= 0) continue;

        on_receive_bytes(buffer, count);
    }
}

void ComPort::start_receiving(void)
{
    m_continue_receiving = true;
    m_receiver_thread = thread(
        [this]{this->receive_process();}
    );
}

void ComPort::stop_receiving(void)
{
    LOG_INFO("stop receiving on port %s", m_port_name.c_str());
    m_continue_receiving = false;
    if(!m_receiver_thread.joinable()) return;
    m_receiver_thread.join();
    LOG_INFO("receiving stopped on port %s", m_port_name.c_str());
}

void ComPort::on_receive_bytes(const uint8_t *buffer, const uint32_t length)
{
    LOG_WARN("Unimplemented receive bytes virtual function. bytes received : %d", length);
}

int ComPort::send_bytes(const uint8_t *buffer, const uint32_t length)
{
    int bytes_written;
    if ( m_fd == 0 )
    {
        LOG_ERROR("impossible to send bytes on %s, the port is not opened", m_port_name.c_str());
        return -1;
    }
    m_write_mutex.lock();
    bytes_written = write(m_fd, buffer, (size_t)length);
    m_write_mutex .unlock();
    LOG_DEBUG("%d bytes written on port %s", bytes_written, m_port_name.c_str());
    return bytes_written;
}

int main(void)
{
    ComPort o;
    o.open_port("/dev/ttyACM1");
    o.start_receiving();
    sleep(5);
}