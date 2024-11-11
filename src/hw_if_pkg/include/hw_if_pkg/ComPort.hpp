#ifndef COM_PORT_HPP__
#define COM_PORT_HPP__

#include <string>
#include <mutex>
#include <thread>
#include <functional>

#define DEFAULT_BAUDRATE 2000000
#define DEFAULT_RECEIVE_BUFFER_SIZE 1024

#ifndef LOG
#include <cstdio>
#define LOG
#define LOG_DEBUG(format, ...)  printf("[DEBUG] " format "\n", ##__VA_ARGS__)
#define LOG_INFO(format, ...)   printf("[INFO ] " format "\n", ##__VA_ARGS__)
#define LOG_WARN(format, ...)   printf("[WARN ] " format "\n", ##__VA_ARGS__)
#define LOG_ERROR(format, ...)  printf("[ERROR] " format "\n", ##__VA_ARGS__)
#endif

#define USE_RECEIVE_CALLBACK
#define USE_RECEIVE_VIRTUAL_FUNCTION

using namespace std;

class ComPort
{

    public : 
        ComPort();
        ~ComPort();

        bool open_port(string port_name);

        void start_receiving(void);
        void stop_receiving(void);
#ifdef USE_RECEIVE_VIRTUAL_FUNCTION
        virtual void on_receive_bytes(const uint8_t *buffer, const uint32_t length);    
#endif
        int send_bytes(const uint8_t *buffer, const uint32_t length);

    private :
        int m_fd;
        mutex m_write_mutex;
        string m_port_name;
        uint32_t m_receive_timeout;
        thread m_receiver_thread;
        bool m_continue_receiving;

        void receive_process(void);

};

#endif //COM_PORT_HPP__